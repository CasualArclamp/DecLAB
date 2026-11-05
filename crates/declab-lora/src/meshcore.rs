// made by CasualArclamp
//! MeshCore packets: a header byte (route type, payload type, version),
//! transport codes on transport routes, the path (hashes of the nodes it
//! went through), then the payload. Adverts (a node's public key, name,
//! type and position) are in the clear; direct messages are encrypted to
//! the two nodes' shared secret; group messages to the channel's key, and
//! the "Public" channel's key is published, so its messages can be read.
//!
//! Layouts from MeshCore (<https://github.com/meshcore-dev/MeshCore>, MIT):
//! `docs/packet_format.md`, `docs/payloads.md`, `src/Packet.h`,
//! `src/Utils.cpp` (`MACThenDecrypt`: AES-128-ECB, a 2-byte HMAC-SHA-256),
//! `src/helpers/AdvertDataHelpers.cpp`, `src/helpers/BaseChatMesh.cpp`.

use crate::crypto::{Aes, base64_decode, hmac_sha256, sha256};

/// LoRa sync word MeshCore uses (RadioLib's private 0x12).
pub const SYNC_WORD: u8 = 0x12;

/// The Public channel's key (`PUBLIC_GROUP_PSK`).
pub const PUBLIC_PSK: &str = "izOH6cXN6mrJ5e26oRXNcg==";

pub const ROUTES: [&str; 4] = ["transport flood", "flood", "direct", "transport direct"];

pub fn payload_type_name(t: u8) -> &'static str {
    match t {
        0 => "REQ",
        1 => "RESPONSE",
        2 => "TXT_MSG",
        3 => "ACK",
        4 => "ADVERT",
        5 => "GRP_TXT",
        6 => "GRP_DATA",
        7 => "ANON_REQ",
        8 => "PATH",
        9 => "TRACE",
        10 => "MULTIPART",
        11 => "CONTROL",
        15 => "RAW_CUSTOM",
        _ => "reserved",
    }
}

/// A group channel: its name, key and hash (the first byte of SHA-256 of
/// the key).
#[derive(Clone)]
pub struct Channel {
    pub name: String,
    /// 16 or 32 bytes; AES uses the first 16, the MAC all 32 (zero padded).
    secret: [u8; 32],
    pub hash: u8,
    aes: Aes,
}

impl Channel {
    pub fn new(name: &str, psk_base64: &str) -> Result<Channel, String> {
        let k = base64_decode(psk_base64).ok_or_else(|| format!("{psk_base64:?} is not base64"))?;
        if k.len() != 16 && k.len() != 32 {
            return Err(format!("a MeshCore key is 16 or 32 bytes, not {}", k.len()));
        }
        Ok(Channel::from_key(name, &k))
    }

    fn from_key(name: &str, k: &[u8]) -> Channel {
        let mut secret = [0u8; 32];
        secret[..k.len()].copy_from_slice(k);
        Channel {
            name: name.to_string(),
            secret,
            hash: sha256(&[k])[0],
            aes: Aes::new(&secret[..16]).expect("16 bytes"),
        }
    }

    /// A hashtag channel ("#test"): its key is the first 16 bytes of
    /// SHA-256 of its name, as the MeshCore apps make it.
    pub fn hashtag(name: &str) -> Channel {
        let name = if name.starts_with('#') {
            name.to_string()
        } else {
            format!("#{name}")
        };
        let h = sha256(&[name.as_bytes()]);
        Channel::from_key(&name, &h[..16])
    }

    pub fn public() -> Channel {
        Channel::new("Public", PUBLIC_PSK).expect("the published key")
    }

    /// MAC then decrypt (`Utils::MACThenDecrypt`); None if the MAC fails.
    pub fn open(&self, mac_and_cipher: &[u8]) -> Option<Vec<u8>> {
        let (mac, ct) = mac_and_cipher.split_at_checked(2)?;
        if ct.is_empty()
            || !ct.len().is_multiple_of(16)
            || hmac_sha256(&self.secret, ct)[..2] != *mac
        {
            return None;
        }
        let mut out = ct.to_vec();
        for b in out.as_chunks_mut::<16>().0 {
            self.aes.decrypt(b);
        }
        Some(out)
    }

    /// Encrypt then MAC (`Utils::encryptThenMAC`; zero padded).
    pub fn seal(&self, plain: &[u8]) -> Vec<u8> {
        let mut ct = plain.to_vec();
        ct.resize(plain.len().div_ceil(16) * 16, 0);
        for b in ct.as_chunks_mut::<16>().0 {
            self.aes.encrypt(b);
        }
        let mut out = hmac_sha256(&self.secret, &ct)[..2].to_vec();
        out.extend_from_slice(&ct);
        out
    }
}

/// An advert's contents.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Advert {
    pub public_key: [u8; 32],
    pub timestamp: u32,
    /// 1 chat, 2 repeater, 3 room server, 4 sensor.
    pub node_type: u8,
    pub position: Option<(f64, f64)>,
    pub name: Option<String>,
}

pub fn node_type_name(t: u8) -> &'static str {
    match t {
        1 => "chat",
        2 => "repeater",
        3 => "room server",
        4 => "sensor",
        _ => "?",
    }
}

/// One decoded packet.
#[derive(Debug, Clone, Default)]
pub struct Packet {
    pub route: u8,
    pub payload_type: u8,
    pub version: u8,
    pub transport: Option<(u16, u16)>,
    /// Bytes per path hash (1–3) and the hashes.
    pub hash_size: u8,
    pub path: Vec<Vec<u8>>,
    pub payload: Vec<u8>,
    pub advert: Option<Advert>,
    /// Group text read with a channel key: (channel, text with sender).
    pub group_text: Option<(String, String)>,
    pub summary: String,
    pub fields: Vec<(String, String)>,
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Reads a MeshCore packet (a LoRa payload with sync word 0x12).
pub fn decode(b: &[u8], channels: &[Channel]) -> Option<Packet> {
    let h = *b.first()?;
    let mut p = Packet {
        route: h & 3,
        payload_type: (h >> 2) & 15,
        version: h >> 6,
        ..Default::default()
    };
    let mut i = 1;
    if p.route == 0 || p.route == 3 {
        let t = b.get(1..5)?;
        p.transport = Some((
            u16::from_le_bytes([t[0], t[1]]),
            u16::from_le_bytes([t[2], t[3]]),
        ));
        i = 5;
    }
    let pl = *b.get(i)?;
    i += 1;
    p.hash_size = (pl >> 6) + 1;
    if p.hash_size == 4 {
        return None;
    }
    let count = (pl & 63) as usize;
    let path_bytes = count * p.hash_size as usize;
    if path_bytes > 64 {
        return None;
    }
    let path = b.get(i..i + path_bytes)?;
    p.path = path
        .chunks(p.hash_size as usize)
        .map(<[u8]>::to_vec)
        .collect();
    i += path_bytes;
    p.payload = b[i..].to_vec();
    if p.payload.len() > 184 || !plausible(p.version, p.payload_type, &p.payload) {
        return None;
    }
    let ty = payload_type_name(p.payload_type);
    p.fields
        .push(("route".into(), ROUTES[p.route as usize].into()));
    p.fields.push(("type".into(), ty.into()));
    if p.version != 0 {
        p.fields
            .push(("version".into(), (p.version + 1).to_string()));
    }
    if let Some((a, c)) = p.transport {
        p.fields
            .push(("transport codes".into(), format!("{a:04x} {c:04x}")));
    }
    if !p.path.is_empty() {
        let s: Vec<String> = p.path.iter().map(|x| hex(x)).collect();
        p.fields.push(("path".into(), s.join(" ")));
    }
    let pay = p.payload.clone();
    match p.payload_type {
        4 => advert(&mut p, &pay)?,
        5 | 6 => {
            let ch_hash = *pay.first()?;
            p.fields
                .push(("channel hash".into(), format!("{ch_hash:02x}")));
            let opened = channels
                .iter()
                .filter(|c| c.hash == ch_hash)
                .find_map(|c| c.open(&pay[1..]).map(|t| (c, t)));
            match opened {
                Some((c, plain)) if p.payload_type == 5 && plain.len() >= 5 => {
                    let ts = u32::from_le_bytes(plain[..4].try_into().unwrap());
                    let text: String = String::from_utf8_lossy(&plain[5..])
                        .trim_end_matches('\0')
                        .to_string();
                    p.fields.push(("channel".into(), c.name.clone()));
                    p.fields
                        .push(("sent".into(), crate::meshtastic::unix_time(ts)));
                    p.fields.push(("text".into(), text.clone()));
                    p.summary = format!("{}: \"{text}\"", c.name);
                    p.group_text = Some((c.name.clone(), text));
                }
                Some((c, plain)) => {
                    p.fields.push(("channel".into(), c.name.clone()));
                    p.summary = format!("{}: data {} bytes", c.name, plain.len());
                }
                None => p.summary = format!("group {ty}, channel {ch_hash:02x} (no key)"),
            }
        }
        0 | 1 | 2 | 8 => {
            let (d, s) = (*pay.first()?, *pay.get(1)?);
            p.fields.push(("to".into(), format!("{d:02x}")));
            p.fields.push(("from".into(), format!("{s:02x}")));
            p.summary = format!(
                "{s:02x} → {d:02x} encrypted ({} bytes)",
                pay.len().saturating_sub(4)
            );
        }
        7 => {
            let d = *pay.first()?;
            let k = pay.get(1..33)?;
            p.fields.push(("to".into(), format!("{d:02x}")));
            p.fields.push(("from key".into(), hex(k)));
            p.summary = format!("anonymous request to {d:02x} from {}…", hex(&k[..4]));
        }
        3 => {
            let c = pay.get(..4)?;
            p.fields.push(("ack checksum".into(), hex(c)));
            p.summary = format!("ack {}", hex(c));
        }
        9 => {
            let t = pay.get(..9)?;
            p.fields.push(("trace tag".into(), hex(&t[..4])));
            p.summary = format!("trace, {} SNRs so far", p.path.len());
        }
        11 => {
            let f = *pay.first()?;
            let sub = f >> 4;
            p.fields.push(("control".into(), format!("sub type {sub}")));
            p.summary = match sub {
                8 => "discover request".into(),
                9 => {
                    let snr = *pay.get(1)? as i8 as f32 / 4.0;
                    format!("discover response, {} SNR {snr} dB", node_type_name(f & 15))
                }
                _ => format!("control {sub}"),
            };
        }
        _ => p.summary = format!("{ty} {} bytes", pay.len()),
    }
    Some(p)
}

/// Whether a payload has the shape its type needs: other users of the
/// private sync word 0x12 send frames that parse as anything otherwise.
/// Only version 1 exists; encrypted parts are whole AES blocks.
fn plausible(version: u8, ty: u8, b: &[u8]) -> bool {
    let blocks = |n: usize| b.len() >= n + 16 && (b.len() - n).is_multiple_of(16);
    version == 0
        && match ty {
            // Hashes and MAC, then the ciphertext.
            0..=2 | 8 => blocks(4),
            3 => b.len() == 4,
            4 => b.len() >= 101,
            5 | 6 => blocks(3),
            // Destination hash, sender's key, MAC.
            7 => blocks(35),
            9 => b.len() >= 9,
            10 | 11 | 15 => !b.is_empty(),
            _ => false,
        }
}

fn advert(p: &mut Packet, b: &[u8]) -> Option<()> {
    if b.len() < 101 {
        return None;
    }
    let mut a = Advert {
        public_key: b[..32].try_into().unwrap(),
        timestamp: u32::from_le_bytes(b[32..36].try_into().unwrap()),
        ..Default::default()
    };
    // b[36..100] is the Ed25519 signature (not checked here).
    let app = &b[100..];
    if let Some(&flags) = app.first() {
        a.node_type = flags & 15;
        let mut i = 1;
        if flags & 0x10 != 0 {
            let lat = i32::from_le_bytes(app.get(i..i + 4)?.try_into().unwrap());
            let lon = i32::from_le_bytes(app.get(i + 4..i + 8)?.try_into().unwrap());
            a.position = Some((lat as f64 * 1e-6, lon as f64 * 1e-6));
            i += 8;
        }
        if flags & 0x20 != 0 {
            i += 2;
        }
        if flags & 0x40 != 0 {
            i += 2;
        }
        if flags & 0x80 != 0 && i <= app.len() {
            a.name = Some(
                String::from_utf8_lossy(&app[i..])
                    .trim_end_matches('\0')
                    .to_string(),
            );
        }
    }
    p.fields.push(("public key".into(), hex(&a.public_key)));
    p.fields
        .push(("node type".into(), node_type_name(a.node_type).into()));
    if let Some(n) = &a.name {
        p.fields.push(("name".into(), n.clone()));
    }
    if let Some((la, lo)) = a.position {
        p.fields
            .push(("position".into(), format!("{la:.5}, {lo:.5}")));
    }
    p.fields.push((
        "advertised".into(),
        crate::meshtastic::unix_time(a.timestamp),
    ));
    p.summary = format!(
        "{} \"{}\"{}",
        node_type_name(a.node_type),
        a.name.clone().unwrap_or_default(),
        a.position
            .map_or(String::new(), |(la, lo)| format!(" {la:.4}, {lo:.4}"))
    );
    p.advert = Some(a);
    Some(())
}

/// A node's ID as shown: the first 4 bytes of its key, hex.
pub fn key_id(k: &[u8]) -> String {
    hex(&k[..4.min(k.len())])
}

/// Packet bytes (for the simulator): flood route, an empty or given path.
pub fn build(payload_type: u8, route: u8, path: &[u8], payload: &[u8]) -> Vec<u8> {
    let mut b = vec![(payload_type << 2) | route];
    if route == 0 || route == 3 {
        b.extend_from_slice(&[0, 0, 0, 0]);
    }
    b.push(path.len() as u8);
    b.extend_from_slice(path);
    b.extend_from_slice(payload);
    b
}

/// An advert's payload (signature left zero: not checked).
pub fn advert_payload(
    key: &[u8; 32],
    ts: u32,
    node_type: u8,
    pos: Option<(f64, f64)>,
    name: &str,
) -> Vec<u8> {
    let mut b = key.to_vec();
    b.extend_from_slice(&ts.to_le_bytes());
    b.extend_from_slice(&[0u8; 64]);
    let mut flags = node_type | 0x80;
    if pos.is_some() {
        flags |= 0x10;
    }
    b.push(flags);
    if let Some((la, lo)) = pos {
        b.extend_from_slice(&((la * 1e6).round() as i32).to_le_bytes());
        b.extend_from_slice(&((lo * 1e6).round() as i32).to_le_bytes());
    }
    b.extend_from_slice(name.as_bytes());
    b
}

/// A group text payload: channel hash, MAC, ciphertext of timestamp,
/// flags 0 and "sender: text".
pub fn group_text_payload(ch: &Channel, ts: u32, sender_and_text: &str) -> Vec<u8> {
    let mut plain = ts.to_le_bytes().to_vec();
    plain.push(0);
    plain.extend_from_slice(sender_and_text.as_bytes());
    let mut b = vec![ch.hash];
    b.extend_from_slice(&ch.seal(&plain));
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn advert_round_trip() {
        let key = [7u8; 32];
        let pay = advert_payload(&key, 1_790_000_000, 2, Some((51.5, -0.12)), "Test Rptr");
        let b = build(4, 1, &[0xaa, 0xbb], &pay);
        let p = decode(&b, &[]).unwrap();
        let a = p.advert.unwrap();
        assert_eq!(a.name.as_deref(), Some("Test Rptr"));
        assert_eq!(a.node_type, 2);
        let (la, lo) = a.position.unwrap();
        assert!((la - 51.5).abs() < 1e-6 && (lo + 0.12).abs() < 1e-6);
        assert_eq!(p.path, vec![vec![0xaa], vec![0xbb]]);
    }

    #[test]
    fn other_private_users_are_not_meshcore() {
        // Short frames other 0x12 users send: no MeshCore type fits them.
        for b in [
            &[0x05u8, 0x00, 0x12, 0x34][..],
            &[0x21, 0x00],
            &[0x31, 0x00, 1, 2, 3],
        ] {
            assert!(decode(b, &[]).is_none(), "{b:02x?}");
        }
    }

    #[test]
    fn public_group_text() {
        let ch = Channel::public();
        let pay = group_text_payload(&ch, 1_790_000_000, "alice: hello all");
        let p = decode(&build(5, 1, &[], &pay), &[Channel::public()]).unwrap();
        assert_eq!(
            p.group_text,
            Some(("Public".into(), "alice: hello all".into()))
        );
        // Without the key: header only.
        let p = decode(&build(5, 1, &[], &pay), &[]).unwrap();
        assert!(p.group_text.is_none() && p.summary.contains("no key"));
        // A hashtag channel.
        let t = Channel::hashtag("test");
        let pay = group_text_payload(&t, 5, "bob: hi");
        let p = decode(
            &build(5, 1, &[], &pay),
            &[Channel::public(), Channel::hashtag("#test")],
        )
        .unwrap();
        assert_eq!(p.group_text.unwrap().0, "#test");
    }
}
