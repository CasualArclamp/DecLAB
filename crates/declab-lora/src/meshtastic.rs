// made by CasualArclamp
//! Meshtastic packets as they go over the air: a 16-byte radio header in
//! the clear, then a `Data` protobuf encrypted with the channel's key
//! (AES-CTR). Every node starts on the default channel, named after its
//! modem preset ("LongFast") with the published default key, so most of
//! what is heard can be read.
//!
//! Layouts from the Meshtastic firmware (`RadioInterface.h`
//! `PacketHeader`, `CryptoEngine::initNonce`, `Channels::generateHash`,
//! GPL-3.0) and field numbers from meshtastic/protobufs (`mesh.proto`,
//! `telemetry.proto`, `mqtt.proto`, `portnums.proto`, GPL-3.0).

use crate::crypto::{Aes, base64_decode};
use crate::mt_names::{hw_model_name, port_name, role_name, routing_error_name};
use crate::proto::{self, Wire, get};

/// LoRa sync word Meshtastic uses (`RadioLibInterface`: 0x2B).
pub const SYNC_WORD: u8 = 0x2B;

/// The default channel key ("AQ==", key index 1, expands to this;
/// `Channels.cpp` `defaultpsk`).
pub const DEFAULT_KEY: [u8; 16] = [
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
];

/// The modem presets (`RadioInterface.cpp`, `ModemPreset` in config.proto):
/// name (also the default channel's name), bandwidth, SF, coding rate 4/x.
pub const PRESETS: &[(&str, f64, u8, u8)] = &[
    ("ShortTurbo", 500e3, 7, 5),
    ("ShortFast", 250e3, 7, 5),
    ("ShortSlow", 250e3, 8, 5),
    ("MediumFast", 250e3, 9, 5),
    ("MediumSlow", 250e3, 10, 5),
    ("LongFast", 250e3, 11, 5),
    ("LongTurbo", 500e3, 11, 8),
    ("LongModerate", 125e3, 11, 8),
    ("LongSlow", 125e3, 12, 8),
    ("VLongSlow", 62.5e3, 12, 8),
];

/// The preset a bandwidth and SF belong to (the first if two share them).
pub fn preset_name(bw_hz: f64, sf: u8) -> Option<&'static str> {
    PRESETS
        .iter()
        .find(|p| (p.1 - bw_hz).abs() < 1.0 && p.2 == sf)
        .map(|p| p.0)
}

/// A channel the decoder can read: its name and key.
#[derive(Debug, Clone)]
pub struct Channel {
    pub name: String,
    /// The key as used: 16 or 32 bytes, or empty (no encryption).
    pub key: Vec<u8>,
    /// What the radio header carries to say which channel (`generateHash`:
    /// the name's bytes XORed, XOR the key's).
    pub hash: u8,
    aes: Option<Aes>,
}

impl Channel {
    /// From a name and a PSK as the apps show it (base64): one byte is a
    /// key index (0: no encryption; n: the default key with n − 1 added to
    /// its last byte), else 16 or 32 bytes of key.
    pub fn new(name: &str, psk_base64: &str) -> Result<Channel, String> {
        let psk =
            base64_decode(psk_base64).ok_or_else(|| format!("{psk_base64:?} is not base64"))?;
        let key = match psk.len() {
            0 => Vec::new(),
            1 if psk[0] == 0 => Vec::new(),
            1 => {
                let mut k = DEFAULT_KEY.to_vec();
                k[15] = k[15].wrapping_add(psk[0] - 1);
                k
            }
            16 | 32 => psk,
            n => return Err(format!("a key is 1, 16 or 32 bytes, not {n}")),
        };
        let x = |b: &[u8]| b.iter().fold(0u8, |a, &c| a ^ c);
        Ok(Channel {
            name: name.to_string(),
            hash: x(name.as_bytes()) ^ x(&key),
            aes: Aes::new(&key),
            key,
        })
    }

    /// The default channel of a preset ("LongFast", key "AQ==").
    pub fn preset_default(name: &str) -> Channel {
        Channel::new(name, "AQ==").expect("the default key")
    }

    /// Decrypts (or encrypts: CTR is its own inverse) a payload in place.
    /// The nonce is the packet ID (as 64 bits) then the sender, little
    /// endian (`CryptoEngine::initNonce`).
    pub fn crypt(&self, from: u32, id: u32, data: &mut [u8]) {
        if let Some(aes) = &self.aes {
            let mut iv = [0u8; 16];
            iv[0..4].copy_from_slice(&id.to_le_bytes());
            iv[8..12].copy_from_slice(&from.to_le_bytes());
            aes.ctr(&iv, data);
        }
    }
}

/// Every preset's default channel: what a receiver tries first.
pub fn default_channels() -> Vec<Channel> {
    PRESETS
        .iter()
        .map(|p| Channel::preset_default(p.0))
        .collect()
}

/// The 16 bytes before the payload (`PacketHeader`, little endian).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Header {
    pub to: u32,
    pub from: u32,
    pub id: u32,
    pub flags: u8,
    pub channel: u8,
    /// Last byte of the next hop's node number (0: none).
    pub next_hop: u8,
    /// Last byte of the node that relayed it to us (0: none).
    pub relay: u8,
}

pub const BROADCAST: u32 = 0xFFFF_FFFF;

impl Header {
    pub fn parse(b: &[u8]) -> Option<Header> {
        let u = |i: usize| u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
        (b.len() >= 16).then(|| Header {
            to: u(0),
            from: u(4),
            id: u(8),
            flags: b[12],
            channel: b[13],
            next_hop: b[14],
            relay: b[15],
        })
    }

    pub fn bytes(&self) -> [u8; 16] {
        let mut b = [0u8; 16];
        b[0..4].copy_from_slice(&self.to.to_le_bytes());
        b[4..8].copy_from_slice(&self.from.to_le_bytes());
        b[8..12].copy_from_slice(&self.id.to_le_bytes());
        b[12] = self.flags;
        b[13] = self.channel;
        b[14] = self.next_hop;
        b[15] = self.relay;
        b
    }

    /// Hops it may still take (`PACKET_FLAGS_HOP_LIMIT_MASK`).
    pub fn hop_limit(&self) -> u8 {
        self.flags & 7
    }
    /// Hops it was sent with (`PACKET_FLAGS_HOP_START_MASK`, bits 5–7).
    pub fn hop_start(&self) -> u8 {
        self.flags >> 5
    }
    pub fn want_ack(&self) -> bool {
        self.flags & 8 != 0
    }
    pub fn via_mqtt(&self) -> bool {
        self.flags & 16 != 0
    }
}

/// A node number as the apps write it: "!a1b2c3d4"; broadcast is "all".
pub fn node_name(n: u32) -> String {
    if n == BROADCAST {
        "all".into()
    } else {
        format!("!{n:08x}")
    }
}

/// What a packet said about its sender, for the nodes list.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NodeNews {
    pub long_name: Option<String>,
    pub short_name: Option<String>,
    pub hw_model: Option<String>,
    pub role: Option<String>,
    /// Degrees and metres.
    pub position: Option<(f64, f64, Option<i32>)>,
    pub battery_pct: Option<u32>,
    pub voltage: Option<f32>,
    /// Temperature °C, humidity %, pressure hPa.
    pub environment: Option<(Option<f32>, Option<f32>, Option<f32>)>,
}

/// One decoded packet.
#[derive(Debug, Clone, Default)]
pub struct Packet {
    pub header: Header,
    /// The channel it was read with, None if no key fitted.
    pub channel: Option<String>,
    /// `Data.portnum`, if decrypted.
    pub port: Option<u32>,
    /// "TEXT_MESSAGE", "POSITION" … or "encrypted".
    pub kind: String,
    /// One line for the packet list.
    pub summary: String,
    /// A text message's text.
    pub text: Option<String>,
    /// Every field, as label and value, nested ones as "device.voltage".
    pub fields: Vec<(String, String)>,
    pub node: NodeNews,
    /// The decrypted `Data` message, for the hex view.
    pub plain: Vec<u8>,
}

/// Reads a Meshtastic packet (a LoRa payload with sync word 0x2B): the
/// header, then each channel whose hash matches is tried until one gives a
/// `Data` message that parses.
pub fn decode(bytes: &[u8], channels: &[Channel]) -> Option<Packet> {
    let header = Header::parse(bytes)?;
    let enc = &bytes[16..];
    let mut p = Packet {
        header,
        ..Default::default()
    };
    for ch in channels.iter().filter(|c| c.hash == header.channel) {
        let mut plain = enc.to_vec();
        ch.crypt(header.from, header.id, &mut plain);
        let Some(fields) = proto::parse(&plain) else {
            continue;
        };
        let Some(port) = get(&fields, 1).and_then(|w| w.u32()) else {
            continue;
        };
        if port_name(port).is_none() && port < 256 {
            continue;
        }
        p.channel = Some(ch.name.clone());
        p.port = Some(port);
        describe_data(&mut p, &fields, port);
        p.plain = plain;
        return Some(p);
    }
    // No key: a direct message to one node (channel hash 0, PKI) or a
    // channel we have no key for.
    p.kind = "encrypted".into();
    p.summary = if header.channel == 0 && header.to != BROADCAST {
        format!("direct message, PKI encrypted ({} bytes)", enc.len())
    } else {
        format!(
            "channel hash 0x{:02x}, no key ({} bytes)",
            header.channel,
            enc.len()
        )
    };
    Some(p)
}

fn deg(i: i32) -> f64 {
    i as f64 * 1e-7
}

fn f(v: f32) -> String {
    format!("{v:.2}")
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}

fn describe_data(p: &mut Packet, d: &[(u32, Wire<'_>)], port: u32) {
    let name = port_name(port).unwrap_or("PRIVATE_APP");
    p.kind = name.trim_end_matches("_APP").to_string();
    let payload = get(d, 2).and_then(|w| w.bytes()).unwrap_or(&[]);
    let mut push = |k: &str, v: String| p.fields.push((k.to_string(), v));
    push("port", format!("{port} {name}"));
    if let Some(r) = get(d, 6).and_then(|w| w.u32()).filter(|&r| r != 0) {
        push("request id", format!("{r:08x}"));
    }
    if let Some(r) = get(d, 7).and_then(|w| w.u32()).filter(|&r| r != 0) {
        push("reply to", format!("{r:08x}"));
    }
    if get(d, 3).and_then(|w| w.u64()) == Some(1) {
        push("want response", "yes".into());
    }
    if get(d, 10).is_some() {
        push("signed", "XEdDSA".into());
    }
    let msg = proto::parse(payload);
    match (port, msg) {
        (1, _) => {
            let t = String::from_utf8_lossy(payload).into_owned();
            p.summary = format!("\"{t}\"");
            p.fields.push(("text".into(), t.clone()));
            p.text = Some(t);
        }
        (3, Some(m)) => position(p, &m, ""),
        (4, Some(m)) => user(p, &m),
        (5, Some(m)) => {
            if let Some(e) = get(&m, 3).and_then(|w| w.u32()) {
                let n = routing_error_name(e).unwrap_or("?");
                p.summary = if e == 0 {
                    "ack".into()
                } else {
                    format!("error {n}")
                };
                p.fields.push(("routing".into(), n.into()));
            } else {
                p.summary = "route request/reply".into();
                route(p, &m);
            }
        }
        (67, Some(m)) => telemetry(p, &m),
        (70, Some(m)) => {
            route(p, &m);
            p.summary = "traceroute".into();
        }
        (71, Some(m)) => {
            let n: Vec<String> = m
                .iter()
                .filter(|(k, _)| *k == 4)
                .filter_map(|(_, w)| proto::parse(w.bytes()?))
                .map(|nb| {
                    let id = get(&nb, 1).and_then(|w| w.u32()).unwrap_or(0);
                    let snr = get(&nb, 2).and_then(|w| w.f32()).unwrap_or(0.0);
                    format!("{} {} dB", node_name(id), f(snr))
                })
                .collect();
            p.summary = format!("{} neighbours", n.len());
            p.fields.push(("neighbours".into(), n.join(", ")));
        }
        (8, Some(m)) => {
            let name = get(&m, 6).and_then(|w| w.str()).unwrap_or_default();
            let lat = get(&m, 2).and_then(|w| w.i32()).map(deg);
            let lon = get(&m, 3).and_then(|w| w.i32()).map(deg);
            p.summary = format!("waypoint \"{name}\"");
            p.fields.push(("waypoint".into(), name));
            if let (Some(a), Some(b)) = (lat, lon) {
                p.fields
                    .push(("position".into(), format!("{a:.5}, {b:.5}")));
            }
            if let Some(t) = get(&m, 7).and_then(|w| w.str()) {
                p.fields.push(("description".into(), t));
            }
        }
        (73, Some(m)) => {
            let s = |n| get(&m, n).and_then(|w| w.str());
            p.node.long_name = s(1);
            p.node.short_name = s(2);
            p.node.role = get(&m, 3)
                .and_then(|w| w.u32())
                .map(|r| role_name(r).unwrap_or("?").into());
            p.node.hw_model = get(&m, 4)
                .and_then(|w| w.u32())
                .map(|h| hw_model_name(h).unwrap_or("?").into());
            for (k, v) in [("long name", 1), ("short name", 2), ("firmware", 5)] {
                if let Some(x) = s(v) {
                    p.fields.push((k.into(), x));
                }
            }
            if let (Some(a), Some(b)) = (
                get(&m, 9).and_then(|w| w.i32()),
                get(&m, 10).and_then(|w| w.i32()),
            ) {
                let alt = get(&m, 11).and_then(|w| w.i32());
                p.node.position = Some((deg(a), deg(b), alt));
                p.fields
                    .push(("position".into(), format!("{:.5}, {:.5}", deg(a), deg(b))));
            }
            p.summary = format!(
                "map report {}",
                p.node.long_name.clone().unwrap_or_default()
            );
        }
        (_, Some(m)) => {
            p.summary = format!("{} fields", m.len());
            generic(p, &m, "");
        }
        (_, None) => p.summary = format!("{} bytes", payload.len()),
    }
}

fn position(p: &mut Packet, m: &[(u32, Wire<'_>)], prefix: &str) {
    let lat = get(m, 1).and_then(|w| w.i32());
    let lon = get(m, 2).and_then(|w| w.i32());
    let alt = get(m, 3).and_then(|w| w.i32());
    let mut s = String::new();
    if let (Some(a), Some(b)) = (lat, lon) {
        s = format!("{:.5}, {:.5}", deg(a), deg(b));
        p.node.position = Some((deg(a), deg(b), alt));
        p.fields.push((format!("{prefix}position"), s.clone()));
    }
    if let Some(a) = alt {
        p.fields
            .push((format!("{prefix}altitude"), format!("{a} m")));
        s += &format!(" {a} m");
    }
    for (num, label, unit) in [
        (19, "satellites", ""),
        (15, "speed", " m/s"),
        (16, "track", "°"),
        (23, "precision bits", ""),
    ] {
        if let Some(v) = get(m, num).and_then(|w| w.u32()) {
            p.fields
                .push((format!("{prefix}{label}"), format!("{v}{unit}")));
        }
    }
    if let Some(t) = get(m, 4).and_then(|w| w.u32()).filter(|&t| t != 0) {
        p.fields.push((format!("{prefix}time"), unix_time(t)));
    }
    p.summary = if s.is_empty() {
        "position (none)".into()
    } else {
        s
    };
}

fn user(p: &mut Packet, m: &[(u32, Wire<'_>)]) {
    let s = |n| get(m, n).and_then(|w| w.str());
    p.node.long_name = s(2);
    p.node.short_name = s(3);
    p.node.hw_model = get(m, 5)
        .and_then(|w| w.u32())
        .map(|h| hw_model_name(h).unwrap_or("?").into());
    p.node.role = Some(
        role_name(get(m, 7).and_then(|w| w.u32()).unwrap_or(0))
            .unwrap_or("?")
            .into(),
    );
    for (label, v) in [
        ("id", s(1)),
        ("long name", p.node.long_name.clone()),
        ("short name", p.node.short_name.clone()),
        ("hardware", p.node.hw_model.clone()),
        ("role", p.node.role.clone()),
    ] {
        if let Some(v) = v {
            p.fields.push((label.into(), v));
        }
    }
    if get(m, 6).and_then(|w| w.u64()) == Some(1) {
        p.fields.push(("licensed".into(), "yes".into()));
    }
    if let Some(k) = get(m, 8).and_then(|w| w.bytes()) {
        p.fields
            .push(("public key".into(), format!("{} bytes", k.len())));
    }
    p.summary = format!(
        "{} ({}) {}",
        p.node.long_name.clone().unwrap_or_default(),
        p.node.short_name.clone().unwrap_or_default(),
        p.node.hw_model.clone().unwrap_or_default()
    );
}

fn telemetry(p: &mut Packet, m: &[(u32, Wire<'_>)]) {
    let mut parts = Vec::new();
    if let Some(dm) = get(m, 2).and_then(|w| proto::parse(w.bytes()?)) {
        p.node.battery_pct = get(&dm, 1).and_then(|w| w.u32());
        p.node.voltage = get(&dm, 2).and_then(|w| w.f32());
        if let Some(b) = p.node.battery_pct {
            parts.push(if b > 100 {
                "powered".to_string()
            } else {
                format!("batt {b} %")
            });
        }
        if let Some(v) = p.node.voltage {
            parts.push(format!("{} V", f(v)));
        }
        if let Some(c) = get(&dm, 3).and_then(|w| w.f32()) {
            parts.push(format!("ch util {} %", f(c)));
        }
        let labels = [
            (1, "battery", "%"),
            (2, "voltage", "V"),
            (3, "channel util", "%"),
            (4, "air util tx", "%"),
            (5, "uptime", "s"),
        ];
        metrics(p, &dm, "device.", &labels);
    }
    if let Some(em) = get(m, 3).and_then(|w| proto::parse(w.bytes()?)) {
        let t = get(&em, 1).and_then(|w| w.f32());
        let h = get(&em, 2).and_then(|w| w.f32());
        let pr = get(&em, 3).and_then(|w| w.f32());
        p.node.environment = Some((t, h, pr));
        if let Some(t) = t {
            parts.push(format!("{} °C", f(t)));
        }
        if let Some(h) = h {
            parts.push(format!("{} %RH", f(h)));
        }
        if let Some(x) = pr {
            parts.push(format!("{} hPa", f(x)));
        }
        let labels = [
            (1, "temperature", "°C"),
            (2, "humidity", "%"),
            (3, "pressure", "hPa"),
            (4, "gas resistance", "MΩ"),
            (5, "voltage", "V"),
            (6, "current", "mA"),
            (7, "IAQ", ""),
            (9, "lux", "lx"),
            (13, "wind direction", "°"),
            (14, "wind speed", "m/s"),
            (16, "wind gust", "m/s"),
            (19, "rain 1 h", "mm"),
        ];
        metrics(p, &em, "env.", &labels);
    }
    if let Some(aq) = get(m, 4).and_then(|w| proto::parse(w.bytes()?)) {
        parts.push("air quality".into());
        let labels = [
            (1, "PM1.0", "µg/m³"),
            (2, "PM2.5", "µg/m³"),
            (3, "PM10", "µg/m³"),
            (13, "CO2", "ppm"),
        ];
        metrics(p, &aq, "air.", &labels);
    }
    if let Some(pm) = get(m, 5).and_then(|w| proto::parse(w.bytes()?)) {
        parts.push("power".into());
        let labels = [
            (1, "ch1 voltage", "V"),
            (2, "ch1 current", "mA"),
            (3, "ch2 voltage", "V"),
            (4, "ch2 current", "mA"),
            (5, "ch3 voltage", "V"),
            (6, "ch3 current", "mA"),
        ];
        metrics(p, &pm, "power.", &labels);
    }
    if let Some(ls) = get(m, 6).and_then(|w| proto::parse(w.bytes()?)) {
        if let Some(n) = get(&ls, 7).and_then(|w| w.u32()) {
            parts.push(format!("{n} nodes online"));
        }
        let labels = [
            (1, "uptime", "s"),
            (2, "channel util", "%"),
            (3, "air util tx", "%"),
            (4, "packets tx", ""),
            (5, "packets rx", ""),
            (6, "packets rx bad", ""),
            (7, "nodes online", ""),
            (8, "nodes", ""),
            (15, "noise floor", "dBm"),
        ];
        metrics(p, &ls, "stats.", &labels);
    }
    if let Some(t) = get(m, 1).and_then(|w| w.u32()).filter(|&t| t != 0) {
        p.fields.push(("time".into(), unix_time(t)));
    }
    p.summary = if parts.is_empty() {
        "telemetry".into()
    } else {
        parts.join(" ")
    };
}

/// Named metrics of a telemetry sub-message: floats or integers.
fn metrics(p: &mut Packet, m: &[(u32, Wire<'_>)], prefix: &str, labels: &[(u32, &str, &str)]) {
    for (num, w) in m {
        let (label, unit) = labels
            .iter()
            .find(|l| l.0 == *num)
            .map_or((format!("#{num}"), ""), |l| (l.1.to_string(), l.2));
        let v = match w {
            Wire::Fixed32(_) => f(w.f32().unwrap_or(0.0)),
            Wire::Varint(_) if label == "noise floor" => w.i32().unwrap_or(0).to_string(),
            _ => w.u64().map_or_else(|| "…".into(), |v| v.to_string()),
        };
        let v = if unit.is_empty() {
            v
        } else {
            format!("{v} {unit}")
        };
        p.fields.push((format!("{prefix}{label}"), v));
    }
}

fn route(p: &mut Packet, m: &[(u32, Wire<'_>)]) {
    for (num, label) in [(1, "route"), (3, "route back")] {
        let hops: Vec<String> = m
            .iter()
            .filter(|(k, _)| *k == num)
            .filter_map(|(_, w)| match w {
                Wire::Fixed32(v) => Some(node_name(*v)),
                // Packed repeated fixed32.
                Wire::Bytes(b) => Some(
                    b.as_chunks::<4>()
                        .0
                        .iter()
                        .map(|c| node_name(u32::from_le_bytes(*c)))
                        .collect::<Vec<_>>()
                        .join(" "),
                ),
                _ => None,
            })
            .collect();
        if !hops.is_empty() {
            p.fields.push((label.into(), hops.join(" ")));
        }
    }
}

/// Fields of a message we have no names for, by number.
fn generic(p: &mut Packet, m: &[(u32, Wire<'_>)], prefix: &str) {
    for (num, w) in m {
        let v = match w {
            Wire::Bytes(b) if b.iter().all(|c| (0x20..0x7f).contains(c)) => {
                format!("\"{}\"", String::from_utf8_lossy(b))
            }
            Wire::Bytes(b) => format!("{} bytes", b.len()),
            _ => w.u64().unwrap_or(0).to_string(),
        };
        p.fields.push((format!("{prefix}#{num}"), v));
    }
}

/// Unix seconds as "2026-11-05 12:34:56 UTC".
pub fn unix_time(t: u32) -> String {
    let days = (t / 86_400) as i64;
    let s = t % 86_400;
    // Civil from days (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + (m <= 2) as i64;
    format!(
        "{y}-{m:02}-{d:02} {:02}:{:02}:{:02} UTC",
        s / 3600,
        s / 60 % 60,
        s % 60
    )
}

/// A packet as a node would send it on a channel (for the simulator):
/// header, then `Data { portnum, payload }` encrypted.
pub fn build(ch: &Channel, mut h: Header, port: u32, payload: &[u8]) -> Vec<u8> {
    h.channel = ch.hash;
    let mut data = proto::Writer::default()
        .varint(1, port as u64)
        .bytes(2, payload)
        .done();
    ch.crypt(h.from, h.id, &mut data);
    let mut out = h.bytes().to_vec();
    out.extend_from_slice(&data);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::Writer;

    #[test]
    fn longfast_hash_is_8() {
        // The well-known hash of the default LongFast channel.
        assert_eq!(Channel::preset_default("LongFast").hash, 0x08);
    }

    #[test]
    fn text_round_trip() {
        let ch = Channel::preset_default("LongFast");
        let h = Header {
            to: BROADCAST,
            from: 0x1234_5678,
            id: 0x0bad_cafe,
            flags: 3 | (3 << 5),
            ..Default::default()
        };
        let pkt = build(&ch, h, 1, b"hello mesh");
        let p = decode(&pkt, &default_channels()).unwrap();
        assert_eq!(p.channel.as_deref(), Some("LongFast"));
        assert_eq!(p.text.as_deref(), Some("hello mesh"));
        assert_eq!(p.header.hop_start(), 3);
        // A key that does not fit leaves it encrypted.
        let other = Channel::new("Private", "AAECAwQFBgcICQoLDA0ODw==").unwrap();
        let p = decode(&build(&other, h, 1, b"secret"), &default_channels()).unwrap();
        assert_eq!(p.kind, "encrypted");
    }

    #[test]
    fn position_and_telemetry() {
        let ch = Channel::preset_default("MediumFast");
        let h = Header {
            to: BROADCAST,
            from: 7,
            id: 9,
            ..Default::default()
        };
        let pos = Writer::default()
            .fixed32(1, 515_000_000u32)
            .fixed32(2, (-1_200_000i32) as u32)
            .varint(3, 35)
            .done();
        let p = decode(&build(&ch, h, 3, &pos), &default_channels()).unwrap();
        let (lat, lon, alt) = p.node.position.unwrap();
        assert!((lat - 51.5).abs() < 1e-6 && (lon + 0.12).abs() < 1e-6 && alt == Some(35));
        let dm = Writer::default().varint(1, 87).float(2, 4.02).done();
        let t = Writer::default().bytes(2, &dm).done();
        let p = decode(&build(&ch, h, 67, &t), &default_channels()).unwrap();
        assert_eq!(p.node.battery_pct, Some(87));
        assert!(p.summary.contains("batt 87 %"), "{}", p.summary);
    }

    #[test]
    fn dates() {
        assert_eq!(unix_time(0), "1970-01-01 00:00:00 UTC");
        assert_eq!(unix_time(1_794_000_000), "2026-11-06 21:20:00 UTC");
    }
}
