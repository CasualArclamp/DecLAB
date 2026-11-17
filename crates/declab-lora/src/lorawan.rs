// made by CasualArclamp
//! LoRaWAN's MAC header (LoRaWAN 1.0.x / 1.1 §4): what is sent in clear on
//! sync word 0x34. The application payload is AES-encrypted with keys only
//! the network and the device hold, so what can be read is who sent it
//! (DevAddr, or a join request's DevEUI and JoinEUI), its counter, port and
//! MAC flags.

/// LoRaWAN's public sync word.
pub const SYNC_WORD: u8 = 0x34;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MType {
    JoinRequest,
    JoinAccept,
    UnconfirmedUp,
    UnconfirmedDown,
    ConfirmedUp,
    ConfirmedDown,
    /// Rejoin request (1.1); RFU in 1.0.
    Rejoin,
    Proprietary,
}

impl MType {
    pub fn name(self) -> &'static str {
        match self {
            MType::JoinRequest => "Join request",
            MType::JoinAccept => "Join accept",
            MType::UnconfirmedUp => "Unconfirmed data up",
            MType::UnconfirmedDown => "Unconfirmed data down",
            MType::ConfirmedUp => "Confirmed data up",
            MType::ConfirmedDown => "Confirmed data down",
            MType::Rejoin => "Rejoin request",
            MType::Proprietary => "Proprietary",
        }
    }

    pub fn uplink(self) -> bool {
        matches!(self, MType::JoinRequest | MType::UnconfirmedUp | MType::ConfirmedUp | MType::Rejoin)
    }
}

/// A data frame's header.
#[derive(Debug, Clone, PartialEq)]
pub struct Data {
    pub dev_addr: u32,
    pub adr: bool,
    /// Uplink: ADRACKReq; downlink: RFU.
    pub adr_ack_req: bool,
    pub ack: bool,
    /// Uplink: Class B; downlink: FPending.
    pub class_b_or_pending: bool,
    pub fcnt: u16,
    /// MAC commands in the header (in clear in 1.0.x).
    pub fopts: Vec<u8>,
    pub fport: Option<u8>,
    /// Bytes of (encrypted) FRMPayload.
    pub payload_len: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Body {
    Data(Data),
    Join { join_eui: u64, dev_eui: u64, dev_nonce: u16 },
    /// Encrypted (join accept) or not LoRaWAN's to define.
    Opaque(usize),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub mtype: MType,
    /// 0: LoRaWAN R1.
    pub major: u8,
    pub body: Body,
    pub mic: u32,
}

/// A frame's MAC header, if `b` is long enough to be one.
pub fn parse(b: &[u8]) -> Option<Frame> {
    let (&mhdr, rest) = b.split_first()?;
    if rest.len() < 4 {
        return None;
    }
    let (body, mic) = rest.split_at(rest.len() - 4);
    let mic = u32::from_le_bytes(mic.try_into().ok()?);
    let mtype = match mhdr >> 5 {
        0 => MType::JoinRequest,
        1 => MType::JoinAccept,
        2 => MType::UnconfirmedUp,
        3 => MType::UnconfirmedDown,
        4 => MType::ConfirmedUp,
        5 => MType::ConfirmedDown,
        6 => MType::Rejoin,
        _ => MType::Proprietary,
    };
    let major = mhdr & 3;
    let le = |s: &[u8]| s.iter().rev().fold(0u64, |a, &x| a << 8 | x as u64);
    let body = match mtype {
        MType::JoinRequest => {
            if body.len() != 18 {
                return None;
            }
            Body::Join { join_eui: le(&body[..8]), dev_eui: le(&body[8..16]), dev_nonce: le(&body[16..]) as u16 }
        }
        MType::UnconfirmedUp | MType::UnconfirmedDown | MType::ConfirmedUp | MType::ConfirmedDown => {
            if body.len() < 7 {
                return None;
            }
            let fctrl = body[4];
            let n = (fctrl & 0x0F) as usize;
            if body.len() < 7 + n {
                return None;
            }
            let after = &body[7 + n..];
            Body::Data(Data {
                dev_addr: le(&body[..4]) as u32,
                adr: fctrl & 0x80 != 0,
                adr_ack_req: fctrl & 0x40 != 0,
                ack: fctrl & 0x20 != 0,
                class_b_or_pending: fctrl & 0x10 != 0,
                fcnt: le(&body[5..7]) as u16,
                fopts: body[7..7 + n].to_vec(),
                fport: after.first().copied(),
                payload_len: after.len().saturating_sub(1),
            })
        }
        _ => Body::Opaque(body.len()),
    };
    Some(Frame { mtype, major, body, mic })
}

/// The network a DevAddr belongs to, for the NetIDs most heard: The Things
/// Network (NetID 0x000013, DevAddr 26/27…) and Helium (0x00003C, 78/79…)
/// are type 0 NetIDs, whose DevAddr carries 6 bits of NwkID after a 0.
pub fn network(dev_addr: u32) -> Option<&'static str> {
    if dev_addr >> 31 != 0 {
        return None;
    }
    match dev_addr >> 25 {
        0x13 => Some("The Things Network"),
        0x3C => Some("Helium"),
        _ => None,
    }
}

impl Frame {
    /// A line for the packets list: "DevAddr 260B1A2C · FCnt 42 · port 1 · 12 bytes".
    pub fn summary(&self) -> String {
        match &self.body {
            Body::Data(d) => {
                let mut s = format!("DevAddr {:08X} · FCnt {}", d.dev_addr, d.fcnt);
                if let Some(p) = d.fport {
                    s += &format!(" · port {p} · {} bytes", d.payload_len);
                }
                if d.ack {
                    s += " · ACK";
                }
                if !d.fopts.is_empty() {
                    s += &format!(" · {} bytes of MAC commands", d.fopts.len());
                }
                s
            }
            Body::Join { join_eui, dev_eui, dev_nonce } => {
                format!("DevEUI {dev_eui:016X} · JoinEUI {join_eui:016X} · DevNonce {dev_nonce}")
            }
            Body::Opaque(n) => format!("{n} bytes"),
        }
    }

    /// Who sent it, for the packets list's from → to column.
    pub fn from_to(&self) -> String {
        match &self.body {
            Body::Data(d) if self.mtype.uplink() => format!("{:08X} → network", d.dev_addr),
            Body::Data(d) => format!("network → {:08X}", d.dev_addr),
            Body::Join { dev_eui, .. } => format!("{dev_eui:016X} → network"),
            Body::Opaque(_) => String::new(),
        }
    }

    /// Name and value pairs for the packet's details.
    pub fn fields(&self) -> Vec<(String, String)> {
        let mut f = vec![("LoRaWAN".to_string(), format!("{} (R{})", self.mtype.name(), self.major + 1))];
        match &self.body {
            Body::Data(d) => {
                let net = network(d.dev_addr).map_or(String::new(), |n| format!(" ({n})"));
                f.push(("DevAddr".into(), format!("{:08X}{net}", d.dev_addr)));
                f.push(("FCnt".into(), d.fcnt.to_string()));
                let mut flags = Vec::new();
                if d.adr {
                    flags.push("ADR");
                }
                if d.adr_ack_req && self.mtype.uplink() {
                    flags.push("ADRACKReq");
                }
                if d.ack {
                    flags.push("ACK");
                }
                if d.class_b_or_pending {
                    flags.push(if self.mtype.uplink() { "Class B" } else { "FPending" });
                }
                if !flags.is_empty() {
                    f.push(("FCtrl".into(), flags.join(" ")));
                }
                if !d.fopts.is_empty() {
                    f.push(("FOpts".into(), d.fopts.iter().map(|b| format!("{b:02X}")).collect()));
                }
                match d.fport {
                    Some(0) => f.push(("FPort".into(), format!("0 (MAC commands, {} bytes)", d.payload_len))),
                    Some(p) => f.push(("FPort".into(), format!("{p} ({} bytes, encrypted)", d.payload_len))),
                    None => {}
                }
            }
            Body::Join { join_eui, dev_eui, dev_nonce } => {
                f.push(("DevEUI".into(), format!("{dev_eui:016X}")));
                f.push(("JoinEUI".into(), format!("{join_eui:016X}")));
                f.push(("DevNonce".into(), dev_nonce.to_string()));
            }
            Body::Opaque(n) => f.push(("body".into(), format!("{n} bytes (encrypted or proprietary)"))),
        }
        f.push(("MIC".into(), format!("{:08X}", self.mic)));
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unconfirmed_uplink() {
        // MHDR 0x40, DevAddr 260B1A2C, FCtrl ADR + 1 byte FOpts, FCnt 42,
        // FOpts 02 (LinkCheckReq), FPort 1, 3 bytes, MIC.
        let b = [0x40, 0x2C, 0x1A, 0x0B, 0x26, 0x81, 0x2A, 0x00, 0x02, 0x01, 0xAA, 0xBB, 0xCC, 1, 2, 3, 4];
        let f = parse(&b).unwrap();
        assert_eq!(f.mtype, MType::UnconfirmedUp);
        let Body::Data(d) = &f.body else { panic!() };
        assert_eq!((d.dev_addr, d.fcnt, d.fport, d.payload_len), (0x260B1A2C, 42, Some(1), 3));
        assert!(d.adr && !d.ack);
        assert_eq!(d.fopts, vec![0x02]);
        assert_eq!(f.mic, 0x04030201);
        assert_eq!(network(d.dev_addr), Some("The Things Network"));
        assert_eq!(f.summary(), "DevAddr 260B1A2C · FCnt 42 · port 1 · 3 bytes · 1 bytes of MAC commands");
    }

    #[test]
    fn join_request() {
        let mut b = vec![0x00];
        b.extend_from_slice(&0x70B3D57ED0000001u64.to_le_bytes());
        b.extend_from_slice(&0x0004A30B001C0530u64.to_le_bytes());
        b.extend_from_slice(&0x1234u16.to_le_bytes());
        b.extend_from_slice(&[9, 9, 9, 9]);
        let f = parse(&b).unwrap();
        assert_eq!(
            f.body,
            Body::Join { join_eui: 0x70B3D57ED0000001, dev_eui: 0x0004A30B001C0530, dev_nonce: 0x1234 }
        );
        assert_eq!(f.from_to(), "0004A30B001C0530 → network");
    }

    #[test]
    fn too_short_is_not_a_frame() {
        assert!(parse(&[0x40, 1, 2, 3]).is_none());
        assert!(parse(&[0x40, 1, 2, 3, 4, 0x0F, 0, 0, 1, 2, 3, 4]).is_none());
        assert!(parse(&[0x00, 1, 2, 3, 4, 5]).is_none());
    }
}
