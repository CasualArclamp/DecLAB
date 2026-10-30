// made by CasualArclamp
//! P25 phase 1 (TIA-102.BAAA, C4FM): conventional and trunked voice and its
//! signalling.
//!
//! The TIA documents are sold, not published, so the layouts follow open
//! implementations that work with real radios:
//! - MMDVMHost (GPL-2.0-or-later, © Jonathan Naylor G4KLX), which builds
//!   and reads every frame type: NID, HDU, LDU1/2, TDU, TSDU, and the
//!   spec's Reed-Solomon matrices.
//! - DSD (ISC) for the IMBE interleave and the TDULC.
//! - SDRTrunk (GPL-3.0, © Dennis Sheirer) for the link control and TSBK
//!   opcodes and their fields.
//!
//! A frame starts with the 48-bit sync and the 64-bit NID; every 72nd and
//! 71st bit (counting from the sync) is a status symbol, dropped before
//! anything is decoded. With those gone the NID ends at bit 112, where the
//! payload starts:
//!
//! ```text
//!  HDU   (792 bits)  Golay (18,6) ×36 → RS (36,20): MI, MFID, ALGID, KID, TGID
//!  LDU1  (1728)      9 IMBE frames, link control in Hamming (10,6) → RS (24,12)
//!  LDU2  (1728)      9 IMBE frames, encryption sync → RS (24,16)
//!  TDU   (144)       end of a call
//!  TDULC (432)       end of a call, with link control: Golay (24,12) → RS (24,12)
//!  TSDU  (360+)      one to three TSBKs, rate ½ trellis, CRC-CCITT
//! ```

pub mod fec;
pub mod imbe_frame;
pub mod receiver;
pub mod sim;

use std::fmt;

pub use receiver::{Event, Receiver, Stats};

/// Symbols a second.
pub const SYMBOL_RATE: f64 = 4800.0;
/// The outer symbols' deviation (±3 at 1.8 kHz, ±1 at 600 Hz).
pub const OUTER_DEV_HZ: f64 = 1800.0;
/// The roll-off of the Nyquist shaping (C4FM's raised cosine).
pub const ALPHA: f64 = 0.2;
/// The frame sync, 48 bits (MMDVMHost `P25_SYNC_BYTES`).
pub const FS: u64 = 0x5575_F5FF_77FF;

/// The data unit IDs in the NID.
pub mod duid {
    pub const HDU: u8 = 0;
    pub const TDU: u8 = 3;
    pub const LDU1: u8 = 5;
    pub const TSDU: u8 = 7;
    pub const LDU2: u8 = 10;
    pub const PDU: u8 = 12;
    pub const TDULC: u8 = 15;
}

pub fn duid_name(d: u8) -> &'static str {
    match d {
        duid::HDU => "HDU",
        duid::TDU => "TDU",
        duid::LDU1 => "LDU1",
        duid::TSDU => "TSDU",
        duid::LDU2 => "LDU2",
        duid::PDU => "PDU",
        duid::TDULC => "TDULC",
        _ => "?",
    }
}

/// A raw frame position (status symbols counted) → its position once they
/// are dropped (the status pair sits at bits 70, 71 of each 72).
pub const fn stripped(raw: usize) -> usize {
    raw - 2 * ((raw + 1) / 72)
}

/// The frame length, bits as sent (status included), of a DUID; a TSDU with
/// `tsbks` blocks.
pub fn frame_bits(d: u8, tsbks: usize) -> usize {
    match d {
        duid::HDU => 792,
        duid::LDU1 | duid::LDU2 => 1728,
        duid::TDU => 144,
        duid::TDULC => 432,
        // 112 + 196 per TSBK once the status is out, rounded up to whole
        // 72-bit microslots as sent (MMDVMHost's single TSBK makes 360).
        _ => {
            let payload = 112 + 196 * tsbks.max(1);
            payload.div_ceil(70) * 72
        }
    }
}

/// The nine IMBE frames of an LDU: their raw start bits (each 144 bits once
/// the status is out; MMDVMHost `CP25Audio`).
pub const IMBE_AT: [usize; 9] = [114, 262, 452, 640, 830, 1020, 1208, 1398, 1578];
/// The six 40-bit pieces of an LDU's link control or encryption sync (raw
/// start bits; MMDVMHost `CP25Data`).
pub const LDU_LC_AT: [usize; 6] = [410, 600, 788, 978, 1168, 1356];

/// Service options (as DMR's): emergency, encrypted, priority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ServiceOptions(pub u8);

impl ServiceOptions {
    pub fn emergency(self) -> bool {
        self.0 & 0x80 != 0
    }
    pub fn encrypted(self) -> bool {
        self.0 & 0x40 != 0
    }
}

/// A link control word (9 bytes), SDRTrunk's layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lc {
    /// LCF 0x00.
    GroupVoice {
        opts: ServiceOptions,
        group: u16,
        source: u32,
    },
    /// LCF 0x03.
    UnitVoice {
        opts: ServiceOptions,
        target: u32,
        source: u32,
    },
    Other {
        lcf: u8,
        mfid: u8,
        bytes: [u8; 9],
    },
}

/// Standard link control opcode names (SDRTrunk `LinkControlOpcode`).
pub fn lc_name(lcf: u8) -> &'static str {
    match lcf {
        0 => "GROUP VOICE CHANNEL USER",
        2 => "GROUP VOICE CHANNEL UPDATE",
        3 => "UNIT-TO-UNIT VOICE CHANNEL USER",
        4 => "GROUP VOICE CHANNEL UPDATE EXPLICIT",
        5 => "UNIT-TO-UNIT ANSWER REQUEST",
        6 => "TELEPHONE INTERCONNECT VOICE CHANNEL USER",
        7 => "TELEPHONE INTERCONNECT ANSWER REQUEST",
        9 => "SOURCE ID EXTENSION",
        10 => "UNIT-TO-UNIT VOICE CHANNEL USER EXTENDED",
        15 => "CALL TERMINATION",
        16 => "GROUP AFFILIATION QUERY",
        17 => "UNIT REGISTRATION COMMAND",
        18 => "UNIT AUTHENTICATION COMMAND",
        19 => "STATUS QUERY",
        20 => "STATUS UPDATE",
        21 => "MESSAGE UPDATE",
        22 => "CALL ALERT",
        23 => "EXTENDED FUNCTION COMMAND",
        24 => "CHANNEL IDENTIFIER UPDATE",
        25 => "CHANNEL IDENTIFIER UPDATE EXPLICIT",
        32 => "SYSTEM SERVICE BROADCAST",
        33 => "SECONDARY CONTROL CHANNEL BROADCAST",
        34 => "ADJACENT SITE STATUS",
        35 => "RFSS STATUS BROADCAST",
        36 => "NET STATUS BROADCAST",
        37 => "ENCRYPTION PARAMETERS BROADCAST",
        42 => "CONVENTIONAL FALLBACK INDICATION",
        _ => "reserved",
    }
}

fn be(b: &[u8]) -> u32 {
    b.iter().fold(0u32, |v, &x| (v << 8) | u32::from(x))
}

impl Lc {
    /// From 9 bytes. The first byte: encryption flag, standard-vendor flag,
    /// 6-bit LCF; the standard ones are parsed when the MFID is standard.
    pub fn parse(b: &[u8; 9]) -> Lc {
        let lcf = b[0] & 0x3f;
        let standard = b[0] & 0x40 == 0 || matches!(b[1], 0 | 1);
        let opts = ServiceOptions(b[2]);
        match (standard, lcf) {
            (true, 0) => Lc::GroupVoice {
                opts,
                group: be(&b[4..6]) as u16,
                source: be(&b[6..9]),
            },
            (true, 3) => Lc::UnitVoice {
                opts,
                target: be(&b[3..6]),
                source: be(&b[6..9]),
            },
            _ => Lc::Other {
                lcf,
                mfid: b[1],
                bytes: *b,
            },
        }
    }

    pub fn to_bytes(&self) -> [u8; 9] {
        let mut b = [0u8; 9];
        match *self {
            Lc::GroupVoice {
                opts,
                group,
                source,
            } => {
                b[2] = opts.0;
                b[4..6].copy_from_slice(&group.to_be_bytes());
                b[6..9].copy_from_slice(&source.to_be_bytes()[1..]);
            }
            Lc::UnitVoice {
                opts,
                target,
                source,
            } => {
                b[0] = 3;
                b[2] = opts.0;
                b[3..6].copy_from_slice(&target.to_be_bytes()[1..]);
                b[6..9].copy_from_slice(&source.to_be_bytes()[1..]);
            }
            Lc::Other { bytes, .. } => return bytes,
        }
        b
    }

    /// (source, destination, group?, options) of a voice call.
    pub fn call(&self) -> Option<(u32, u32, bool, ServiceOptions)> {
        match *self {
            Lc::GroupVoice {
                opts,
                group,
                source,
            } => Some((source, u32::from(group), true, opts)),
            Lc::UnitVoice {
                opts,
                target,
                source,
            } => Some((source, target, false, opts)),
            Lc::Other { .. } => None,
        }
    }
}

impl fmt::Display for Lc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Lc::GroupVoice {
                opts,
                group,
                source,
            } => {
                write!(f, "group call TG {group} from {source}")?;
                if opts.emergency() {
                    write!(f, " EMERGENCY")?;
                }
                Ok(())
            }
            Lc::UnitVoice {
                opts,
                target,
                source,
            } => {
                write!(f, "private call {source} → {target}")?;
                if opts.emergency() {
                    write!(f, " EMERGENCY")?;
                }
                Ok(())
            }
            Lc::Other {
                lcf, mfid: 0 | 1, ..
            } => write!(f, "{}", lc_name(lcf)),
            Lc::Other { lcf, mfid, .. } => write!(f, "LC {lcf:#04x} MFID {mfid:#04x}"),
        }
    }
}

/// A channel identifier band (IDEN_UP and its variants): what turns a
/// grant's 4-bit band and 12-bit channel into a frequency.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Band {
    pub base_hz: f64,
    pub spacing_hz: f64,
    /// Timeslots a carrier (2 for a phase 2 TDMA band, else 1).
    pub slots: u32,
}

impl Band {
    pub fn downlink_hz(&self, channel: u16) -> f64 {
        self.base_hz + self.spacing_hz * f64::from(u32::from(channel) / self.slots.max(1))
    }
}

/// A TSBK (12 bytes, CRC good), SDRTrunk's layouts for the standard
/// outbound ones.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tsbk {
    /// GRP_VCH_GRANT (0).
    GroupGrant {
        opts: ServiceOptions,
        band: u8,
        channel: u16,
        group: u16,
        source: u32,
    },
    /// GRP_VCH_GRNT_UPD (2): one or two talkgroups.
    GroupGrantUpdate {
        a: (u8, u16, u16),
        b: Option<(u8, u16, u16)>,
    },
    /// UU_VCH_GRANT (4).
    UnitGrant {
        band: u8,
        channel: u16,
        target: u32,
        source: u32,
    },
    /// IDEN_UPDATE (61), _VUHF (52), _TDMA (51).
    Iden {
        id: u8,
        band: Band,
    },
    /// RFSS_STATUS_BCST (58).
    RfssStatus {
        system: u16,
        rfss: u8,
        site: u8,
        band: u8,
        channel: u16,
    },
    /// NET_STATUS_BCAST (59).
    NetStatus {
        wacn: u32,
        system: u16,
        band: u8,
        channel: u16,
    },
    /// ADJ SITE STATUS (60).
    AdjacentStatus {
        system: u16,
        rfss: u8,
        site: u8,
        band: u8,
        channel: u16,
    },
    Other {
        opcode: u8,
        mfid: u8,
        last: bool,
    },
}

/// Standard outbound TSBK names (SDRTrunk `Opcode`).
pub fn tsbk_name(op: u8) -> &'static str {
    match op {
        0 => "GRP_VCH_GRANT",
        2 => "GRP_VCH_GRNT_UPD",
        3 => "GRP_VCH_GRNT_UPX",
        4 => "UU_VCH_GRANT",
        5 => "UU_ANS_REQ",
        6 => "UU_VCH_GRANT_UPD",
        8 => "TEL_INT_VCH_GRNT",
        9 => "TEL_INT_VCH_GRNU",
        10 => "TEL_INT_ANS_RQST",
        20 => "SNDCP_DCH_GRANT",
        21 => "SNDCP_DCH_PAG_RQ",
        22 => "SNDCP_DCH_ANN_EX",
        24 => "STATUS_UPDATE",
        26 => "STATUS_QUERY",
        28 => "MESSAGE_UPDATE",
        29 => "RADIO_MONITR_CMD",
        31 => "CALL_ALERT",
        32 => "ACK_RESPONSE_FNE",
        33 => "QUEUED_RESPONSE",
        36 => "EXTNDED_FUNC_CMD",
        39 => "DENY_RESPONSE",
        40 => "GRP_AFFIL_RESP",
        41 => "SCCB_CCH_BCST_EX",
        42 => "GRP_AFFIL_QUERY",
        43 => "LOCN_REG_RESPONS",
        44 => "UNIT_REG_RESPONS",
        45 => "UNIT_REG_COMMAND",
        46 => "AUTH_COMMAND",
        47 => "DE_REGIST_ACK",
        48 => "TDMA_SYNC_BCST",
        49 => "AUTH_DEMAND",
        50 => "AUTH_FNE_RESP",
        51 => "IDEN_UPDATE_TDMA",
        52 => "IDEN_UPDATE_VUHF",
        53 => "TIME_DATE_ANNOUN",
        54 => "ROAM_ADDR_CMD",
        55 => "ROAM_ADDR_UPDATE",
        56 => "SYS_SVC_BCAST",
        57 => "SEC_CCH_BROADCST",
        58 => "RFSS_STATUS_BCST",
        59 => "NET_STATUS_BCAST",
        60 => "ADJ SITE STATUS",
        61 => "IDEN_UPDATE",
        62 => "ADJ_STS_BCST_UNC",
        _ => "reserved",
    }
}

/// `len` bits of `b` from bit `start` (0 = the first byte's MSB).
fn bits(b: &[u8], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0u64, |v, i| {
        (v << 1) | u64::from((b[i / 8] >> (7 - i % 8)) & 1)
    })
}

fn put(b: &mut [u8], start: usize, len: usize, v: u64) {
    for k in 0..len {
        let i = start + k;
        let bit = ((v >> (len - 1 - k)) & 1) as u8;
        b[i / 8] = (b[i / 8] & !(0x80 >> (i % 8))) | (bit << (7 - i % 8));
    }
}

impl Tsbk {
    /// From the 12 bytes (the CRC already checked).
    pub fn parse(b: &[u8]) -> Tsbk {
        let last = b[0] & 0x80 != 0;
        let opcode = b[0] & 0x3f;
        let mfid = b[1];
        let f = |s: usize, l: usize| bits(b, s, l);
        if !matches!(mfid, 0 | 1) {
            return Tsbk::Other { opcode, mfid, last };
        }
        match opcode {
            0 => Tsbk::GroupGrant {
                opts: ServiceOptions(f(16, 8) as u8),
                band: f(24, 4) as u8,
                channel: f(28, 12) as u16,
                group: f(40, 16) as u16,
                source: f(56, 24) as u32,
            },
            2 => {
                let a = (f(16, 4) as u8, f(20, 12) as u16, f(32, 16) as u16);
                let b2 = (f(48, 4) as u8, f(52, 12) as u16, f(64, 16) as u16);
                let b = (b2.2 != a.2 && b2.2 != 0).then_some(b2);
                Tsbk::GroupGrantUpdate { a, b }
            }
            4 => Tsbk::UnitGrant {
                band: f(16, 4) as u8,
                channel: f(20, 12) as u16,
                target: f(32, 24) as u32,
                source: f(56, 24) as u32,
            },
            61 | 52 | 51 => {
                let spacing_hz = f(38, 10) as f64 * 125.0;
                // TDMA bands: the channel type's slots (two for the common
                // phase 2 types); a plain identifier update has one.
                let slots = if opcode == 51 { 2 } else { 1 };
                Tsbk::Iden {
                    id: f(16, 4) as u8,
                    band: Band {
                        base_hz: f(48, 32) as f64 * 5.0,
                        spacing_hz,
                        slots,
                    },
                }
            }
            58 | 60 => {
                let (system, rfss, site, band, channel) = (
                    f(28, 12) as u16,
                    f(40, 8) as u8,
                    f(48, 8) as u8,
                    f(56, 4) as u8,
                    f(60, 12) as u16,
                );
                if opcode == 58 {
                    Tsbk::RfssStatus {
                        system,
                        rfss,
                        site,
                        band,
                        channel,
                    }
                } else {
                    Tsbk::AdjacentStatus {
                        system,
                        rfss,
                        site,
                        band,
                        channel,
                    }
                }
            }
            59 => Tsbk::NetStatus {
                wacn: f(24, 20) as u32,
                system: f(44, 12) as u16,
                band: f(56, 4) as u8,
                channel: f(60, 12) as u16,
            },
            _ => Tsbk::Other { opcode, mfid, last },
        }
    }

    /// The 10 bytes before the CRC (for the simulator), last block set.
    pub fn to_bytes(&self) -> [u8; 10] {
        let mut b = [0u8; 10];
        let mut set = |s: usize, l: usize, v: u64| put(&mut b, s, l, v);
        let op = match *self {
            Tsbk::GroupGrant {
                opts,
                band,
                channel,
                group,
                source,
            } => {
                set(16, 8, u64::from(opts.0));
                set(24, 4, u64::from(band));
                set(28, 12, u64::from(channel));
                set(40, 16, u64::from(group));
                set(56, 24, u64::from(source));
                0
            }
            Tsbk::UnitGrant {
                band,
                channel,
                target,
                source,
            } => {
                set(16, 4, u64::from(band));
                set(20, 12, u64::from(channel));
                set(32, 24, u64::from(target));
                set(56, 24, u64::from(source));
                4
            }
            Tsbk::Iden { id, band } => {
                set(16, 4, u64::from(id));
                set(29, 1, 1);
                set(30, 8, 0x80);
                set(38, 10, (band.spacing_hz / 125.0) as u64);
                set(48, 32, (band.base_hz / 5.0) as u64);
                61
            }
            Tsbk::RfssStatus {
                system,
                rfss,
                site,
                band,
                channel,
            } => {
                set(28, 12, u64::from(system));
                set(40, 8, u64::from(rfss));
                set(48, 8, u64::from(site));
                set(56, 4, u64::from(band));
                set(60, 12, u64::from(channel));
                58
            }
            Tsbk::NetStatus {
                wacn,
                system,
                band,
                channel,
            } => {
                set(24, 20, u64::from(wacn));
                set(44, 12, u64::from(system));
                set(56, 4, u64::from(band));
                set(60, 12, u64::from(channel));
                59
            }
            Tsbk::AdjacentStatus {
                system,
                rfss,
                site,
                band,
                channel,
            } => {
                set(28, 12, u64::from(system));
                set(40, 8, u64::from(rfss));
                set(48, 8, u64::from(site));
                set(56, 4, u64::from(band));
                set(60, 12, u64::from(channel));
                60
            }
            Tsbk::GroupGrantUpdate { a, b: bb } => {
                set(16, 4, u64::from(a.0));
                set(20, 12, u64::from(a.1));
                set(32, 16, u64::from(a.2));
                let b2 = bb.unwrap_or(a);
                set(48, 4, u64::from(b2.0));
                set(52, 12, u64::from(b2.1));
                set(64, 16, u64::from(b2.2));
                2
            }
            Tsbk::Other { opcode, .. } => opcode,
        };
        b[0] = 0x80 | (op & 0x3f);
        b
    }

    pub fn last(&self) -> bool {
        match *self {
            Tsbk::Other { last, .. } => last,
            _ => true,
        }
    }
}

impl fmt::Display for Tsbk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Tsbk::GroupGrant {
                opts,
                band,
                channel,
                group,
                source,
            } => {
                write!(f, "GRP_VCH_GRANT {source} → TG {group} on {band}-{channel}")?;
                if opts.emergency() {
                    write!(f, " EMERGENCY")?;
                }
                Ok(())
            }
            Tsbk::GroupGrantUpdate { a, b } => {
                write!(f, "GRP_VCH_GRNT_UPD TG {} on {}-{}", a.2, a.0, a.1)?;
                if let Some(b) = b {
                    write!(f, ", TG {} on {}-{}", b.2, b.0, b.1)?;
                }
                Ok(())
            }
            Tsbk::UnitGrant {
                band,
                channel,
                target,
                source,
            } => {
                write!(f, "UU_VCH_GRANT {source} → {target} on {band}-{channel}")
            }
            Tsbk::Iden { id, band } => write!(
                f,
                "IDEN_UPDATE band {id}: base {:.5} MHz, spacing {:.2} kHz",
                band.base_hz / 1e6,
                band.spacing_hz / 1e3
            ),
            Tsbk::RfssStatus {
                system,
                rfss,
                site,
                band,
                channel,
            } => {
                write!(
                    f,
                    "RFSS_STATUS system {system:03X} RFSS {rfss} site {site}, control {band}-{channel}"
                )
            }
            Tsbk::NetStatus {
                wacn,
                system,
                band,
                channel,
            } => {
                write!(
                    f,
                    "NET_STATUS WACN {wacn:05X} system {system:03X}, control {band}-{channel}"
                )
            }
            Tsbk::AdjacentStatus {
                system,
                rfss,
                site,
                band,
                channel,
            } => {
                write!(
                    f,
                    "ADJ_SITE system {system:03X} RFSS {rfss} site {site} on {band}-{channel}"
                )
            }
            Tsbk::Other {
                opcode,
                mfid: 0 | 1,
                ..
            } => {
                write!(f, "{}", tsbk_name(opcode))
            }
            Tsbk::Other { opcode, mfid, .. } => write!(f, "TSBK {opcode:#04x} MFID {mfid:#04x}"),
        }
    }
}

/// The HDU's fields (MMDVMHost `CP25Data::encodeHeader`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub mi: [u8; 9],
    pub mfid: u8,
    /// 0x80: clear.
    pub algid: u8,
    pub kid: u16,
    pub tgid: u16,
}

impl Header {
    pub fn parse(b: &[u8; 15]) -> Header {
        Header {
            mi: b[..9].try_into().expect("9"),
            mfid: b[9],
            algid: b[10],
            kid: u16::from_be_bytes([b[11], b[12]]),
            tgid: u16::from_be_bytes([b[13], b[14]]),
        }
    }

    pub fn to_bytes(&self) -> [u8; 15] {
        let mut b = [0u8; 15];
        b[..9].copy_from_slice(&self.mi);
        b[9] = self.mfid;
        b[10] = self.algid;
        b[11..13].copy_from_slice(&self.kid.to_be_bytes());
        b[13..15].copy_from_slice(&self.tgid.to_be_bytes());
        b
    }

    pub fn encrypted(&self) -> bool {
        self.algid != 0x80
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stripped_positions() {
        assert_eq!(stripped(114), 112);
        assert_eq!(stripped(262) - stripped(114), 144);
        assert_eq!(stripped(452) - stripped(410), 40);
        assert_eq!(stripped(780) - stripped(114), 648);
    }

    #[test]
    fn lc_and_tsbk_round_trip() {
        let lc = Lc::GroupVoice {
            opts: ServiceOptions(0),
            group: 101,
            source: 1_234_567,
        };
        assert_eq!(Lc::parse(&lc.to_bytes()), lc);
        let lc = Lc::UnitVoice {
            opts: ServiceOptions(0x80),
            target: 7,
            source: 99,
        };
        assert_eq!(Lc::parse(&lc.to_bytes()), lc);
        for t in [
            Tsbk::GroupGrant {
                opts: ServiceOptions(0),
                band: 1,
                channel: 0x123,
                group: 101,
                source: 1_234_567,
            },
            Tsbk::UnitGrant {
                band: 2,
                channel: 77,
                target: 5,
                source: 6,
            },
            Tsbk::Iden {
                id: 1,
                band: Band {
                    base_hz: 851_006_250.0,
                    spacing_hz: 6250.0,
                    slots: 1,
                },
            },
            Tsbk::RfssStatus {
                system: 0x3a1,
                rfss: 2,
                site: 9,
                band: 1,
                channel: 0x21,
            },
            Tsbk::NetStatus {
                wacn: 0xbee00,
                system: 0x3a1,
                band: 1,
                channel: 0x21,
            },
        ] {
            let b = t.to_bytes();
            let mut full = [0u8; 12];
            full[..10].copy_from_slice(&b);
            assert_eq!(Tsbk::parse(&full), t);
        }
    }
}
