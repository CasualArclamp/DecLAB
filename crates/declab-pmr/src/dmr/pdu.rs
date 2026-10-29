// made by CasualArclamp
//! The PDUs: Full LC (ETSI TS 102 361-2 clause 7.1.1), CSBK with Tier III's
//! (TS 102 361-4 clause 7.1 and table B.1) and Short LC (TS 102 361-2
//! clause 7.1.3, TS 102 361-4 clause 7.1.2). Each parses from and writes to
//! the bytes the burst carries, so the modulator and the tests share them.

use std::fmt;

/// `len` bits of `bytes` from bit `start` (0 = the first byte's MSB).
pub fn field(bytes: &[u8], start: usize, len: usize) -> u32 {
    (start..start + len).fold(0, |v, i| {
        (v << 1) | u32::from((bytes[i / 8] >> (7 - i % 8)) & 1)
    })
}

/// Write `len` bits of `v` into `bytes` from bit `start`.
pub fn put(bytes: &mut [u8], start: usize, len: usize, v: u32) {
    for k in 0..len {
        let i = start + k;
        let bit = ((v >> (len - 1 - k)) & 1) as u8;
        bytes[i / 8] = (bytes[i / 8] & !(0x80 >> (i % 8))) | (bit << (7 - i % 8));
    }
}

/// Service Options, TS 102 361-2 table 7.11.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ServiceOptions(pub u8);

impl ServiceOptions {
    pub fn emergency(self) -> bool {
        self.0 & 0x80 != 0
    }
    /// Privacy (encryption) is in use; DMR does not define how.
    pub fn privacy(self) -> bool {
        self.0 & 0x40 != 0
    }
    pub fn broadcast(self) -> bool {
        self.0 & 0x08 != 0
    }
    pub fn ovcm(self) -> bool {
        self.0 & 0x04 != 0
    }
    pub fn priority(self) -> u8 {
        self.0 & 3
    }
}

impl fmt::Display for ServiceOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut tags = Vec::new();
        if self.emergency() {
            tags.push("EMERGENCY".to_string());
        }
        if self.privacy() {
            tags.push("encrypted".into());
        }
        if self.broadcast() {
            tags.push("broadcast".into());
        }
        if self.ovcm() {
            tags.push("OVCM".into());
        }
        if self.priority() > 0 {
            tags.push(format!("priority {}", self.priority()));
        }
        write!(f, "{}", tags.join(", "))
    }
}

/// A Full LC: in the voice header, the terminator and embedded in voice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FullLc {
    /// Grp_V_Ch_Usr, FLCO 000000.
    GroupVoice {
        opts: ServiceOptions,
        group: u32,
        source: u32,
    },
    /// UU_V_Ch_Usr, FLCO 000011.
    UnitVoice {
        opts: ServiceOptions,
        target: u32,
        source: u32,
    },
    /// Anything else (GPS, talker alias, a manufacturer's).
    Other { flco: u8, fid: u8, bytes: [u8; 9] },
}

impl FullLc {
    pub fn parse(b: &[u8; 9]) -> FullLc {
        let (flco, fid) = (b[0] & 0x3f, b[1]);
        let opts = ServiceOptions(b[2]);
        let (a, s) = (field(b, 24, 24), field(b, 48, 24));
        match (flco, fid) {
            (0, 0) => FullLc::GroupVoice {
                opts,
                group: a,
                source: s,
            },
            (3, 0) => FullLc::UnitVoice {
                opts,
                target: a,
                source: s,
            },
            _ => FullLc::Other {
                flco,
                fid,
                bytes: *b,
            },
        }
    }

    pub fn to_bytes(&self) -> [u8; 9] {
        let mut b = [0u8; 9];
        let (flco, opts, a, s) = match *self {
            FullLc::GroupVoice {
                opts,
                group,
                source,
            } => (0, opts, group, source),
            FullLc::UnitVoice {
                opts,
                target,
                source,
            } => (3, opts, target, source),
            FullLc::Other { bytes, .. } => return bytes,
        };
        b[0] = flco;
        b[2] = opts.0;
        put(&mut b, 24, 24, a);
        put(&mut b, 48, 24, s);
        b
    }

    /// (source, destination, group?) of a voice call.
    pub fn call(&self) -> Option<(u32, u32, bool, ServiceOptions)> {
        match *self {
            FullLc::GroupVoice {
                opts,
                group,
                source,
            } => Some((source, group, true, opts)),
            FullLc::UnitVoice {
                opts,
                target,
                source,
            } => Some((source, target, false, opts)),
            FullLc::Other { .. } => None,
        }
    }
}

impl fmt::Display for FullLc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FullLc::GroupVoice {
                opts,
                group,
                source,
            } => {
                write!(f, "group call TG {group} from {source}")?;
                if opts.0 != 0 {
                    write!(f, " ({opts})")?;
                }
                Ok(())
            }
            FullLc::UnitVoice {
                opts,
                target,
                source,
            } => {
                write!(f, "private call {source} → {target}")?;
                if opts.0 != 0 {
                    write!(f, " ({opts})")?;
                }
                Ok(())
            }
            FullLc::Other { flco, fid, .. } => write!(f, "LC FLCO {flco:#04x} FID {fid:#04x}"),
        }
    }
}

/// The System Identity Code, TS 102 361-4 clause 6.3.2.2.1.1: the model
/// sets how its 12 middle bits split between NET and SITE (table 6.5);
/// the last two are PAR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SysCode(pub u16);

impl SysCode {
    pub fn model(self) -> u8 {
        (self.0 >> 14) as u8
    }

    /// (NET, SITE): Tiny 9+3 bits, Small 7+5, Large 4+8, Huge 2+10.
    pub fn net_site(self) -> (u16, u16) {
        let ns = (self.0 >> 2) & 0xfff;
        let site_bits = [3, 5, 8, 10][self.model() as usize];
        (ns >> site_bits, ns & ((1 << site_bits) - 1))
    }

    pub fn par(self) -> u8 {
        (self.0 & 3) as u8
    }

    fn model_name(self) -> &'static str {
        ["Tiny", "Small", "Large", "Huge"][self.model() as usize]
    }
}

impl fmt::Display for SysCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (net, site) = self.net_site();
        // Sites are numbered from 1 on air (the field is site − 1).
        write!(f, "{} net {} site {}", self.model_name(), net, site + 1)
    }
}

/// Which grant (table B.1, opcodes 48–56).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrantKind {
    PrivateVoice,
    TalkgroupVoice,
    BroadcastVoice,
    PrivateData,
    TalkgroupData,
    DuplexPrivateVoice,
    DuplexPrivateData,
    PrivateDataMulti,
    TalkgroupDataMulti,
}

impl GrantKind {
    fn from_opcode(op: u8) -> Option<GrantKind> {
        use GrantKind::*;
        Some(match op {
            48 => PrivateVoice,
            49 => TalkgroupVoice,
            50 => BroadcastVoice,
            51 => PrivateData,
            52 => TalkgroupData,
            53 => DuplexPrivateVoice,
            54 => DuplexPrivateData,
            55 => PrivateDataMulti,
            56 => TalkgroupDataMulti,
            _ => return None,
        })
    }

    fn opcode(self) -> u8 {
        use GrantKind::*;
        match self {
            PrivateVoice => 48,
            TalkgroupVoice => 49,
            BroadcastVoice => 50,
            PrivateData => 51,
            TalkgroupData => 52,
            DuplexPrivateVoice => 53,
            DuplexPrivateData => 54,
            PrivateDataMulti => 55,
            TalkgroupDataMulti => 56,
        }
    }

    pub fn alias(self) -> &'static str {
        use GrantKind::*;
        match self {
            PrivateVoice => "PV_GRANT",
            TalkgroupVoice => "TV_GRANT",
            BroadcastVoice => "BTV_GRANT",
            PrivateData => "PD_GRANT",
            TalkgroupData => "TD_GRANT",
            DuplexPrivateVoice => "PV_GRANT_DX",
            DuplexPrivateData => "PD_GRANT_DX",
            PrivateDataMulti => "PD_GRANT_MI",
            TalkgroupDataMulti => "TD_GRANT_MI",
        }
    }

    pub fn to_group(self) -> bool {
        matches!(
            self,
            GrantKind::TalkgroupVoice
                | GrantKind::BroadcastVoice
                | GrantKind::TalkgroupData
                | GrantKind::TalkgroupDataMulti
        )
    }
}

/// The logical physical channel number that means "the frequencies follow
/// in an appended block" (TS 102 361-4 clause 7.1.1.1.1).
pub const LPCN_ABSOLUTE: u16 = 0xfff;

/// A CSBK (or an MBC's first block). Tier III's, with the standard FID 0,
/// are parsed; the rest keep their bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Csbk {
    /// Table 7.11 (and its siblings 7.9–7.15).
    Grant {
        kind: GrantKind,
        lpcn: u16,
        /// 0 or 1: TDMA channel 1 or 2.
        slot: u8,
        late_entry: bool,
        emergency: bool,
        offset: bool,
        target: u32,
        source: u32,
    },
    /// C_ALOHA, table 7.19.
    Aloha {
        syscode: SysCode,
        ms: u32,
        reg: bool,
        backoff: u8,
        mask: u8,
        version: u8,
    },
    /// C_BCAST, table 7.20.
    Bcast {
        ann: u8,
        parms1: u16,
        reg: bool,
        backoff: u8,
        syscode: SysCode,
        parms2: u32,
    },
    /// P_CLEAR, table 7.30.
    Clear {
        lpcn: u16,
        group: bool,
        target: u32,
        source: u32,
    },
    /// Every other opcode: its alias if known.
    Other {
        lb: bool,
        opcode: u8,
        fid: u8,
        bytes: [u8; 10],
    },
}

/// The CSBK aliases of table B.1 not parsed field by field.
pub fn csbk_alias(op: u8) -> Option<&'static str> {
    Some(match op {
        25 => "C_ALOHA",
        26 => "C_UDTHD",
        27 => "C_UDTHU",
        28 => "C_AHOY",
        30 => "C_ACKVIT",
        31 => "C_RAND",
        32 => "C_ACKD",
        33 => "C_ACKU",
        34 => "P_ACKD",
        35 => "P_ACKU",
        36 => "C_DGNAHD",
        37 => "C_DGNAHU",
        40 => "C_BCAST",
        42 => "P_MAINT",
        46 => "P_CLEAR",
        47 => "P_PROTECT",
        57 => "C_MOVE",
        op => return GrantKind::from_opcode(op).map(GrantKind::alias),
    })
}

/// Announcement types, TS 102 361-4 table 7.68.
pub fn announcement(t: u8) -> &'static str {
    match t {
        0 => "Ann_WD_TSCC",
        1 => "CallTimer_Parms",
        2 => "Vote_Now",
        3 => "Local_Time",
        4 => "MassReg",
        5 => "Chan_Freq",
        6 => "Adjacent_Site",
        7 => "Gen_Site_Params",
        30 | 31 => "manufacturer",
        _ => "reserved",
    }
}

impl Csbk {
    pub fn parse(b: &[u8; 10]) -> Csbk {
        let lb = b[0] & 0x80 != 0;
        let (opcode, fid) = (b[0] & 0x3f, b[1]);
        let f = |s: usize, l: usize| field(b, 16 + s, l);
        if fid != 0 {
            return Csbk::Other {
                lb,
                opcode,
                fid,
                bytes: *b,
            };
        }
        if let Some(kind) = GrantKind::from_opcode(opcode) {
            return Csbk::Grant {
                kind,
                lpcn: f(0, 12) as u16,
                slot: f(12, 1) as u8,
                late_entry: f(13, 1) == 1,
                emergency: f(14, 1) == 1,
                offset: f(15, 1) == 1,
                target: f(16, 24),
                source: f(40, 24),
            };
        }
        match opcode {
            25 => Csbk::Aloha {
                version: f(3, 3) as u8,
                mask: f(8, 5) as u8,
                reg: f(19, 1) == 1,
                backoff: f(20, 4) as u8,
                syscode: SysCode(f(24, 16) as u16),
                ms: f(40, 24),
            },
            40 => Csbk::Bcast {
                ann: f(0, 5) as u8,
                parms1: f(5, 14) as u16,
                reg: f(19, 1) == 1,
                backoff: f(20, 4) as u8,
                syscode: SysCode(f(24, 16) as u16),
                parms2: f(40, 24),
            },
            46 => Csbk::Clear {
                lpcn: f(0, 12) as u16,
                group: f(15, 1) == 1,
                target: f(16, 24),
                source: f(40, 24),
            },
            _ => Csbk::Other {
                lb,
                opcode,
                fid,
                bytes: *b,
            },
        }
    }

    /// The 10 bytes (before the CRC), as a single-block CSBK (LB set).
    pub fn to_bytes(&self) -> [u8; 10] {
        let mut b = [0u8; 10];
        let mut set = |s: usize, l: usize, v: u32| put(&mut b, 16 + s, l, v);
        let opcode = match *self {
            Csbk::Grant {
                kind,
                lpcn,
                slot,
                late_entry,
                emergency,
                offset,
                target,
                source,
            } => {
                set(0, 12, u32::from(lpcn));
                set(12, 1, u32::from(slot));
                set(13, 1, u32::from(late_entry));
                set(14, 1, u32::from(emergency));
                set(15, 1, u32::from(offset));
                set(16, 24, target);
                set(40, 24, source);
                kind.opcode()
            }
            Csbk::Aloha {
                syscode,
                ms,
                reg,
                backoff,
                mask,
                version,
            } => {
                set(3, 3, u32::from(version));
                set(8, 5, u32::from(mask));
                set(19, 1, u32::from(reg));
                set(20, 4, u32::from(backoff));
                set(24, 16, u32::from(syscode.0));
                set(40, 24, ms);
                25
            }
            Csbk::Bcast {
                ann,
                parms1,
                reg,
                backoff,
                syscode,
                parms2,
            } => {
                set(0, 5, u32::from(ann));
                set(5, 14, u32::from(parms1));
                set(19, 1, u32::from(reg));
                set(20, 4, u32::from(backoff));
                set(24, 16, u32::from(syscode.0));
                set(40, 24, parms2);
                40
            }
            Csbk::Clear {
                lpcn,
                group,
                target,
                source,
            } => {
                set(0, 12, u32::from(lpcn));
                set(15, 1, u32::from(group));
                set(16, 24, target);
                set(40, 24, source);
                46
            }
            Csbk::Other { bytes, .. } => return bytes,
        };
        b[0] = 0x80 | opcode;
        b
    }
}

impl fmt::Display for Csbk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Csbk::Grant {
                kind,
                lpcn,
                slot,
                emergency,
                target,
                source,
                late_entry,
                ..
            } => {
                let to = if kind.to_group() {
                    format!("TG {target}")
                } else {
                    target.to_string()
                };
                write!(f, "{} {source} → {to} on ", kind.alias())?;
                if lpcn == LPCN_ABSOLUTE {
                    write!(f, "an absolute channel")?;
                } else {
                    write!(f, "LPCN {lpcn}")?;
                }
                write!(f, " TS{}", slot + 1)?;
                if emergency {
                    write!(f, " EMERGENCY")?;
                }
                if late_entry {
                    write!(f, " (late entry)")?;
                }
                Ok(())
            }
            Csbk::Aloha {
                syscode, ms, reg, ..
            } => {
                write!(f, "C_ALOHA {syscode}")?;
                if ms != 0 {
                    write!(f, " to {ms}")?;
                }
                if reg {
                    write!(f, ", registration required")?;
                }
                Ok(())
            }
            Csbk::Bcast { ann, syscode, .. } => {
                write!(f, "C_BCAST {} {syscode}", announcement(ann))
            }
            Csbk::Clear {
                lpcn,
                group,
                target,
                ..
            } => {
                let to = if group {
                    format!("TG {target}")
                } else {
                    target.to_string()
                };
                write!(f, "P_CLEAR {to} back to the TSCC (LPCN {lpcn})")
            }
            Csbk::Other { opcode, fid, .. } => match (fid, csbk_alias(opcode)) {
                (0, Some(a)) => write!(f, "{a}"),
                _ => write!(f, "CSBK opcode {opcode} FID {fid:#04x}"),
            },
        }
    }
}

/// A Short LC from the CACH (28 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShortLc {
    Null,
    /// Act_Updt, TS 102 361-2 table 7.10: each slot's activity and the hash
    /// of its destination.
    Activity {
        ts1: u8,
        ts2: u8,
        hash1: u8,
        hash2: u8,
    },
    /// C_SYS_Parms (TSCC), TS 102 361-4 table 7.33.
    ControlParms {
        syscode14: u16,
        reg: bool,
        counter: u16,
    },
    /// P_SYS_Parms (payload channel), table 7.34.
    PayloadParms {
        syscode14: u16,
        data: u16,
    },
    Other {
        slco: u8,
        data: u32,
    },
}

impl ShortLc {
    pub fn parse(v: u32) -> ShortLc {
        let slco = (v >> 24) as u8;
        let d = v & 0xff_ffff;
        match slco {
            0 => ShortLc::Null,
            1 => ShortLc::Activity {
                ts1: (d >> 20) as u8 & 15,
                ts2: (d >> 16) as u8 & 15,
                hash1: (d >> 8) as u8,
                hash2: d as u8,
            },
            2 => ShortLc::ControlParms {
                syscode14: (d >> 10) as u16,
                reg: (d >> 9) & 1 == 1,
                counter: (d & 0x1ff) as u16,
            },
            3 => ShortLc::PayloadParms {
                syscode14: (d >> 10) as u16,
                data: (d & 0x3ff) as u16,
            },
            slco => ShortLc::Other { slco, data: d },
        }
    }

    pub fn to_u32(&self) -> u32 {
        match *self {
            ShortLc::Null => 0,
            ShortLc::Activity {
                ts1,
                ts2,
                hash1,
                hash2,
            } => {
                (1 << 24)
                    | (u32::from(ts1 & 15) << 20)
                    | (u32::from(ts2 & 15) << 16)
                    | (u32::from(hash1) << 8)
                    | u32::from(hash2)
            }
            ShortLc::ControlParms {
                syscode14,
                reg,
                counter,
            } => {
                (2 << 24)
                    | (u32::from(syscode14 & 0x3fff) << 10)
                    | (u32::from(reg) << 9)
                    | u32::from(counter & 0x1ff)
            }
            ShortLc::PayloadParms { syscode14, data } => {
                (3 << 24) | (u32::from(syscode14 & 0x3fff) << 10) | u32::from(data & 0x3ff)
            }
            ShortLc::Other { slco, data } => (u32::from(slco & 15) << 24) | (data & 0xff_ffff),
        }
    }
}

fn activity(a: u8) -> &'static str {
    match a {
        0 => "idle",
        2 => "group CSBK",
        3 => "individual CSBK",
        8 => "group voice",
        9 => "individual voice",
        10 => "individual data",
        11 => "group data",
        12 => "emergency group voice",
        13 => "emergency individual voice",
        _ => "reserved",
    }
}

impl fmt::Display for ShortLc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            ShortLc::Null => write!(f, "null"),
            ShortLc::Activity { ts1, ts2, .. } => {
                write!(f, "activity TS1 {}, TS2 {}", activity(ts1), activity(ts2))
            }
            ShortLc::ControlParms {
                syscode14,
                reg,
                counter,
            } => {
                write!(f, "C_SYS_Parms {}", SysCode(syscode14 << 2))?;
                if reg {
                    write!(f, ", registration required")?;
                }
                write!(f, ", slot counter {counter}")
            }
            ShortLc::PayloadParms { syscode14, .. } => {
                write!(f, "P_SYS_Parms {}", SysCode(syscode14 << 2))
            }
            ShortLc::Other { slco, .. } => write!(f, "SLCO {slco}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let lc = FullLc::GroupVoice {
            opts: ServiceOptions(0x80),
            group: 2350,
            source: 2_341_234,
        };
        assert_eq!(FullLc::parse(&lc.to_bytes()), lc);
        let sys = SysCode((0b01 << 14) | (5 << 7) | (3 << 2));
        assert_eq!(sys.net_site(), (5, 3));
        for c in [
            Csbk::Grant {
                kind: GrantKind::TalkgroupVoice,
                lpcn: 7,
                slot: 1,
                late_entry: false,
                emergency: true,
                offset: false,
                target: 9,
                source: 1_234_567,
            },
            Csbk::Aloha {
                syscode: sys,
                ms: 0,
                reg: true,
                backoff: 3,
                mask: 0,
                version: 1,
            },
            Csbk::Bcast {
                ann: 5,
                parms1: 0x123,
                reg: false,
                backoff: 2,
                syscode: sys,
                parms2: 0xabcdef,
            },
            Csbk::Clear {
                lpcn: 2,
                group: true,
                target: 9,
                source: 0xfffffd,
            },
        ] {
            assert_eq!(Csbk::parse(&c.to_bytes()), c);
        }
        for s in [
            ShortLc::ControlParms {
                syscode14: sys.0 >> 2,
                reg: true,
                counter: 300,
            },
            ShortLc::Activity {
                ts1: 8,
                ts2: 2,
                hash1: 0x12,
                hash2: 0x34,
            },
        ] {
            assert_eq!(ShortLc::parse(s.to_u32()), s);
        }
    }
}
