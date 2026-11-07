// made by CasualArclamp
//! P25 phase 2 (TIA-102.BBAB physical layer, TIA-102.BBAC MAC): the
//! two-slot TDMA traffic channel a phase 1 or phase 2 control channel
//! grants calls onto. Outbound (the site's transmitter) only: H-DQPSK at
//! 6000 symbols a second, 12 kb/s, two voice channels interleaved.
//!
//! The TIA documents are sold, not published, so the layouts follow OP25
//! (GPL-3.0-or-later, © 2013–2025 Max H. Parke KA1RBI and Graham J.
//! Norbury: `p25p2_framer`, `_isch`, `_duid`, `_tdma`, `_vf`, and
//! `apps/tdma/lfsr.py`), checked against DSD-FME's `p25p2_*.c` for the
//! MAC message fields.
//!
//! ```text
//!  superframe  12 timeslots, 360 ms; each logical channel (slot) has 6
//!  timeslot    180 symbols: ISCH (20) + burst (160, its DUID in 4 dibits)
//!  ISCH        S-ISCH (sync, 0x575D57F7FF) in timeslots 2,3,6,7,10,11;
//!              I-ISCH (a 40-bit code of 7 bits) in 0,1,4,5,8,9: the
//!              timeslot's place in the superframe
//!  4V burst    4 AMBE+2 frames (72 bits each) + ESS-B (4 hexbits)
//!  2V burst    2 AMBE+2 frames + ESS-A (28 hexbits): RS (44,16) over ESS
//!  FACCH       RS (63,35) shortened and punctured to 45 hexbits, CRC-12
//!  SACCH       the same code to 52 hexbits, CRC-12
//! ```
//!
//! Everything after the ISCH is scrambled by a 4320-bit sequence (one per
//! superframe) from a 44-bit LFSR seeded by WACN, system ID and NAC, which
//! a receiver normally learns from the control channel. Here the seed is
//! also *found* from the traffic channel alone: the sequence is linear in
//! the seed, and so is every Reed-Solomon parity check (RS over GF(2⁶) is
//! a binary linear code too), so one clean SACCH or FACCH gives over a
//! hundred linear equations in the 44 unknowns ([`scramble`]).

pub mod receiver;
pub mod rs63;
pub mod scramble;
pub mod sim;

use std::fmt;

pub use receiver::{Event, Receiver, Stats};

/// Symbols a second.
pub const SYMBOL_RATE: f64 = 6000.0;
/// Symbols in a timeslot (ISCH + burst).
pub const SLOT_SYMBOLS: usize = 180;
/// Timeslots in a superframe.
pub const SUPERFRAME: usize = 12;
/// The S-ISCH (sync), 40 bits, all outer symbols (OP25
/// `P25P2_FRAME_SYNC_MAGIC`).
pub const S_ISCH: u64 = 0x57_5D57_F7FF;

/// The logical channel (0 or 1) each timeslot of the superframe belongs
/// to: alternate, but the last two swap (the "inverted" SACCH slots; OP25
/// `which_slot`).
pub const CHANNEL_OF: [u8; SUPERFRAME] = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0];

/// The I-ISCH codewords, by their 7-bit value (OP25 `p25p2_isch.cc`): the
/// value's bits 6–5 are the channel, 4–3 the location (which pair of
/// timeslots), 2 a flag and 1–0 a count. Timeslot = location × 4 + channel.
#[rustfmt::skip]
pub const I_ISCH: [u64; 128] = [
    0x184229d461, 0x18761451f6, 0x181ae27e2f, 0x182edffbb8, 0x18df8a7510, 0x18ebb7f087, 0x188741df5e, 0x18b37c5ac9,
    0x1146a44f13, 0x117299ca84, 0x111e6fe55d, 0x112a5260ca, 0x11db07ee62, 0x11ef3a6bf5, 0x1183cc442c, 0x11b7f1c1bb,
    0x1a4a2e239e, 0x1a7e13a609, 0x1a12e589d0, 0x1a26d80c47, 0x1ad78d82ef, 0x1ae3b00778, 0x1a8f4628a1, 0x1abb7bad36,
    0x134ea3b8ec, 0x137a9e3d7b, 0x13166812a2, 0x1322559735, 0x13d300199d, 0x13e73d9c0a, 0x138bcbb3d3, 0x13bff63644,
    0x1442f705ef, 0x1476ca8078, 0x141a3cafa1, 0x142e012a36, 0x14df54a49e, 0x14eb692109, 0x14879f0ed0, 0x14b3a28b47,
    0x1d467a9e9d, 0x1d72471b0a, 0x1d1eb134d3, 0x1d2a8cb144, 0x1ddbd93fec, 0x1defe4ba7b, 0x1d831295a2, 0x1db72f1035,
    0x164af0f210, 0x167ecd7787, 0x16123b585e, 0x162606ddc9, 0x16d7535361, 0x16e36ed6f6, 0x168f98f92f, 0x16bba57cb8,
    0x1f4e7d6962, 0x1f7a40ecf5, 0x1f16b6c32c, 0x1f228b46bb, 0x1fd3dec813, 0x1fe7e34d84, 0x1f8b15625d, 0x1fbf28e7ca,
    0x084d62c339, 0x08795f46ae, 0x0815a96977, 0x082194ece0, 0x08d0c16248, 0x08e4fce7df, 0x08880ac806, 0x08bc374d91,
    0x0149ef584b, 0x017dd2dddc, 0x011124f205, 0x0125197792, 0x01d44cf93a, 0x01e0717cad, 0x018c875374, 0x01b8bad6e3,
    0x0a456534c6, 0x0a7158b151, 0x0a1dae9e88, 0x0a29931b1f, 0x0ad8c695b7, 0x0aecfb1020, 0x0a800d3ff9, 0x0ab430ba6e,
    0x0341e8afb4, 0x0375d52a23, 0x03192305fa, 0x032d1e806d, 0x03dc4b0ec5, 0x03e8768b52, 0x038480a48b, 0x03b0bd211c,
    0x044dbc12b7, 0x0479819720, 0x041577b8f9, 0x04214a3d6e, 0x04d01fb3c6, 0x04e4223651, 0x0488d41988, 0x04bce99c1f,
    0x0d493189c5, 0x0d7d0c0c52, 0x0d11fa238b, 0x0d25c7a61c, 0x0dd49228b4, 0x0de0afad23, 0x0d8c5982fa, 0x0db864076d,
    0x0645bbe548, 0x06718660df, 0x061d704f06, 0x06294dca91, 0x06d8184439, 0x06ec25c1ae, 0x0680d3ee77, 0x06b4ee6be0,
    0x0f41367e3a, 0x0f750bfbad, 0x0f19fdd474, 0x0f2dc051e3, 0x0fdc95df4b, 0x0fe8a85adc, 0x0f845e7505, 0x0fb063f092,
];

/// The I-ISCH value a timeslot sends (count and flag zero; the receiver
/// reads only the location and channel).
pub fn i_isch_value(timeslot: usize) -> Option<usize> {
    let (loc, ch) = (timeslot / 4, timeslot % 4);
    (ch < 2 && loc < 3).then_some((ch << 5) | (loc << 3))
}

/// The timeslot an I-ISCH value names, if it names one.
pub fn i_isch_timeslot(value: usize) -> Option<usize> {
    let (ch, loc) = ((value >> 5) & 3, (value >> 3) & 3);
    (ch < 2 && loc < 3).then_some(loc * 4 + ch)
}

/// Whether a timeslot carries the S-ISCH rather than an I-ISCH.
pub fn has_s_isch(timeslot: usize) -> bool {
    i_isch_value(timeslot).is_none()
}

/// The burst types the DUID gives (OP25 `handle_packet`).
pub mod duid {
    pub const V4: u8 = 0;
    pub const SACCH_S: u8 = 3;
    pub const LCCH_S: u8 = 4;
    pub const V2: u8 = 6;
    pub const FACCH_S: u8 = 9;
    pub const SACCH: u8 = 12;
    pub const LCCH: u8 = 13;
    pub const FACCH: u8 = 15;
}

pub fn duid_name(d: u8) -> &'static str {
    match d {
        duid::V4 => "4V",
        duid::V2 => "2V",
        duid::SACCH_S | duid::SACCH => "SACCH",
        duid::FACCH_S | duid::FACCH => "FACCH",
        duid::LCCH_S | duid::LCCH => "LCCH",
        _ => "?",
    }
}

/// Whether a burst type is scrambled.
pub fn scrambled(d: u8) -> bool {
    matches!(d, duid::V4 | duid::V2 | duid::SACCH_S | duid::FACCH_S | duid::LCCH_S)
}

/// The DUID's 8-bit codeword for a 4-bit type: the word whose eight
/// one-bit neighbours all decode to it in OP25's `_duid_lookup` (an
/// (8,4) code correcting one error).
pub fn duid_codeword(d: u8) -> u8 {
    (0..=255u8)
        .find(|&c| DUID_LOOKUP[c as usize] == d as i8 && (0..8).all(|b| DUID_LOOKUP[(c ^ (1 << b)) as usize] == d as i8))
        .expect("every type has a codeword")
}

/// The DUID's four dibits sit at these burst dibits (counted from the
/// timeslot's 10th symbol, as OP25 counts: the ISCH's second half is 0–9).
pub const DUID_AT: [usize; 4] = [10, 47, 132, 169];

/// OP25 `p25p2_duid.cc`: received codeword → type, −1 when it cannot say.
#[rustfmt::skip]
pub const DUID_LOOKUP: [i8; 256] = [
    0,0,0,-1,0,-1,-1,1,0,-1,-1,4,-1,8,2,-1,
    0,-1,-1,1,-1,1,1,1,-1,3,9,-1,5,-1,-1,1,
    0,-1,-1,10,-1,6,2,-1,-1,3,2,-1,2,-1,2,2,
    -1,3,7,-1,11,-1,-1,1,3,3,-1,3,-1,3,2,-1,
    0,-1,-1,4,-1,6,12,-1,-1,4,4,4,5,-1,-1,4,
    -1,13,7,-1,5,-1,-1,1,5,-1,-1,4,5,5,5,-1,
    -1,6,7,-1,6,6,-1,6,14,-1,-1,4,-1,6,2,-1,
    7,-1,7,7,-1,6,7,-1,-1,3,7,-1,5,-1,-1,15,
    0,-1,-1,10,-1,8,12,-1,-1,8,9,-1,8,8,-1,8,
    -1,13,9,-1,11,-1,-1,1,9,-1,9,9,-1,8,9,-1,
    -1,10,10,10,11,-1,-1,10,14,-1,-1,10,-1,8,2,-1,
    11,-1,-1,10,11,11,11,-1,-1,3,9,-1,11,-1,-1,15,
    -1,13,12,-1,12,-1,12,12,14,-1,-1,4,-1,8,12,-1,
    13,13,-1,13,-1,13,12,-1,-1,13,9,-1,5,-1,-1,15,
    14,-1,-1,10,-1,6,12,-1,14,14,14,-1,14,-1,-1,15,
    -1,13,7,-1,11,-1,-1,15,14,-1,-1,15,-1,15,15,15,
];

/// The AMBE+2 frames of a 4V burst start at these burst dibits (36 each);
/// a 2V burst has the first two (OP25 `handle_packet`).
pub const VOICE_AT: [usize; 4] = [11, 48, 96, 133];
/// A 4V burst's ESS-B: 12 dibits (4 hexbits) here.
pub const ESS_B_AT: usize = 84;
/// A 2V burst's ESS-A: 28 hexbits from here, skipping DUID dibit 132.
pub const ESS_A_AT: usize = 84;

/// The burst dibits carrying a FACCH (fast) or SACCH, in order (OP25
/// `handle_acch_frame`).
pub fn acch_dibits(fast: bool) -> Vec<usize> {
    if fast {
        (11..47).chain(48..79).chain(100..132).chain(133..169).collect()
    } else {
        (11..47).chain(48..132).chain(133..169).collect()
    }
}

/// Where an ACCH's hexbits sit in the RS (63,35) codeword (index 0 the
/// highest term): the first one sent, how many are sent, and the data
/// length in bits before the CRC-12. Positions before the first are
/// shortened (zero); those after the last sent are punctured parity.
pub const fn acch_layout(fast: bool) -> (usize, usize, usize) {
    if fast { (9, 45, 144) } else { (5, 52, 168) }
}

/// CRC-12 (x¹² + x¹¹ + x⁷ + x⁴ + x² + x + 1, inverted; OP25 `crc12`) of
/// `bits`.
pub fn crc12(bits: &[u8]) -> u16 {
    let mut r: u16 = 0;
    for &b in bits {
        let fb = ((r >> 11) & 1) ^ u16::from(b & 1);
        r = (r << 1) & 0xfff;
        if fb == 1 {
            r ^= 0x897;
        }
    }
    r ^ 0xfff
}

/// The MAC PDU's opcode (its first byte's top three bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacOp {
    Signal,
    Ptt,
    EndPtt,
    Idle,
    Active,
    Hangtime,
    Other(u8),
}

impl MacOp {
    pub fn from_byte(b: u8) -> MacOp {
        match b >> 5 {
            0 => MacOp::Signal,
            1 => MacOp::Ptt,
            2 => MacOp::EndPtt,
            3 => MacOp::Idle,
            4 => MacOp::Active,
            6 => MacOp::Hangtime,
            n => MacOp::Other(n),
        }
    }

    fn code(self) -> u8 {
        match self {
            MacOp::Signal => 0,
            MacOp::Ptt => 1,
            MacOp::EndPtt => 2,
            MacOp::Idle => 3,
            MacOp::Active => 4,
            MacOp::Hangtime => 6,
            MacOp::Other(n) => n,
        }
    }
}

/// One MAC message (TIA-102.BBAC; fields as DSD-FME's `p25p2_vpdu.c`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MacMsg {
    /// Group voice channel user (opcode 0x01 abbreviated, 0x21 extended):
    /// who is talking on this channel.
    GroupVoice { opts: u8, group: u16, source: u32 },
    /// Unit-to-unit voice channel user (0x02 / 0x22).
    UnitVoice { opts: u8, target: u32, source: u32 },
    /// Anything else, by opcode, with its bytes.
    Other { op: u8, bytes: Vec<u8> },
}

impl MacMsg {
    /// The call it names: (source, destination, group, emergency, encrypted).
    pub fn call(&self) -> Option<(u32, u32, bool, u8)> {
        match *self {
            MacMsg::GroupVoice { opts, group, source } => Some((source, u32::from(group), true, opts)),
            MacMsg::UnitVoice { opts, target, source } => Some((source, target, false, opts)),
            MacMsg::Other { .. } => None,
        }
    }
}

/// The MAC message names, by opcode (TIA-102.BBAC as SDRTrunk names them),
/// for those shown by number.
pub fn mac_msg_name(op: u8) -> &'static str {
    match op {
        0x00 => "null",
        0x01 | 0x21 => "group voice channel user",
        0x02 | 0x22 => "unit-to-unit voice channel user",
        0x03 => "telephone interconnect voice channel user",
        0x05 => "group voice channel grant update multiple",
        0x08 => "null (avoid zero bias)",
        0x11 => "indirect group paging",
        0x12 => "individual paging with priority",
        0x25 => "group voice channel grant update multiple explicit",
        0x30 => "power control signal quality",
        0x31 => "MAC release",
        0x40 => "group voice channel grant",
        0x42 => "group voice channel grant update",
        0x44 => "unit-to-unit voice service channel grant",
        0x70 => "SNDCP data channel announcement",
        0x73 => "identifier update TDMA",
        0x74 => "identifier update VHF/UHF",
        0x76 => "secondary control channel broadcast",
        0x78 => "system service broadcast",
        0x79 => "secondary control channel broadcast",
        0x7a => "RFSS status broadcast",
        0x7b => "network status broadcast",
        0x7c => "adjacent status broadcast",
        0x7d => "identifier update",
        _ => "MAC message",
    }
}

impl fmt::Display for MacMsg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flags = |opts: u8| {
            let mut s = String::new();
            if opts & 0x80 != 0 {
                s += " (emergency)";
            }
            s
        };
        match self {
            MacMsg::GroupVoice { opts, group, source } => {
                write!(f, "group call TG {group} from {source}{}", flags(*opts))
            }
            MacMsg::UnitVoice { opts, target, source } => {
                write!(f, "private call {source} → {target}{}", flags(*opts))
            }
            MacMsg::Other { op, .. } => write!(f, "{} (0x{op:02X})", mac_msg_name(*op)),
        }
    }
}

/// The MAC messages' lengths in bytes, by opcode, where the message does not
/// say (OP25 `mac_msg_len`).
#[rustfmt::skip]
const MAC_MSG_LEN: [u8; 256] = [
    0,  7,  8,  7,  0, 16,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0, 14, 15,  0,  0, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    5,  7,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    9,  7,  9,  0,  9,  8,  9,  0, 10, 10,  9,  0, 10,  0,  0,  0,
    0,  0,  0,  0,  9,  7,  0,  0, 10,  0,  7,  0, 10,  8, 14,  7,
    9,  9,  0,  0,  9,  0,  0,  9, 10,  0,  7, 10, 10,  7,  0,  9,
    9, 29,  9,  9,  9,  9, 10, 13,  9,  9,  9, 11,  9,  9,  0,  0,
    8,  0,  0,  7, 11,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    16, 0,  0, 11, 13, 11, 11, 11, 10,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    11, 0,  0,  8, 15, 12, 15, 32, 12, 12,  0, 27, 14, 29, 29, 32,
    0,  0,  0,  0,  0,  0,  9,  0, 14, 29, 11, 27, 14,  0, 40, 11,
    28, 0,  0, 14, 17, 14,  0,  0, 16,  8, 11,  0, 13, 19,  0,  0,
    0,  0, 16, 14,  0,  0, 12,  0, 22,  0, 11, 13, 11,  0, 15,  0,
];

/// A MAC PDU from a FACCH or SACCH whose CRC passed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MacPdu {
    pub op: MacOp,
    /// MAC_PTT and MAC_END_PTT: the talker, the group, and (PTT only) the
    /// encryption: (source, group, ALGID, KID). ALGID 0x80 is clear.
    pub ptt: Option<(u32, u16, u8, u16)>,
    pub messages: Vec<MacMsg>,
}

impl MacPdu {
    /// Parse a PDU's bytes (18 from a FACCH, 21 from a SACCH; OP25
    /// `process_mac_pdu` and `decode_mac_msg`).
    pub fn parse(b: &[u8]) -> MacPdu {
        let op = MacOp::from_byte(b[0]);
        let mut pdu = MacPdu { op, ptt: None, messages: Vec::new() };
        match op {
            MacOp::Ptt | MacOp::EndPtt => {
                let src = (u32::from(b[13]) << 16) | (u32::from(b[14]) << 8) | u32::from(b[15]);
                let grp = u16::from_be_bytes([b[16], b[17]]);
                let (alg, kid) = if op == MacOp::Ptt {
                    (b[10], u16::from_be_bytes([b[11], b[12]]))
                } else {
                    (0x80, 0)
                };
                pdu.ptt = Some((src, grp, alg, kid));
            }
            MacOp::Signal | MacOp::Idle | MacOp::Active | MacOp::Hangtime => {
                let len = if op == MacOp::Signal { 18.min(b.len()) } else { b.len() };
                let mut at = 1;
                while at < len {
                    let opc = b[at];
                    let left = len - at;
                    let n = match opc {
                        0x00 => left,
                        0x08 => usize::from(b.get(at + 1).copied().unwrap_or(0) & 0x3f),
                        0x11 => usize::from(b.get(at + 1).copied().unwrap_or(0) & 3) * 2 + 4,
                        0x12 => usize::from(b.get(at + 1).copied().unwrap_or(0) & 3) * 3 + 5,
                        _ => {
                            let own = if opc >> 6 == 2 {
                                usize::from(b.get(at + 2).copied().unwrap_or(0) & 0x3f)
                            } else {
                                0
                            };
                            if own != 0 { own } else { usize::from(MAC_MSG_LEN[opc as usize]) }
                        }
                    };
                    if n == 0 || at + n > len {
                        break;
                    }
                    let m = &b[at..at + n];
                    let u24 = |i: usize| (u32::from(m[i]) << 16) | (u32::from(m[i + 1]) << 8) | u32::from(m[i + 2]);
                    match opc {
                        0x00 | 0x08 => {}
                        0x01 | 0x21 if n >= 7 => pdu.messages.push(MacMsg::GroupVoice {
                            opts: m[1],
                            group: u16::from_be_bytes([m[2], m[3]]),
                            source: if opc == 0x21 && n >= 14 { u24(11) } else { u24(4) },
                        }),
                        0x02 | 0x22 if n >= 8 => pdu.messages.push(MacMsg::UnitVoice {
                            opts: m[1],
                            target: u24(2),
                            source: if opc == 0x22 && n >= 15 { u24(12) } else { u24(5) },
                        }),
                        _ => pdu.messages.push(MacMsg::Other { op: opc, bytes: m.to_vec() }),
                    }
                    at += n;
                }
            }
            MacOp::Other(_) => {}
        }
        pdu
    }

    /// The PDU's bytes, `len` of them (for the simulator).
    pub fn to_bytes(&self, len: usize) -> Vec<u8> {
        let mut b = vec![0u8; len];
        b[0] = self.op.code() << 5;
        if let Some((src, grp, alg, kid)) = self.ptt {
            b[10] = alg;
            b[11..13].copy_from_slice(&kid.to_be_bytes());
            b[13] = (src >> 16) as u8;
            b[14] = (src >> 8) as u8;
            b[15] = src as u8;
            b[16..18].copy_from_slice(&grp.to_be_bytes());
            return b;
        }
        let mut at = 1;
        for m in &self.messages {
            let bytes: Vec<u8> = match *m {
                MacMsg::GroupVoice { opts, group, source } => {
                    let g = group.to_be_bytes();
                    vec![0x01, opts, g[0], g[1], (source >> 16) as u8, (source >> 8) as u8, source as u8]
                }
                MacMsg::UnitVoice { opts, target, source } => vec![
                    0x02,
                    opts,
                    (target >> 16) as u8,
                    (target >> 8) as u8,
                    target as u8,
                    (source >> 16) as u8,
                    (source >> 8) as u8,
                    source as u8,
                ],
                MacMsg::Other { ref bytes, .. } => bytes.clone(),
            };
            if at + bytes.len() > len {
                break;
            }
            b[at..at + bytes.len()].copy_from_slice(&bytes);
            at += bytes.len();
        }
        b
    }
}

impl fmt::Display for MacPdu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self.op {
            MacOp::Signal => "MAC_SIGNAL",
            MacOp::Ptt => "MAC_PTT",
            MacOp::EndPtt => "MAC_END_PTT",
            MacOp::Idle => "MAC_IDLE",
            MacOp::Active => "MAC_ACTIVE",
            MacOp::Hangtime => "MAC_HANGTIME",
            MacOp::Other(_) => "MAC ?",
        };
        write!(f, "{name}")?;
        if let Some((s, g, a, _)) = self.ptt {
            write!(f, " TG {g} from {s}")?;
            if a != 0x80 {
                write!(f, " (encrypted, ALGID 0x{a:02X})")?;
            }
        }
        for m in &self.messages {
            write!(f, ", {m}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duid_codewords_decode_to_themselves() {
        for d in [duid::V4, duid::V2, duid::SACCH_S, duid::FACCH_S, duid::SACCH, duid::FACCH, duid::LCCH] {
            let c = duid_codeword(d);
            assert_eq!(DUID_LOOKUP[c as usize], d as i8);
        }
    }

    #[test]
    fn i_isch_timeslots_round_trip() {
        let named: Vec<usize> = (0..12).filter(|&t| !has_s_isch(t)).collect();
        assert_eq!(named, [0, 1, 4, 5, 8, 9]);
        for t in named {
            assert_eq!(i_isch_timeslot(i_isch_value(t).unwrap()), Some(t));
        }
    }

    #[test]
    fn crc12_matches_op25s_long_division() {
        // OP25's: the message times x^12 divided bit by bit, inverted.
        let op25 = |bits: &[u8]| {
            let poly = [1u8, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1];
            let mut buf = bits.to_vec();
            buf.extend([0u8; 12]);
            for i in 0..bits.len() {
                if buf[i] == 1 {
                    for j in 0..13 {
                        buf[i + j] ^= poly[j];
                    }
                }
            }
            buf[bits.len()..].iter().fold(0u16, |c, &b| (c << 1) | u16::from(b)) ^ 0xfff
        };
        let bits: Vec<u8> = (0..144).map(|i| ((i * 7 + i / 3) % 5 == 0) as u8).collect();
        assert_eq!(crc12(&bits), op25(&bits));
    }

    #[test]
    fn mac_pdus_round_trip() {
        let p = MacPdu {
            op: MacOp::Active,
            ptt: None,
            messages: vec![MacMsg::GroupVoice { opts: 0, group: 4001, source: 1_234_567 }],
        };
        assert_eq!(MacPdu::parse(&p.to_bytes(21)), p);
        let q = MacPdu { op: MacOp::Ptt, ptt: Some((7_654_321, 33, 0x80, 0)), messages: vec![] };
        assert_eq!(MacPdu::parse(&q.to_bytes(18)), q);
    }
}
