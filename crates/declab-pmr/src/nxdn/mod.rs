// made by CasualArclamp
//! NXDN48 (NXDN at 4800 bit/s in 6.25 kHz): conventional voice and its
//! signalling.
//!
//! The NXDN Forum's specifications are behind a registration form, so the
//! layouts here follow two open implementations that interoperate with real
//! radios: MMDVMHost (`NXDN*.cpp`, GPL-2.0-or-later, © Jonathan Naylor
//! G4KLX), which both builds and reads NXDN frames, and DSD (`nxdn_const.h`,
//! ISC licence, © 2010 DSD Author) for the voice frames.
//!
//! A frame is 384 bits (192 symbols, 80 ms at 2400 baud):
//!
//! ```text
//!  FSW 20 | LICH 16 | SACCH 60 | 288: 4 voice frames, or FACCH1 in either half or both
//! ```
//!
//! Everything after the frame sync word is scrambled by a fixed sequence
//! (MMDVMHost's `SCRAMBLER`), which touches only each dibit's first bit.
//!
//! - [`receiver`]: IQ in, [`Event`]s out.
//! - [`sim`]: a modulator for the tests and the sweep.
//!
//! Not yet: the control channel (RCCH: frames are recognised but their CAC
//! is not decoded), the data channel's UDCH/FACCH2, and NXDN96 (9600 bit/s).

pub mod receiver;
pub mod sim;

use std::fmt;

use crate::conv;

pub use receiver::{Event, Receiver, Stats};

/// Symbols a second (NXDN48).
pub const SYMBOL_RATE: f64 = 2400.0;
/// The outer symbols' deviation (±3), NXDN48.
pub const OUTER_DEV_HZ: f64 = 1050.0;
/// The Nyquist roll-off.
pub const ALPHA: f64 = 0.2;
/// Bits and symbols a frame.
pub const FRAME_BITS: usize = 384;
pub const FRAME_SYMBOLS: usize = 192;

/// The frame sync word, 20 bits (MMDVMHost `NXDN_FSW_BYTES` 0xCDF59).
pub const FSW: u32 = 0xC_DF59;

/// Where the channels sit in a frame (bits).
pub mod layout {
    pub const LICH: usize = 20;
    pub const SACCH: usize = 36;
    pub const PAYLOAD: usize = 96;
    pub const FACCH1_BITS: usize = 144;
    pub const VCH_BITS: usize = 72;
}

/// The scrambler, XORed over the whole frame (MMDVMHost `SCRAMBLER`).
const SCRAMBLER: [u8; 48] = [
    0x00, 0x00, 0x00, 0x82, 0xA0, 0x88, 0x8A, 0x00, 0xA2, 0xA8, 0x82, 0x8A, 0x82, 0x02, 0x20, 0x08,
    0x8A, 0x20, 0xAA, 0xA2, 0x82, 0x08, 0x22, 0x8A, 0xAA, 0x08, 0x28, 0x88, 0x28, 0x28, 0x00, 0x0A,
    0x02, 0x82, 0x20, 0x28, 0x82, 0x2A, 0xAA, 0x20, 0x22, 0x80, 0xA8, 0x8A, 0x08, 0xA0, 0xAA, 0x02,
];

/// Frame bit `i` of the scrambler.
#[inline]
pub fn scrambler_bit(i: usize) -> u8 {
    (SCRAMBLER[i / 8] >> (7 - i % 8)) & 1
}

/// The symbol a dibit is sent as (as DMR: 01 → +3, 00 → +1, 10 → −1,
/// 11 → −3).
pub use crate::dmr::symbol;

/// The LICH: what the frame carries (MMDVMHost `CNXDNLICH`). Each of its 8
/// bits is sent as the first bit of a dibit whose second bit is 1, so every
/// LICH symbol is ±3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Lich(pub u8);

/// Radio channel types (the LICH's RFCT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rfct {
    /// Control channel (trunking).
    Rcch,
    /// Traffic channel: voice.
    Rtch,
    /// Data channel.
    Rdch,
    /// Composite control channel's traffic.
    RtchC,
}

/// What a traffic frame's 288 payload bits are (the LICH's option, "steal").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Steal {
    /// Four voice frames.
    None,
    /// FACCH1 in the first half, voice in the second.
    First,
    /// Voice in the first half, FACCH1 in the second.
    Second,
    /// FACCH1 in both halves.
    Both,
}

impl Lich {
    pub fn new(rfct: Rfct, fct: u8, steal: Steal, outbound: bool) -> Lich {
        let r = match rfct {
            Rfct::Rcch => 0,
            Rfct::Rtch => 1,
            Rfct::Rdch => 2,
            Rfct::RtchC => 3,
        };
        let o = match steal {
            Steal::Both => 0,
            Steal::First => 1,
            Steal::Second => 2,
            Steal::None => 3,
        };
        let v = (r << 6) | ((fct & 3) << 4) | (o << 2) | (u8::from(outbound) << 1);
        Lich(v | u8::from(Lich::parity(v)))
    }

    /// MMDVMHost `getParity`: set for 0x8x and 0xBx.
    fn parity(v: u8) -> bool {
        matches!(v & 0xF0, 0x80 | 0xB0)
    }

    pub fn parity_ok(self) -> bool {
        (self.0 & 1 == 1) == Lich::parity(self.0)
    }

    pub fn rfct(self) -> Rfct {
        match self.0 >> 6 {
            0 => Rfct::Rcch,
            1 => Rfct::Rtch,
            2 => Rfct::Rdch,
            _ => Rfct::RtchC,
        }
    }

    /// Functional channel type: on a traffic channel 0 is SACCH
    /// (non-superframe), 1 UDCH, 2 SACCH in a superframe, 3 idle SACCH.
    pub fn fct(self) -> u8 {
        (self.0 >> 4) & 3
    }

    pub fn steal(self) -> Steal {
        match (self.0 >> 2) & 3 {
            0 => Steal::Both,
            1 => Steal::First,
            2 => Steal::Second,
            _ => Steal::None,
        }
    }

    pub fn outbound(self) -> bool {
        self.0 & 2 != 0
    }
}

/// SACCH interleave: coded bit `i` is sent at `(i % 12) * 5 + i / 12`
/// (MMDVMHost `CNXDNSACCH` INTERLEAVE_TABLE).
fn sacch_pos(i: usize) -> usize {
    (i % 12) * 5 + i / 12
}

/// FACCH1 interleave, 16 by 9 (MMDVMHost `CNXDNFACCH1`).
fn facch1_pos(i: usize) -> usize {
    (i % 16) * 9 + i / 16
}

/// The punctured places: SACCH drops coded bits 5, 11, … 71 of 72; FACCH1
/// drops 1, 5, … 189 of 192.
fn sacch_punctured(i: usize) -> bool {
    i % 6 == 5
}

fn facch1_punctured(i: usize) -> bool {
    i % 4 == 1
}

/// MMDVMHost's CRCs: CRC-6 (x⁶+x⁵+x²+x+1, all ones to start), CRC-12
/// (0x80F) and CRC-15 (0x4CC5), no final inversion.
fn crc(bits: &[u8], width: u32, poly: u32) -> u32 {
    let top = 1 << (width - 1);
    let mask = (1 << width) - 1;
    let mut c = mask;
    for &b in bits {
        let fb = u32::from(b & 1) ^ u32::from(c & top != 0);
        c = (c << 1) & mask;
        if fb == 1 {
            c ^= poly;
        }
    }
    c
}

pub fn crc6(bits: &[u8]) -> u32 {
    crc(bits, 6, 0x27)
}

pub fn crc12(bits: &[u8]) -> u32 {
    crc(bits, 12, 0x80F)
}

pub fn crc15(bits: &[u8]) -> u32 {
    crc(bits, 15, 0x4CC5)
}

fn push_bits(v: u32, n: usize, out: &mut Vec<u8>) {
    out.extend((0..n).rev().map(|i| ((v >> i) & 1) as u8));
}

/// Encode `data` bits with a CRC of `width` (via `f`), the 4 tail bits,
/// the code, puncturing (`punct`) and interleaving (`pos`) into `n` sent
/// bits.
fn channel_encode(
    data: &[u8],
    width: usize,
    f: fn(&[u8]) -> u32,
    punct: fn(usize) -> bool,
    pos: fn(usize) -> usize,
    n: usize,
) -> Vec<u8> {
    let mut bits = data.to_vec();
    push_bits(f(data), width, &mut bits);
    bits.extend([0, 0, 0, 0]);
    let coded = conv::encode(&bits);
    let kept: Vec<u8> = coded
        .iter()
        .enumerate()
        .filter(|(i, _)| !punct(*i))
        .map(|(_, &b)| b)
        .collect();
    debug_assert_eq!(kept.len(), n);
    let mut out = vec![0u8; n];
    for (i, &b) in kept.iter().enumerate() {
        out[pos(i)] = b;
    }
    out
}

/// The inverse, from `n` received LLRs: the data bits if the CRC checks.
fn channel_decode(
    llr: &[f32],
    data_bits: usize,
    width: usize,
    f: fn(&[u8]) -> u32,
    punct: fn(usize) -> bool,
    pos: fn(usize) -> usize,
) -> Option<Vec<u8>> {
    let total = 2 * (data_bits + width + 4);
    let mut coded = Vec::with_capacity(total);
    let mut k = 0;
    for i in 0..total {
        if punct(i) {
            coded.push(0.0);
        } else {
            coded.push(llr[pos(k)]);
            k += 1;
        }
    }
    let (bits, _) = conv::decode(&coded);
    let (data, rest) = bits.split_at(data_bits);
    let got = rest[..width]
        .iter()
        .fold(0u32, |v, &b| (v << 1) | u32::from(b));
    (f(data) == got).then(|| data.to_vec())
}

/// SACCH: 26 bits (structure 2, RAN 6, 18 of a message) into 60.
pub fn sacch_encode(bits26: &[u8]) -> Vec<u8> {
    channel_encode(bits26, 6, crc6, sacch_punctured, sacch_pos, 60)
}

pub fn sacch_decode(llr: &[f32]) -> Option<Vec<u8>> {
    channel_decode(llr, 26, 6, crc6, sacch_punctured, sacch_pos)
}

/// FACCH1: 80 bits (a 10-byte message) into 144.
pub fn facch1_encode(bits80: &[u8]) -> Vec<u8> {
    channel_encode(bits80, 12, crc12, facch1_punctured, facch1_pos, 144)
}

pub fn facch1_decode(llr: &[f32]) -> Option<Vec<u8>> {
    channel_decode(llr, 80, 12, crc12, facch1_punctured, facch1_pos)
}

/// A layer 3 message (the first bytes of FACCH1's 10 or a SACCH
/// superframe's 9), as MMDVMHost `CNXDNLayer3` reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message(pub Vec<u8>);

/// Message types MMDVMHost knows (`NXDNDefines.h`).
pub fn message_name(t: u8) -> &'static str {
    match t {
        0x01 => "VCALL",
        0x03 => "VCALL_IV",
        0x08 => "TX_REL",
        0x09 => "DCALL_HDR",
        0x0B => "DCALL_DATA",
        0x0C => "DCALL_ACK",
        0x0F => "HEAD_DLY",
        0x10 => "IDLE",
        0x28 => "AUTH_INQ_REQ",
        0x29 => "AUTH_INQ_RESP",
        0x30 => "STAT_INQ_REQ",
        0x31 => "STAT_INQ_RESP",
        0x32 => "STAT_REQ",
        0x33 => "STAT_RESP",
        0x34 => "REM_CON_REQ",
        0x35 => "REM_CON_RESP",
        0x38 => "SDCALL_REQ_HDR",
        0x39 => "SDCALL_REQ_DATA",
        0x3A => "SDCALL_IV",
        0x3B => "SDCALL_RESP",
        0x3F => "PROP_FORM",
        _ => "unknown",
    }
}

impl Message {
    pub fn kind(&self) -> u8 {
        self.0[0] & 0x3F
    }

    /// Calls and releases: (source, destination, group?).
    pub fn call(&self) -> Option<(u16, u16, bool)> {
        matches!(self.kind(), 0x01 | 0x08 | 0x09).then(|| {
            let b = &self.0;
            (
                u16::from_be_bytes([b[3], b[4]]),
                u16::from_be_bytes([b[5], b[6]]),
                b[2] & 0x80 == 0,
            )
        })
    }

    /// A voice call message (for the simulator).
    pub fn vcall(kind: u8, source: u16, dest: u16, group: bool) -> Message {
        let mut b = vec![0u8; 10];
        b[0] = kind & 0x3F;
        b[2] = if group { 0x20 } else { 0xA0 };
        b[3..5].copy_from_slice(&source.to_be_bytes());
        b[5..7].copy_from_slice(&dest.to_be_bytes());
        Message(b)
    }
}

impl fmt::Display for Message {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = message_name(self.kind());
        match (self.kind(), self.call()) {
            (0x01, Some((s, d, true))) => write!(f, "group call TG {d} from {s}"),
            (0x01, Some((s, d, false))) => write!(f, "private call {s} → {d}"),
            (0x08, Some((s, d, g))) => write!(f, "TX_REL {s} → {}{d}", if g { "TG " } else { "" }),
            (_, Some((s, d, g))) => write!(f, "{name} {s} → {}{d}", if g { "TG " } else { "" }),
            _ => write!(f, "{name}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interleaves_are_permutations() {
        let mut s: Vec<usize> = (0..60).map(sacch_pos).collect();
        s.sort();
        assert_eq!(s, (0..60).collect::<Vec<_>>());
        let mut f: Vec<usize> = (0..144).map(facch1_pos).collect();
        f.sort();
        assert_eq!(f, (0..144).collect::<Vec<_>>());
        // MMDVMHost's tables start 0, 5, 10 … and 0, 9, 18 …
        assert_eq!((sacch_pos(1), sacch_pos(12)), (5, 1));
        assert_eq!((facch1_pos(1), facch1_pos(16)), (9, 1));
    }

    #[test]
    fn sacch_and_facch1_round_trip() {
        let d26: Vec<u8> = (0..26).map(|i| (i % 3 == 0) as u8).collect();
        let tx = sacch_encode(&d26);
        let mut llr: Vec<f32> = tx
            .iter()
            .map(|&b| if b == 0 { 2.0 } else { -2.0 })
            .collect();
        llr[7] = -llr[7];
        assert_eq!(sacch_decode(&llr).as_deref(), Some(&d26[..]));
        let d80: Vec<u8> = (0..80).map(|i| (i % 7 < 3) as u8).collect();
        let tx = facch1_encode(&d80);
        let mut llr: Vec<f32> = tx
            .iter()
            .map(|&b| if b == 0 { 2.0 } else { -2.0 })
            .collect();
        for i in [3, 40, 99] {
            llr[i] = -llr[i];
        }
        assert_eq!(facch1_decode(&llr).as_deref(), Some(&d80[..]));
    }

    #[test]
    fn lich_fields_and_parity() {
        let l = Lich::new(Rfct::Rtch, 2, Steal::None, true);
        assert!(l.parity_ok());
        assert_eq!(
            (l.rfct(), l.fct(), l.steal(), l.outbound()),
            (Rfct::Rtch, 2, Steal::None, true)
        );
        let m = Message::vcall(1, 1234, 77, true);
        assert_eq!(m.call(), Some((1234, 77, true)));
        assert_eq!(m.to_string(), "group call TG 77 from 1234");
    }
}
