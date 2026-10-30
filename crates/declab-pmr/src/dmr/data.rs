// made by CasualArclamp
//! DMR packet data (ETSI TS 102 361-1 clauses 8.2, 9.2 and annex B): the
//! data headers, rate ½ and ¾ blocks, their CRCs, and the reassembly of a
//! whole packet from a header and the blocks that follow it.
//!
//! Rate ¾ blocks are trellis coded (B.2.4) and decoded here by a soft
//! Viterbi over the 8-state machine, every branch scored on the four bits'
//! LLRs: about 2 dB better than slicing the dibits first.

use super::bptc::to_bits;
use super::mask;
use crate::fec;

/// The data header's format (DPF), table 9.30.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    Udt,
    Response,
    Unconfirmed,
    Confirmed,
    ShortDefined,
    ShortRaw,
    Proprietary,
    Other(u8),
}

impl Format {
    fn from_u8(v: u8) -> Format {
        match v {
            0 => Format::Udt,
            1 => Format::Response,
            2 => Format::Unconfirmed,
            3 => Format::Confirmed,
            13 => Format::ShortDefined,
            14 => Format::ShortRaw,
            15 => Format::Proprietary,
            v => Format::Other(v),
        }
    }
}

/// Service access points, table 9.31.
pub mod sap {
    pub const UDT: u8 = 0;
    pub const TCP_HC: u8 = 2;
    pub const UDP_HC: u8 = 3;
    pub const IP: u8 = 4;
    pub const ARP: u8 = 5;
    pub const PROPRIETARY: u8 = 9;
    pub const SHORT_DATA: u8 = 10;
}

/// A data header (C_HEAD, U_HEAD and the rest share the first 8 bytes'
/// layout closely enough for reassembly), tables 9.10 and 9.15.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataHeader {
    pub group: bool,
    pub format: Format,
    pub sap: u8,
    /// Pad octets at the end of the user data (POC).
    pub pad: u8,
    pub dst: u32,
    pub src: u32,
    pub blocks: u8,
}

impl DataHeader {
    pub fn parse(b: &[u8; 10]) -> DataHeader {
        DataHeader {
            group: b[0] & 0x80 != 0,
            format: Format::from_u8(b[0] & 15),
            sap: b[1] >> 4,
            pad: ((b[0] >> 4) & 1) << 4 | (b[1] & 15),
            dst: u32::from_be_bytes([0, b[2], b[3], b[4]]),
            src: u32::from_be_bytes([0, b[5], b[6], b[7]]),
            blocks: b[8] & 0x7f,
        }
    }

    pub fn to_bytes(&self) -> [u8; 10] {
        let dpf = match self.format {
            Format::Udt => 0,
            Format::Response => 1,
            Format::Unconfirmed => 2,
            Format::Confirmed => 3,
            Format::ShortDefined => 13,
            Format::ShortRaw => 14,
            Format::Proprietary => 15,
            Format::Other(v) => v & 15,
        };
        let mut b = [0u8; 10];
        b[0] = (u8::from(self.group) << 7) | ((self.pad >> 4) & 1) << 4 | dpf;
        b[1] = (self.sap << 4) | (self.pad & 15);
        b[2..5].copy_from_slice(&self.dst.to_be_bytes()[1..]);
        b[5..8].copy_from_slice(&self.src.to_be_bytes()[1..]);
        // Full message flag set; S, N(S), FSN zero.
        b[8] = 0x80 | (self.blocks & 0x7f);
        b
    }

    /// The header's CRC as received is right (under the data header mask).
    pub fn crc_ok(b: &[u8; 12]) -> bool {
        let mut bits = Vec::with_capacity(80);
        to_bits(&b[..10], &mut bits);
        fec::crc_ccitt(&bits) ^ mask::DATA_HEADER == u16::from_be_bytes([b[10], b[11]])
    }
}

/// CRC-9 of a confirmed block, B.3.10: the block data then the 7-bit serial
/// number, x^9+x^6+x^4+x^3+1, inverted.
pub fn crc9(data: &[u8], serial: u8) -> u16 {
    let mut bits = Vec::with_capacity(data.len() * 8 + 7);
    to_bits(data, &mut bits);
    bits.extend((0..7).rev().map(|i| (serial >> i) & 1));
    let mut crc: u16 = 0;
    for b in bits {
        let top = ((crc >> 8) as u8 & 1) ^ b;
        crc = (crc << 1) & 0x1ff;
        if top == 1 {
            crc ^= 0x059;
        }
    }
    crc ^ 0x1ff
}

/// The message CRC-32, B.3.9: over the user data and pad, each 16-bit
/// word's octets swapped, x^32+…+1 (0x04C11DB7), no inversion. Returned as
/// the four octets in sending order (least significant first, figure B.8C).
pub fn crc32(data: &[u8]) -> [u8; 4] {
    let mut crc: u32 = 0;
    let mut feed = |byte: u8| {
        for i in (0..8).rev() {
            let top = (crc >> 31) as u8 ^ ((byte >> i) & 1);
            crc <<= 1;
            if top == 1 {
                crc ^= 0x04C1_1DB7;
            }
        }
    };
    for pair in data.chunks(2) {
        if pair.len() == 2 {
            feed(pair[1]);
        }
        feed(pair[0]);
    }
    crc.to_le_bytes()
}

/// Table B.7: the constellation point for (state, tribit); the next state
/// is the tribit.
const TRELLIS: [[u8; 8]; 8] = [
    [0, 8, 4, 12, 2, 10, 6, 14],
    [4, 12, 2, 10, 6, 14, 0, 8],
    [1, 9, 5, 13, 3, 11, 7, 15],
    [5, 13, 3, 11, 7, 15, 1, 9],
    [3, 11, 7, 15, 1, 9, 5, 13],
    [7, 15, 1, 9, 5, 13, 3, 11],
    [2, 10, 6, 14, 0, 8, 4, 12],
    [6, 14, 0, 8, 4, 12, 2, 10],
];

/// Table B.8: each point's two dibits, as the symbols (+3 … −3) sent.
const POINTS: [(i8, i8); 16] = [
    (1, -1),
    (-1, -1),
    (3, -3),
    (-3, -3),
    (-3, -1),
    (3, -1),
    (-1, -3),
    (1, -3),
    (-3, 3),
    (3, 3),
    (-1, 1),
    (1, 1),
    (1, 3),
    (-1, 3),
    (3, 1),
    (-3, 1),
];

/// A symbol's dibit, table 10.3.
fn dibit(sym: i8) -> u8 {
    match sym {
        3 => 0b01,
        1 => 0b00,
        -1 => 0b10,
        _ => 0b11,
    }
}

/// Table B.9: sent dibit `i` is encoder dibit `ILV[i]`.
fn interleave() -> [usize; 98] {
    let mut t = [0usize; 98];
    let mut i = 0;
    for g in 0..4 {
        let mut base = 2 * g;
        while base < 98 {
            t[i] = base;
            t[i + 1] = base + 1;
            i += 2;
            base += 8;
        }
    }
    t
}

/// Table B.7's rate ½ sibling (P25's TSBKs and PDUs; MMDVMHost
/// `ENCODE_TABLE_12`): the constellation point for (state, dibit); the next
/// state is the dibit.
const TRELLIS12: [[u8; 4]; 4] = [[0, 15, 12, 3], [4, 11, 8, 7], [13, 2, 1, 14], [9, 6, 5, 10]];

/// Encode `bits` (`per` bits a step: 3 for rate ¾, 2 for rate ½, the flush
/// step included) through the machine `point(state, input)` into the 196
/// bits sent.
fn trellis_encode_with(bits: &[u8], per: usize, point: impl Fn(usize, usize) -> u8) -> [u8; 196] {
    let mut enc = [0u8; 98];
    let mut state = 0usize;
    for k in 0..49 {
        let t = bits[per * k..per * k + per]
            .iter()
            .fold(0usize, |v, &b| (v << 1) | usize::from(b));
        let (a, b) = POINTS[point(state, t) as usize];
        enc[2 * k] = dibit(a);
        enc[2 * k + 1] = dibit(b);
        state = t;
    }
    let ilv = interleave();
    let mut out = [0u8; 196];
    for (i, &e) in ilv.iter().enumerate() {
        out[2 * i] = enc[e] >> 1;
        out[2 * i + 1] = enc[e] & 1;
    }
    out
}

/// Soft Viterbi over a machine of `states` states (the input is the next
/// state): the 49 inputs, and how many steps the path disagreed with the
/// slicer on.
fn trellis_decode_with(
    llr: &[f32; 196],
    states: usize,
    point: impl Fn(usize, usize) -> u8,
) -> ([u8; 49], u32) {
    let ilv = interleave();
    // Encoder-order LLR pairs.
    let mut enc = [[0f32; 2]; 98];
    for (i, &e) in ilv.iter().enumerate() {
        enc[e] = [llr[2 * i], llr[2 * i + 1]];
    }
    // How well a symbol matches a dibit's two LLRs (positive LLR: bit 0).
    let score = |l: [f32; 2], sym: i8| {
        let d = dibit(sym);
        let s = |b: u8, x: f32| if b == 0 { x } else { -x };
        s(d >> 1, l[0]) + s(d & 1, l[1])
    };
    let mut metric = [f32::MIN; 8];
    metric[0] = 0.0;
    let mut from = [[0u8; 8]; 49];
    for k in 0..49 {
        let mut next = [f32::MIN; 8];
        for s in 0..states {
            if metric[s] == f32::MIN {
                continue;
            }
            for t in 0..states {
                let (a, b) = POINTS[point(s, t) as usize];
                let m = metric[s] + score(enc[2 * k], a) + score(enc[2 * k + 1], b);
                if m > next[t] {
                    next[t] = m;
                    from[k][t] = s as u8;
                }
            }
        }
        metric = next;
    }
    // The flush step leaves the machine in state 0.
    let mut inputs = [0u8; 49];
    let mut s = 0usize;
    for k in (0..49).rev() {
        inputs[k] = s as u8;
        s = from[k][s] as usize;
    }
    let mut state = 0usize;
    let mut bad = 0;
    for (k, &t) in inputs.iter().enumerate() {
        let (a, b) = POINTS[point(state, t as usize) as usize];
        if score(enc[2 * k], a) < 0.0 || score(enc[2 * k + 1], b) < 0.0 {
            bad += 1;
        }
        state = t as usize;
    }
    (inputs, bad)
}

/// Rate ¾ trellis encode: 18 bytes → the 196 bits sent.
pub fn trellis_encode(data: &[u8; 18]) -> [u8; 196] {
    let mut bits = Vec::with_capacity(147);
    to_bits(data, &mut bits);
    bits.extend([0, 0, 0]);
    trellis_encode_with(&bits, 3, |s, t| TRELLIS[s][t])
}

/// Rate ¾ trellis decode from the 196 bits' LLRs (sending order): 18 bytes
/// and how many tribits the path disagreed with the slicer on.
pub fn trellis_decode(llr: &[f32; 196]) -> ([u8; 18], u32) {
    let (tribits, bad) = trellis_decode_with(llr, 8, |s, t| TRELLIS[s][t]);
    let mut bits = Vec::with_capacity(144);
    for &t in &tribits[..48] {
        bits.extend([(t >> 2) & 1, (t >> 1) & 1, t & 1]);
    }
    let mut out = [0u8; 18];
    super::bptc::to_bytes(&bits, &mut out);
    (out, bad)
}

/// Rate ½ trellis encode (P25 TSBKs): 12 bytes → the 196 bits sent.
pub fn trellis12_encode(data: &[u8; 12]) -> [u8; 196] {
    let mut bits = Vec::with_capacity(98);
    to_bits(data, &mut bits);
    bits.extend([0, 0]);
    trellis_encode_with(&bits, 2, |s, t| TRELLIS12[s][t])
}

/// Rate ½ trellis decode: 12 bytes and the steps that disagreed.
pub fn trellis12_decode(llr: &[f32; 196]) -> ([u8; 12], u32) {
    let (dibits, bad) = trellis_decode_with(llr, 4, |s, t| TRELLIS12[s][t]);
    let mut bits = Vec::with_capacity(96);
    for &d in &dibits[..48] {
        bits.extend([(d >> 1) & 1, d & 1]);
    }
    let mut out = [0u8; 12];
    super::bptc::to_bytes(&bits, &mut out);
    (out, bad)
}

/// A reassembled packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub header: DataHeader,
    /// The user data, pad and CRC removed.
    pub data: Vec<u8>,
    /// The message CRC-32 checked.
    pub crc_ok: bool,
    /// Confirmed blocks whose CRC-9 failed.
    pub bad_blocks: u32,
}

/// The block rates a data packet may use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rate {
    Half,
    ThreeQuarter,
}

/// Collects a header's blocks on one slot.
#[derive(Clone, Default)]
pub struct Assembler {
    header: Option<DataHeader>,
    data: Vec<u8>,
    left: u8,
    serial: u8,
    bad: u32,
}

impl Assembler {
    pub fn header(&mut self, h: DataHeader) {
        let collect = matches!(h.format, Format::Unconfirmed | Format::Confirmed) && h.blocks > 0;
        self.header = collect.then_some(h);
        self.data.clear();
        self.left = h.blocks;
        self.serial = 0;
        self.bad = 0;
    }

    pub fn abandon(&mut self) {
        self.header = None;
    }

    /// One block's decoded bytes (12 for rate ½, 18 for rate ¾). A packet
    /// comes back when the last block is in.
    pub fn block(&mut self, b: &[u8], rate: Rate) -> Option<Packet> {
        let h = self.header?;
        let user = if h.format == Format::Confirmed {
            // DBSN (7 bits) and CRC-9 (9 bits) lead each confirmed block.
            let serial = b[0] >> 1;
            let got = (u16::from(b[0] & 1) << 8) | u16::from(b[1]);
            let m = match rate {
                Rate::Half => 0x0F0,
                Rate::ThreeQuarter => 0x1FF,
            };
            if crc9(&b[2..], serial) ^ m != got {
                self.bad += 1;
            }
            self.serial = serial;
            &b[2..]
        } else {
            b
        };
        self.data.extend_from_slice(user);
        self.left -= 1;
        if self.left > 0 {
            return None;
        }
        self.header = None;
        let n = self.data.len();
        if n < 4 {
            return None;
        }
        let (body, crc) = self.data.split_at(n - 4);
        let calc = crc32(body);
        let rev = [calc[3], calc[2], calc[1], calc[0]];
        let crc_ok = crc == calc || crc == rev;
        let keep = body.len().saturating_sub(usize::from(h.pad));
        Some(Packet {
            header: h,
            data: body[..keep].to_vec(),
            crc_ok,
            bad_blocks: self.bad,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interleave_is_table_b9() {
        let t = interleave();
        assert_eq!(&t[..4], &[0, 1, 8, 9]);
        assert_eq!(&t[24..28], &[96, 97, 2, 3]);
        assert_eq!(&t[72..76], &[92, 93, 6, 7]);
        assert_eq!(t[97], 95);
    }

    #[test]
    fn trellis_round_trip_through_errors() {
        let data: [u8; 18] = std::array::from_fn(|i| (i * 37 + 11) as u8);
        let tx = trellis_encode(&data);
        let mut llr: [f32; 196] = std::array::from_fn(|i| if tx[i] == 0 { 2.0 } else { -2.0 });
        for i in [3, 50, 51, 120, 190] {
            llr[i] *= -0.5;
        }
        let (out, bad) = trellis_decode(&llr);
        assert_eq!(out, data);
        assert!(bad > 0);
    }

    #[test]
    fn trellis12_round_trip_through_errors() {
        let data: [u8; 12] = std::array::from_fn(|i| (i * 53 + 7) as u8);
        let tx = trellis12_encode(&data);
        let mut llr: [f32; 196] = std::array::from_fn(|i| if tx[i] == 0 { 2.0 } else { -2.0 });
        for i in [4, 30, 31, 90, 150, 191] {
            llr[i] *= -0.5;
        }
        assert_eq!(trellis12_decode(&llr).0, data);
    }

    #[test]
    fn header_round_trip() {
        let h = DataHeader {
            group: false,
            format: Format::Confirmed,
            sap: sap::IP,
            pad: 17,
            dst: 1234,
            src: 2_340_001,
            blocks: 5,
        };
        assert_eq!(DataHeader::parse(&h.to_bytes()), h);
    }
}
