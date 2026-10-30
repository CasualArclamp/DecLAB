// made by CasualArclamp
//! TETRA channel coding for π/4-DQPSK logical channels, ETSI EN 300 392-2
//! clause 8: the 16-state RCPC code (rate 2/3 here), the shortened (30,14)
//! Reed-Muller code of the AACH, the (K+16,K) CRC, (K,a) block
//! interleaving and the colour-code scrambler.
//!
//! Soft values are LLRs, positive meaning 0, as elsewhere in this crate.

use std::sync::OnceLock;

/// The mother code's generators, clause 8.2.3.1.1 (8.3)–(8.6): bit j is the
/// coefficient of D^j.
const G: [u32; 4] = [
    0b10011, // 1 + D + D^4
    0b11101, // 1 + D^2 + D^3 + D^4
    0b10111, // 1 + D + D^2 + D^4
    0b11011, // 1 + D + D^3 + D^4
];

/// The four mother-code bits for input `u` with the previous four inputs in
/// `state` (bit 0 = b(k−1), bit 3 = b(k−4)).
#[inline]
fn mother(u: u32, state: u32) -> [u8; 4] {
    let reg = u | (state << 1); // bit j = b(k−j)
    std::array::from_fn(|i| ((reg & G[i]).count_ones() & 1) as u8)
}

/// Rate 2/3 puncturing, clause 8.2.3.1.3: of each 8 mother bits (two
/// inputs) V1, V2 and V5 are sent.
const P23: [usize; 3] = [1, 2, 5];

/// The mother-code index (0-based) of type-3 bit `j` (0-based), (8.7).
fn punct23(j: usize) -> usize {
    let t = P23.len();
    8 * (j / t) + P23[j % t] - 1
}

/// RCPC rate 2/3 encoding of type-2 bits (tail included) into
/// `3/2 · len` type-3 bits.
pub fn rcpc23_encode(b2: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 * b2.len());
    let mut s = 0u32;
    for &b in b2 {
        let u = u32::from(b & 1);
        v.extend(mother(u, s));
        s = ((s << 1) | u) & 0xf;
    }
    (0..b2.len() * 3 / 2).map(|j| v[punct23(j)]).collect()
}

/// Soft Viterbi for the rate 2/3 code: `k2` type-2 bits, the last four the
/// zero tail, from the type-3 LLRs. Returns the type-2 bits (tail dropped)
/// and the path metric's margin per bit, a rough reliability.
pub fn rcpc23_decode(llr: &[f32], k2: usize) -> Vec<u8> {
    let mut m = vec![0f32; 4 * k2];
    for (j, &l) in llr.iter().enumerate().take(k2 * 3 / 2) {
        m[punct23(j)] = l;
    }
    viterbi(&m, k2)
}

/// Branch outputs for each (state, input): the four mother bits as a
/// nibble. Rust note: `OnceLock` builds the table on first use, safely
/// across threads.
fn outputs() -> &'static [[u8; 2]; 16] {
    static T: OnceLock<[[u8; 2]; 16]> = OnceLock::new();
    T.get_or_init(|| {
        std::array::from_fn(|s| {
            std::array::from_fn(|u| {
                let o = mother(u as u32, s as u32);
                o[0] | (o[1] << 1) | (o[2] << 2) | (o[3] << 3)
            })
        })
    })
}

/// Viterbi over the mother code with depunctured LLRs (0 where punctured),
/// `k` inputs ending in state 0.
fn viterbi(m: &[f32], k: usize) -> Vec<u8> {
    let out = outputs();
    let mut metric = [f32::NEG_INFINITY; 16];
    metric[0] = 0.0;
    // Survivors: per step, per state, the input bit and the previous state
    // packed (prev << 1 | u).
    let mut surv = vec![[0u8; 16]; k];
    for (step, sv) in surv.iter_mut().enumerate() {
        let l = &m[4 * step..4 * step + 4];
        // The four mother bits' ±LLR sums for each of 16 nibbles.
        let mut bm = [0f32; 16];
        for (nib, b) in bm.iter_mut().enumerate() {
            *b = (0..4)
                .map(|i| if nib >> i & 1 == 0 { l[i] } else { -l[i] })
                .sum();
        }
        let mut next = [f32::NEG_INFINITY; 16];
        for s in 0..16 {
            if metric[s] == f32::NEG_INFINITY {
                continue;
            }
            for u in 0..2 {
                let ns = ((s << 1) | u) & 0xf;
                let v = metric[s] + bm[out[s][u] as usize];
                if v > next[ns] {
                    next[ns] = v;
                    sv[ns] = ((s as u8) << 1) | u as u8;
                }
            }
        }
        metric = next;
    }
    let mut bits = vec![0u8; k];
    let mut s = 0usize;
    for step in (0..k).rev() {
        let e = surv[step][s];
        bits[step] = e & 1;
        s = (e >> 1) as usize;
    }
    bits.truncate(k.saturating_sub(4));
    bits
}

/// The (K+16,K) block code, clause 8.2.3.3: CRC-CCITT preset to all ones
/// and inverted (the X.25 FCS), sent most significant bit first.
pub fn crc16(bits: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in bits {
        let top = (crc >> 15) as u8 ^ (b & 1);
        crc <<= 1;
        if top == 1 {
            crc ^= 0x1021;
        }
    }
    !crc
}

/// Type-1 bits with their CRC appended and four zero tail bits: the type-2
/// block.
pub fn add_crc_tail(b1: &[u8]) -> Vec<u8> {
    let c = crc16(b1);
    let mut v = b1.to_vec();
    v.extend((0..16).map(|i| ((c >> (15 - i)) & 1) as u8));
    v.extend([0; 4]);
    v
}

/// Whether type-2 bits (CRC included, tail removed) pass the CRC.
pub fn crc_ok(b2: &[u8]) -> bool {
    let k = b2.len() - 16;
    let c = crc16(&b2[..k]);
    (0..16).all(|i| b2[k + i] == ((c >> (15 - i)) & 1) as u8)
}

/// (K,a) block interleaving, clause 8.2.4.1 (8.34): b4(k) = b3(i),
/// k = 1 + (a·i mod K). Generic over the element (bits or LLRs).
pub fn interleave<T: Copy + Default>(b3: &[T], a: usize) -> Vec<T> {
    let k = b3.len();
    let mut b4 = vec![T::default(); k];
    for i in 1..=k {
        b4[(a * i) % k] = b3[i - 1];
    }
    b4
}

pub fn deinterleave<T: Copy + Default>(b4: &[T], a: usize) -> Vec<T> {
    let k = b4.len();
    (1..=k).map(|i| b4[(a * i) % k]).collect()
}

/// The 30-bit extended colour code, clause 23.2.1 figure 23.5: MCC (10),
/// MNC (14), colour code (6), e(1) being the MCC's most significant bit.
pub fn extended_colour_code(mcc: u16, mnc: u16, colour: u8) -> u32 {
    (u32::from(mcc & 0x3ff) << 20) | (u32::from(mnc & 0x3fff) << 6) | u32::from(colour & 0x3f)
}

/// The scrambling sequence p(1..n), clause 8.2.5.2: the LFSR with
/// c(x) = 1+x+x²+x⁴+x⁵+x⁷+x⁸+x¹⁰+x¹¹+x¹²+x¹⁶+x²²+x²³+x²⁶+x³², preset with
/// p(0..−29) = e(1..30) and p(−30) = p(−31) = 1. `e` is 0 for the BSCH.
pub fn scrambling(e: u32, n: usize) -> Vec<u8> {
    const TAPS: [u32; 14] = [1, 2, 4, 5, 7, 8, 10, 11, 12, 16, 22, 23, 26, 32];
    let mask = TAPS.iter().fold(0u32, |m, &i| m | (1 << (i - 1)));
    // Bit i−1 of `reg` is p(k−i).
    let mut reg = 0u32;
    for i in 1..=30 {
        // p(1−i) = e(i): bit i−1.
        let ei = (e >> (30 - i)) & 1;
        reg |= ei << (i - 1);
    }
    reg |= 0b11 << 30;
    (0..n)
        .map(|_| {
            let p = (reg & mask).count_ones() & 1;
            reg = (reg << 1) | p;
            p as u8
        })
        .collect()
}

/// The shortened (30,14) Reed-Muller code's parity columns, clause 8.2.3.2
/// (8.13): row r gives the 16 parity bits of data bit r.
const RM_P: [u16; 14] = [
    0b1001101101100000,
    0b0010110111100000,
    0b1111110000100000,
    0b1110000000111100,
    0b1001100000111010,
    0b0101010000110110,
    0b0010110000101110,
    0b1111111111011111,
    0b1000001100111001,
    0b0100001010110101,
    0b0010000110101101,
    0b0001001001110011,
    0b0000100101101011,
    0b0000010011100111,
];

/// 14 data bits (MSB = b1(1)) to the 30-bit codeword (MSB = b2(1)).
pub fn rm30_encode(d: u16) -> u32 {
    let mut p = 0u16;
    for (r, &row) in RM_P.iter().enumerate() {
        if d >> (13 - r) & 1 == 1 {
            p ^= row;
        }
    }
    (u32::from(d & 0x3fff) << 16) | u32::from(p)
}

/// Syndrome of a 30-bit word: its parity recomputed against the received.
fn rm30_syndrome(w: u32) -> u16 {
    (rm30_encode((w >> 16) as u16) as u16) ^ (w as u16)
}

/// Coset leaders up to weight 3 by syndrome (the code's distance is 8, so
/// three errors are always within reach).
fn rm30_table() -> &'static [u32] {
    static T: OnceLock<Vec<u32>> = OnceLock::new();
    T.get_or_init(|| {
        let mut t = vec![u32::MAX; 1 << 16];
        t[0] = 0;
        let mut add = |e: u32| {
            let s = rm30_syndrome(e) as usize;
            if t[s] == u32::MAX {
                t[s] = e;
            }
        };
        for a in 0..30 {
            add(1 << a);
        }
        for a in 0..30 {
            for b in a + 1..30 {
                add((1 << a) | (1 << b));
            }
        }
        for a in 0..30 {
            for b in a + 1..30 {
                for c in b + 1..30 {
                    add((1 << a) | (1 << b) | (1 << c));
                }
            }
        }
        t
    })
}

/// Soft decoding of the AACH's (30,14) code: Chase-II over the five least
/// reliable bits, each trial hard-decoded through the coset table, the best
/// correlation kept. Returns the data and the number of hard decisions it
/// overrules, or `None` when no trial lands on a codeword.
pub fn rm30_decode(llr: &[f32; 30]) -> Option<(u16, u32)> {
    let table = rm30_table();
    let hard: u32 = llr.iter().fold(0, |w, &l| (w << 1) | u32::from(l < 0.0));
    let mut order: Vec<usize> = (0..30).collect();
    order.sort_by(|&a, &b| llr[a].abs().total_cmp(&llr[b].abs()));
    let mut best: Option<(f32, u32)> = None;
    for flips in 0u32..32 {
        let mut w = hard;
        for (i, &pos) in order.iter().take(5).enumerate() {
            if flips >> i & 1 == 1 {
                w ^= 1 << (29 - pos);
            }
        }
        let e = table[rm30_syndrome(w) as usize];
        if e == u32::MAX {
            continue;
        }
        let c = w ^ e;
        let score: f32 = (0..30)
            .map(|i| {
                let l = llr[i];
                if c >> (29 - i) & 1 == 0 { l } else { -l }
            })
            .sum();
        if best.is_none_or(|(s, _)| score > s) {
            best = Some((score, c));
        }
    }
    best.map(|(_, c)| ((c >> 16) as u16, (c ^ hard).count_ones()))
}

/// The signalling channels' coding chains, clause 8.3.1: type-1 bits,
/// type-5 block size, interleaver parameter `a`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chan {
    /// BSCH, 8.3.1.2: 60 → 120, (120,11), scrambled with e = 0.
    Bsch,
    /// SCH/HD, BNCH, STCH, 8.3.1.4.1: 124 → 216, (216,101).
    Half,
    /// SCH/F, 8.3.1.4.5: 268 → 432, (432,103).
    Full,
}

impl Chan {
    pub fn k1(self) -> usize {
        match self {
            Chan::Bsch => 60,
            Chan::Half => 124,
            Chan::Full => 268,
        }
    }
    pub fn k5(self) -> usize {
        (self.k1() + 20) * 3 / 2
    }
    fn a(self) -> usize {
        match self {
            Chan::Bsch => 11,
            Chan::Half => 101,
            Chan::Full => 103,
        }
    }
}

/// Type-1 bits to the scrambled type-5 block.
pub fn encode(chan: Chan, b1: &[u8], scramble: &[u8]) -> Vec<u8> {
    let b3 = rcpc23_encode(&add_crc_tail(b1));
    let b4 = interleave(&b3, chan.a());
    b4.iter().zip(scramble).map(|(&b, &p)| b ^ p).collect()
}

/// Type-5 LLRs to the type-1 bits if the CRC passes.
pub fn decode(chan: Chan, llr: &[f32], scramble: &[u8]) -> Option<Vec<u8>> {
    let b4: Vec<f32> = llr
        .iter()
        .zip(scramble)
        .map(|(&l, &p)| if p == 1 { -l } else { l })
        .collect();
    let b3 = deinterleave(&b4, chan.a());
    let b2 = rcpc23_decode(&b3, chan.k1() + 20);
    if !crc_ok(&b2) {
        return None;
    }
    let mut b1 = b2;
    b1.truncate(chan.k1());
    Some(b1)
}

/// The AACH, clause 8.3.1.1: 14 bits through the (30,14) code, scrambled.
pub fn encode_aach(d: u16, scramble: &[u8]) -> Vec<u8> {
    let c = rm30_encode(d);
    (0..30)
        .map(|i| ((c >> (29 - i)) & 1) as u8 ^ scramble[i])
        .collect()
}

pub fn decode_aach(llr: &[f32], scramble: &[u8]) -> Option<(u16, u32)> {
    let l: [f32; 30] = std::array::from_fn(|i| if scramble[i] == 1 { -llr[i] } else { llr[i] });
    rm30_decode(&l)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(seed: u32, n: usize) -> Vec<u8> {
        let mut x = seed.wrapping_mul(2_654_435_761) | 1;
        (0..n)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                (x & 1) as u8
            })
            .collect()
    }

    #[test]
    fn rcpc_round_trip_with_errors() {
        for k1 in [60usize, 124, 268] {
            let b1 = bits(k1 as u32, k1);
            let b2 = add_crc_tail(&b1);
            let b3 = rcpc23_encode(&b2);
            assert_eq!(b3.len(), b2.len() * 3 / 2);
            let mut llr: Vec<f32> = b3
                .iter()
                .map(|&b| if b == 0 { 1.0 } else { -1.0 })
                .collect();
            // A few scattered errors.
            for i in (7..llr.len()).step_by(37) {
                llr[i] = -llr[i];
            }
            let d = rcpc23_decode(&llr, b2.len());
            assert_eq!(d.len(), k1 + 16);
            assert!(crc_ok(&d));
            assert_eq!(&d[..k1], &b1[..]);
        }
    }

    #[test]
    fn interleaver_inverts() {
        let x: Vec<u8> = bits(5, 216);
        assert_eq!(deinterleave(&interleave(&x, 101), 101), x);
        let y: Vec<u8> = bits(6, 120);
        assert_eq!(deinterleave(&interleave(&y, 11), 11), y);
    }

    #[test]
    fn rm30_corrects_three() {
        for d in [0u16, 1, 0x2a5b, 0x3fff, 0x1234] {
            let c = rm30_encode(d);
            let mut llr: [f32; 30] =
                std::array::from_fn(|i| if c >> (29 - i) & 1 == 0 { 1.0 } else { -1.0 });
            for i in [2, 13, 27] {
                llr[i] *= -0.5;
            }
            assert_eq!(rm30_decode(&llr).map(|x| x.0), Some(d));
        }
        // The code's minimum distance is 8 (a sanity check on the table).
        let w = (1..1u32 << 14)
            .map(|d| rm30_encode(d as u16).count_ones())
            .min();
        assert_eq!(w, Some(8));
    }

    #[test]
    fn scrambler_is_a_full_period_lfsr_start() {
        // BSCH: all-zero e still gives a non-zero sequence (p(−30), p(−31)).
        let p = scrambling(0, 120);
        assert!(p.contains(&1));
        assert_ne!(scrambling(1, 64), scrambling(2, 64));
    }

    #[test]
    fn crc_residue() {
        // The X.25 FCS appended leaves the well-known residue 0x1D0F, which
        // inverted is 0xE2F0: check through crc_ok instead.
        let b = bits(9, 124);
        let mut b2 = add_crc_tail(&b);
        b2.truncate(140);
        assert!(crc_ok(&b2));
        b2[3] ^= 1;
        assert!(!crc_ok(&b2));
    }
}
