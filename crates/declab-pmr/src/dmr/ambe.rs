// made by CasualArclamp
//! The AMBE+2 (3600×2450) voice frame as DMR sends it: 72 bits, three to a
//! voice burst, carrying the vocoder's 49 parameter bits.
//!
//! The FEC is DVSI's, not ETSI's: C0 is Golay (24,12) on parameter bits
//! 0–11; C1 is Golay (23,12) on bits 12–23, whitened by a pseudo-random
//! sequence seeded from C0's data; C2 (11 bits) and C3 (14 bits) are sent
//! bare. The interleave and the bit orders are DSD's and mbelib's
//! (`dmr_const.h`, `ecc.c`, `ambe3600x2450.c`; ISC licence, © 2010 DSD
//! Author and © 2010 mbelib Author: "Permission to use, copy, modify,
//! and/or distribute this software for any purpose with or without fee is
//! hereby granted, provided that the above copyright notice and this
//! permission notice appear in all copies.").
//!
//! C0 and C1 are decoded soft (Chase over the four weakest bits). Their
//! corrections, out of 46 coded bits, estimate the channel bit error rate,
//! as DSD's voice error count does.

use crate::fec::{Golay23, hard};

/// Dibit `i` of a frame: its high bit goes to `fr[RW[i]][RX[i]]`, its low
/// bit to `fr[RY[i]][RZ[i]]` (DSD `dmr_const.h`).
const RW: [usize; 36] = [
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2,
    0, 2, 0, 2,
];
const RX: [usize; 36] = [
    23, 10, 22, 9, 21, 8, 20, 7, 19, 6, 18, 5, 17, 4, 16, 3, 15, 2, 14, 1, 13, 0, 12, 10, 11, 9,
    10, 8, 9, 7, 8, 6, 7, 5, 6, 4,
];
const RY: [usize; 36] = [
    0, 2, 0, 2, 0, 2, 0, 2, 0, 3, 0, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3,
    1, 3, 1, 3,
];
const RZ: [usize; 36] = [
    5, 3, 4, 2, 3, 1, 2, 0, 1, 13, 0, 12, 22, 11, 21, 10, 20, 9, 19, 8, 18, 7, 17, 6, 16, 5, 15, 4,
    14, 3, 13, 2, 12, 1, 11, 0,
];

/// One decoded voice frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AmbeFrame {
    /// The vocoder's 49 parameter bits, 0/1, in mbelib's `ambe_d` order.
    pub bits: [u8; 49],
    /// Bits C0 and C1's Golay codes corrected (of 46).
    pub errors: u32,
}

/// The C1 whitening (mbelib `mbe_demodulateAmbe3600x2450Data`): 23 bits
/// from a linear congruential generator seeded by C0's 12 data bits, the
/// mask for `fr[1][22]` first.
fn pn(c0_data: u32) -> [u8; 23] {
    let mut x = 16 * c0_data;
    let mut m = [0u8; 23];
    for b in m.iter_mut() {
        x = (173 * x + 13849) % 65536;
        *b = (x >> 15) as u8;
    }
    m
}

/// The frame's 72 soft bits (sending order) → the 49 parameter bits.
pub fn decode(llr: &[f32; 72]) -> AmbeFrame {
    let mut fr = [[0f32; 24]; 4];
    for i in 0..36 {
        fr[RW[i]][RX[i]] = llr[2 * i];
        fr[RY[i]][RZ[i]] = llr[2 * i + 1];
    }
    let golay = Golay23::get();
    // Golay word bit b (bit 22 the data MSB) is fr[0][b + 1] for C0 and
    // fr[1][b] for C1; the decoder takes the word most significant first.
    let mut w = [0f32; 23];
    for (j, x) in w.iter_mut().enumerate() {
        *x = fr[0][23 - j];
    }
    let h0 = w.iter().fold(0u32, |a, &l| (a << 1) | hard(l));
    let c0 = golay.soft(&w);
    let d0 = c0 >> 11;
    let mask = pn(d0);
    for (j, x) in w.iter_mut().enumerate() {
        // fr[1][22 - j] carries mask bit j.
        let l = fr[1][22 - j];
        *x = if mask[j] == 1 { -l } else { l };
    }
    let h1 = w.iter().fold(0u32, |a, &l| (a << 1) | hard(l));
    let c1 = golay.soft(&w);
    let d1 = c1 >> 11;
    let mut bits = [0u8; 49];
    for i in 0..12 {
        bits[i] = ((d0 >> (11 - i)) & 1) as u8;
        bits[12 + i] = ((d1 >> (11 - i)) & 1) as u8;
    }
    for i in 0..11 {
        bits[24 + i] = hard(fr[2][10 - i]) as u8;
    }
    for i in 0..14 {
        bits[35 + i] = hard(fr[3][13 - i]) as u8;
    }
    AmbeFrame {
        bits,
        errors: (c0 ^ h0).count_ones() + (c1 ^ h1).count_ones(),
    }
}

/// The 49 parameter bits → the frame's 72 bits, sending order (for the
/// modulator).
pub fn encode(d: &[u8; 49]) -> [u8; 72] {
    let word = |b: &[u8]| b.iter().fold(0u32, |a, &x| (a << 1) | u32::from(x & 1));
    let mut fr = [[0u8; 24]; 4];
    let c0 = Golay23::encode(word(&d[0..12]));
    for b in 0..23 {
        fr[0][b + 1] = ((c0 >> b) & 1) as u8;
    }
    // The (24,12) extension: even parity over the 23.
    fr[0][0] = (c0.count_ones() & 1) as u8;
    let c1 = Golay23::encode(word(&d[12..24]));
    let mask = pn(word(&d[0..12]));
    for b in 0..23 {
        fr[1][b] = ((c1 >> b) & 1) as u8 ^ mask[22 - b];
    }
    for i in 0..11 {
        fr[2][10 - i] = d[24 + i];
    }
    for i in 0..14 {
        fr[3][13 - i] = d[35 + i];
    }
    let mut out = [0u8; 72];
    for i in 0..36 {
        out[2 * i] = fr[RW[i]][RX[i]];
        out[2 * i + 1] = fr[RY[i]][RZ[i]];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interleave_covers_every_bit_once() {
        let mut seen = [[0u8; 24]; 4];
        for i in 0..36 {
            seen[RW[i]][RX[i]] += 1;
            seen[RY[i]][RZ[i]] += 1;
        }
        let lens = [24, 23, 11, 14];
        for (row, &n) in seen.iter().zip(&lens) {
            assert!(row[..n].iter().all(|&c| c == 1), "{row:?}");
            assert!(row[n..].iter().all(|&c| c == 0));
        }
    }

    #[test]
    fn round_trip_through_errors() {
        let mut d = [0u8; 49];
        for (i, b) in d.iter_mut().enumerate() {
            *b = ((i * 7 + 3) % 5 == 0) as u8;
        }
        let tx = encode(&d);
        let mut llr: [f32; 72] = std::array::from_fn(|i| if tx[i] == 0 { 3.0 } else { -3.0 });
        // Errors that land in C0 and C1 (dibits 0 and 12 carry their bits).
        llr[0] = -llr[0];
        llr[2] = -llr[2];
        llr[25] = -llr[25];
        let f = decode(&llr);
        assert_eq!(f.bits, d);
        assert_eq!(f.errors, 3);
    }
}
