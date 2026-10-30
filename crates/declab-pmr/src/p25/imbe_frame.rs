// made by CasualArclamp
//! The IMBE (7200×4400) voice frame as P25 phase 1 sends it: 144 bits,
//! nine to an LDU, carrying the vocoder's 88 parameter bits.
//!
//! The interleave is DSD's (`p25p1_const.h` iW/iX/iY/iZ: dibit `i` puts its
//! high bit at `fr[IW[i]][IX[i]]`, its low bit at `fr[IY[i]][IZ[i]]`); the
//! FEC is mbelib's (`imbe7200x4400.c`): four Golay (23,12) words, three
//! Hamming (15,11), seven bits bare, and rows 1–6 whitened by the LCG that
//! C0's data seeds. Both ISC licence, © 2010 DSD Author and mbelib Author.
//! Decoded soft: Chase for the Golay and Hamming words.

// The frame is a table of rows and columns, indexed as mbelib indexes it.
#![allow(clippy::needless_range_loop)]

use super::fec::{imbe_hamming_decode, imbe_hamming_encode};
use crate::fec::{Golay23, hard, pack_hard};

const IW: [usize; 72] = [
    0, 2, 4, 1, 3, 5, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3, 6, 0, 2,
    4, 1, 3, 6, 0, 2, 5, 1, 3, 6, 0, 2, 5, 1, 3, 6, 0, 2, 5, 1, 3, 7, 0, 2, 5, 1, 3, 7, 0, 2, 5, 1,
    4, 7, 0, 3, 5, 2, 4, 7,
];
const IX: [usize; 72] = [
    22, 20, 10, 20, 18, 0, 20, 18, 8, 18, 16, 13, 18, 16, 6, 16, 14, 11, 16, 14, 4, 14, 12, 9, 14,
    12, 2, 12, 10, 7, 12, 10, 0, 10, 8, 5, 10, 8, 13, 8, 6, 3, 8, 6, 11, 6, 4, 1, 6, 4, 9, 4, 2, 6,
    4, 2, 7, 2, 0, 4, 2, 0, 5, 0, 13, 2, 0, 21, 3, 21, 11, 0,
];
const IY: [usize; 72] = [
    1, 3, 5, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3, 6, 0, 2, 4, 1, 3,
    6, 0, 2, 5, 1, 3, 6, 0, 2, 5, 1, 3, 6, 0, 2, 5, 1, 3, 6, 0, 2, 5, 1, 3, 7, 0, 2, 5, 1, 4, 7, 0,
    3, 5, 2, 4, 7, 1, 3, 5,
];
const IZ: [usize; 72] = [
    21, 19, 1, 21, 19, 9, 19, 17, 14, 19, 17, 7, 17, 15, 12, 17, 15, 5, 15, 13, 10, 15, 13, 3, 13,
    11, 8, 13, 11, 1, 11, 9, 6, 11, 9, 14, 9, 7, 4, 9, 7, 12, 7, 5, 2, 7, 5, 10, 5, 3, 0, 5, 3, 8,
    3, 1, 5, 3, 1, 6, 1, 14, 3, 1, 22, 4, 22, 12, 1, 22, 20, 2,
];
/// Bits in each row of the frame (`imbe_fr[8][23]`).
const ROW: [usize; 8] = [23, 23, 23, 23, 15, 15, 15, 7];

/// One decoded voice frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImbeFrame {
    /// The vocoder's 88 parameter bits, 0/1, in mbelib's `imbe_d` order.
    pub bits: [u8; 88],
    /// Coded bits the Golay and Hamming words corrected (of 137).
    pub errors: u32,
    /// mbelib's `errs2`: Golay data-bit corrections plus Hamming words
    /// corrected; over 5 the vocoder repeats the last frame.
    pub data_errors: u32,
}

/// The whitening (mbelib `mbe_demodulateImbe7200x4400Data`): 114 bits from
/// the LCG seeded by C0's 12 data bits; bit k−1 masks the k-th of rows 1–6
/// taken from their top bit down.
fn pn(c0_data: u32) -> [u8; 114] {
    let mut x = 16 * c0_data;
    let mut m = [0u8; 114];
    for b in m.iter_mut() {
        x = (173 * x + 13849) % 65536;
        *b = (x >> 15) as u8;
    }
    m
}

/// The (row, column) of each of rows 1–6's bits in PN order.
fn pn_cells() -> impl Iterator<Item = (usize, usize)> {
    (1..7).flat_map(|r| (0..ROW[r]).rev().map(move |c| (r, c)))
}

/// The frame's 144 soft bits (sending order) → the 88 parameter bits.
pub fn decode(llr: &[f32]) -> ImbeFrame {
    let mut fr = [[0f32; 23]; 8];
    for i in 0..72 {
        fr[IW[i]][IX[i]] = llr[2 * i];
        fr[IY[i]][IZ[i]] = llr[2 * i + 1];
    }
    let golay = Golay23::get();
    // A row's word, most significant (column 22) first.
    let word = |row: &[f32; 23]| -> [f32; 23] { std::array::from_fn(|j| row[22 - j]) };
    let w0 = word(&fr[0]);
    let c0 = golay.soft(&w0);
    let h0 = pack_hard(&w0);
    let mut errors = (c0 ^ h0).count_ones();
    let mut data_errors = ((c0 ^ h0) >> 11).count_ones();
    let mask = pn(c0 >> 11);
    for ((r, c), &m) in pn_cells().zip(&mask) {
        if m == 1 {
            fr[r][c] = -fr[r][c];
        }
    }
    let mut bits = [0u8; 88];
    let mut k = 0;
    let mut push = |v: u32, n: usize| {
        for b in (0..n).rev() {
            bits[k] = ((v >> b) & 1) as u8;
            k += 1;
        }
    };
    push(c0 >> 11, 12);
    for row in &fr[1..4] {
        let w = word(row);
        let c = golay.soft(&w);
        let h = pack_hard(&w);
        errors += (c ^ h).count_ones();
        data_errors += ((c ^ h) >> 11).count_ones();
        push(c >> 11, 12);
    }
    for row in &fr[4..7] {
        let w: [f32; 15] = std::array::from_fn(|j| row[14 - j]);
        let c = imbe_hamming_decode(&w);
        let h = pack_hard(&w);
        if c != h {
            errors += (c ^ h).count_ones();
            data_errors += 1;
        }
        push(c >> 4, 11);
    }
    for c in (0..7).rev() {
        bits[81 + 6 - c] = hard(fr[7][c]) as u8;
    }
    ImbeFrame {
        bits,
        errors,
        data_errors,
    }
}

/// The 88 parameter bits → the frame's 144 bits, sending order (for the
/// simulator).
pub fn encode(d: &[u8; 88]) -> [u8; 144] {
    let word = |b: &[u8]| b.iter().fold(0u32, |a, &x| (a << 1) | u32::from(x & 1));
    let mut fr = [[0u8; 23]; 8];
    let set = |fr: &mut [[u8; 23]; 8], r: usize, w: u32, n: usize| {
        for c in 0..n {
            fr[r][c] = ((w >> c) & 1) as u8;
        }
    };
    let c0 = Golay23::encode(word(&d[0..12]));
    set(&mut fr, 0, c0, 23);
    for r in 1..4 {
        set(
            &mut fr,
            r,
            Golay23::encode(word(&d[12 * r..12 * r + 12])),
            23,
        );
    }
    for r in 4..7 {
        let at = 48 + 11 * (r - 4);
        set(&mut fr, r, imbe_hamming_encode(word(&d[at..at + 11])), 15);
    }
    set(&mut fr, 7, word(&d[81..88]), 7);
    let mask = pn(word(&d[0..12]));
    for ((r, c), &m) in pn_cells().zip(&mask) {
        fr[r][c] ^= m;
    }
    let mut out = [0u8; 144];
    for i in 0..72 {
        out[2 * i] = fr[IW[i]][IX[i]];
        out[2 * i + 1] = fr[IY[i]][IZ[i]];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interleave_covers_every_bit_once() {
        let mut seen = [[0u8; 23]; 8];
        for i in 0..72 {
            seen[IW[i]][IX[i]] += 1;
            seen[IY[i]][IZ[i]] += 1;
        }
        for (row, &n) in seen.iter().zip(&ROW) {
            assert!(
                row[..n].iter().all(|&c| c == 1) && row[n..].iter().all(|&c| c == 0),
                "{row:?}"
            );
        }
    }

    #[test]
    fn round_trip_through_errors() {
        let d: [u8; 88] = std::array::from_fn(|i| ((i * 5 + 1) % 3 == 0) as u8);
        let tx = encode(&d);
        let mut llr: Vec<f32> = tx
            .iter()
            .map(|&b| if b == 0 { 3.0 } else { -3.0 })
            .collect();
        // One error in each of the seven coded words (a dibit's high bit
        // that lands in that row).
        for r in 0..7 {
            let i = (0..72).find(|&i| IW[i] == r).expect("every row is reached");
            llr[2 * i] = -llr[2 * i];
        }
        let f = decode(&llr);
        assert_eq!(f.bits, d);
        assert_eq!(f.errors, 7);
        assert_eq!(f.data_errors, 4 + 3);
    }
}
