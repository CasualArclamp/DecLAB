// made by CasualArclamp
//! DMR's block product turbo codes, ETSI TS 102 361-1 annex B:
//! BPTC (196,96) for full bursts, the 128-bit variable BPTC of embedded
//! signalling (over voice bursts B–E) and the 68-bit one of the CACH's Short
//! LC (over four CACH bursts).
//!
//! The (196,96) rows are decoded soft (Chase), then rows and columns are
//! iterated hard; the short ones decode their Hamming rows soft and use the
//! column parity and the checksum or CRC to accept.

use crate::fec::{self, BlockCode};

/// Bits 0/1 to bytes, most significant first.
pub fn to_bytes(bits: &[u8], out: &mut [u8]) {
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = bits[i * 8..i * 8 + 8]
            .iter()
            .fold(0, |b, &x| (b << 1) | (x & 1));
    }
}

/// Bytes to 0/1 bits, most significant first.
pub fn to_bits(bytes: &[u8], out: &mut Vec<u8>) {
    for &b in bytes {
        out.extend((0..8).rev().map(|i| (b >> i) & 1));
    }
}

/// Where matrix index `i` (figure B.1 numbering, R(3) at 0) is sent: (B.1).
#[inline]
fn interleave(i: usize) -> usize {
    (i * 181) % 196
}

/// Matrix positions of the 96 information bits, I(95) first (B.1): row 0
/// columns 3–10, rows 1–8 columns 0–10.
fn info_positions() -> impl Iterator<Item = usize> {
    (0..9).flat_map(|r| {
        let first = if r == 0 { 3 } else { 0 };
        (first..11).map(move |c| 1 + 15 * r + c)
    })
}

/// Crossings tried by [`crossing`] (the least reliable), and the most flips.
const CROSSING_CANDIDATES: usize = 24;
const CROSSING_FLIPS: usize = 4;

/// Plain hard decoding from the channel's decisions: rows, then columns,
/// single errors each, a few rounds. The soft pass can lose a pattern this
/// clears at once: an error the channel was sure of (a glitch) makes Chase
/// prefer flipping weaker bits instead.
fn hard(first: &[u32; 13]) -> Option<[u32; 13]> {
    let (row, col) = (fec::hamming_15_11(), fec::hamming_13_9());
    let col_word =
        |w: &[u32; 13], c: usize| (0..13).fold(0u32, |x, r| (x << 1) | ((w[r] >> (14 - c)) & 1));
    let mut w = *first;
    for _ in 0..4 {
        for word in w.iter_mut().take(9) {
            if let Some(fixed) = row.correct1(*word) {
                *word = fixed;
            }
        }
        for c in 0..15 {
            let cw = col_word(&w, c);
            if let Some(fixed) = col.correct1(cw) {
                for (r, word) in w.iter_mut().enumerate() {
                    let bit = (fixed >> (12 - r)) & 1;
                    *word = (*word & !(1 << (14 - c))) | (bit << (14 - c));
                }
            }
        }
        if w[..9].iter().all(|&x| row.syndrome(x) == 0)
            && (0..15).all(|c| col.syndrome(col_word(&w, c)) == 0)
        {
            // Accepted only within the code's reach (distance 9).
            let flips: u32 = w.iter().zip(first).map(|(a, b)| (a ^ b).count_ones()).sum();
            return (flips <= 4).then_some(w);
        }
    }
    None
}

/// The (196,96) product code's fallback when the iterations fail: an error
/// sits where a failing row crosses a failing column (rows 9–12 carry only
/// column parity, so they are always candidates). The least reliable
/// crossings are flipped, fewest first, and the cheapest set (sum of |LLR|)
/// that clears every row and column wins. The code's distance is 9, so up
/// to 4 errors have one answer, and no more are tried. Seen on a real MOTOTRBO repeater (2026-10-30): idle and data
/// bursts with 3–5 clustered bit errors the iterations left failing, one
/// in ten at 22 dB, and with them LRRP blocks.
fn crossing(first: &[u32; 13], soft: &[f32; 196]) -> Option<[u32; 13]> {
    let (row, col) = (fec::hamming_15_11(), fec::hamming_13_9());
    let col_word =
        |w: &[u32; 13], c: usize| (0..13).fold(0u32, |x, r| (x << 1) | ((w[r] >> (14 - c)) & 1));
    let ok = |w: &[u32; 13]| {
        w[..9].iter().all(|&x| row.syndrome(x) == 0)
            && (0..15).all(|c| col.syndrome(col_word(w, c)) == 0)
    };
    let rows: Vec<usize> = (0..9)
        .filter(|&r| row.syndrome(first[r]) != 0)
        .chain(9..13)
        .collect();
    let cols: Vec<usize> = (0..15)
        .filter(|&c| col.syndrome(col_word(first, c)) != 0)
        .collect();
    // (reliability, row, column), least reliable first.
    let mut cand: Vec<(f32, usize, usize)> = rows
        .iter()
        .flat_map(|&r| {
            cols.iter()
                .map(move |&c| (soft[1 + 15 * r + c].abs(), r, c))
        })
        .collect();
    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
    cand.truncate(CROSSING_CANDIDATES);
    let n = cand.len();
    let mut best: Option<(f32, [u32; 13])> = None;
    for k in 1..=CROSSING_FLIPS.min(n) {
        // Rust note: `idx` walks every k-subset of 0..n in order
        // (0,1,…,k−1), (0,1,…,k), … like nested loops of any depth.
        let mut idx: Vec<usize> = (0..k).collect();
        loop {
            let mut w = *first;
            let mut cost = 0.0;
            for &i in &idx {
                let (rel, r, c) = cand[i];
                w[r] ^= 1 << (14 - c);
                cost += rel;
            }
            if ok(&w) && best.as_ref().is_none_or(|b| cost < b.0) {
                best = Some((cost, w));
            }
            // The next subset: the rightmost index that can move, then the
            // ones after it packed behind.
            let Some(j) = (0..k).rev().find(|&j| idx[j] < n - k + j) else {
                break;
            };
            idx[j] += 1;
            for m in j + 1..k {
                idx[m] = idx[m - 1] + 1;
            }
        }
        if best.is_some() {
            break;
        }
    }
    best.map(|b| b.1)
}

/// Encode 12 bytes (96 bits) into the 196 bits sent, in sending order.
pub fn encode_196(data: &[u8; 12]) -> [u8; 196] {
    let mut bits = Vec::with_capacity(96);
    to_bits(data, &mut bits);
    let mut m = [0u8; 196];
    for (p, &b) in info_positions().zip(&bits) {
        m[p] = b;
    }
    let (row, col) = (fec::hamming_15_11(), fec::hamming_13_9());
    for r in 0..9 {
        let d = fec::pack(&m[1 + 15 * r..1 + 15 * r + 11]) as u32;
        let w = row.encode(d);
        for c in 0..15 {
            m[1 + 15 * r + c] = ((w >> (14 - c)) & 1) as u8;
        }
    }
    for c in 0..15 {
        let d = (0..9).fold(0u32, |w, r| (w << 1) | u32::from(m[1 + 15 * r + c]));
        let w = col.encode(d);
        for r in 0..13 {
            m[1 + 15 * r + c] = ((w >> (12 - r)) & 1) as u8;
        }
    }
    let mut tx = [0u8; 196];
    for (i, &b) in m.iter().enumerate() {
        tx[interleave(i)] = b;
    }
    tx
}

/// Decode the 196 bits as received (LLRs, sending order): the 12 data bytes
/// and the bits the code corrected, or `None` if a row or column still fails.
pub fn decode_196(llr: &[f32; 196]) -> Option<([u8; 12], u32)> {
    let (row, col) = (fec::hamming_15_11(), fec::hamming_13_9());
    let mut soft = [0f32; 196];
    for (i, s) in soft.iter_mut().enumerate() {
        *s = llr[interleave(i)];
    }
    // 13 rows of 15 bits as words.
    let mut w = [0u32; 13];
    for (r, word) in w.iter_mut().enumerate() {
        *word = fec::pack_hard(&soft[1 + 15 * r..1 + 15 * r + 15]);
    }
    let first = w;
    // Soft iterations, a simplified Pyndiah turbo product decoder: rows and
    // columns take turns, each Chase-decoded on the channel LLRs plus a
    // nudge (α × the mean reliability) towards what the other direction
    // decided. The hard row/column clean-up below then finishes the job.
    let mean = soft.iter().map(|x| x.abs()).sum::<f32>() / 196.0;
    let mut ext = [0f32; 195];
    let mut input = [0f32; 15];
    for alpha in [0.4f32, 0.6, 0.8] {
        let mut row_ext = [0f32; 195];
        for (r, word) in w.iter_mut().enumerate().take(9) {
            for (c, x) in input.iter_mut().enumerate() {
                *x = soft[1 + 15 * r + c] + ext[15 * r + c];
            }
            if let Some(d) = row.chase(&input, 3) {
                *word = d;
                for c in 0..15 {
                    row_ext[15 * r + c] = if (d >> (14 - c)) & 1 == 0 {
                        alpha * mean
                    } else {
                        -alpha * mean
                    };
                }
            }
        }
        let mut col_in = [0f32; 13];
        for c in 0..15 {
            for (r, x) in col_in.iter_mut().enumerate() {
                *x = soft[1 + 15 * r + c] + row_ext[15 * r + c];
            }
            if let Some(d) = col.chase(&col_in, 3) {
                for r in 0..13 {
                    let bit = (d >> (12 - r)) & 1;
                    w[r] = (w[r] & !(1 << (14 - c))) | (bit << (14 - c));
                    ext[15 * r + c] = if bit == 0 {
                        alpha * mean
                    } else {
                        -alpha * mean
                    };
                }
            }
        }
    }
    let col_word =
        |w: &[u32; 13], c: usize| (0..13).fold(0u32, |x, r| (x << 1) | ((w[r] >> (14 - c)) & 1));
    for _ in 0..3 {
        let mut changed = false;
        for c in 0..15 {
            let cw = col_word(&w, c);
            if let Some(fixed) = col.correct1(cw)
                && fixed != cw
            {
                for (r, word) in w.iter_mut().enumerate() {
                    let bit = (fixed >> (12 - r)) & 1;
                    *word = (*word & !(1 << (14 - c))) | (bit << (14 - c));
                }
                changed = true;
            }
        }
        for word in w.iter_mut().take(9) {
            if let Some(fixed) = row.correct1(*word)
                && fixed != *word
            {
                *word = fixed;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    let rows_ok = w[..9].iter().all(|&x| row.syndrome(x) == 0);
    let cols_ok = (0..15).all(|c| col.syndrome(col_word(&w, c)) == 0);
    if !(rows_ok && cols_ok) {
        w = hard(&first).or_else(|| crossing(&first, &soft))?;
    }
    let fixed: u32 = w
        .iter()
        .zip(&first)
        .map(|(a, b)| (a ^ b).count_ones())
        .sum();
    let mut bits = [0u8; 96];
    for (b, p) in bits.iter_mut().zip(info_positions()) {
        let (r, c) = ((p - 1) / 15, (p - 1) % 15);
        *b = ((w[r] >> (14 - c)) & 1) as u8;
    }
    let mut out = [0u8; 12];
    to_bytes(&bits, &mut out);
    Some((out, fixed))
}

/// The encode matrix of a variable BPTC read out column by column (top to
/// bottom, left to right), as sent: matrix (row, col) is sent at
/// `col * rows + row`.
#[inline]
fn column_index(row: usize, col: usize, rows: usize) -> usize {
    col * rows + row
}

/// Embedded LC (figure B.3): the 9 LC bytes into 128 bits, the 32 sent in
/// each of voice bursts B, C, D and E in turn.
// Matrix code reads best with (row, column) indices.
#[allow(clippy::needless_range_loop)]
pub fn encode_embedded(lc: &[u8; 9]) -> [u8; 128] {
    let code = fec::hamming_16_11();
    let mut bits = Vec::with_capacity(72);
    to_bits(lc, &mut bits);
    let cs = fec::cs5(lc);
    let mut m = [[0u8; 16]; 8];
    let mut k = 0;
    for (r, row) in m.iter_mut().enumerate().take(7) {
        let n = if r < 2 { 11 } else { 10 };
        row[..n].copy_from_slice(&bits[k..k + n]);
        k += n;
        if r >= 2 {
            row[10] = (cs >> (6 - r)) & 1;
        }
        let w = code.encode(fec::pack(&row[..11]) as u32);
        for (c, b) in row.iter_mut().enumerate() {
            *b = ((w >> (15 - c)) & 1) as u8;
        }
    }
    for c in 0..16 {
        m[7][c] = (0..7).fold(0, |p, r| p ^ m[r][c]);
    }
    let mut tx = [0u8; 128];
    for (r, row) in m.iter().enumerate() {
        for (c, &b) in row.iter().enumerate() {
            tx[column_index(r, c, 8)] = b;
        }
    }
    tx
}

/// Decode an embedded LC from its 128 bits (LLRs, sending order): the LC
/// bytes if the rows, column parity and checksum all agree.
pub fn decode_embedded(llr: &[f32; 128]) -> Option<[u8; 9]> {
    let code = fec::hamming_16_11();
    let mut rows = [0u32; 8];
    let mut row_llr = [0f32; 16];
    for (r, word) in rows.iter_mut().enumerate() {
        for (c, l) in row_llr.iter_mut().enumerate() {
            *l = llr[column_index(r, c, 8)];
        }
        *word = if r < 7 {
            code.chase(&row_llr, 3)?
        } else {
            fec::pack_hard(&row_llr)
        };
    }
    if rows.iter().fold(0, |p, &w| p ^ w) != 0 {
        return None;
    }
    let mut bits = Vec::with_capacity(72);
    let mut cs = 0u8;
    for (r, &w) in rows.iter().enumerate().take(7) {
        let n = if r < 2 { 11 } else { 10 };
        bits.extend((0..n).map(|c| ((w >> (15 - c)) & 1) as u8));
        if r >= 2 {
            cs = (cs << 1) | ((w >> 5) & 1) as u8;
        }
    }
    let mut lc = [0u8; 9];
    to_bytes(&bits, &mut lc);
    (fec::cs5(&lc) == cs).then_some(lc)
}

/// Short LC in the CACH (figure B.6): 28 bits (SLCO and data) into 68, the
/// 17 sent in each of four CACH bursts.
// Matrix code reads best with (row, column) indices.
#[allow(clippy::needless_range_loop)]
pub fn encode_short_lc(slc: u32) -> [u8; 68] {
    let code = fec::hamming_17_12();
    let mut bits = Vec::with_capacity(36);
    fec::unpack(u64::from(slc & 0x0fff_ffff), 28, &mut bits);
    let crc = fec::crc8(&bits);
    fec::unpack(u64::from(crc), 8, &mut bits);
    let mut m = [[0u8; 17]; 4];
    for r in 0..3 {
        let w = code.encode(fec::pack(&bits[12 * r..12 * r + 12]) as u32);
        for c in 0..17 {
            m[r][c] = ((w >> (16 - c)) & 1) as u8;
        }
    }
    for c in 0..17 {
        m[3][c] = m[0][c] ^ m[1][c] ^ m[2][c];
    }
    let mut tx = [0u8; 68];
    for (r, row) in m.iter().enumerate() {
        for (c, &b) in row.iter().enumerate() {
            tx[column_index(r, c, 4)] = b;
        }
    }
    tx
}

/// Decode a Short LC from the 68 bits of four CACH bursts (LLRs): its 28
/// bits if the rows, column parity and CRC agree.
pub fn decode_short_lc(llr: &[f32; 68]) -> Option<u32> {
    let code: &BlockCode = fec::hamming_17_12();
    let mut rows = [0u32; 4];
    let mut row_llr = [0f32; 17];
    for (r, word) in rows.iter_mut().enumerate() {
        for (c, l) in row_llr.iter_mut().enumerate() {
            *l = llr[column_index(r, c, 4)];
        }
        *word = if r < 3 {
            code.chase(&row_llr, 3)?
        } else {
            fec::pack_hard(&row_llr)
        };
    }
    if rows.iter().fold(0, |p, &w| p ^ w) != 0 {
        return None;
    }
    let data = rows[..3]
        .iter()
        .fold(0u64, |d, &w| (d << 12) | u64::from(w >> 5));
    let mut bits = Vec::with_capacity(36);
    fec::unpack(data, 36, &mut bits);
    let crc = fec::pack(&bits[28..]) as u8;
    (fec::crc8(&bits[..28]) == crc).then_some((data >> 8) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn llr_of(bits: &[u8]) -> Vec<f32> {
        bits.iter()
            .map(|&b| if b == 0 { 2.0 } else { -2.0 })
            .collect()
    }

    #[test]
    fn bptc196_round_trip_with_errors() {
        let data = [
            0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x0F, 0x1E, 0x2D, 0x3C,
        ];
        let tx = encode_196(&data);
        // Table B.3: R(3) goes first, H_R1(2) second (matrix index 13).
        assert_eq!(interleave(13), 1);
        let mut llr: [f32; 196] = llr_of(&tx).try_into().unwrap();
        for i in [5, 40, 77, 123, 150, 190] {
            llr[i] = -llr[i];
        }
        let (out, fixed) = decode_196(&llr).unwrap();
        assert_eq!(out, data);
        assert_eq!(fixed, 6);
    }

    /// Four confident errors, one to a row and column: the shape a radio's
    /// glitch left in real idle and LRRP bursts, which the soft pass alone
    /// did not clear (Chase flipped weaker bits instead).
    #[test]
    fn bptc196_confident_errors() {
        let data = [
            0xff, 0x83, 0xdf, 0x17, 0x32, 0x09, 0x4e, 0xd1, 0xe7, 0xcd, 0x8a, 0x91,
        ];
        let tx = encode_196(&data);
        let mut llr: [f32; 196] = llr_of(&tx).try_into().unwrap();
        // Matrix (row, column) of each error, sent at interleave(index).
        for (r, c) in [(5, 8), (6, 6), (7, 4), (8, 2)] {
            let i = interleave(1 + 15 * r + c);
            llr[i] *= -4.0;
        }
        let (out, fixed) = decode_196(&llr).unwrap();
        assert_eq!(out, data);
        assert_eq!(fixed, 4);
    }

    #[test]
    fn embedded_and_short_lc_round_trip() {
        let lc = [0x00, 0x00, 0x00, 0x00, 0x23, 0x2F, 0x00, 0x30, 0x39];
        let tx = encode_embedded(&lc);
        let mut llr: [f32; 128] = llr_of(&tx).try_into().unwrap();
        llr[17] *= -0.1;
        assert_eq!(decode_embedded(&llr), Some(lc));
        let slc = 0x0212_3456;
        let tx = encode_short_lc(slc);
        let mut llr: [f32; 68] = llr_of(&tx).try_into().unwrap();
        llr[30] *= -0.1;
        assert_eq!(decode_short_lc(&llr), Some(slc));
    }
}
