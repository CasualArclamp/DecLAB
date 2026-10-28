// made by CasualArclamp
//! The BCH(31,21) code with an even parity bit that both POCSAG and FLEX use
//! for every 32-bit codeword.
//!
//! POCSAG: ITU-R M.584-2 Annex 1 §4.4 — generator x¹⁰+x⁹+x⁸+x⁶+x⁵+x³+1,
//! 21 information bits, 10 check bits, then even parity over all 32.
//! FLEX (ARIB STD-43A / multimon-ng's `bch.c`): the same code — roots
//! α¹…α⁴ of GF(2⁵) with x⁵+x²+1, whose minimal polynomials multiply to the
//! same generator — with the bits sent in the same order.
//!
//! Words here are in *transmission order*: the first bit on air is bit 31.
//! Bits 31..11 carry the information, 10..1 the check bits, 0 the parity.
//! FLEX stores words LSB-first (first bit on air in bit 0); its decoder
//! reverses them with `u32::reverse_bits` before and after.

use std::sync::OnceLock;

/// x¹⁰+x⁹+x⁸+x⁶+x⁵+x³+1 (ITU-R M.584-2 Annex 1 §4.4).
const GEN: u32 = 0x769;

/// Remainder of a 31-bit codeword (bit 30 = x³⁰) divided by the generator:
/// the syndrome, zero for a codeword.
fn remainder(cw31: u32) -> u32 {
    let mut r = cw31;
    for i in (10..31).rev() {
        if r & (1 << i) != 0 {
            r ^= GEN << (i - 10);
        }
    }
    r & 0x3FF
}

/// Syndrome → error pattern (31-bit) for every single and double error. A
/// double-error-correcting BCH code gives all 496 of them distinct syndromes.
fn table() -> &'static [u32; 1024] {
    static T: OnceLock<[u32; 1024]> = OnceLock::new();
    // Rust note: `get_or_init` runs the closure once, on first use, from
    // whichever thread gets there first; later calls just return the table.
    T.get_or_init(|| {
        let mut t = [0u32; 1024];
        for i in 0..31 {
            t[remainder(1 << i) as usize] = 1 << i;
            for j in i + 1..31 {
                let e = (1 << i) | (1 << j);
                t[remainder(e) as usize] = e;
            }
        }
        t
    })
}

/// Encode 21 information bits into a 32-bit transmission-order codeword.
pub fn encode(info21: u32) -> u32 {
    let m = (info21 & 0x1F_FFFF) << 10;
    let w = (m | remainder(m)) << 1;
    w | (w.count_ones() & 1)
}

/// Correct up to two bit errors. Returns the corrected word and the number
/// of bits changed, or `None` when the word is beyond repair.
pub fn decode(w: u32) -> Option<(u32, u32)> {
    let c31 = w >> 1;
    let s = remainder(c31);
    let parity_bad = w.count_ones() & 1 != 0;
    if s == 0 {
        return Some(if parity_bad { (w ^ 1, 1) } else { (w, 0) });
    }
    let e = table()[s as usize];
    if e == 0 {
        return None;
    }
    let c = ((c31 ^ e) << 1) | (w & 1);
    if c.count_ones() & 1 == 0 {
        Some((c, e.count_ones()))
    } else if e.count_ones() == 1 {
        // One data error and the parity bit itself: still two errors.
        Some((c ^ 1, 2))
    } else {
        // Two corrected plus a parity mismatch would be three: refuse.
        None
    }
}

/// Least reliable bits tried by [`decode_soft`]: 2⁶ = 64 test patterns.
const CHASE_BITS: usize = 6;
/// The most bits a soft decode may change: t + 2.
const CHASE_MAX_FLIPS: u32 = 4;

/// Soft-decision decoding (Chase algorithm II): `rel[i]` is how sure the
/// demodulator was of bit `i` (bit 31 first on air). Every pattern of flips
/// over the least reliable bits is hard-decoded; of the codewords found,
/// the one whose differences from `w` cost the least reliability wins.
/// Clean words (0 or 1 error) take the hard path alone.
pub fn decode_soft(w: u32, rel: &[f32; 32]) -> Option<(u32, u32)> {
    if let Some((c, n)) = decode(w)
        && n <= 1
    {
        return Some((c, n));
    }
    let mut order: [u8; 32] = std::array::from_fn(|i| i as u8);
    // Rust note: `select_nth_unstable_by` partitions in O(n) so the
    // CHASE_BITS least reliable positions come first, unsorted.
    order.select_nth_unstable_by(CHASE_BITS, |&a, &b| {
        rel[a as usize].total_cmp(&rel[b as usize])
    });
    let mut best: Option<(u32, f32)> = None;
    for pattern in 0u32..1 << CHASE_BITS {
        let mut t = w;
        for (k, &pos) in order[..CHASE_BITS].iter().enumerate() {
            if pattern >> k & 1 == 1 {
                t ^= 1 << pos;
            }
        }
        let Some((c, _)) = decode(t) else { continue };
        let diff = c ^ w;
        if diff.count_ones() > CHASE_MAX_FLIPS {
            continue;
        }
        let cost: f32 = (0..32)
            .filter(|&i| diff >> i & 1 == 1)
            .map(|i| rel[i])
            .sum();
        if best.is_none_or(|(_, b)| cost < b) {
            best = Some((c, cost));
        }
    }
    best.map(|(c, _)| (c, (c ^ w).count_ones()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_pocsag_codewords_are_codewords() {
        // Sync and idle codewords (ITU-R M.584-2 Annex 1 §4.3, §4.6).
        for w in [0x7CD2_15D8u32, 0x7A89_C197] {
            assert_eq!(encode(w >> 11), w);
            assert_eq!(decode(w), Some((w, 0)));
        }
    }

    #[test]
    fn corrects_one_and_two_errors_and_refuses_three() {
        let w = encode(0x1A_BCDE);
        for i in 0..32 {
            assert_eq!(decode(w ^ (1 << i)), Some((w, 1)), "bit {i}");
            for j in i + 1..32 {
                assert_eq!(
                    decode(w ^ (1 << i) ^ (1 << j)),
                    Some((w, 2)),
                    "bits {i},{j}"
                );
            }
        }
        let mut refused = 0;
        for i in 0..32 {
            for j in i + 1..32 {
                for k in j + 1..32 {
                    match decode(w ^ (1 << i) ^ (1 << j) ^ (1 << k)) {
                        None => refused += 1,
                        Some((c, _)) => assert_ne!(c, w),
                    }
                }
            }
        }
        // With the parity bit the code has distance 6: every triple error is
        // detected, none mis-corrected.
        assert_eq!(refused, 32 * 31 * 30 / 6);
    }

    #[test]
    fn soft_decoding_corrects_errors_in_unreliable_bits() {
        let w = encode(0x0F_0F0F);
        // Four errors, all where the demodulator was unsure.
        let e = w ^ (1 << 3) ^ (1 << 9) ^ (1 << 20) ^ (1 << 30);
        let mut rel = [1.0f32; 32];
        for i in [3, 9, 20, 30, 12, 25] {
            rel[i] = 0.1;
        }
        assert_ne!(decode(e).map(|d| d.0), Some(w));
        assert_eq!(decode_soft(e, &rel), Some((w, 4)));
    }
}
