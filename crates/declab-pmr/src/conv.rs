// made by CasualArclamp
//! The rate ½, K = 5 convolutional code NXDN (and P25 phase 1's trellis
//! neighbours) protect their channels with: G1 = 1 + D³ + D⁴, G2 = 1 + D +
//! D² + D⁴, output G1 then G2 for each input bit (as MMDVMHost's
//! `CNXDNConvolution::encode`, GPL-2.0-or-later, © Jonathan Naylor G4KLX).
//!
//! Decoded here by a soft Viterbi: branch metrics are sums of LLRs (positive
//! for a 0, as everywhere in this crate), punctured bits enter as LLR 0, and
//! the 4 tail bits bring the path home to state 0. MMDVMHost slices first
//! and counts hard distances; the soft metric is worth about 2 dB.

/// Encode `bits` (0/1, the tail already appended) to 2 bits each.
pub fn encode(bits: &[u8]) -> Vec<u8> {
    let (mut d1, mut d2, mut d3, mut d4) = (0u8, 0u8, 0u8, 0u8);
    let mut out = Vec::with_capacity(2 * bits.len());
    for &d in bits {
        let d = d & 1;
        out.push(d ^ d3 ^ d4);
        out.push(d ^ d1 ^ d2 ^ d4);
        d4 = d3;
        d3 = d2;
        d2 = d1;
        d1 = d;
    }
    out
}

/// The two coded bits leaving `state` (the last four inputs, newest in bit
/// 0) on input `d`.
#[inline]
fn outputs(state: usize, d: usize) -> (usize, usize) {
    let b = |k: usize| (state >> k) & 1;
    let (d1, d2, d3, d4) = (b(0), b(1), b(2), b(3));
    (d ^ d3 ^ d4, d ^ d1 ^ d2 ^ d4)
}

/// Soft Viterbi over `llr` (two per input bit, sending order, punctured
/// places 0.0): the `llr.len() / 2` input bits, the tail included, the path
/// ending in state 0. Also the path metric's shortfall from a perfect match
/// (0 = every LLR agreed), a quality figure.
// `s` is both a state number and an index: a range loop says so plainest.
#[allow(clippy::needless_range_loop)]
pub fn decode(llr: &[f32]) -> (Vec<u8>, f32) {
    let steps = llr.len() / 2;
    let mut metric = [f32::MIN; 16];
    metric[0] = 0.0;
    // Rust note: one u16 of survivor bits per step, bit s for state s.
    let mut from = vec![0u16; steps];
    let mut next = [f32::MIN; 16];
    // Rust note: `as_chunks::<2>()` views the slice as `[f32; 2]` pairs.
    for (k, pair) in llr.as_chunks::<2>().0.iter().enumerate() {
        next.fill(f32::MIN);
        let mut choice = 0u16;
        for s in 0..16 {
            if metric[s] == f32::MIN {
                continue;
            }
            for d in 0..2 {
                let (g1, g2) = outputs(s, d);
                let m = metric[s]
                    + if g1 == 0 { pair[0] } else { -pair[0] }
                    + if g2 == 0 { pair[1] } else { -pair[1] };
                let t = ((s << 1) | d) & 15;
                // The state shifted out (bit 3 of `s`) tells the two
                // predecessors of `t` apart.
                if m > next[t] {
                    next[t] = m;
                    if s & 8 != 0 {
                        choice |= 1 << t;
                    } else {
                        choice &= !(1 << t);
                    }
                }
            }
        }
        from[k] = choice;
        metric = next;
    }
    let mut bits = vec![0u8; steps];
    let mut s = 0usize;
    for k in (0..steps).rev() {
        bits[k] = (s & 1) as u8;
        let hi = usize::from((from[k] >> s) & 1);
        s = (s >> 1) | (hi << 3);
    }
    let best: f32 = llr.iter().map(|l| l.abs()).sum();
    (bits, best - metric[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_with_errors_and_punctures() {
        let mut data: Vec<u8> = (0..92).map(|i| ((i * 7 + 3) % 5 == 0) as u8).collect();
        data.extend([0, 0, 0, 0]);
        let coded = encode(&data);
        let mut llr: Vec<f32> = coded
            .iter()
            .map(|&b| if b == 0 { 2.0 } else { -2.0 })
            .collect();
        for i in (1..llr.len()).step_by(4) {
            llr[i] = 0.0;
        }
        for i in [10, 50, 90, 130, 170] {
            llr[i] = -llr[i];
        }
        let (bits, _) = decode(&llr);
        assert_eq!(bits, data);
    }
}
