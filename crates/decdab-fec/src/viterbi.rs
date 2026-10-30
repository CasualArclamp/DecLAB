// made by CasualArclamp
//! A block Viterbi decoder for DAB's rate-1/4, K = 7 mother code.
//!
//! Every DAB coded block starts in the zero state and ends there after six
//! tail bits (§11.1.1), so a block is decoded whole: forward through the
//! trellis keeping each step's 64 decisions, then back from state 0.
//!
//! **The butterfly** (as in DecDVB-T's decoder): the register holds the new
//! bit in bit 6 and the state is the six older bits. All four generators
//! have their first and last taps, so the branches from states 2j and 2j+1
//! to j and j+32 have outputs ±c_j in the pattern + − − +, and one branch
//! metric B_j = Σ_g s_g·c_{g,j} decides all four. The first and last
//! generators are equal (133), so their soft bits are simply added.
//! Metrics are correlations, the larger the better, in i32 (no
//! renormalisation needed for any DAB block length).

use crate::conv::GENERATORS;

pub struct Viterbi {
    /// ±1 signs of the three distinct generators' outputs for the
    /// butterflies' first branch (register 2j, input 0).
    sign: [[i32; 32]; 3],
    decisions: Vec<u64>,
}

impl Default for Viterbi {
    fn default() -> Self {
        Self::new()
    }
}

impl Viterbi {
    pub fn new() -> Viterbi {
        let mut sign = [[0i32; 32]; 3];
        for (row, g) in sign.iter_mut().zip([GENERATORS[0], GENERATORS[1], GENERATORS[2]]) {
            for (j, s) in row.iter_mut().enumerate() {
                *s = if ((2 * j as u8) & g).count_ones().is_multiple_of(2) {
                    1
                } else {
                    -1
                };
            }
        }
        Viterbi {
            sign,
            decisions: Vec::new(),
        }
    }

    /// Decode a block: `mother` holds 4·(n + 6) soft bits (depunctured),
    /// `out` receives the n information bits (one per byte). Returns the
    /// final metric of state 0 minus the best other state's: a margin, ≥ 0
    /// when the zero-tail path won.
    pub fn decode(&mut self, mother: &[i8], out: &mut Vec<u8>) -> i32 {
        let steps = mother.len() / 4;
        assert!(steps >= 6, "a block is at least its tail");
        self.decisions.clear();
        self.decisions.resize(steps, 0);
        const NEG: i32 = i32::MIN / 4;
        let mut m = [NEG; 64];
        m[0] = 0;
        let mut next = [0i32; 64];
        for (t, s) in mother.as_chunks::<4>().0.iter().enumerate() {
            let sa = s[0] as i32 + s[3] as i32;
            let (sb, sc) = (s[1] as i32, s[2] as i32);
            let mut bits = 0u64;
            for j in 0..32 {
                let b = sa * self.sign[0][j] + sb * self.sign[1][j] + sc * self.sign[2][j];
                let (e, o) = (m[2 * j], m[2 * j + 1]);
                let (p0, q0) = (e + b, o - b);
                let (p1, q1) = (e - b, o + b);
                next[j] = p0.max(q0);
                next[j + 32] = p1.max(q1);
                bits |= (u64::from(q0 > p0) << j) | (u64::from(q1 > p1) << (j + 32));
            }
            m = next;
            self.decisions[t] = bits;
        }
        let best_other = m[1..].iter().copied().max().unwrap_or(NEG);
        // Back from state 0: the state's top bit is the input that led to
        // it; the decision says which predecessor (its oldest bit).
        let mut st = 0usize;
        let start = out.len();
        out.resize(start + steps - 6, 0);
        for t in (0..steps).rev() {
            let d = (self.decisions[t] >> st) & 1;
            if t < steps - 6 {
                out[start + t] = (st >> 5) as u8;
            }
            st = ((st << 1) & 63) | d as usize;
        }
        m[0] - best_other
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conv::{EepSet, Plan, encode};

    fn noisy(bits: &[u8], seed: &mut u64, amp: f32, sigma: f32) -> Vec<i8> {
        bits.iter()
            .map(|&b| {
                // Box–Muller from a small LCG.
                let mut u = || {
                    *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                    ((*seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12)
                };
                let n = (-2.0 * u().ln()).sqrt() * (std::f64::consts::TAU * u()).cos();
                let v = if b == 0 { amp } else { -amp } + sigma * n as f32;
                v.clamp(-127.0, 127.0) as i8
            })
            .collect()
    }

    #[test]
    fn decodes_clean_and_noisy_blocks() {
        let mut seed = 1u64;
        for (plan, sigma) in [
            (Plan::fic(), 0.0),
            (Plan::fic(), 25.0),
            (Plan::eep(EepSet::A, 3, 48).unwrap(), 20.0),
            (Plan::uep(35).unwrap(), 14.0),
        ] {
            let bits: Vec<u8> = (0..plan.info_bits())
                .map(|i| ((i * 2654435761usize) >> 7 & 1) as u8)
                .collect();
            let mut mother = Vec::new();
            encode(&bits, &mut mother);
            let mut tx = Vec::new();
            plan.puncture(&mother, &mut tx);
            let rx = noisy(&tx, &mut seed, 30.0, sigma);
            let mut dep = Vec::new();
            plan.depuncture(&rx, &mut dep);
            let mut out = Vec::new();
            let margin = Viterbi::new().decode(&dep, &mut out);
            assert_eq!(out, bits, "{plan:?} sigma {sigma}");
            assert!(margin >= 0);
        }
    }
}
