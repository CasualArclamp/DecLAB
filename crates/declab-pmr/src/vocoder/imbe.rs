// made by CasualArclamp
//! The IMBE (7200×4400) speech decoder P25 phase 1 uses: 88 parameter bits
//! every 20 ms in, 160 samples at 8 kHz out.
//!
//! A port of mbelib's `mbe_decodeImbe4400Parms` and
//! `mbe_processImbe4400Dataf` (ISC licence, © 2010 mbelib Author; the notice
//! is in `imbe_tables.rs`); the synthesis is the one AMBE+2 shares
//! (`mbe.rs`). As there, mbelib's order of operations is kept.

// The loops keep mbelib's 1-based indices, so the port reads against the C.
#![allow(clippy::needless_range_loop)]

use std::f32::consts::PI;

use super::ambe2::FrameKind;
use super::imbe_tables as t;
use super::mbe::{FRAME, Synth};

/// The decoder's memory between frames.
pub struct ImbeDecoder {
    s: Synth,
}

impl Default for ImbeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ImbeDecoder {
    pub fn new() -> ImbeDecoder {
        ImbeDecoder { s: Synth::new() }
    }

    /// Forget the past (a new call).
    pub fn reset(&mut self) {
        self.s.init();
    }

    /// Decode one frame: its 88 parameter bits (`imbe_d`) and the FEC's
    /// corrections (`errs2`), into 160 samples, ±1.0 full scale
    /// (`mbe_processImbe4400Dataf`).
    pub fn decode(&mut self, bits: &[u8; 88], errors: u32, out: &mut [f32; FRAME]) -> FrameKind {
        let bad = !self.decode_parms(bits);
        let kind = if bad || errors > 5 {
            self.s.use_last();
            self.s.cur.repeat += 1;
            FrameKind::Repeat
        } else {
            self.s.cur.repeat = 0;
            FrameKind::Speech
        };
        if self.s.cur.repeat <= 3 {
            self.s.speak(out);
            kind
        } else {
            self.s.silence(out);
            FrameKind::Silence
        }
    }

    /// mbelib's `mbe_decodeImbe4400Parms`: false for an invalid frame (or
    /// silence, which mbelib treats the same way).
    fn decode_parms(&mut self, d: &[u8; 88]) -> bool {
        let (cur, prev) = (&mut self.s.cur, &mut self.s.prev);
        cur.repeat = prev.repeat;
        let b0 = [0, 1, 2, 3, 4, 5, 85, 86]
            .iter()
            .fold(0usize, |v, &i| (v << 1) | usize::from(d[i] & 1));
        if b0 > 207 {
            return false;
        }
        cur.w0 = 4.0 * PI / (b0 as f32 + 39.5);
        let l = (0.9254 * ((PI / cur.w0 + 0.25) as i32) as f64) as usize;
        if !(9..=56).contains(&l) {
            return false;
        }
        cur.l = l;
        let l9 = l - 9;
        let k = if l < 37 { l.div_ceil(3) } else { 12 };

        // Bits 6..84 into b0..bL+1 (mbelib `bb`), as table `bo` says.
        let mut bb = [[0u8; 12]; 58];
        for (n, i) in (6..85).enumerate() {
            let [r, c] = t::BO[l9][n];
            bb[usize::from(r)][usize::from(c)] = d[i] & 1;
        }

        // V/UV: one bit per three harmonics, the last covering the rest.
        let (mut j, mut kk) = (1, k - 1);
        for i in 1..=l {
            cur.vl[i] = bb[1][kk];
            if j == 3 {
                j = 1;
                kk = kk.saturating_sub(1);
            } else {
                j += 1;
            }
        }

        // G1 from b2, G2..G6 from b3..b7 (equation 68).
        let mut gm = [0f32; 7];
        let b2 = (0..6)
            .rev()
            .fold(0usize, |v, b| (v << 1) | usize::from(bb[2][b]));
        gm[1] = t::B2[b2];
        for (i, g) in gm.iter_mut().enumerate().skip(2) {
            let [nbits, step] = t::BA[l9][i - 2];
            let nb = nbits as usize;
            let bm = (0..nb)
                .rev()
                .fold(0u32, |v, b| (v << 1) | u32::from(bb[i + 1][b]));
            *g = step * (bm as f32 - 2f32.powi(nb as i32 - 1) + 0.5);
        }

        // Inverse DCT of G gives each block's first coefficient.
        let mut ri = [0f32; 7];
        for (i, r) in ri.iter_mut().enumerate().skip(1) {
            let mut sum = 0f32;
            for (m, &g) in gm.iter().enumerate().skip(1) {
                let am = if m == 1 { 1.0 } else { 2.0 };
                sum += am
                    * g
                    * ((std::f64::consts::PI * (m - 1) as f64 * (i as f64 - 0.5)) / 6.0).cos()
                        as f32;
            }
            *r = sum;
        }

        // The higher-order coefficients from b8..bL+1.
        let mut cik = [[0f32; 11]; 7];
        let mut m = 8;
        for i in 1..=6 {
            cik[i][1] = ri[i];
            for kc in 2..=usize::from(t::JI[l9][i - 1]) {
                let nb = usize::from(t::HOBA[l9][m - 8]);
                cik[i][kc] = if nb == 0 {
                    0.0
                } else {
                    let bm = (0..nb)
                        .rev()
                        .fold(0u32, |v, b| (v << 1) | u32::from(bb[m][b]));
                    t::QUANTSTEP[nb - 1]
                        * t::STANDDEV[kc - 2]
                        * (bm as f32 - 2f32.powi(nb as i32 - 1) + 0.5)
                };
                m += 1;
            }
        }

        // Inverse DCT of each block gives the residuals Tl.
        let mut tl = [0f32; 57];
        let mut li = 1;
        for i in 1..=6 {
            let ji = usize::from(t::JI[l9][i - 1]);
            for jj in 1..=ji {
                let mut sum = 0f32;
                for kc in 1..=ji {
                    let ak = if kc == 1 { 1.0 } else { 2.0 };
                    sum += ak
                        * cik[i][kc]
                        * ((std::f64::consts::PI * (kc - 1) as f64 * (jj as f64 - 0.5)) / ji as f64)
                            .cos() as f32;
                }
                if li < 57 {
                    tl[li] = sum;
                }
                li += 1;
            }
        }

        // Prediction from the previous frame (equations 75–77).
        let rho = if cur.l <= 15 {
            0.4
        } else if cur.l <= 24 {
            0.03 * cur.l as f32 - 0.05
        } else {
            0.7
        };
        if cur.l > prev.l {
            for x in prev.l + 1..=cur.l {
                prev.ml[x] = prev.ml[prev.l];
                prev.log2ml[x] = prev.log2ml[prev.l];
            }
        }
        let mut intkl = [0usize; 57];
        let mut deltal = [0f32; 57];
        let mut sum77 = 0f32;
        for x in 1..=cur.l {
            let flokl = (prev.l as f32 / cur.l as f32) * x as f32;
            intkl[x] = flokl as usize;
            deltal[x] = flokl - intkl[x] as f32;
            sum77 += (1.0 - deltal[x]) * prev.log2ml[intkl[x]]
                + deltal[x] * prev.log2ml[(intkl[x] + 1).min(56)];
        }
        sum77 *= rho / cur.l as f32;
        for x in 1..=cur.l {
            let c1 = rho * (1.0 - deltal[x]) * prev.log2ml[intkl[x]];
            let c2 = rho * deltal[x] * prev.log2ml[(intkl[x] + 1).min(56)];
            cur.log2ml[x] = tl[x] + c1 + c2 - sum77;
            cur.ml[x] = 2f32.powf(cur.log2ml[x]);
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn random_frames_stay_finite_and_bad_ones_repeat() {
        let mut dec = ImbeDecoder::new();
        let mut x: u64 = 7;
        let mut out = [0f32; FRAME];
        let mut kinds = [0u32; 3];
        for _ in 0..3000 {
            let bits: [u8; 88] = std::array::from_fn(|_| {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                (x & 1) as u8
            });
            let k = dec.decode(&bits, (x % 4) as u32, &mut out);
            kinds[match k {
                FrameKind::Speech => 0,
                FrameKind::Repeat => 1,
                _ => 2,
            }] += 1;
            assert!(
                out.iter().all(|v| v.is_finite() && v.abs() <= 1.0),
                "{out:?}"
            );
        }
        // About a fifth of random b0 values are invalid (> 207): repeats.
        assert!(kinds[0] > 2000 && kinds[1] > 300, "{kinds:?}");
    }

    #[test]
    fn steady_frames_come_out_at_their_pitch() {
        // b0 = 100: w0 = 4π / 139.5, a period of 69.75 samples.
        let mut d = [0u8; 88];
        let b0 = 100u8;
        for (k, &i) in [0usize, 1, 2, 3, 4, 5, 85, 86].iter().enumerate() {
            d[i] = (b0 >> (7 - k)) & 1;
        }
        // Every band voiced, and every gain and coefficient at the middle of
        // its range (its top bit set, the rest clear), found through the
        // bit allocation for this L.
        let l9 = 32 - 9;
        for (n, i) in (6..85).enumerate() {
            let [r, c] = t::BO[l9][n];
            let (r, c) = (usize::from(r), usize::from(c));
            let top = match r {
                1 => None,
                2 => Some(5),
                3..=7 => Some(t::BA[l9][r - 3][0] as usize - 1),
                _ => Some(usize::from(t::HOBA[l9][r - 8]).saturating_sub(1)),
            };
            d[i] = u8::from(top.is_none_or(|t| t == c));
        }
        let mut dec = ImbeDecoder::new();
        let mut pcm = Vec::new();
        let mut out = [0f32; FRAME];
        for _ in 0..12 {
            dec.decode(&d, 0, &mut out);
            pcm.extend_from_slice(&out);
        }
        let s = &pcm[FRAME * 6..];
        let ac = |lag: usize| s[lag..].iter().zip(s).map(|(a, b)| a * b).sum::<f32>();
        let best = (40..120).max_by(|&a, &b| ac(a).total_cmp(&ac(b))).unwrap();
        let period = 2.0 * PI / (4.0 * PI / 139.5);
        assert!(
            (best as f32 - period).abs() <= 2.0,
            "autocorrelation peak at {best}, pitch period {period:.1}"
        );
    }
}
