// made by CasualArclamp
//! The AMBE+2 (3600×2450) speech decoder DMR, NXDN and P25 phase 2 use:
//! 49 parameter bits every 20 ms in, 160 samples at 8 kHz out.
//!
//! A port of mbelib's `mbe_decodeAmbe2450Parms`, `mbe_processAmbe2450Dataf`
//! and the synthesis in `mbelib.c` (spectral amplitude enhancement, then a
//! bank of harmonic oscillators for the voiced bands and noise-like
//! multisines for the unvoiced ones, overlap-added with the window `WS`).
//! The order of operations, including mbelib's quirks (it reads one
//! harmonic past the previous frame's last, and mutates the previous frame
//! while decoding the current one), is kept so the output matches mbelib's;
//! `tests` compares against mbelib itself where the C compiler is there.
//!
//! mbelib is © 2010 mbelib Author under the ISC licence (the notice is in
//! `ambe_tables.rs`). Its README warns that AMBE+2 may be covered by patents
//! in some countries; that is for whoever ships this to decide.
//!
//! The one deliberate difference: mbelib's noise comes from C's `rand()`;
//! here it is a xorshift32, so the unvoiced bands' fine detail differs from
//! a given mbelib build (it differs between C libraries anyway).

// The loops keep mbelib's 1-based harmonic indices, so the port reads
// against the C line by line.
#![allow(clippy::needless_range_loop)]

use std::f32::consts::{E, PI};

use super::ambe_tables as t;

/// Samples a frame (20 ms at 8 kHz).
pub const FRAME: usize = 160;
/// The output's sample rate.
pub const RATE: u32 = 8_000;
/// mbelib's `uvquality`: sines in each unvoiced band (its default).
const UV_QUALITY: usize = 3;
/// mbelib scales its float output by 7 into 16 bits; this brings the same
/// level to ±1.0 full scale.
pub const TO_UNIT: f32 = 7.0 / 32768.0;

/// What a frame turned out to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameKind {
    Speech,
    /// The previous frame said again (too many bit errors).
    Repeat,
    /// Silence: speech stopped, too many repeats, or an erasure or tone
    /// frame (tones are not synthesised, as in mbelib).
    Silence,
    Erasure,
    Tone,
}

/// One frame's model parameters (mbelib's `mbe_parms`), harmonics 1-based.
#[derive(Clone)]
struct Parms {
    w0: f32,
    l: usize,
    vl: [u8; 57],
    ml: [f32; 57],
    log2ml: [f32; 57],
    phil: [f32; 57],
    psil: [f32; 57],
    gamma: f32,
    repeat: u32,
}

impl Parms {
    /// mbelib's `mbe_initMbeParms`.
    fn initial() -> Parms {
        Parms {
            w0: 0.09378,
            l: 30,
            vl: [0; 57],
            ml: [0.0; 57],
            log2ml: [0.0; 57],
            phil: [0.0; 57],
            psil: [PI / 2.0; 57],
            gamma: 0.0,
            repeat: 0,
        }
    }

    /// mbelib's `mbe_moveMbeParms(self, to)`.
    fn copy_to(&self, to: &mut Parms) {
        to.clone_from(self);
        to.ml[0] = self.ml[0];
    }
}

/// The decoder's memory between frames.
pub struct Ambe2Decoder {
    cur: Parms,
    prev: Parms,
    prev_enhanced: Parms,
    rng: u32,
}

impl Default for Ambe2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Ambe2Decoder {
    pub fn new() -> Ambe2Decoder {
        let p = Parms::initial();
        Ambe2Decoder {
            cur: p.clone(),
            prev: p.clone(),
            prev_enhanced: p,
            rng: 0x1234_5678,
        }
    }

    /// Forget the past (a new call).
    pub fn reset(&mut self) {
        let rng = self.rng;
        *self = Ambe2Decoder::new();
        self.rng = rng;
    }

    /// A uniform number in [0, 1] (mbelib's `mbe_rand`).
    #[inline]
    fn rand(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x as f32 / u32::MAX as f32
    }

    #[inline]
    fn rand_phase(&mut self) -> f32 {
        self.rand() * (PI * 2.0) - PI
    }

    /// Decode one frame: its 49 parameter bits (`ambe_d`) and the Golay
    /// data-bit corrections (`errs2`), into 160 samples, ±1.0 full scale.
    pub fn decode(
        &mut self,
        bits: &[u8; 49],
        data_errors: u32,
        out: &mut [f32; FRAME],
    ) -> FrameKind {
        let bad = self.decode_parms(bits);
        let mut kind = match bad {
            2 => {
                self.cur.repeat = 0;
                FrameKind::Erasure
            }
            3 => {
                self.cur.repeat = 0;
                FrameKind::Tone
            }
            _ if data_errors > 3 => {
                // mbe_useLastMbeParms, then count the repeat.
                self.prev.copy_to(&mut self.cur);
                self.cur.repeat += 1;
                FrameKind::Repeat
            }
            _ => {
                self.cur.repeat = 0;
                FrameKind::Speech
            }
        };
        if bad == 0 && self.cur.repeat <= 3 {
            self.cur.copy_to(&mut self.prev);
            self.spectral_amp_enhance();
            self.synthesize(out);
            self.cur.copy_to(&mut self.prev_enhanced);
            out.iter_mut().for_each(|x| *x *= TO_UNIT);
        } else {
            out.fill(0.0);
            let p = Parms::initial();
            self.cur = p.clone();
            self.prev = p.clone();
            self.prev_enhanced = p;
            if kind == FrameKind::Repeat || kind == FrameKind::Speech {
                kind = FrameKind::Silence;
            }
        }
        kind
    }

    /// mbelib's `mbe_decodeAmbe2450Parms`: 0 for speech or silence, 2 for
    /// an erasure, 3 for a tone.
    fn decode_parms(&mut self, d: &[u8; 49]) -> u8 {
        let bitsof = |idx: &[usize]| {
            idx.iter()
                .fold(0usize, |v, &i| (v << 1) | usize::from(d[i] & 1))
        };
        let (cur, prev) = (&mut self.cur, &mut self.prev);
        cur.repeat = prev.repeat;

        let b0 = bitsof(&[0, 1, 2, 3, 37, 38, 39]);
        let silence = match b0 {
            120..=123 => return 2,
            126 | 127 => return 3,
            124 | 125 => true,
            _ => false,
        };
        let (f0, l) = if silence {
            cur.w0 = 2.0 * PI / 32.0;
            for v in &mut cur.vl[1..=14] {
                *v = 0;
            }
            (1.0 / 32.0, 14)
        } else {
            let f0 = t::W0[b0];
            cur.w0 = f0 * 2.0 * PI;
            (f0, t::L[b0] as usize)
        };
        cur.l = l;
        let unvc = 0.2046 / cur.w0.sqrt();

        let b1 = bitsof(&[4, 5, 6, 7, 35]);
        if !silence {
            for li in 1..=l {
                let jl = (li as f32 * 16.0 * f0) as usize;
                cur.vl[li] = t::VUV[b1][jl];
            }
        }

        let b2 = bitsof(&[8, 9, 10, 11, 36]);
        cur.gamma = t::DG[b2] + 0.5 * prev.gamma;

        let b3 = bitsof(&[12, 13, 14, 15, 16, 17, 18, 19, 40]);
        let b4 = bitsof(&[20, 21, 22, 23, 41, 42, 43]);
        let mut gm = [0f32; 9];
        gm[2..5].copy_from_slice(&t::PRBA24[b3]);
        gm[5..9].copy_from_slice(&t::PRBA58[b4]);
        let mut ri = [0f32; 9];
        for (i, r) in ri.iter_mut().enumerate().skip(1) {
            let mut sum = 0f32;
            for (m, &g) in gm.iter().enumerate().skip(1) {
                let am = if m == 1 { 1.0 } else { 2.0 };
                sum += am
                    * g
                    * ((std::f64::consts::PI * (m - 1) as f64 * (i as f64 - 0.5)) / 8.0).cos()
                        as f32;
            }
            *r = sum;
        }

        let mut cik = [[0f32; 18]; 5];
        let rconst = 1.0 / (2.0 * std::f32::consts::SQRT_2);
        for i in 1..=4 {
            cik[i][1] = 0.5 * (ri[2 * i - 1] + ri[2 * i]);
            cik[i][2] = rconst * (ri[2 * i - 1] - ri[2 * i]);
        }
        let b5 = bitsof(&[24, 25, 26, 27, 44]);
        let b6 = bitsof(&[28, 29, 30, 45]);
        let b7 = bitsof(&[31, 32, 33, 46]);
        let b8 = bitsof(&[34, 47, 48]);
        let ji: [usize; 5] = [0, 0, 0, 0, 0];
        let ji = {
            let mut j = ji;
            for (k, x) in j.iter_mut().enumerate().skip(1) {
                *x = usize::from(t::LMPRBL[l][k - 1]);
            }
            j
        };
        let hoc: [&[f32; 4]; 5] = [
            &[0.0; 4],
            &t::HOC_B5[b5],
            &t::HOC_B6[b6],
            &t::HOC_B7[b7],
            &t::HOC_B8[b8],
        ];
        for i in 1..=4 {
            for k in 3..=ji[i] {
                cik[i][k] = if k > 6 { 0.0 } else { hoc[i][k - 3] };
            }
        }

        // Inverse DCT of each block gives the prediction residuals Tl.
        let mut tl = [0f32; 57];
        let mut li = 1;
        for i in 1..=4 {
            let n = ji[i];
            for j in 1..=n {
                let mut sum = 0f32;
                for k in 1..=n {
                    let ak = if k == 1 { 1.0 } else { 2.0 };
                    sum += ak
                        * cik[i][k]
                        * ((std::f64::consts::PI * (k - 1) as f64 * (j as f64 - 0.5)) / n as f64)
                            .cos() as f32;
                }
                if li < 57 {
                    tl[li] = sum;
                }
                li += 1;
            }
        }

        // The prediction from the previous frame's log magnitudes.
        if cur.l > prev.l {
            for x in prev.l + 1..=cur.l {
                prev.ml[x] = prev.ml[prev.l];
                prev.log2ml[x] = prev.log2ml[prev.l];
            }
        }
        prev.log2ml[0] = prev.log2ml[1];
        prev.ml[0] = prev.ml[1];

        let mut intkl = [0usize; 57];
        let mut deltal = [0f32; 57];
        let mut sum43 = 0f32;
        for x in 1..=cur.l {
            let flokl = (prev.l as f32 / cur.l as f32) * x as f32;
            intkl[x] = flokl as usize;
            deltal[x] = flokl - intkl[x] as f32;
            sum43 += (1.0 - deltal[x]) * prev.log2ml[intkl[x]]
                + deltal[x] * prev.log2ml[(intkl[x] + 1).min(56)];
        }
        sum43 *= 0.65 / cur.l as f32;
        let sum42 = tl[1..=cur.l].iter().sum::<f32>() / cur.l as f32;
        let big_gamma = cur.gamma - 0.5 * ((cur.l as f64).ln() / 2f64.ln()) as f32 - sum42;
        for x in 1..=cur.l {
            let c1 = 0.65 * (1.0 - deltal[x]) * prev.log2ml[intkl[x]];
            let c2 = 0.65 * deltal[x] * prev.log2ml[(intkl[x] + 1).min(56)];
            cur.log2ml[x] = tl[x] + c1 + c2 - sum43 + big_gamma;
            let m = (f64::from(0.693 * cur.log2ml[x])).exp() as f32;
            cur.ml[x] = if cur.vl[x] == 1 { m } else { unvc * m };
        }
        0
    }

    /// mbelib's `mbe_spectralAmpEnhance`.
    fn spectral_amp_enhance(&mut self) {
        let c = &mut self.cur;
        let (mut rm0, mut rm1) = (0f32, 0f32);
        for l in 1..=c.l {
            let m2 = c.ml[l] * c.ml[l];
            rm0 += m2;
            rm1 += m2 * (c.w0 * l as f32).cos();
        }
        let (r2m0, r2m1) = (rm0 * rm0, rm1 * rm1);
        for l in 1..=c.l {
            if c.ml[l] != 0.0 {
                let wl = c.ml[l].sqrt()
                    * ((0.96 * PI * ((r2m0 + r2m1) - 2.0 * rm0 * rm1 * (c.w0 * l as f32).cos()))
                        / (c.w0 * rm0 * (r2m0 - r2m1)))
                        .powf(0.25);
                if 8 * l <= c.l {
                } else if wl > 1.2 {
                    c.ml[l] *= 1.2;
                } else if wl < 0.5 {
                    c.ml[l] *= 0.5;
                } else {
                    c.ml[l] *= wl;
                }
            }
        }
        let sum: f32 = c.ml[1..=c.l].iter().map(|m| m * m).sum();
        let gamma = if sum == 0.0 { 1.0 } else { (rm0 / sum).sqrt() };
        for m in &mut c.ml[1..=c.l] {
            *m *= gamma;
        }
    }

    /// mbelib's `mbe_synthesizeSpeechf` (against `prev_enhanced`).
    fn synthesize(&mut self, out: &mut [f32; FRAME]) {
        const N: usize = FRAME;
        let uvthreshold = 2700.0 * PI / 4000.0;
        let uvsine = 1.359_140_9 * E;
        let uvrand = 2.0f32;
        let q = UV_QUALITY;
        let loguvquality = if q == 1 {
            1.0 / E
        } else {
            (q as f32).ln() / q as f32
        };
        let uvstep = 1.0 / q as f32;
        let qfactor = loguvquality;
        let uvoffset = (uvstep * (q - 1) as f32) / 2.0;

        let num_uv = (1..=self.cur.l).filter(|&l| self.cur.vl[l] == 0).count() as f32;
        let (cw0, pw0) = (self.cur.w0, self.prev_enhanced.w0);
        out.fill(0.0);

        let maxl = if self.cur.l > self.prev_enhanced.l {
            for l in self.prev_enhanced.l + 1..=self.cur.l {
                self.prev_enhanced.ml[l] = 0.0;
                self.prev_enhanced.vl[l] = 1;
            }
            self.cur.l
        } else {
            for l in self.cur.l + 1..=self.prev_enhanced.l {
                self.cur.ml[l] = 0.0;
                self.cur.vl[l] = 1;
            }
            self.prev_enhanced.l
        };

        for l in 1..=56 {
            self.cur.psil[l] = self.prev_enhanced.psil[l] + (pw0 + cw0) * ((l * N) as f32 / 2.0);
            self.cur.phil[l] = if l <= self.cur.l / 4 {
                self.cur.psil[l]
            } else {
                let r = self.rand_phase();
                self.cur.psil[l] + (num_uv * r) / self.cur.l as f32
            };
        }

        let ws = &t::WS;
        let mut rphase = [0f32; 64];
        let mut rphase2 = [0f32; 64];
        for l in 1..=maxl {
            let (cv, pv) = (self.cur.vl[l], self.prev_enhanced.vl[l]);
            let (cm, pm) = (self.cur.ml[l], self.prev_enhanced.ml[l]);
            let (cphi, pphi) = (self.cur.phil[l], self.prev_enhanced.phil[l]);
            let cw0l = cw0 * l as f32;
            let pw0l = pw0 * l as f32;
            let lf = l as f32;
            if cv == 0 && pv == 1 {
                for r in &mut rphase[..q] {
                    *r = self.rand_phase();
                }
                for (n, o) in out.iter_mut().enumerate() {
                    let c1 = ws[n + N] * pm * (pw0l * n as f32 + pphi).cos();
                    let mut c3 = 0f32;
                    for (i, &rp) in rphase[..q].iter().enumerate() {
                        c3 += (cw0 * n as f32 * (lf + i as f32 * uvstep - uvoffset) + rp).cos();
                        if cw0l > uvthreshold {
                            c3 += (cw0l - uvthreshold) * uvrand * self.rand();
                        }
                    }
                    c3 *= uvsine * ws[n] * cm * qfactor;
                    *o += c1 + c3;
                }
            } else if cv == 1 && pv == 0 {
                for r in &mut rphase[..q] {
                    *r = self.rand_phase();
                }
                for (n, o) in out.iter_mut().enumerate() {
                    let c1 = ws[n] * cm * (cw0l * (n as f32 - N as f32) + cphi).cos();
                    let mut c3 = 0f32;
                    for (i, &rp) in rphase[..q].iter().enumerate() {
                        c3 += (pw0 * n as f32 * (lf + i as f32 * uvstep - uvoffset) + rp).cos();
                        if pw0l > uvthreshold {
                            c3 += (pw0l - uvthreshold) * uvrand * self.rand();
                        }
                    }
                    c3 *= uvsine * ws[n + N] * pm * qfactor;
                    *o += c1 + c3;
                }
            } else if cv == 1 || pv == 1 {
                for (n, o) in out.iter_mut().enumerate() {
                    let c1 = ws[n + N] * pm * (pw0l * n as f32 + pphi).cos();
                    let c2 = ws[n] * cm * (cw0l * (n as f32 - N as f32) + cphi).cos();
                    *o += c1 + c2;
                }
            } else {
                for r in &mut rphase[..q] {
                    *r = self.rand_phase();
                }
                for r in &mut rphase2[..q] {
                    *r = self.rand_phase();
                }
                for (n, o) in out.iter_mut().enumerate() {
                    let mut c3 = 0f32;
                    for (i, &rp) in rphase[..q].iter().enumerate() {
                        c3 += (pw0 * n as f32 * (lf + i as f32 * uvstep - uvoffset) + rp).cos();
                        if pw0l > uvthreshold {
                            c3 += (pw0l - uvthreshold) * uvrand * self.rand();
                        }
                    }
                    c3 *= uvsine * ws[n + N] * pm * qfactor;
                    let mut c4 = 0f32;
                    for (i, &rp) in rphase2[..q].iter().enumerate() {
                        c4 += (cw0 * n as f32 * (lf + i as f32 * uvstep - uvoffset) + rp).cos();
                        if cw0l > uvthreshold {
                            c4 += (cw0l - uvthreshold) * uvrand * self.rand();
                        }
                    }
                    c4 *= uvsine * ws[n] * cm * qfactor;
                    *o += c3 + c4;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 49 bits for parameter indices b0..b8 (the inverse of the bit
    /// gathering in `decode_parms`).
    fn pack(b: [usize; 9]) -> [u8; 49] {
        let map: [&[usize]; 9] = [
            &[0, 1, 2, 3, 37, 38, 39],
            &[4, 5, 6, 7, 35],
            &[8, 9, 10, 11, 36],
            &[12, 13, 14, 15, 16, 17, 18, 19, 40],
            &[20, 21, 22, 23, 41, 42, 43],
            &[24, 25, 26, 27, 44],
            &[28, 29, 30, 45],
            &[31, 32, 33, 46],
            &[34, 47, 48],
        ];
        let mut d = [0u8; 49];
        for (v, idx) in b.iter().zip(map) {
            for (k, &i) in idx.iter().enumerate() {
                d[i] = ((v >> (idx.len() - 1 - k)) & 1) as u8;
            }
        }
        d
    }

    #[test]
    fn steady_voiced_frames_come_out_at_their_pitch() {
        let mut dec = Ambe2Decoder::new();
        let b0 = 40;
        let period = 1.0 / t::W0[b0];
        // All bands voiced (b1 = 0), a moderate gain.
        let frame = pack([b0, 0, 16, 100, 60, 10, 5, 5, 3]);
        let mut pcm = Vec::new();
        let mut out = [0f32; FRAME];
        for _ in 0..12 {
            assert_eq!(dec.decode(&frame, 0, &mut out), FrameKind::Speech);
            assert!(out.iter().all(|x| x.is_finite() && x.abs() < 2.0));
            pcm.extend_from_slice(&out);
        }
        let s = &pcm[FRAME * 6..];
        let ac = |lag: usize| s[lag..].iter().zip(s).map(|(a, b)| a * b).sum::<f32>();
        let best = (20..200).max_by(|&a, &b| ac(a).total_cmp(&ac(b))).unwrap();
        assert!(
            (best as f32 - period).abs() <= 2.0,
            "autocorrelation peak at {best}, pitch period {period:.1}"
        );
        assert!(ac(0) > 0.0);
    }

    #[test]
    fn erasures_tones_and_errors() {
        let mut dec = Ambe2Decoder::new();
        let mut out = [1f32; FRAME];
        assert_eq!(
            dec.decode(&pack([120, 0, 0, 0, 0, 0, 0, 0, 0]), 0, &mut out),
            FrameKind::Erasure
        );
        assert!(out.iter().all(|&x| x == 0.0));
        assert_eq!(
            dec.decode(&pack([127, 0, 0, 0, 0, 0, 0, 0, 0]), 0, &mut out),
            FrameKind::Tone
        );
        let speech = pack([50, 3, 20, 7, 9, 1, 2, 3, 4]);
        assert_eq!(dec.decode(&speech, 0, &mut out), FrameKind::Speech);
        // Too many errors: the last frame is said again, up to three times.
        for _ in 0..3 {
            assert_eq!(dec.decode(&speech, 5, &mut out), FrameKind::Repeat);
        }
        assert_eq!(dec.decode(&speech, 5, &mut out), FrameKind::Silence);
    }

    #[test]
    fn random_frames_stay_finite() {
        let mut dec = Ambe2Decoder::new();
        let mut x: u64 = 1;
        let mut out = [0f32; FRAME];
        for _ in 0..3000 {
            let bits: [u8; 49] = std::array::from_fn(|_| {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                (x & 1) as u8
            });
            dec.decode(&bits, (x % 5) as u32, &mut out);
            assert!(out.iter().all(|v| v.is_finite()), "{out:?}");
        }
    }
}
