// made by CasualArclamp
//! What the MBE vocoders share: a frame's model parameters, and mbelib's
//! synthesis (spectral amplitude enhancement, then harmonic oscillators for
//! the voiced bands and multisines for the unvoiced ones, overlap-added with
//! the window `WS`). AMBE+2 (`ambe2`) and IMBE (`imbe`) each decode their
//! bits into `Synth::cur` and then call `Synth::speak`.
//!
//! A port of `mbelib.c` (ISC licence, © 2010 mbelib Author; the notice is in
//! `ambe_tables.rs`), keeping its order of operations. Its noise comes from
//! C's `rand()`; here from a xorshift32.

// The loops keep mbelib's 1-based harmonic indices, so the port reads
// against the C line by line.
#![allow(clippy::needless_range_loop)]

use std::f32::consts::{E, PI};

use super::ambe_tables as t;

/// Samples a frame (20 ms at 8 kHz).
pub const FRAME: usize = 160;
/// mbelib's `uvquality`: sines in each unvoiced band (its default).
const UV_QUALITY: usize = 3;
/// mbelib scales its float output by 7 into 16 bits; this brings the same
/// level to ±1.0 full scale.
pub const TO_UNIT: f32 = 7.0 / 32768.0;

/// One frame's model parameters (mbelib's `mbe_parms`), harmonics 1-based.
#[derive(Clone)]
pub(crate) struct Parms {
    pub(crate) w0: f32,
    pub(crate) l: usize,
    pub(crate) vl: [u8; 57],
    pub(crate) ml: [f32; 57],
    pub(crate) log2ml: [f32; 57],
    pub(crate) phil: [f32; 57],
    pub(crate) psil: [f32; 57],
    pub(crate) gamma: f32,
    pub(crate) repeat: u32,
}

impl Parms {
    /// mbelib's `mbe_initMbeParms`.
    pub(crate) fn initial() -> Parms {
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
    pub(crate) fn copy_to(&self, to: &mut Parms) {
        to.clone_from(self);
        to.ml[0] = self.ml[0];
    }
}

/// The synthesiser and the three frames of parameters it keeps.
pub(crate) struct Synth {
    pub(crate) cur: Parms,
    pub(crate) prev: Parms,
    pub(crate) prev_enhanced: Parms,
    rng: u32,
}

impl Synth {
    pub(crate) fn new() -> Synth {
        let p = Parms::initial();
        Synth {
            cur: p.clone(),
            prev: p.clone(),
            prev_enhanced: p,
            rng: 0x1234_5678,
        }
    }

    /// mbelib's `mbe_initMbeParms` on all three (the noise runs on).
    pub(crate) fn init(&mut self) {
        let p = Parms::initial();
        self.cur = p.clone();
        self.prev = p.clone();
        self.prev_enhanced = p;
    }

    /// mbelib's `mbe_useLastMbeParms`: say the previous frame again.
    pub(crate) fn use_last(&mut self) {
        self.prev.copy_to(&mut self.cur);
    }

    /// Speak `cur` (as mbelib's process functions do once a frame is
    /// decoded): move it to `prev`, enhance, synthesise against
    /// `prev_enhanced`, keep it there; ±1.0 full scale out.
    pub(crate) fn speak(&mut self, out: &mut [f32; FRAME]) {
        self.cur.copy_to(&mut self.prev);
        self.spectral_amp_enhance();
        self.synthesize(out);
        self.cur.copy_to(&mut self.prev_enhanced);
        // mbelib clips at ±32760 when it converts to 16 bits.
        out.iter_mut()
            .for_each(|x| *x = (*x * TO_UNIT).clamp(-1.0, 1.0));
    }

    /// Silence, and forget the past.
    pub(crate) fn silence(&mut self, out: &mut [f32; FRAME]) {
        out.fill(0.0);
        self.init();
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
