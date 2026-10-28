// made by CasualArclamp
//! Non-coherent FSK detection: a bank of tone correlators, one per FSK
//! level, each integrating over exactly one symbol.
//!
//! The FM discriminator is wideband: below about 10 dB C/N its output is
//! dominated by clicks, and the box filter after it cannot undo that. Each
//! correlator here is a matched filter for its tone with unknown phase, so
//! the decision (largest energy) is the optimum non-coherent one; that gains
//! several dB at low C/N, more the slower the symbol rate.
//!
//! Each correlator is a sliding sum S_k[n] = Σ z[m]·e^(−jω_k m) over the last
//! L samples, updated in O(1) a sample: S += p_n·(z_n − e^(jω_k L)·z_(n−L)),
//! with p_n = e^(−jω_k n). The sums are recomputed from the sample ring every
//! [`RESUM_EVERY`] samples, which also applies frequency updates and bounds
//! rounding drift.
//!
//! Each tone's frequency is tracked by its own frequency-locked loop
//! (decision directed: only the winning tone is updated): the phase step
//! between the two halves of its window, arg(S_recent·S_older*), is the
//! frequency error times π/baud. That follows tuning offset and the
//! transmitter's actual deviation, which often differs from nominal by more
//! than a slow symbol's filter can stand.

use declab_core::Iq;

/// Samples between recomputing the sums (and applying the FLLs).
const RESUM_EVERY: u32 = 1024;

/// The levels' symbol numbers: tone `k` is symbol `k` (0 lowest).
pub const LEVELS4: [f32; 4] = [-1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0];

/// What the bank says at a symbol instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Decision {
    /// 0 (lowest tone) to 3; 0 or 3 in 2-level mode.
    pub sym: u8,
    /// Energy-weighted level, ±1 outer, ±⅓ inner: for plots.
    pub soft: f32,
    /// Reliability of the first (sign) bit and, in 4-level, the second.
    pub rel: [f32; 2],
}

struct Tone {
    /// Nominal offset from the carrier (Hz), and the tracked correction.
    nominal: f64,
    delta: f64,
    /// Frequency the sums were last built for.
    applied: f64,
    /// e^(−jωn) now; e^(−jω); e^(+jωL); e^(+jωL/2).
    ph: (f64, f64),
    step: (f64, f64),
    back_full: Iq,
    back_half: Iq,
    full: Iq,
    half: Iq,
}

pub struct ToneBank {
    rate: f64,
    baud: u32,
    levels: u8,
    len: usize,
    half_len: usize,
    ring: Vec<Iq>,
    pos: usize,
    /// Tones 0..4 lowest first; 2-level uses 0 and 3.
    tones: [Tone; 4],
    since_resum: u32,
    /// Common offset (Hz) to start from while unlocked.
    coarse: f64,
    locked: bool,
    /// Samples since sync was lost (saturating).
    unlocked_for: u32,
}

fn rot(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

impl ToneBank {
    /// `dev` is the outer deviation (POCSAG 4.5 kHz, FLEX 4.8 kHz);
    /// `min_baud` sizes the ring for the slowest rate it will be set to.
    pub fn new(rate: f64, baud: u32, levels: u8, dev: f64, min_baud: u32) -> Self {
        let cap = (rate / f64::from(min_baud.min(baud))).ceil() as usize + 2;
        let tone = |n: f64| Tone {
            nominal: n,
            delta: 0.0,
            applied: f64::NAN,
            ph: (1.0, 0.0),
            step: (1.0, 0.0),
            back_full: Iq::new(1.0, 0.0),
            back_half: Iq::new(1.0, 0.0),
            full: Iq::new(0.0, 0.0),
            half: Iq::new(0.0, 0.0),
        };
        let mut b = ToneBank {
            rate,
            baud: 0,
            levels,
            len: 1,
            half_len: 1,
            ring: vec![Iq::new(0.0, 0.0); cap],
            pos: 0,
            tones: [tone(-dev), tone(-dev / 3.0), tone(dev / 3.0), tone(dev)],
            since_resum: 0,
            coarse: 0.0,
            locked: false,
            unlocked_for: u32::MAX,
        };
        b.set_mode(baud, levels);
        b
    }

    pub fn set_mode(&mut self, baud: u32, levels: u8) {
        if baud == self.baud && levels == self.levels {
            return;
        }
        self.baud = baud;
        self.levels = levels;
        self.len = ((self.rate / f64::from(baud)).round() as usize).clamp(2, self.ring.len() - 1);
        self.half_len = self.len / 2;
        self.resum();
    }

    /// Unlocked, the tones follow `coarse_hz` (the discriminator's blind DC
    /// estimate of the tuning offset); locked, their own FLLs.
    pub fn set_locked(&mut self, locked: bool, coarse_hz: f32) {
        self.locked = locked;
        if locked {
            self.unlocked_for = 0;
        }
        self.coarse = f64::from(coarse_hz);
    }

    fn active(&self) -> &'static [usize] {
        if self.levels == 4 {
            &[0, 1, 2, 3]
        } else {
            &[0, 3]
        }
    }

    /// Rebuild every active tone's sums from the ring at its current
    /// frequency.
    fn resum(&mut self) {
        self.since_resum = 0;
        let n = self.ring.len();
        let (len, half_len, rate) = (self.len, self.half_len, self.rate);
        let locked = self.locked;
        let coarse = self.coarse;
        // Just after losing sync (a FLEX frame's end, a gap in a POCSAG
        // transmission) the tones stay where their FLLs left them: at low
        // C/N the discriminator's estimate wanders by kHz. Only after a
        // second without sync do they go back to it.
        if !locked && self.unlocked_for as f64 > rate {
            // Unlocked: every tone (the inner ones too, though 2-level does
            // not use them) sits at the common (discriminator) estimate.
            // The FLLs alias beyond ±baud, so they only run with sync.
            for t in &mut self.tones {
                t.delta = coarse;
            }
        }
        for &k in self.active() {
            let t = &mut self.tones[k];
            let f = t.nominal + t.delta;
            t.applied = f;
            let w = -std::f64::consts::TAU * f / rate;
            t.step = (w.cos(), w.sin());
            let wl = -w * len as f64;
            t.back_full = Iq::new(wl.cos() as f32, wl.sin() as f32);
            let wh = -w * half_len as f64;
            t.back_half = Iq::new(wh.cos() as f32, wh.sin() as f32);
            // The newest sample gets phasor 1; older ones e^(+jω·age).
            t.ph = (1.0, 0.0);
            let back = (t.step.0, -t.step.1);
            let mut p = (1.0f64, 0.0f64);
            let (mut full, mut half) = (Iq::new(0.0, 0.0), Iq::new(0.0, 0.0));
            for age in 0..len {
                let z = self.ring[(self.pos + n - 1 - age) % n];
                let pz = Iq::new(p.0 as f32, p.1 as f32) * z;
                full += pz;
                if age < half_len {
                    half += pz;
                }
                p = rot(p, back);
            }
            t.full = full;
            t.half = half;
            // The next sample is one step on from the newest.
            t.ph = t.step;
        }
    }

    /// One channel sample.
    #[inline]
    pub fn push(&mut self, z: Iq) {
        if !self.locked {
            self.unlocked_for = self.unlocked_for.saturating_add(1);
        }
        let n = self.ring.len();
        let old_full = self.ring[(self.pos + n - self.len) % n];
        let old_half = self.ring[(self.pos + n - self.half_len) % n];
        self.ring[self.pos] = z;
        self.pos = if self.pos + 1 == n { 0 } else { self.pos + 1 };
        let levels4 = self.levels == 4;
        for k in 0..4 {
            if !levels4 && (k == 1 || k == 2) {
                continue;
            }
            let t = &mut self.tones[k];
            let p = Iq::new(t.ph.0 as f32, t.ph.1 as f32);
            t.full += p * (z - t.back_full * old_full);
            t.half += p * (z - t.back_half * old_half);
            t.ph = rot(t.ph, t.step);
        }
        self.since_resum += 1;
        if self.since_resum >= RESUM_EVERY {
            self.resum();
        }
    }

    /// The decision at this instant (call at the symbol's end), and one FLL
    /// step for the tone that won.
    pub fn decide(&mut self) -> Decision {
        let e: [f32; 4] = std::array::from_fn(|k| self.tones[k].full.norm_sqr());
        let a: [f32; 4] = std::array::from_fn(|k| e[k].sqrt());
        let (sym, soft, rel) = if self.levels == 4 {
            let sym = (0..4).max_by(|&i, &j| e[i].total_cmp(&e[j])).unwrap_or(0);
            let tot = e.iter().sum::<f32>().max(1e-30);
            let soft = (0..4).map(|k| LEVELS4[k] * e[k]).sum::<f32>() / tot;
            let rel_a = (a[2].max(a[3]) - a[0].max(a[1])).abs();
            let rel_b = (a[1].max(a[2]) - a[0].max(a[3])).abs();
            (sym, soft, [rel_a, rel_b])
        } else {
            let sym = if e[3] > e[0] { 3 } else { 0 };
            let soft = (e[3] - e[0]) / (e[3] + e[0]).max(1e-30);
            (sym, soft, [(a[3] - a[0]).abs(), 0.0])
        };
        // FLL on the winner: phase step from its older to its newer half.
        let t = &mut self.tones[sym];
        let older = t.full - t.half;
        let x = t.half * older.conj();
        if self.locked && x.norm_sqr() > 0.0 {
            let err_hz = f64::from(x.im.atan2(x.re)) * self.rate
                / (std::f64::consts::TAU * self.half_len as f64);
            // Clamp: past ±baud the estimate aliases.
            let lim = f64::from(self.baud);
            let g = 0.02;
            t.delta = (t.delta + g * err_hz.clamp(-lim, lim))
                .clamp(-3.0 * lim - 2_000.0, 3.0 * lim + 2_000.0);
        }
        Decision {
            sym: sym as u8,
            soft,
            rel,
        }
    }

    /// The energy-weighted level now (±1 outer, ±⅓ inner): what the
    /// timing slicer runs on. Far cleaner than the discriminator at low C/N,
    /// since each correlator has only one symbol's bandwidth of noise.
    #[inline]
    pub fn metric(&self) -> f32 {
        let e = |k: usize| self.tones[k].full.norm_sqr();
        if self.levels == 4 {
            let (e0, e1, e2, e3) = (e(0), e(1), e(2), e(3));
            (e3 - e0 + (e2 - e1) / 3.0) / (e0 + e1 + e2 + e3).max(1e-30)
        } else {
            let (e0, e3) = (e(0), e(3));
            (e3 - e0) / (e3 + e0).max(1e-30)
        }
    }

    /// Tracked tone frequencies, Hz (lowest first; the 2-level ones at 0, 3).
    pub fn tone_hz(&self) -> [f64; 4] {
        std::array::from_fn(|k| self.tones[k].nominal + self.tones[k].delta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picks_the_tone_and_follows_offset_and_deviation() {
        let rate = 50_000.0;
        // 1200 baud 2-FSK at ±4.1 kHz (not the nominal 4.5) and +350 Hz.
        let mut b = ToneBank::new(rate, 1200, 2, 4500.0, 1200);
        b.set_locked(true, 0.0);
        b.resum();
        let sps = rate / 1200.0;
        let mut phase = 0f64;
        let mut seed = 7u32;
        let mut errs = 0;
        let mut t = 0.0f64;
        let mut next = sps;
        let mut bit = true;
        for n in 0..(2000.0 * sps) as usize {
            let f = 350.0 + if bit { 4100.0 } else { -4100.0 };
            phase += std::f64::consts::TAU * f / rate;
            b.push(Iq::new(phase.cos() as f32, phase.sin() as f32));
            t += 1.0;
            if t >= next {
                next += sps;
                let d = b.decide();
                if n > (500.0 * sps) as usize && (d.sym == 3) != bit {
                    errs += 1;
                }
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
                bit = seed >> 16 & 1 == 1;
            }
        }
        assert_eq!(errs, 0);
        let hz = b.tone_hz();
        assert!(
            (hz[3] - 4450.0).abs() < 60.0 && (hz[0] + 3750.0).abs() < 60.0,
            "{hz:?}"
        );
    }
}
