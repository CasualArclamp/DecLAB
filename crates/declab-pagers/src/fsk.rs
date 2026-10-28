// made by CasualArclamp
//! The FSK symbol slicer: 2- or 4-level, any symbol rate, one per decoder.
//!
//! Input is the FM discriminator's output, one value a sample. A box filter
//! one symbol long is the matched filter for NRZ; its output is read once a
//! symbol, at the end of each symbol, by a phase accumulator that need not
//! divide the sample rate. Timing comes from zero crossings: the box
//! output ramps linearly from one symbol's level `a` to the next's `b`, so
//! it crosses zero at a fraction a/(a−b) of the way; the error from that is
//! fed back into the phase.
//!
//! DC (tuning offset) and the outer level are tracked: fast and blind while
//! the decoder is searching (preambles are balanced), slow and
//! decision-directed once it has sync.

use crate::tones::Decision;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Symbol {
    /// 0 (lowest frequency) to 3 (highest); 0 or 3 in 2-level mode.
    pub sym: u8,
    /// The sample relative to the outer level (±1 outer, ±⅓ inner).
    pub soft: f32,
    /// Reliability of the symbol's first bit (its sign) and, in 4-level,
    /// its second (inner or outer), for soft-decision decoding.
    pub rel: [f32; 2],
}

pub struct Slicer {
    rate: f64,
    baud: u32,
    levels: u8,
    /// Symbols per sample, and the fraction of a symbol since the last
    /// symbol instant.
    inc: f64,
    phase: f64,
    /// Recent samples for the box filter, enough for the slowest rate.
    ring: Vec<f32>,
    pos: usize,
    len: usize,
    sum: f32,
    prev_y: f32,
    dc: f32,
    amp: f32,
    /// Level (±1, ±⅓) of the last symbol, and the phase of the last zero
    /// crossing since it.
    last_level: f32,
    crossing: Option<f64>,
    locked: bool,
    /// Input is the tone bank's energy-weighted level, already integrated
    /// over a symbol: no box filter, and the crossing between levels a and
    /// b falls where a·α² + b·(1−α)² = 0 rather than on a straight ramp.
    energy: bool,
    /// Box-filter sums drift with rounding; recompute this often.
    since_resum: u32,
    /// Symbols decided, for the eye diagram's two-symbol window.
    nsym: u64,
}

impl Slicer {
    /// `min_baud` sizes the filter memory: the slowest rate this slicer will
    /// be set to.
    pub fn new(rate: f64, baud: u32, levels: u8, min_baud: u32) -> Self {
        let cap = (rate / f64::from(min_baud.min(baud))).ceil() as usize + 2;
        let mut s = Slicer {
            rate,
            baud: 0,
            levels,
            inc: 0.0,
            phase: 0.0,
            ring: vec![0.0; cap],
            pos: 0,
            len: 1,
            sum: 0.0,
            prev_y: 0.0,
            dc: 0.0,
            amp: 1.0,
            last_level: 1.0,
            crossing: None,
            locked: false,
            energy: false,
            since_resum: 0,
            nsym: 0,
        };
        s.set_mode(baud, levels);
        s
    }

    /// A slicer for [`ToneBank::metric`](crate::tones::ToneBank::metric)
    /// samples: they are integrated already, so no box filter.
    pub fn for_energy(rate: f64, baud: u32, levels: u8) -> Self {
        let mut s = Slicer::new(rate, baud, levels, baud);
        s.energy = true;
        s.len = 1;
        s.resum();
        s
    }

    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Change speed or levels; symbol boundaries carry on where they were.
    pub fn set_mode(&mut self, baud: u32, levels: u8) {
        self.levels = levels;
        if baud == self.baud {
            return;
        }
        if self.baud != 0 {
            self.phase *= f64::from(baud) / f64::from(self.baud);
        }
        self.baud = baud;
        self.inc = f64::from(baud) / self.rate;
        self.len = if self.energy {
            1
        } else {
            ((self.rate / f64::from(baud)).round() as usize).clamp(1, self.ring.len() - 1)
        };
        self.resum();
    }

    /// The latest sample for an eye diagram: where it falls across two
    /// symbols (0..2, symbol instants at 0, 1 and 2) and its level against
    /// the outer level (±1 outer, ±⅓ inner).
    pub fn eye_point(&self) -> (f32, f32) {
        (
            (self.phase + (self.nsym & 1) as f64) as f32,
            self.prev_y / self.amp.max(1e-9),
        )
    }

    /// While unlocked, DC and level follow quickly; locked, slowly.
    pub fn set_locked(&mut self, locked: bool) {
        self.locked = locked;
    }

    fn resum(&mut self) {
        let n = self.ring.len();
        self.sum = (0..self.len)
            .map(|k| self.ring[(self.pos + n - 1 - k) % n])
            .sum();
        self.since_resum = 0;
    }

    /// One discriminator sample; a symbol when one is due.
    #[inline]
    pub fn push(&mut self, x: f32) -> Option<Symbol> {
        self.push_with(x, || None)
    }

    /// The tracked DC: with discriminator input, the tuning offset in Hz.
    pub fn dc(&self) -> f32 {
        self.dc
    }

    /// As [`push`](Self::push), but at a symbol instant `hint` may supply
    /// the decision (the tone bank's); timing and level tracking still run
    /// on `x`. Rust note: `impl FnOnce` takes any closure, called at most
    /// once, so the bank is only asked when a symbol is due.
    #[inline]
    pub fn push_with(&mut self, x: f32, hint: impl FnOnce() -> Option<Decision>) -> Option<Symbol> {
        let n = self.ring.len();
        let old = self.ring[(self.pos + n - self.len) % n];
        self.ring[self.pos] = x;
        self.pos = if self.pos + 1 == n { 0 } else { self.pos + 1 };
        self.sum += x - old;
        self.since_resum += 1;
        if self.since_resum >= 1 << 16 {
            self.resum();
        }
        let y = self.sum / self.len as f32 - self.dc;

        self.phase += self.inc;
        if (self.prev_y < 0.0) != (y < 0.0) && self.prev_y != y {
            let frac = f64::from(self.prev_y / (self.prev_y - y));
            self.crossing = Some(self.phase - self.inc * (1.0 - frac));
        }
        let out = if self.phase >= 1.0 {
            self.phase -= 1.0;
            // Interpolate back to the exact instant.
            let back = (self.phase / self.inc).min(1.0) as f32;
            let ys = y - (y - self.prev_y) * back;
            Some(self.decide(ys, hint()))
        } else {
            None
        };
        self.prev_y = y;
        out
    }

    fn decide(&mut self, ys: f32, hint: Option<Decision>) -> Symbol {
        self.nsym += 1;
        let amp = self.amp.max(1e-9);
        let v = ys / amp;
        // The slicer's own decision drives its timing and its DC and level
        // tracking, so a wrong tone-bank lock cannot pull the offset
        // estimate it is itself steered by; the bank's decision (if any) is
        // what goes out.
        let (own, level) = if self.levels == 4 {
            if v > 2.0 / 3.0 {
                (3, 1.0)
            } else if v > 0.0 {
                (2, 1.0 / 3.0)
            } else if v > -2.0 / 3.0 {
                (1, -1.0 / 3.0)
            } else {
                (0, -1.0)
            }
        } else if v > 0.0 {
            (3, 1.0)
        } else {
            (0, -1.0)
        };
        let sym = hint.map_or(own, |h| h.sym);

        // Timing: where the crossing should have been between the two levels.
        if let Some(cp) = self.crossing.take()
            && (self.last_level > 0.0) != (level > 0.0)
        {
            let expected = if self.energy {
                // Energies of the old and new tone go as α² and (1−α)².
                1.0 / (1.0 + f64::from((-level / self.last_level).sqrt()))
            } else {
                f64::from(self.last_level / (self.last_level - level))
            };
            let err = (cp - expected).clamp(-0.5, 0.5);
            let gain = if self.locked { 0.03 } else { 0.1 };
            self.phase -= gain * err;
        }
        self.last_level = level;

        if self.locked {
            let a = 0.01;
            self.dc += a * (ys - level * amp);
            self.amp += a * level.abs() * (ys / level - amp);
        } else {
            // DC slowly (~100 symbols): a faster blind estimate follows the
            // imbalance of the last few bits, which put it several hundred
            // Hz out at the sync word, too far for 512 baud's tone filters.
            self.dc += 0.01 * ys;
            self.amp += 0.05 * (ys.abs() - self.amp);
        }
        match hint {
            Some(h) => Symbol {
                sym,
                soft: h.soft,
                rel: h.rel,
            },
            None => Symbol {
                sym,
                soft: v,
                rel: [v.abs(), (v.abs() - 2.0 / 3.0).abs()],
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A noiseless NRZ waveform with fractional samples per symbol.
    fn nrz(levels: &[f32], rate: f64, baud: f64, offset: f32) -> Vec<f32> {
        let n = (levels.len() as f64 * rate / baud) as usize;
        (0..n)
            .map(|i| levels[((i as f64 + 0.5) * baud / rate) as usize] * 4500.0 + offset)
            .collect()
    }

    #[test]
    fn recovers_symbols_at_a_fractional_rate_with_dc() {
        let mut lv = Vec::new();
        for i in 0..600 {
            lv.push(if i % 2 == 0 { 1.0 } else { -1.0 });
        }
        let mut seed = 1u32;
        for _ in 0..2000 {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            lv.push(if seed >> 16 & 1 == 1 { 1.0 } else { -1.0 });
        }
        // 512 baud at 38.4k + 30 ppm clock error: 75.0023 samples a symbol.
        let x = nrz(&lv, 38_400.0, 512.0 * (1.0 - 30e-6), 1200.0);
        let mut s = Slicer::new(38_400.0, 512, 2, 512);
        let syms: Vec<u8> = x.iter().filter_map(|&v| s.push(v)).map(|s| s.sym).collect();
        let want: Vec<u8> = lv.iter().map(|&l| if l > 0.0 { 3 } else { 0 }).collect();
        // Find the alignment and count errors over the random part.
        let tail = &want[1000..2500];
        let pos = syms.windows(tail.len()).position(|w| w == tail);
        assert!(pos.is_some(), "random symbols not found");
    }
}
