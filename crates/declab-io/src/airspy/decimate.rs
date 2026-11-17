// made by CasualArclamp
//! Decimation of the Airspy's samples by 2 to 64, as libairspy's hosts offer
//! it: each halving is a half-band low-pass and every other sample kept, so
//! the band shrinks while the 12-bit ADC's quantisation noise is spread over
//! less of it (one more bit's worth for every four times). What is left is
//! a narrower, cleaner band at a lower rate, which also costs the decoders
//! and the waterfall less.
//!
//! The half-band is 63 taps (a Kaiser β 9 windowed sinc, 16 multiplies an
//! output): flat to within 0.01 dB over the inner 80 % of each output band
//! and at least 80 dB down wherever it would alias onto that, so only the
//! last stage's outer 10 % either side is not clean.

use declab_core::Iq;

/// Taps either side of the centre: the filter is `2 * HALF + 1` long.
const HALF: usize = 31;
/// Kaiser window β: about 90 dB of stopband.
const BETA: f64 = 9.0;

/// The factors offered: 1 (none) to 64, powers of two.
pub const FACTORS: [u32; 7] = [1, 2, 4, 8, 16, 32, 64];

/// Whether `n` is a factor [`Decimator`] takes.
pub fn valid(n: u32) -> bool {
    FACTORS.contains(&n)
}

/// One 2:1 stage. Only the centre and odd taps of a half-band are non-zero.
#[derive(Clone)]
struct Halfband {
    /// The odd taps h[1], h[3], … h[HALF] (the filter is symmetric).
    taps: Vec<f32>,
    /// Input not yet used by an output, with the HALF samples before it.
    hist: Vec<Iq>,
}

impl Halfband {
    fn new() -> Self {
        let i0 = |x: f64| {
            let (mut s, mut t) = (1.0, 1.0);
            for k in 1..40 {
                t *= (x / 2.0) * (x / 2.0) / (k * k) as f64;
                s += t;
            }
            s
        };
        let m = (HALF + 1) as f64;
        let taps = (1..=HALF)
            .step_by(2)
            .map(|n| {
                let n = n as f64;
                let x = std::f64::consts::FRAC_PI_2 * n;
                let w = i0(BETA * (1.0 - (n / m) * (n / m)).sqrt()) / i0(BETA);
                (0.5 * x.sin() / x * w) as f32
            })
            .collect();
        Halfband { taps, hist: vec![Iq::default(); 2 * HALF] }
    }

    /// Filter `x` and append every other output to `out`.
    fn process(&mut self, x: &[Iq], out: &mut Vec<Iq>) {
        self.hist.extend_from_slice(x);
        let h = &self.hist;
        // Output centred on h[c]: needs h[c - HALF ..= c + HALF].
        let mut c = HALF;
        while c + HALF < h.len() {
            let mut acc = h[c] * 0.5;
            for (i, &t) in self.taps.iter().enumerate() {
                let k = 2 * i + 1;
                acc += (h[c - k] + h[c + k]) * t;
            }
            out.push(acc);
            c += 2;
        }
        self.hist.drain(..c - HALF);
    }
}

/// Decimation by a power of two, 1 to 64: a cascade of half-bands.
#[derive(Clone)]
pub struct Decimator {
    stages: Vec<Halfband>,
    scratch: Vec<Iq>,
}

impl Decimator {
    /// `factor` must be one of [`FACTORS`].
    pub fn new(factor: u32) -> Self {
        assert!(valid(factor), "Airspy decimation {factor} is not a power of two from 1 to 64");
        Decimator { stages: (0..factor.trailing_zeros()).map(|_| Halfband::new()).collect(), scratch: Vec::new() }
    }

    pub fn factor(&self) -> u32 {
        1 << self.stages.len()
    }

    /// Decimate `buf` in place.
    pub fn process(&mut self, buf: &mut Vec<Iq>) {
        for s in &mut self.stages {
            self.scratch.clear();
            s.process(buf, &mut self.scratch);
            std::mem::swap(buf, &mut self.scratch);
        }
    }

    /// Forget the past (after a lost buffer, as the converter does).
    pub fn reset(&mut self) {
        *self = Decimator::new(self.factor());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tone at `f` (cycles per input sample), through `factor`: its level
    /// out, dB, measured once the filters have filled.
    fn gain_db(factor: u32, f: f64) -> f64 {
        let mut d = Decimator::new(factor);
        let n = 4096 * factor as usize;
        let mut out = Vec::new();
        // In pieces of an awkward size, as the USB transfers come.
        // The phase wrapped in f64 first: f32 at thousands of radians is
        // itself noise near -76 dB.
        let x: Vec<Iq> =
            (0..n).map(|k| Iq::from_polar(1.0, (std::f64::consts::TAU * (f * k as f64).fract()) as f32)).collect();
        for piece in x.chunks(1531) {
            let mut b = piece.to_vec();
            d.process(&mut b);
            out.extend(b);
        }
        assert_eq!(out.len(), n / factor as usize);
        let tail = &out[out.len() / 2..];
        let p = tail.iter().map(|z| z.norm_sqr() as f64).sum::<f64>() / tail.len() as f64;
        10.0 * p.log10()
    }

    #[test]
    fn none_passes_untouched() {
        let mut d = Decimator::new(1);
        let mut b = vec![Iq::new(0.25, -0.5); 100];
        d.process(&mut b);
        assert_eq!(b, vec![Iq::new(0.25, -0.5); 100]);
    }

    #[test]
    fn the_inner_band_is_flat_at_every_factor() {
        for factor in FACTORS {
            // 0.4 of the output rate either side: the inner 80 %.
            for edge in [-0.4, -0.2, 0.0, 0.1, 0.4] {
                let g = gain_db(factor, edge / factor as f64);
                assert!(g.abs() < 0.01, "×{factor} at {edge}: {g:.4} dB");
            }
        }
    }

    #[test]
    fn what_would_alias_onto_it_is_80_db_down() {
        for factor in [2, 8, 64] {
            // Anything from 0.6 of the output rate up folds back inside the
            // inner 80 %.
            for f_out in [0.6, 0.8, 1.3, 2.7, -0.6, -1.9] {
                let f = f_out / factor as f64;
                if f.abs() >= 0.5 {
                    continue;
                }
                let g = gain_db(factor, f);
                assert!(g < -80.0, "×{factor} at {f_out}: {g:.1} dB");
            }
        }
    }
}
