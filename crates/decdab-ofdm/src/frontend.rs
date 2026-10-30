// made by CasualArclamp
//! From the radio's rate to DAB's 2.048 MS/s.
//!
//! At 2.048·2^m MS/s (2.048, 4.096, 8.192 …) a chain of halfband filters,
//! each halving the rate: half the taps of a halfband are zero, and the
//! rest are symmetric, so a stage costs eight multiplies an output sample
//! here. Any other rate goes through the polyphase resampler.
//!
//! A halfband's transition is centred on a quarter of its input rate: at
//! 4.096 MS/s it passes 0.768 MHz (the ensemble's edge) and stops from
//! 1.28 MHz, where aliases would start to land on the ensemble.

use decdab_core::Iq;
use decdab_dsp::Resampler;
use decdab_dsp::resample::kaiser_lowpass;

use crate::params::SAMPLE_RATE;

/// Taps of each halfband (4m + 3, so the outermost taps are not zeros).
const HB_TAPS: usize = 47;
/// Kaiser β: about 60 dB of stopband in f32 at this length.
const HB_BETA: f64 = 7.0;

/// One halving stage.
struct Halfband {
    /// The non-zero taps either side of the centre, for offsets 1, 3, 5 …
    side: Vec<f32>,
    centre: f32,
    hist: Vec<Iq>,
    /// Whether the next input sample is an odd one (its output skipped).
    odd: bool,
}

impl Halfband {
    fn new() -> Halfband {
        let h = kaiser_lowpass(HB_TAPS, 0.25, HB_BETA);
        let sum: f32 = h.iter().sum();
        let c = HB_TAPS / 2;
        Halfband {
            side: (1..=c).step_by(2).map(|i| h[c + i] / sum).collect(),
            centre: h[c] / sum,
            hist: vec![Iq::default(); HB_TAPS - 1],
            odd: false,
        }
    }

    fn process(&mut self, input: &[Iq], out: &mut Vec<Iq>) {
        let c = HB_TAPS / 2;
        let keep = HB_TAPS - 1;
        let mut x = std::mem::take(&mut self.hist);
        x.extend_from_slice(input);
        // Output j is centred on x[j + c]; every other input sample makes
        // one.
        let n_out = x.len() - keep;
        let mut j = if self.odd { 1 } else { 0 };
        while j < n_out {
            let m = j + c;
            let mut acc = x[m] * self.centre;
            for (t, &h) in self.side.iter().enumerate() {
                let o = 2 * t + 1;
                acc += (x[m - o] + x[m + o]) * h;
            }
            out.push(acc);
            j += 2;
        }
        self.odd = (j - n_out) == 1;
        x.drain(..n_out);
        self.hist = x;
    }
}

enum Chain {
    Halfbands(Vec<Halfband>, Vec<Iq>),
    Resample(Box<Resampler>),
}

/// The decimator in front of the demodulator.
pub struct Frontend {
    chain: Chain,
    in_rate: f64,
    /// The NCO that brings an ensemble off the centre to DC: its phasor and
    /// step, when there is an offset.
    nco: Option<(Iq, Iq)>,
    mixed: Vec<Iq>,
}

impl Frontend {
    /// For samples at `in_rate` Hz.
    pub fn new(in_rate: f64) -> Result<Frontend, String> {
        let ratio = in_rate / SAMPLE_RATE;
        let m = ratio.log2().round();
        let chain = if (0.0..=4.0).contains(&m) && (ratio - 2f64.powf(m)).abs() < 1e-9 {
            Chain::Halfbands((0..m as usize).map(|_| Halfband::new()).collect(), Vec::new())
        } else {
            Chain::Resample(Box::new(Resampler::for_rates(in_rate, SAMPLE_RATE, 1.6e6)?))
        };
        Ok(Frontend {
            chain,
            in_rate,
            nco: None,
            mixed: Vec::new(),
        })
    }

    /// For an ensemble `offset_hz` from the centre of samples at `in_rate`.
    pub fn with_offset(in_rate: f64, offset_hz: f64) -> Result<Frontend, String> {
        if offset_hz.abs() + 0.8e6 > in_rate / 2.0 {
            return Err(format!(
                "an ensemble {:.3} MHz off centre is outside a {:.3} MS/s span",
                offset_hz / 1e6,
                in_rate / 1e6
            ));
        }
        let mut f = Frontend::new(in_rate)?;
        if offset_hz != 0.0 {
            let step = Iq::from_polar(
                1.0,
                (-std::f64::consts::TAU * offset_hz / in_rate) as f32,
            );
            f.nco = Some((Iq::new(1.0, 0.0), step));
        }
        Ok(f)
    }

    pub fn in_rate(&self) -> f64 {
        self.in_rate
    }

    /// Decimate `input` into `out` (cleared first).
    pub fn process(&mut self, input: &[Iq], out: &mut Vec<Iq>) {
        out.clear();
        let mut mixed = std::mem::take(&mut self.mixed);
        let input = match &mut self.nco {
            Some((ph, step)) => {
                mixed.clear();
                for (i, s) in input.iter().enumerate() {
                    mixed.push(s * *ph);
                    *ph *= *step;
                    if i % 1024 == 1023 {
                        *ph /= ph.norm();
                    }
                }
                &mixed[..]
            }
            None => input,
        };
        match &mut self.chain {
            Chain::Halfbands(stages, tmp) if stages.is_empty() => {
                let _ = tmp;
                out.extend_from_slice(input);
            }
            Chain::Halfbands(stages, tmp) => {
                let n = stages.len();
                tmp.clear();
                tmp.extend_from_slice(input);
                for (i, st) in stages.iter_mut().enumerate() {
                    if i + 1 == n {
                        st.process(tmp, out);
                    } else {
                        let mut next = Vec::with_capacity(tmp.len() / 2 + 1);
                        st.process(tmp, &mut next);
                        *tmp = next;
                    }
                }
            }
            Chain::Resample(r) => r.process(input, out),
        }
        self.mixed = mixed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(f: f64, rate: f64, n: usize) -> Vec<Iq> {
        (0..n)
            .map(|i| Iq::from_polar(1.0, (std::f64::consts::TAU * f * i as f64 / rate) as f32))
            .collect()
    }

    fn power_out(f: f64) -> f32 {
        let mut fe = Frontend::new(4.096e6).unwrap();
        let x = tone(f, 4.096e6, 40_000);
        let mut out = Vec::new();
        // In odd-sized pieces, to exercise the carry between calls.
        let mut all = Vec::new();
        for c in x.chunks(777) {
            fe.process(c, &mut out);
            all.extend_from_slice(&out);
        }
        assert!((all.len() as i64 - 20_000).abs() <= 1);
        all[100..].iter().map(|s| s.norm_sqr()).sum::<f32>() / (all.len() - 100) as f32
    }

    #[test]
    fn passes_the_ensemble_and_stops_the_aliases() {
        assert!((power_out(700e3) - 1.0).abs() < 0.01);
        assert!((power_out(-750e3) - 1.0).abs() < 0.02);
        // At least 55 dB down (the 8-bit radio's own range is about 48).
        assert!(power_out(1.4e6) < 3e-6);
        assert!(power_out(-1.6e6) < 3e-6);
    }
}
