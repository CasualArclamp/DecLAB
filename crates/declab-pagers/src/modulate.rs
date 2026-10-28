// made by CasualArclamp
//! An FSK modulator for test signals: POCSAG transmissions and FLEX frames
//! laid end to end, FM-modulated to complex baseband with optional tuning
//! offset and noise.

use declab_core::Iq;

use crate::flex::FlexFrame;
use crate::pocsag::{self, PocsagMessage};

/// POCSAG deviation ±4.5 kHz (ITU-R M.584-2 Annex 1 §2); FLEX ±4.8 kHz outer,
/// ±1.6 kHz inner.
pub const POCSAG_DEV: f32 = 4500.0;
pub const FLEX_DEV: f32 = 4800.0;

/// The instantaneous frequency (Hz) and carrier on/off of each sample.
pub struct SignalBuilder {
    rate: f64,
    /// Time already used into the next sample, in samples (keeps symbol
    /// edges exact at fractional samples per symbol).
    carry: f64,
    freq: Vec<f32>,
    on: Vec<bool>,
}

impl SignalBuilder {
    pub fn new(rate: f64) -> Self {
        SignalBuilder {
            rate,
            carry: 0.0,
            freq: Vec::new(),
            on: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.freq.len()
    }

    pub fn is_empty(&self) -> bool {
        self.freq.is_empty()
    }

    pub fn seconds(&self) -> f64 {
        self.freq.len() as f64 / self.rate
    }

    /// Carrier off for `s` seconds.
    pub fn silence(&mut self, s: f64) {
        let n = (s * self.rate) as usize;
        self.freq.extend(std::iter::repeat_n(0.0, n));
        self.on.extend(std::iter::repeat_n(false, n));
    }

    /// Symbols 0–3 (lowest frequency first) at `baud`; `dev` is the outer
    /// deviation.
    pub fn symbols(&mut self, syms: &[u8], baud: u32, levels: u8, dev: f32) {
        let sps = self.rate / f64::from(baud);
        for &s in syms {
            let f = match (levels, s) {
                (2, 0 | 1) => -dev,
                (2, _) => dev,
                (_, 0) => -dev,
                (_, 1) => -dev / 3.0,
                (_, 2) => dev / 3.0,
                _ => dev,
            };
            let end = self.carry + sps;
            let n = end.floor() as usize;
            self.carry = end - n as f64;
            self.freq.extend(std::iter::repeat_n(f, n));
            self.on.extend(std::iter::repeat_n(true, n));
        }
    }

    pub fn pocsag(&mut self, baud: u32, msgs: &[PocsagMessage]) {
        // Binary 1 is the lower frequency.
        let syms: Vec<u8> = pocsag::encode(msgs)
            .iter()
            .map(|&b| if b { 0 } else { 3 })
            .collect();
        self.symbols(&syms, baud, 2, POCSAG_DEV);
    }

    pub fn flex(&mut self, f: &FlexFrame) {
        self.symbols(&f.head, 1600, 2, FLEX_DEV);
        self.symbols(&f.data, f.baud, f.levels, FLEX_DEV);
    }

    /// FM-modulate: a short premodulation filter (a box of `smooth_s`
    /// seconds), `offset_hz` tuning error, and white noise for a
    /// carrier-to-noise ratio of `cn_db` in 16 kHz (`None`: no noise).
    pub fn to_iq(&self, offset_hz: f64, cn_db: Option<f64>, smooth_s: f64, seed: u64) -> Vec<Iq> {
        let l = ((smooth_s * self.rate).round() as usize).max(1);
        let sigma = cn_db.map_or(0.0, |cn| {
            let n_per_sample = 10f64.powf(-cn / 10.0) * self.rate / 16_000.0;
            (n_per_sample / 2.0).sqrt() as f32
        });
        let mut rng = seed.max(1);
        let mut gauss = move || {
            // xorshift64 + Box–Muller: good enough for test noise.
            let mut u = || {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                ((rng >> 11) as f64 + 0.5) / (1u64 << 53) as f64
            };
            let (u1, u2) = (u(), u());
            let r = (-2.0 * u1.ln()).sqrt();
            let t = std::f64::consts::TAU * u2;
            ((r * t.cos()) as f32, (r * t.sin()) as f32)
        };
        let mut out = Vec::with_capacity(self.freq.len());
        let mut phase = 0f64;
        let mut sum = 0f64;
        let w = std::f64::consts::TAU / self.rate;
        for i in 0..self.freq.len() {
            sum += f64::from(self.freq[i]);
            if i >= l {
                sum -= f64::from(self.freq[i - l]);
            }
            phase += w * (sum / l as f64 + offset_hz);
            if phase > std::f64::consts::PI {
                phase -= std::f64::consts::TAU;
            }
            let a = if self.on[i] { 1.0 } else { 0.0 };
            let mut z = Iq::new((phase.cos() * a) as f32, (phase.sin() * a) as f32);
            if sigma > 0.0 {
                let (nr, ni) = gauss();
                z += Iq::new(nr * sigma, ni * sigma);
            }
            out.push(z);
        }
        out
    }
}
