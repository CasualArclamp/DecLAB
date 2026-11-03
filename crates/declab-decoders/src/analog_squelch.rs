// made by CasualArclamp
//! Squelch for AM and FM, three ways of telling a signal from noise:
//!
//! - **Level**: the channel's power against a threshold in dBFS. Simple,
//!   but it moves with the gain and the noise floor.
//! - **SNR**: the carrier-to-noise ratio estimated from the signal itself by
//!   the M2M4 moments estimator (Pauluzzi & Beaulieu, IEEE Trans. Commun.
//!   48(10), 2000): for a constant envelope `A` in complex Gaussian noise
//!   of power `N`, `M2 = A² + N` and `M4 = A⁴ + 4A²N + 2N²`, so
//!   `A² = √(2M2² − M4)`. Independent of the gain. FM has a constant
//!   envelope; AM's modulation counts as noise, so it reads a few dB low.
//! - **Noise**: the receiver's classic noise squelch. The detector's output
//!   (FM: the discriminator; AM: the envelope over its mean) is high-passed
//!   above the audio, where only noise lives. A carrier quiets it; the
//!   threshold is the quieting in dB below what noise alone gives at the
//!   mode's default bandwidth.
//!
//! The metric is smoothed (~16 ms), decided every [`DECIDE`] samples with
//! [`HYSTERESIS_DB`] of hysteresis, and the audio fades in and out over
//! [`FADE_S`] rather than clicking.

use std::f64::consts::TAU;

/// Samples between decisions (1.3 ms at 48 kHz).
const DECIDE: u32 = 64;
/// Closes this far below the opening threshold.
pub const HYSTERESIS_DB: f32 = 2.0;
/// The fade in or out, seconds.
const FADE_S: f64 = 0.005;
/// The metrics' smoothing, Hz.
const SMOOTH_HZ: f64 = 10.0;
const EPS: f32 = 1e-20;

/// How the squelch decides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    Level,
    Snr,
    Noise,
}

impl Method {
    pub const ALL: [Method; 3] = [Method::Level, Method::Snr, Method::Noise];

    /// The value of the `"squelch"` setting.
    pub fn key(self) -> &'static str {
        match self {
            Method::Level => "level",
            Method::Snr => "snr",
            Method::Noise => "noise",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Method::Level => "Level",
            Method::Snr => "SNR",
            Method::Noise => "Noise",
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            Method::Level => "dBFS",
            Method::Snr | Method::Noise => "dB",
        }
    }

    /// The threshold slider's range.
    pub fn range(self) -> (f64, f64) {
        match self {
            Method::Level => (-100.0, -20.0),
            Method::Snr => (-5.0, 30.0),
            Method::Noise => (0.0, 30.0),
        }
    }

    fn parse(s: &str) -> Option<Method> {
        Method::ALL.into_iter().find(|m| m.key() == s)
    }
}

/// The squelch's settings: `"squelch"` = `"off"` | `"level"` | `"snr"` |
/// `"noise"` (or a number: level, in dBFS, as kept by older versions), and
/// each method's threshold, `"squelch_level"`, `"squelch_snr"`,
/// `"squelch_noise"`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub method: Option<Method>,
    pub level_dbfs: f64,
    pub snr_db: f64,
    pub noise_db: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            method: None,
            level_dbfs: -60.0,
            snr_db: 6.0,
            noise_db: 8.0,
        }
    }
}

impl Settings {
    pub fn threshold(&self, m: Method) -> f64 {
        match m {
            Method::Level => self.level_dbfs,
            Method::Snr => self.snr_db,
            Method::Noise => self.noise_db,
        }
    }

    pub fn threshold_mut(&mut self, m: Method) -> &mut f64 {
        match m {
            Method::Level => &mut self.level_dbfs,
            Method::Snr => &mut self.snr_db,
            Method::Noise => &mut self.noise_db,
        }
    }

    /// `Ok(false)`: not a squelch key.
    pub fn set(&mut self, key: &str, value: &str) -> Result<bool, String> {
        let num = || {
            value
                .parse::<f64>()
                .map_err(|_| format!("{key}: {value:?} is not a number"))
        };
        match key {
            "squelch" => {
                self.method = match (value, Method::parse(value)) {
                    ("off", _) => None,
                    (_, Some(m)) => Some(m),
                    _ => {
                        self.level_dbfs = num()?;
                        Some(Method::Level)
                    }
                }
            }
            "squelch_level" => self.level_dbfs = num()?.clamp(-140.0, 0.0),
            "squelch_snr" => self.snr_db = num()?.clamp(-20.0, 60.0),
            "squelch_noise" => self.noise_db = num()?.clamp(0.0, 60.0),
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// The settings as `(key, value)` pairs.
    pub fn pairs(&self) -> [(&'static str, String); 4] {
        [
            ("squelch", self.method.map_or("off", Method::key).into()),
            ("squelch_level", format!("{:.0}", self.level_dbfs)),
            ("squelch_snr", format!("{:.0}", self.snr_db)),
            ("squelch_noise", format!("{:.0}", self.noise_db)),
        ]
    }
}

/// What the noise method listens to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Detector {
    /// The envelope (AM), normalised by its mean.
    Envelope,
    /// The discriminator's phase step, radians (NFM).
    Narrow,
    /// The same at broadcast FM's rate and deviation.
    Wide,
}

/// A biquad, transposed direct form II.
#[derive(Debug, Clone, Copy, Default)]
struct Biquad {
    b: [f32; 3],
    a: [f32; 2],
    z: [f32; 2],
}

impl Biquad {
    /// RBJ's Audio EQ Cookbook high-pass.
    fn highpass(hz: f64, rate: f64, q: f64) -> Biquad {
        let w = TAU * hz / rate;
        let (s, c) = w.sin_cos();
        let alpha = s / (2.0 * q);
        let a0 = 1.0 + alpha;
        let b0 = (1.0 + c) / 2.0 / a0;
        Biquad {
            b: [b0 as f32, (-2.0 * b0) as f32, b0 as f32],
            a: [(-2.0 * c / a0) as f32, ((1.0 - alpha) / a0) as f32],
            z: [0.0; 2],
        }
    }

    #[inline]
    fn tick(&mut self, x: f32) -> f32 {
        let y = self.b[0] * x + self.z[0];
        self.z[0] = self.b[1] * x - self.a[0] * y + self.z[1];
        self.z[1] = self.b[2] * x - self.a[1] * y;
        y
    }
}

pub struct Squelch {
    detector: Detector,
    k: f32,
    fade: f32,
    power: f32,
    m2: f32,
    m4: f32,
    mean_amp: f32,
    /// 4th-order Butterworth high-pass: two biquads (Q 0.541, 1.307).
    hp: [Biquad; 2],
    noise: f32,
    /// What noise alone puts through the high-pass (see `reference`).
    noise_ref: f32,
    count: u32,
    open: bool,
    gain: f32,
    /// The latest metric, in the method's unit.
    pub metric: f32,
}

impl Squelch {
    pub fn new(detector: Detector, rate: f64) -> Squelch {
        // Above the audio: AM's 5 kHz, NFM's voice, broadcast FM's
        // multiplex (RDS ends at 59.4 kHz).
        let hz = match detector {
            Detector::Envelope => 6_500.0,
            Detector::Narrow => 6_000.0,
            Detector::Wide => 80_000.0,
        };
        let k = (1.0 - (-TAU * SMOOTH_HZ / rate).exp()) as f32;
        Squelch {
            detector,
            k,
            fade: (1.0 / (FADE_S * rate)) as f32,
            power: 0.0,
            m2: 0.0,
            m4: 0.0,
            mean_amp: 0.0,
            hp: [
                Biquad::highpass(hz, rate, 0.541_2),
                Biquad::highpass(hz, rate, 1.306_6),
            ],
            noise: 0.0,
            noise_ref: reference(detector),
            count: 0,
            open: false,
            gain: 0.0,
            metric: f32::NEG_INFINITY,
        }
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    /// One channel sample: its power `amp2` and, for FM, the discriminator's
    /// phase step `phase_step` (radians). Returns the audio's gain, 0..1.
    #[inline]
    pub fn tick(&mut self, method: Method, threshold: f32, amp2: f32, phase_step: f32) -> f32 {
        let k = self.k;
        match method {
            Method::Level => self.power += k * (amp2 - self.power),
            Method::Snr => {
                self.m2 += k * (amp2 - self.m2);
                self.m4 += k * (amp2 * amp2 - self.m4);
            }
            Method::Noise => {
                let d = match self.detector {
                    Detector::Envelope => {
                        let amp = amp2.sqrt();
                        self.mean_amp += k * (amp - self.mean_amp);
                        amp / self.mean_amp.max(1e-12)
                    }
                    _ => phase_step,
                };
                let h = self.hp[0].tick(d);
                let h = self.hp[1].tick(h);
                self.noise += k * (h * h - self.noise);
            }
        }
        self.count += 1;
        if self.count >= DECIDE {
            self.count = 0;
            self.metric = self.measure(method);
            if self.open {
                self.open = self.metric >= threshold - HYSTERESIS_DB;
            } else {
                self.open = self.metric >= threshold;
            }
        }
        let target = if self.open { 1.0 } else { 0.0 };
        self.gain += (target - self.gain).clamp(-self.fade, self.fade);
        self.gain
    }

    fn measure(&self, method: Method) -> f32 {
        match method {
            Method::Level => 10.0 * self.power.max(EPS).log10(),
            Method::Snr => {
                let s = (2.0 * self.m2 * self.m2 - self.m4).max(0.0).sqrt();
                let n = (self.m2 - s).max(self.m2 * 1e-4).max(EPS);
                (10.0 * (s.max(EPS) / n).log10()).max(-20.0)
            }
            Method::Noise => 10.0 * (self.noise_ref / self.noise.max(EPS)).log10(),
        }
    }
}

/// The high-passed detector's power for noise alone in the mode's default
/// channel (AM 10 kHz, NFM 12.5 kHz at 48 kS/s; WFM 200 kHz at 384 kS/s),
/// measured by `noise_reference` in the tests below; the noise method's
/// 0 dB. A wider or narrower channel shifts it by a dB or two.
fn reference(d: Detector) -> f32 {
    match d {
        Detector::Envelope => 0.0385,
        Detector::Narrow => 0.240,
        Detector::Wide => 0.576,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Iq;
    use declab_dsp::{Fir, lowpass};

    /// Complex white noise of unit power, filtered to `bw` at `rate`.
    fn noise(n: usize, bw: f64, rate: f64, seed: u64) -> Vec<Iq> {
        let mut s = seed | 1;
        let mut uni = move || {
            // xorshift64*: a reproducible generator for tests.
            s ^= s >> 12;
            s ^= s << 25;
            s ^= s >> 27;
            (s.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
        };
        let raw: Vec<Iq> = (0..n)
            .map(|_| {
                let (u, v) = (uni().max(1e-300), uni());
                let r = (-u.ln()).sqrt();
                Iq::new((r * (TAU * v).cos()) as f32, (r * (TAU * v).sin()) as f32)
            })
            .collect();
        let mut fir = Fir::new(lowpass(bw / 2.0 / rate, 255));
        let mut out = Vec::new();
        fir.process(&raw, &mut out);
        let p: f32 = out.iter().map(|x| x.norm_sqr()).sum::<f32>() / out.len() as f32;
        out.iter().map(|x| x / p.sqrt()).collect()
    }

    /// Runs `iq` through a squelch; the final metric and the share of
    /// decisions (after the first 0.2 s) that were open.
    fn run(d: Detector, rate: f64, m: Method, thr: f32, iq: &[Iq]) -> (f32, f64) {
        let mut sq = Squelch::new(d, rate);
        let mut prev = Iq::new(1.0, 0.0);
        let (mut open, mut total) = (0usize, 0usize);
        let skip = (0.2 * rate) as usize;
        for (i, &x) in iq.iter().enumerate() {
            let step = (x * prev.conj()).arg();
            prev = x;
            sq.tick(m, thr, x.norm_sqr(), step);
            if i >= skip {
                total += 1;
                open += usize::from(sq.is_open());
            }
        }
        (sq.metric, open as f64 / total as f64)
    }

    /// An FM carrier with a 1 kHz tone at `dev` Hz deviation, at `cn_db`
    /// C/N in the channel `bw`.
    fn fm(rate: f64, bw: f64, dev: f64, cn_db: f64, n: usize) -> Vec<Iq> {
        let a = 10f64.powf(cn_db / 20.0) as f32;
        let mut ph = 0f64;
        noise(n, bw, rate, 7)
            .into_iter()
            .enumerate()
            .map(|(i, z)| {
                ph += TAU * dev * (TAU * 1_000.0 * i as f64 / rate).sin() / rate;
                z + Iq::from_polar(a, ph as f32)
            })
            .collect()
    }

    /// An AM carrier, 30 % modulated by 1 kHz, at `cn_db`.
    fn am(rate: f64, cn_db: f64, n: usize) -> Vec<Iq> {
        let a = 10f64.powf(cn_db / 20.0);
        noise(n, 10_000.0, rate, 9)
            .into_iter()
            .enumerate()
            .map(|(i, z)| {
                let e = a * (1.0 + 0.3 * (TAU * 1_000.0 * i as f64 / rate).sin());
                z + Iq::new(e as f32, 0.0)
            })
            .collect()
    }

    /// Prints the noise method's reference for each detector (the high-
    /// passed power for noise alone): `reference()` holds these.
    /// `cargo test -p declab-decoders noise_reference -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn noise_reference() {
        for (d, rate, bw) in [
            (Detector::Envelope, 48_000.0, 10_000.0),
            (Detector::Narrow, 48_000.0, 12_500.0),
            (Detector::Wide, 384_000.0, 200_000.0),
        ] {
            let mut sq = Squelch::new(d, rate);
            sq.noise_ref = 1.0;
            let iq = noise((4.0 * rate) as usize, bw, rate, 3);
            let mut prev = Iq::new(1.0, 0.0);
            let mut acc = 0f64;
            for (i, &x) in iq.iter().enumerate() {
                sq.tick(Method::Noise, 0.0, x.norm_sqr(), (x * prev.conj()).arg());
                prev = x;
                if i > iq.len() / 4 {
                    acc += f64::from(sq.noise);
                }
            }
            println!("{d:?}: {:.4}", acc / (iq.len() - iq.len() / 4 - 1) as f64);
        }
    }

    #[test]
    fn every_method_stays_shut_on_noise_and_opens_on_a_carrier() {
        let n = 48_000;
        let quiet = noise(n, 12_500.0, 48_000.0, 5);
        let loud = fm(48_000.0, 12_500.0, 3_000.0, 20.0, n);
        for (m, thr) in [(Method::Snr, 6.0), (Method::Noise, 8.0)] {
            let (q, shut) = run(Detector::Narrow, 48_000.0, m, thr, &quiet);
            let (l, open) = run(Detector::Narrow, 48_000.0, m, thr, &loud);
            assert!(shut < 0.01, "NFM {m:?} noise {q:.1}: open {shut}");
            assert!(open > 0.99, "NFM {m:?} carrier {l:.1}: open {open}");
        }
        // Level: unit-power noise is 0 dBFS, the carrier 20 dB above it.
        let (_, shut) = run(Detector::Narrow, 48_000.0, Method::Level, 10.0, &quiet);
        let (_, open) = run(Detector::Narrow, 48_000.0, Method::Level, 10.0, &loud);
        assert!(shut < 0.01 && open > 0.99, "{shut} {open}");

        let quiet = noise(n, 10_000.0, 48_000.0, 6);
        let loud = am(48_000.0, 20.0, n);
        for (m, thr) in [(Method::Snr, 6.0), (Method::Noise, 8.0)] {
            let (q, shut) = run(Detector::Envelope, 48_000.0, m, thr, &quiet);
            let (l, open) = run(Detector::Envelope, 48_000.0, m, thr, &loud);
            assert!(shut < 0.01, "AM {m:?} noise {q:.1}: open {shut}");
            assert!(open > 0.99, "AM {m:?} carrier {l:.1}: open {open}");
        }

        let n = 384_000;
        let quiet = noise(n, 200_000.0, 384_000.0, 8);
        let loud = fm(384_000.0, 200_000.0, 60_000.0, 20.0, n);
        for (m, thr) in [(Method::Snr, 6.0), (Method::Noise, 8.0)] {
            let (q, shut) = run(Detector::Wide, 384_000.0, m, thr, &quiet);
            let (l, open) = run(Detector::Wide, 384_000.0, m, thr, &loud);
            assert!(shut < 0.01, "WFM {m:?} noise {q:.1}: open {shut}");
            assert!(open > 0.99, "WFM {m:?} carrier {l:.1}: open {open}");
        }
    }

    #[test]
    fn old_numeric_setting_means_level() {
        let mut s = Settings::default();
        assert_eq!(s.set("squelch", "-55"), Ok(true));
        assert_eq!((s.method, s.level_dbfs), (Some(Method::Level), -55.0));
        assert_eq!(s.set("squelch", "noise"), Ok(true));
        assert_eq!(s.method, Some(Method::Noise));
        assert_eq!(s.set("squelch", "off"), Ok(true));
        assert_eq!(s.method, None);
        assert_eq!(s.set("pitch", "800"), Ok(false));
    }

    /// Each method's reading against C/N, for choosing thresholds:
    /// `cargo test -p declab-decoders --release squelch_sweep -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn squelch_sweep() {
        for (name, d, rate) in [
            ("AM", Detector::Envelope, 48_000.0),
            ("NFM", Detector::Narrow, 48_000.0),
            ("WFM", Detector::Wide, 384_000.0),
        ] {
            let n = rate as usize;
            let mut line = format!("{name:4}");
            for m in Method::ALL {
                let quiet = match d {
                    Detector::Envelope => noise(n, 10_000.0, rate, 4),
                    Detector::Narrow => noise(n, 12_500.0, rate, 4),
                    Detector::Wide => noise(n, 200_000.0, rate, 4),
                };
                line += &format!(
                    " | {:5} noise {:6.1}",
                    m.label(),
                    run(d, rate, m, 99.0, &quiet).0
                );
                for cn in [0.0, 5.0, 10.0, 20.0] {
                    let iq = match d {
                        Detector::Envelope => am(rate, cn, n),
                        Detector::Narrow => fm(rate, 12_500.0, 3_000.0, cn, n),
                        Detector::Wide => fm(rate, 200_000.0, 60_000.0, cn, n),
                    };
                    line += &format!(" {cn:2.0}:{:5.1}", run(d, rate, m, 99.0, &iq).0);
                }
            }
            println!("{line}");
        }
    }
}
