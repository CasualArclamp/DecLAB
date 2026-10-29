// made by CasualArclamp
//! The 4FSK front end DMR, NXDN and P25 phase 1 share: channel IQ in, the
//! instantaneous frequency through the root-raised-cosine matched filter out,
//! in symbol units (±1, ±3 nominal before the receiver's level estimate).
//!
//! Two detectors, chosen by [`Detector`]:
//!
//! - `Discriminator`: the textbook limiter-discriminator, arg(z[n]·z*[n−1]),
//!   then the matched filter on the frequency. Below about 10 dB C/N the
//!   arg throws 2π "clicks" whenever the noise phasor wraps round the
//!   origin, and the filter smears each click across several symbols.
//! - `Product` (the default): the matched filter runs on the complex lag
//!   product z[n]·z*[n−1] itself and the arg is taken once per output. Near
//!   threshold the noise in the product averages out as vectors before any
//!   angle is taken, so there are no clicks to smear; above threshold the two
//!   agree. The sweep (`cargo run --release -p declab-pmr --example
//!   dmr_sweep`) measures what it buys.

use num_complex::Complex32;

pub type Iq = Complex32;

/// Which frequency detector (see the module notes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Detector {
    Discriminator,
    Product,
}

/// Root-raised-cosine taps, `sps` samples a symbol, ±`span` symbols, summed
/// to 1 so a steady frequency comes through unchanged.
pub fn rrc(alpha: f64, sps: usize, span: usize) -> Vec<f32> {
    use std::f64::consts::PI;
    let n = 2 * span * sps + 1;
    let mut h: Vec<f64> = (0..n)
        .map(|i| {
            let t = (i as f64 - (span * sps) as f64) / sps as f64;
            if t.abs() < 1e-9 {
                1.0 - alpha + 4.0 * alpha / PI
            } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-9 {
                alpha / 2f64.sqrt()
                    * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                        + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
            } else {
                ((PI * t * (1.0 - alpha)).sin() + 4.0 * alpha * t * (PI * t * (1.0 + alpha)).cos())
                    / (PI * t * (1.0 - (4.0 * alpha * t).powi(2)))
            }
        })
        .collect();
    let s: f64 = h.iter().sum();
    h.iter_mut().for_each(|x| *x /= s);
    h.into_iter().map(|x| x as f32).collect()
}

/// A windowed-sinc low-pass, cut-off `fc` of the sample rate `fs`.
pub fn lowpass(fc: f64, fs: f64, taps: usize) -> Vec<f32> {
    use std::f64::consts::PI;
    let m = (taps - 1) as f64 / 2.0;
    let wc = 2.0 * fc / fs;
    let mut h: Vec<f64> = (0..taps)
        .map(|i| {
            let t = i as f64 - m;
            let sinc = if t == 0.0 {
                wc
            } else {
                (PI * wc * t).sin() / (PI * t)
            };
            // Blackman window: the stop band matters more than the skirt.
            let w = 0.42 - 0.5 * (2.0 * PI * i as f64 / (taps - 1) as f64).cos()
                + 0.08 * (4.0 * PI * i as f64 / (taps - 1) as f64).cos();
            sinc * w
        })
        .collect();
    let s: f64 = h.iter().sum();
    h.iter_mut().for_each(|x| *x /= s);
    h.into_iter().map(|x| x as f32).collect()
}

/// A FIR over a ring kept twice over, so each output is one contiguous dot
/// product. Rust note: generic over the sample type `T` (real or complex)
/// through the arithmetic traits it needs.
struct Fir<T> {
    taps: Vec<f32>,
    ring: Vec<T>,
    pos: usize,
}

impl<T> Fir<T>
where
    T: Copy + Default + std::ops::Add<Output = T> + std::ops::Mul<f32, Output = T>,
{
    fn new(taps: Vec<f32>) -> Self {
        let n = taps.len();
        Fir {
            taps,
            ring: vec![T::default(); 2 * n],
            pos: 0,
        }
    }

    #[inline]
    fn push(&mut self, x: T) -> T {
        let n = self.taps.len();
        self.ring[self.pos] = x;
        self.ring[self.pos + n] = x;
        self.pos = (self.pos + 1) % n;
        // The window starts at the oldest sample; taps are symmetric, so the
        // order does not matter.
        let w = &self.ring[self.pos..self.pos + n];
        let mut acc = T::default();
        for (s, &h) in w.iter().zip(&self.taps) {
            acc = acc + *s * h;
        }
        acc
    }

    fn reset(&mut self) {
        self.ring.iter_mut().for_each(|x| *x = T::default());
        self.pos = 0;
    }
}

/// IQ at `rate` in, matched-filtered frequency in symbol units out, one value
/// per input sample.
pub struct Fsk4Demod {
    detector: Detector,
    /// The AFC's mixer: IQ is turned by −`afc_step` radians a sample before
    /// the channel filter, so a narrow filter stays centred on the signal.
    afc_step: f64,
    afc_phase: f64,
    rate: f64,
    channel: Fir<Iq>,
    matched_c: Fir<Iq>,
    matched_r: Fir<f32>,
    prev: Iq,
    /// Radians a sample → symbol units.
    scale: f32,
}

impl Fsk4Demod {
    /// `rate` samples a second, `symbol_rate` symbols, the outer symbol (±3)
    /// at `outer_dev_hz`, RRC roll-off `alpha`, channel low-pass `chan_hz`.
    pub fn new(
        rate: f64,
        symbol_rate: f64,
        outer_dev_hz: f64,
        alpha: f64,
        chan_hz: f64,
        detector: Detector,
    ) -> Self {
        let sps = (rate / symbol_rate).round() as usize;
        let taps = rrc(alpha, sps, 6);
        Fsk4Demod {
            detector,
            afc_step: 0.0,
            afc_phase: 0.0,
            rate,
            channel: Fir::new(lowpass(chan_hz, rate, 8 * sps + 1)),
            matched_c: Fir::new(taps.clone()),
            matched_r: Fir::new(taps),
            prev: Iq::new(0.0, 0.0),
            scale: (rate / (2.0 * std::f64::consts::PI) / (outer_dev_hz / 3.0)) as f32,
        }
    }

    pub fn detector(&self) -> Detector {
        self.detector
    }

    /// Shift the input by −`hz` (the carrier's offset) from now on.
    pub fn set_afc(&mut self, hz: f64) {
        self.afc_step = 2.0 * std::f64::consts::PI * hz / self.rate;
    }

    pub fn afc_hz(&self) -> f64 {
        self.afc_step * self.rate / (2.0 * std::f64::consts::PI)
    }

    /// Change the channel low-pass's cut-off. Its length (and so its delay)
    /// stays the same, so the timing of what is already tracked holds.
    pub fn set_channel(&mut self, chan_hz: f64) {
        let n = self.channel.taps.len();
        self.channel.taps = lowpass(chan_hz, self.rate, n);
    }

    /// Symbol units per hertz of frequency.
    pub fn units_per_hz(&self) -> f32 {
        self.scale * (2.0 * std::f64::consts::PI / self.rate) as f32
    }

    pub fn process(&mut self, iq: &[Iq], out: &mut Vec<f32>) {
        out.reserve(iq.len());
        for &x in iq {
            let x = if self.afc_step == 0.0 {
                x
            } else {
                self.afc_phase = (self.afc_phase - self.afc_step) % std::f64::consts::TAU;
                x * Iq::from_polar(1.0, self.afc_phase as f32)
            };
            let z = self.channel.push(x);
            let d = z * self.prev.conj();
            self.prev = z;
            let f = match self.detector {
                Detector::Product => self.matched_c.push(d).arg(),
                Detector::Discriminator => self.matched_r.push(d.arg()),
            };
            out.push(f * self.scale);
        }
    }

    pub fn reset(&mut self) {
        self.channel.reset();
        self.matched_c.reset();
        self.matched_r.reset();
        self.prev = Iq::new(0.0, 0.0);
        self.afc_step = 0.0;
        self.afc_phase = 0.0;
    }
}

/// 4FSK from symbols (±1, ±3) at `rate`, unit amplitude: impulses through
/// the transmit RRC, then deviation (outer symbol at `outer_dev_hz`), with
/// `pad_s` of unmodulated carrier either side. For simulators and tests.
pub fn modulate(
    symbols: &[f32],
    rate: f64,
    symbol_rate: f64,
    outer_dev_hz: f64,
    alpha: f64,
    pad_s: f64,
) -> Vec<Iq> {
    let sps = (rate / symbol_rate).round() as usize;
    let h = rrc(alpha, sps, 8);
    let pad = (pad_s * rate) as usize;
    let mut impulses = vec![0f32; pad + symbols.len() * sps + h.len() + pad];
    for (s, &v) in symbols.iter().enumerate() {
        impulses[pad + s * sps] = v;
    }
    // An RRC summed to 1 passes a symbol held over many symbols at its
    // level only when the impulses are scaled by sps.
    let k = (outer_dev_hz / 3.0) as f32 * sps as f32;
    let mut phase = 0f64;
    let mut out = Vec::with_capacity(impulses.len());
    for i in 0..impulses.len() {
        let mut f = 0f32;
        for (j, &t) in h.iter().enumerate() {
            if i >= j {
                f += impulses[i - j] * t;
            }
        }
        phase += 2.0 * std::f64::consts::PI * f64::from(f * k) / rate;
        out.push(Iq::from_polar(1.0, phase as f32));
    }
    out
}

/// Add white Gaussian noise for a carrier-to-noise ratio `cn_db` measured in
/// `ref_bw_hz` (the carrier has unit power), and a frequency offset.
pub fn add_noise(iq: &mut [Iq], rate: f64, cn_db: f64, ref_bw_hz: f64, offset_hz: f64, seed: u64) {
    let n_power = rate / ref_bw_hz * 10f64.powf(-cn_db / 10.0);
    let sigma = (n_power / 2.0).sqrt() as f32;
    let mut s = seed | 1;
    let mut uniform = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        ((s >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    };
    let w = 2.0 * std::f64::consts::PI * offset_hz / rate;
    for (i, x) in iq.iter_mut().enumerate() {
        let (u1, u2) = (uniform(), uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let a = 2.0 * std::f64::consts::PI * u2;
        let rot = Iq::from_polar(1.0, (w * i as f64) as f32);
        *x = *x * rot + Iq::new((r * a.cos()) as f32 * sigma, (r * a.sin()) as f32 * sigma);
    }
}
