// made by CasualArclamp
//! The burst finder: watches the whole channel with a short FFT, opens a
//! burst where bins rise above their noise floor, cuts each burst out at
//! its own frequency (mixer + decimating FIR, from a little before it was
//! seen), and hands it over when it has been quiet for a while.
//!
//! Each burst runs its own down-converter only while it is open, so the
//! steady cost is the FFT: one 256-point FFT per 256 samples (no overlap).
//! Two sensors on different frequencies at once are two bursts.

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

/// FFT size: 9.8 kHz bins at 2.5 MS/s, 102 µs a frame.
pub const N: usize = 256;
/// A bin is active when its power (two frames averaged) is this far over
/// its floor: 8 dB. Noise alone does that in ~5e-5 of bins; a burst only
/// opens where two frames running were active, and needs [`MIN_FRAMES`]
/// active frames to be kept.
const THRESHOLD: f32 = 6.3;
/// Floor tracking: the mean of quiet frames over ~64 frames (6.5 ms), and
/// a slow creep up while active (so a steady carrier is absorbed in ~1 s).
const FLOOR_ALPHA: f32 = 1.0 / 64.0;
const FLOOR_CREEP: f32 = 1.0 + 2e-4;
/// Inside a burst's range, where this frame was quiet: much slower, so a
/// floor that started low still recovers.
const FLOOR_ALPHA_BUSY: f32 = 1.0 / 1024.0;
/// Frames averaged for the first floor (3 ms) before anything is
/// detected.
const INIT_FRAMES: u64 = 32;
/// Active bins this close (in bins) are one signal; a burst takes in
/// activity this far outside its range (FSK's other tone).
const CLUSTER_GAP: usize = 3;
const BURST_MARGIN: usize = 8;
/// Pre-trigger: samples kept from before a burst was seen.
const PRE_S: f64 = 0.002;
/// A burst closes after this long with nothing in its range…
const HOLD_S: f64 = 0.025;
/// …or when this long.
const MAX_S: f64 = 1.5;
/// Active frames a burst needs to be kept (~0.6 ms of carrier): noise
/// opens short ones now and then. The shortest real transmissions (a
/// meter's 3 ms) have dozens.
const MIN_FRAMES: u32 = 6;
/// Active frames that confirm a burst and start its cut-out. The ring
/// holds the pre-trigger and a hold time, so confirmation can come late.
const CONFIRM_FRAMES: u32 = 3;
/// Bursts open at once.
const MAX_OPEN: usize = 6;
/// The rate bursts are cut out at, or just over it.
pub const BURST_RATE: f64 = 500_000.0;

/// A finished burst: its samples at `rate`, centred on `mix_hz`.
#[derive(Debug, Clone)]
pub struct Burst {
    /// Index (at the input rate) of `iq[0]`'s first input sample.
    pub start_sample: u64,
    pub rate: f32,
    /// The burst's samples are the channel mixed down by this, Hz.
    pub mix_hz: f64,
    /// Where it was seen, Hz from the channel's centre: the active bins'
    /// range and power-weighted centre.
    pub lo_hz: f64,
    pub hi_hz: f64,
    pub centre_hz: f64,
    /// The filter it was cut out with: ±this, Hz.
    pub half_hz: f64,
    /// Peak bin power over the floor, dB.
    pub peak_db: f32,
    /// Frames with something in its range.
    pub active_frames: u32,
    pub iq: Vec<Complex32>,
}

/// Mixer + polyphase decimating FIR for one burst.
struct Ddc {
    phase: Complex32,
    step: Complex32,
    taps: Arc<Vec<f32>>,
    hist: Vec<Complex32>,
    pos: usize,
    count: usize,
    decim: usize,
}

impl Ddc {
    fn new(mix_hz: f64, rate: f64, taps: Arc<Vec<f32>>, decim: usize) -> Self {
        let w = -std::f64::consts::TAU * mix_hz / rate;
        Ddc {
            phase: Complex32::new(1.0, 0.0),
            step: Complex32::new(w.cos() as f32, w.sin() as f32),
            hist: vec![Complex32::default(); taps.len()],
            taps,
            pos: 0,
            count: 0,
            decim,
        }
    }

    #[inline]
    fn push(&mut self, x: Complex32, out: &mut Vec<Complex32>) {
        let y = x * self.phase;
        self.phase *= self.step;
        let n = self.hist.len();
        self.hist[self.pos] = y;
        self.pos = if self.pos + 1 == n { 0 } else { self.pos + 1 };
        self.count += 1;
        if self.count == self.decim {
            self.count = 0;
            // hist[pos] is now the oldest sample: taps run oldest first
            // (the filter is symmetric, so the order does not matter).
            let (a, b) = self.hist.split_at(self.pos);
            let mut acc = Complex32::default();
            for (h, t) in b.iter().chain(a.iter()).zip(self.taps.iter()) {
                acc += h * t;
            }
            out.push(acc);
            // Keep the oscillator on the unit circle.
            let m = self.phase.norm();
            self.phase /= m;
        }
    }
}

struct Open {
    lo: usize,
    hi: usize,
    start_sample: u64,
    last_active: u64,
    first_frame: u64,
    active_frames: u32,
    peak: f32,
    /// Power over the floor summed per bin while active: the burst's
    /// shape across frequency.
    shape: Vec<f32>,
    mix_hz: f64,
    /// Started when the burst is confirmed ([`CONFIRM_FRAMES`]): noise
    /// opens bursts all the time, and cutting each out would cost more
    /// than everything else.
    ddc: Option<Ddc>,
    iq: Vec<Complex32>,
}

/// One row of the band's frequency-time map: the highest power over the
/// floor in each bin over the last stretch, dB.
#[derive(Debug, Clone)]
pub struct MapRow {
    pub db: Vec<f32>,
}

pub struct BurstFinder {
    rate: f64,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    frame: Vec<Complex32>,
    scratch: Vec<Complex32>,
    fill: usize,
    power: Vec<f32>,
    prev: Vec<f32>,
    floor: Vec<f32>,
    frames: u64,
    samples: u64,
    /// The last `pre.len()` samples, a ring written at `pre_pos`; `pre_fill`
    /// counts up to its length after a reset.
    pre: Vec<Complex32>,
    pre_pos: usize,
    pre_fill: usize,
    /// Samples kept from before a burst's first frame.
    pre_len: usize,
    hold_frames: u64,
    max_frames: u64,
    open: Vec<Open>,
    decim: usize,
    taps: Arc<Vec<f32>>,
    /// Bursts opened (kept or not), for the CPU tests.
    pub opened: u64,
    /// Finished bursts, oldest first: the receiver takes them.
    pub done: Vec<Burst>,
    /// The map, when asked for (`map_every` frames a row).
    pub map_every: u32,
    map_acc: Vec<f32>,
    map_count: u32,
    pub map_rows: Vec<MapRow>,
    active: Vec<bool>,
    prev_active: Vec<bool>,
    busy: Vec<bool>,
    filters: std::collections::HashMap<u32, Arc<Vec<f32>>>,
}

/// Windowed-sinc low-pass taps, cutoff `fc` as a fraction of the rate.
pub fn lowpass(n: usize, fc: f64) -> Vec<f32> {
    let m = (n - 1) as f64 / 2.0;
    let mut t: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 - m;
            let s = if x == 0.0 {
                2.0 * fc
            } else {
                (std::f64::consts::TAU * fc * x).sin() / (std::f64::consts::PI * x)
            };
            // Blackman window.
            let w = 0.42 - 0.5 * (std::f64::consts::TAU * i as f64 / (n - 1) as f64).cos()
                + 0.08 * (2.0 * std::f64::consts::TAU * i as f64 / (n - 1) as f64).cos();
            s * w
        })
        .collect();
    let sum: f64 = t.iter().sum();
    for v in &mut t {
        *v /= sum;
    }
    t.into_iter().map(|v| v as f32).collect()
}

impl BurstFinder {
    pub fn new(rate: f64) -> Self {
        let fft = FftPlanner::new().plan_fft_forward(N);
        let window: Vec<f32> = (0..N)
            .map(|i| (0.5 - 0.5 * (std::f64::consts::TAU * i as f64 / N as f64).cos()) as f32)
            .collect();
        let decim = ((rate / BURST_RATE).floor() as usize).max(1);
        // A boxcar of `decim` taps twice over (a triangle of 2·decim − 1):
        // its nulls sit on every alias of 0 Hz, so what folds onto the
        // burst's own band (± tens of kHz, filtered again when the burst
        // ends) is well down, at a fifth of the cost of a sharp low-pass.
        // It droops ~1 dB at ±100 kHz, ~3 dB at ±170 kHz.
        let taps = if decim > 1 {
            let n = 2 * decim - 1;
            let t: Vec<f32> = (0..n)
                .map(|i| (decim - (i as isize + 1 - decim as isize).unsigned_abs()) as f32)
                .collect();
            let sum: f32 = t.iter().sum();
            t.into_iter().map(|v| v / sum).collect()
        } else {
            vec![1.0]
        };
        let frame_s = N as f64 / rate;
        BurstFinder {
            rate,
            scratch: vec![Complex32::default(); fft.get_inplace_scratch_len()],
            fft,
            window,
            frame: vec![Complex32::default(); N],
            fill: 0,
            power: vec![0.0; N],
            prev: vec![0.0; N],
            floor: vec![0.0; N],
            frames: 0,
            samples: 0,
            pre: vec![Complex32::default(); ((PRE_S + HOLD_S + 0.005) * rate) as usize + N],
            pre_len: (PRE_S * rate) as usize + N,
            pre_pos: 0,
            pre_fill: 0,
            hold_frames: (HOLD_S / frame_s).ceil() as u64,
            max_frames: (MAX_S / frame_s) as u64,
            open: Vec::new(),
            decim,
            taps: Arc::new(taps),
            opened: 0,
            done: Vec::new(),
            map_every: 0,
            map_acc: vec![0.0; N],
            map_count: 0,
            map_rows: Vec::new(),
            active: vec![false; N],
            prev_active: vec![false; N],
            busy: vec![false; N],
            filters: Default::default(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// The rate bursts come out at.
    pub fn burst_rate(&self) -> f64 {
        self.rate / self.decim as f64
    }

    pub fn bin_hz(&self) -> f64 {
        self.rate / N as f64
    }

    /// Bursts open now.
    pub fn open_count(&self) -> usize {
        self.open.len()
    }

    /// Forget everything (a retune).
    pub fn reset(&mut self) {
        self.fill = 0;
        self.frames = 0;
        self.floor.fill(0.0);
        self.prev.fill(0.0);
        self.pre_pos = 0;
        self.pre_fill = 0;
        self.open.clear();
        self.map_acc.fill(0.0);
        self.map_count = 0;
    }

    /// Close every open burst now (end of a file).
    pub fn flush(&mut self) {
        for o in std::mem::take(&mut self.open) {
            self.finish(o);
        }
    }

    /// The frequency of bin `k` (FFT order), Hz from the centre.
    fn bin_freq(&self, k: usize) -> f64 {
        let k = k as f64;
        let n = N as f64;
        (if k < n / 2.0 { k } else { k - n }) * self.bin_hz()
    }

    /// Bin `k` in FFT order → its index from the lowest frequency.
    #[inline]
    fn shifted(k: usize) -> usize {
        (k + N / 2) % N
    }

    pub fn process(&mut self, iq: &[Complex32]) {
        let mut i = 0;
        while i < iq.len() {
            let take = (N - self.fill).min(iq.len() - i);
            let chunk = &iq[i..i + take];
            // Every open burst takes the samples as they come.
            for o in &mut self.open {
                for &x in chunk {
                    if let Some(d) = &mut o.ddc {
                        d.push(x, &mut o.iq);
                    }
                }
            }
            // Into the ring, in at most two straight copies.
            let len = self.pre.len();
            let mut rest = chunk;
            while !rest.is_empty() {
                let n = (len - self.pre_pos).min(rest.len());
                self.pre[self.pre_pos..self.pre_pos + n].copy_from_slice(&rest[..n]);
                self.pre_pos = (self.pre_pos + n) % len;
                rest = &rest[n..];
            }
            self.pre_fill = (self.pre_fill + chunk.len()).min(len);
            for (j, &x) in chunk.iter().enumerate() {
                self.frame[self.fill + j] = x * self.window[self.fill + j];
            }
            self.fill += take;
            self.samples += take as u64;
            i += take;
            if self.fill == N {
                self.fill = 0;
                self.frame_done();
            }
        }
    }

    fn frame_done(&mut self) {
        self.fft
            .process_with_scratch(&mut self.frame, &mut self.scratch);
        std::mem::swap(&mut self.active, &mut self.prev_active);
        let first = self.frames < INIT_FRAMES;
        for k in 0..N {
            let p = self.frame[k].norm_sqr();
            let s = 0.5 * (p + self.prev[k]);
            self.prev[k] = p;
            self.power[k] = s;
            let f = self.floor[k];
            if first {
                // Start from the first frames' mean.
                self.floor[k] = if self.frames == 0 {
                    p
                } else {
                    f + (p - f) / (self.frames + 1) as f32
                };
                self.active[k] = false;
                continue;
            }
            self.active[Self::shifted(k)] = s > f * THRESHOLD;
        }
        self.frames += 1;
        if self.map_every > 0 {
            for k in 0..N {
                let r = self.power[k] / self.floor[k].max(1e-30);
                let j = Self::shifted(k);
                if r > self.map_acc[j] {
                    self.map_acc[j] = r;
                }
            }
            self.map_count += 1;
            if self.map_count >= self.map_every {
                let db = self
                    .map_acc
                    .iter()
                    .map(|&r| 10.0 * r.max(1e-3).log10())
                    .collect();
                self.map_rows.push(MapRow { db });
                self.map_acc.fill(0.0);
                self.map_count = 0;
            }
        }
        if first {
            return;
        }
        self.track();
        // The floor follows quiet bins only: inside an open burst's range
        // (even where this frame was below the threshold) it only creeps,
        // or a weak burst would lift its own floor.
        self.busy.fill(false);
        for o in &self.open {
            let a = o.lo.saturating_sub(2);
            let b = (o.hi + 2).min(N - 1);
            self.busy[a..=b].fill(true);
        }
        for k in 0..N {
            let j = Self::shifted(k);
            let f = self.floor[k];
            self.floor[k] = if self.active[j] {
                f * FLOOR_CREEP
            } else if self.busy[j] {
                f + (self.power[k] - f) * FLOOR_ALPHA_BUSY
            } else {
                f + (self.power[k] - f) * FLOOR_ALPHA
            };
        }
    }

    /// Match this frame's active clusters to open bursts, open new ones,
    /// close quiet ones.
    fn track(&mut self) {
        let frame = self.frames;
        // Clusters of active bins (shifted order, low to high frequency).
        let mut j = 0;
        while j < N {
            if !self.active[j] {
                j += 1;
                continue;
            }
            let lo = j;
            let mut hi = j;
            let mut gap = 0;
            j += 1;
            while j < N && gap <= CLUSTER_GAP {
                if self.active[j] {
                    hi = j;
                    gap = 0;
                } else {
                    gap += 1;
                }
                j += 1;
            }
            self.cluster(lo, hi, frame);
        }
        // Close what has been quiet long enough, or is too long.
        let mut k = 0;
        while k < self.open.len() {
            let o = &self.open[k];
            if frame - o.last_active > self.hold_frames || frame - o.first_frame > self.max_frames {
                let o = self.open.swap_remove(k);
                self.finish(o);
            } else {
                k += 1;
            }
        }
    }

    fn cluster(&mut self, lo: usize, hi: usize, frame: u64) {
        let unshift = |j: usize| (j + N / 2) % N;
        let peak = (lo..=hi)
            .map(|j| {
                let k = unshift(j);
                self.power[k] / self.floor[k].max(1e-30)
            })
            .fold(0.0f32, f32::max);
        if let Some(o) = self
            .open
            .iter_mut()
            .find(|o| lo <= o.hi + BURST_MARGIN && hi + BURST_MARGIN >= o.lo)
        {
            o.lo = o.lo.min(lo);
            o.hi = o.hi.max(hi);
            if o.last_active != frame {
                o.active_frames += 1;
            }
            o.last_active = frame;
            o.peak = o.peak.max(peak);
            if o.ddc.is_none() && o.active_frames >= CONFIRM_FRAMES {
                // Confirmed: cut it out from its start, out of the ring.
                let mut ddc = Ddc::new(o.mix_hz, self.rate, self.taps.clone(), self.decim);
                let len = self.pre.len();
                let back = ((self.samples - o.start_sample) as usize).min(self.pre_fill);
                o.start_sample = self.samples - back as u64;
                let first = (self.pre_pos + len - back) % len;
                o.iq.reserve((0.1 * self.rate / self.decim as f64) as usize);
                for i in 0..back {
                    ddc.push(self.pre[(first + i) % len], &mut o.iq);
                }
                o.ddc = Some(ddc);
            }
            for j in lo..=hi {
                let k = unshift(j);
                o.shape[j] += self.power[k] / self.floor[k].max(1e-30);
            }
            return;
        }
        if self.open.len() >= MAX_OPEN {
            return;
        }
        // Only where the last frame was active too: single frames of noise
        // do not open bursts.
        if !(lo.saturating_sub(1)..=(hi + 1).min(N - 1)).any(|j| self.prev_active[j]) {
            return;
        }
        // A new burst, centred on this cluster, starting with what was
        // kept from before it.
        let centre_bin = (lo + hi) / 2;
        let mix_hz = self.bin_freq(unshift(centre_bin));
        let mut shape = vec![0.0; N];
        for (j, s) in shape.iter_mut().enumerate().take(hi + 1).skip(lo) {
            let k = unshift(j);
            *s = self.power[k] / self.floor[k].max(1e-30);
        }
        self.opened += 1;
        self.open.push(Open {
            lo,
            hi,
            start_sample: self.samples - self.pre_fill.min(self.pre_len) as u64,
            last_active: frame,
            first_frame: frame,
            active_frames: 1,
            peak,
            shape,
            mix_hz,
            ddc: None,
            iq: Vec::new(),
        });
    }

    fn finish(&mut self, o: Open) {
        // Single blips are noise: a burst needs a few frames' activity.
        if o.active_frames < MIN_FRAMES || o.ddc.is_none() {
            return;
        }
        let bin = self.bin_hz();
        let f = |j: usize| (j as f64 - (N / 2) as f64) * bin;
        // Its extent: the bins within 20 dB of its strongest (keying
        // splatter from a strong OOK burst reaches far wider than the
        // signal itself), and its power-weighted centre over them.
        let peak_shape = o.shape.iter().fold(0.0f32, |a, &b| a.max(b));
        let (lo, hi) = match (
            o.shape.iter().position(|&v| v >= peak_shape * 0.01),
            o.shape.iter().rposition(|&v| v >= peak_shape * 0.01),
        ) {
            (Some(a), Some(b)) => (a, b),
            _ => (o.lo, o.hi),
        };
        let total: f32 = o.shape[lo..=hi].iter().sum();
        let centre = if total > 0.0 {
            o.shape[lo..=hi]
                .iter()
                .enumerate()
                .map(|(j, &s)| f(lo + j) * s as f64)
                .sum::<f64>()
                / total as f64
        } else {
            f((lo + hi) / 2)
        };
        // Drop the hold time's tail beyond a few milliseconds of quiet.
        let frame_s = N as f64 / self.rate;
        let quiet = (self.frames - o.last_active) as f64 * frame_s;
        let keep_tail = 0.012;
        let mut iq = o.iq;
        if quiet > keep_tail {
            let cut = ((quiet - keep_tail) * self.burst_rate()) as usize;
            let n = iq.len().saturating_sub(cut);
            iq.truncate(n);
        }
        // Now its extent is known: centre it and filter it to its own
        // width (the cut-out passed ±200 kHz), so a narrow sensor keeps
        // its SNR.
        let lo_hz = f(lo) - bin / 2.0;
        let hi_hz = f(hi) + bin / 2.0;
        let half =
            ((hi_hz - centre).max(centre - lo_hz) + 10e3).clamp(20e3, 0.4 * self.burst_rate());
        let iq = self.refilter(iq, centre - o.mix_hz, half);
        self.done.push(Burst {
            start_sample: o.start_sample,
            rate: self.burst_rate() as f32,
            mix_hz: centre,
            lo_hz,
            hi_hz,
            centre_hz: centre,
            half_hz: half,
            peak_db: 10.0 * o.peak.max(1e-3).log10(),
            active_frames: o.active_frames,
            iq,
        });
    }

    /// Mix `iq` (at the burst rate) down by `shift_hz` and low-pass it to
    /// ±`half_hz`.
    fn refilter(&mut self, mut iq: Vec<Complex32>, shift_hz: f64, half_hz: f64) -> Vec<Complex32> {
        let rate = self.burst_rate();
        let w = -std::f64::consts::TAU * shift_hz / rate;
        let step = Complex32::new(w.cos() as f32, w.sin() as f32);
        let mut ph = Complex32::new(1.0, 0.0);
        for (i, x) in iq.iter_mut().enumerate() {
            *x *= ph;
            ph *= step;
            if i % 1024 == 0 {
                ph /= ph.norm();
            }
        }
        if half_hz >= 0.39 * rate {
            return iq;
        }
        // Filters by 5 kHz step, kept.
        let key = (half_hz / 5e3).round() as u32;
        let taps = self
            .filters
            .entry(key)
            .or_insert_with(|| {
                let fc = key as f64 * 5e3 / rate;
                let n = ((4.0 / fc) as usize).clamp(31, 255) | 1;
                Arc::new(lowpass(n, fc))
            })
            .clone();
        let m = taps.len();
        let mut out = Vec::with_capacity(iq.len());
        // Zero-phase in effect: skip the filter's delay so the pulses stay
        // where they were.
        let d = m / 2;
        for i in 0..iq.len() {
            let mut acc = Complex32::default();
            let lo = (i + d + 1).saturating_sub(m);
            let hi = (i + d).min(iq.len() - 1);
            for j in lo..=hi {
                acc += iq[j] * taps[i + d - j];
            }
            out.push(acc);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::Noise;

    #[test]
    fn finds_two_bursts_at_once() {
        let rate = 2.5e6;
        let mut iq = vec![Complex32::default(); (rate * 0.3) as usize];
        let mut noise = Noise::new(7);
        for x in iq.iter_mut() {
            *x = noise.gauss(1e-4);
        }
        // Two carriers keyed on and off, overlapping in time.
        for (f, start, a) in [(-400e3, 0.05, 0.01f32), (300e3, 0.08, 0.003)] {
            let s0 = (start * rate) as usize;
            for i in 0..(0.05 * rate) as usize {
                let on = (i / 1250) % 2 == 0; // 500 µs on, 500 off
                if on {
                    let ph = std::f64::consts::TAU * f * (s0 + i) as f64 / rate;
                    iq[s0 + i] += Complex32::new(a * ph.cos() as f32, a * ph.sin() as f32);
                }
            }
        }
        let mut bf = BurstFinder::new(rate);
        bf.process(&iq);
        bf.flush();
        assert_eq!(
            bf.done.len(),
            2,
            "{:?}",
            bf.done
                .iter()
                .map(|b| (
                    b.centre_hz,
                    b.lo_hz,
                    b.hi_hz,
                    b.active_frames,
                    b.peak_db,
                    b.iq.len()
                ))
                .collect::<Vec<_>>()
        );
        let mut c: Vec<f64> = bf.done.iter().map(|b| b.centre_hz).collect();
        c.sort_by(f64::total_cmp);
        assert!((c[0] + 400e3).abs() < 10e3, "{c:?}");
        assert!((c[1] - 300e3).abs() < 10e3, "{c:?}");
        for b in &bf.done {
            // ~50 ms of burst plus up to 12 ms of tail and 2 ms before.
            let dur = b.iq.len() as f64 / b.rate as f64;
            assert!(
                dur > 0.05 && dur < 0.07,
                "{dur} {} {} {}",
                b.centre_hz,
                b.active_frames,
                b.start_sample
            );
        }
    }
}
