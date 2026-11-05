// made by CasualArclamp
//! LoRa demodulation, every preset at once.
//!
//! The channel (1 MS/s) is halved by half-band filters into four branches
//! at exactly the four bandwidths' chip rates: 500, 250, 125 and 62.5 kS/s.
//! At one sample a chip a dechirped symbol is one FFT bin, and a time
//! offset moves the peak as a frequency offset does, with one difference:
//! a chirp late by a fraction τ of a sample is e^(−j2πτ) out of phase from
//! its frequency wrap on (the wrap falls between samples). That phase step
//! is what the rest has to keep right.
//!
//! **Finding frames.** In each branch, each spreading factor 7–12 watches
//! for a preamble: every other window of 2^SF samples is dechirped and
//! FFT'd, and three such windows in a row with their peak in one place
//! (within two bins, as the phase step splits it; and ln N + 3.5 times the
//! mean bin) are a preamble. The windows need not line up with the
//! symbols. Only every other window is looked at, which halves the cost;
//! Meshtastic and MeshCore send 16 preamble chirps.
//!
//! **Synchronising** (the steps of gr-lora_sdr's `frame_sync`, Tapparel et
//! al., GPL-3.0-or-later, worked out afresh at one sample a chip): windows
//! moved by the preamble's bin, so up-chirps peak in bin 0, are late on
//! the symbols by about minus the carrier offset F (in bins). They are
//! dechirped both ways: the two after the preamble are the sync word, the
//! first a down-chirp fits is the first of the 2.25 down-chirps, peaking
//! at about 2F. With F known every window is moved onto the symbols (so no
//! wrap falls mid-window), and the preamble and down-chirps are measured
//! again to a fraction of a bin (a zoomed DFT: the bins next to the peak
//! are bent by the half-bands at the chirp's jump, the main lobe is not):
//! up-chirps peak at τ + C, down-chirps at C − τ, which gives the carrier
//! offset C and the timing τ. The sync word's nibbles are its peaks ÷ 8.
//!
//! **Clock drift.** The transmitter's crystal moves its chip rate with its
//! carrier, so the symbols drift by N·e samples each (e its error): long
//! frames at SF11–12 drift by several samples. The drift is measured as
//! the slope of the preamble windows' peaks, weighed against what the
//! carrier offset implies (if it is all crystal error), and then followed
//! symbol by symbol by a second-order loop on the phase between each
//! symbol's two sides of its wrap (−π × the timing error, whatever the
//! symbol; its peak's position cannot tell timing from frequency). Whole
//! samples move the window, the rest is taken out as a frequency shift.
//!
//! **Symbols.** Each symbol's spectrum is taken, its value decided, the
//! wrap's phase step put right for that value, and decided again. The
//! header block and LDRO symbols carry SF − 2 bits: their bins are taken
//! in fours (the four's power summed). Then [`crate::codec`].
//!
//! Sensitivity (`lora_sweep`, 50 % of frames, 40 bytes, with offset and
//! drift): SF7 −6.5 dB, SF9 −13, SF11 −18.7, SF12 −22 in the signal's
//! bandwidth (Semtech: −7.5, −12.5, −17.5, −20). CPU at 1 MS/s, every
//! preset watched: ~1.5 % of a core on noise.

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

use crate::codec::{self, Decoder, Frame};

/// The four bandwidths, widest first (branch order).
pub const BANDWIDTHS: [f64; 4] = [500e3, 250e3, 125e3, 62.5e3];
pub const SFS: std::ops::RangeInclusive<u8> = 7..=12;
/// The rate the receiver wants: twice the widest bandwidth.
pub const INPUT_RATE: f64 = 1e6;
/// Samples kept per branch (a power of two), for detections to look back.
const RING: usize = 1 << 16;
/// Preamble windows in a row a detector needs (looking at every other).
const HITS: u32 = 3;
/// A frame given up after this many windows without its down-chirps.
const MAX_PRE_WINDOWS: usize = 80;
/// Rows of the dechirped spectrum kept per symbol for the scope.
pub const ROW_BINS: usize = 64;

/// What the receiver looks for: which bandwidths and spreading factors.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// One flag per [`BANDWIDTHS`] entry.
    pub bw: [bool; 4],
    /// One flag per SF 7..=12.
    pub sf: [bool; 6],
}

impl Default for Config {
    fn default() -> Self {
        Config {
            bw: [true; 4],
            sf: [true; 6],
        }
    }
}

/// A frame off the air, decoded or not.
#[derive(Debug, Clone)]
pub struct RxFrame {
    /// Stream time of the first down-chirp, seconds.
    pub at_s: f64,
    pub bw_hz: f64,
    pub sf: u8,
    /// The sync word (0x2B Meshtastic, 0x12 MeshCore/private, 0x34
    /// LoRaWAN), None if its symbols were not where a sync word's are.
    pub sync: Option<u8>,
    pub ldro: bool,
    /// The frame's header and payload; None if the header failed.
    pub frame: Option<Frame>,
    /// SNR in the signal's bandwidth, from the preamble, dB.
    pub snr_db: f32,
    /// Carrier offset from the channel's centre, Hz.
    pub cfo_hz: f64,
    /// Received power, dB relative to full scale.
    pub level_db: f32,
    /// The transmitter's clock against ours, ppm (positive: fast), as
    /// followed through the payload.
    pub drift_ppm: f32,
    /// Payload symbols as demodulated (bins), header block first.
    pub symbols: Vec<u16>,
    /// The dechirped spectrum of each payload symbol, [`ROW_BINS`] bins,
    /// dB, CFO taken out (bin 0 is symbol 0).
    pub rows: Vec<Vec<f32>>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Stats {
    /// Preambles found.
    pub detections: u64,
    /// Synchronised (down-chirps found).
    pub synced: u64,
    pub header_fail: u64,
    pub crc_fail: u64,
    pub decoded: u64,
}

/// Half-band 2:1 decimator: 23 taps, Kaiser β 7 windowed sinc (only the
/// centre and odd taps are non-zero).
struct HalfBand {
    /// Odd taps h[1], h[3], … h[11] (symmetric).
    taps: [f32; 6],
    hist: Vec<Complex32>,
}

impl HalfBand {
    fn new() -> Self {
        let bessel_i0 = |x: f64| {
            let (mut s, mut t) = (1.0, 1.0);
            for k in 1..30 {
                t *= (x / 2.0) * (x / 2.0) / (k * k) as f64;
                s += t;
            }
            s
        };
        let beta = 7.0;
        let m = 11.0;
        let mut taps = [0f32; 6];
        for (i, t) in taps.iter_mut().enumerate() {
            let n = (2 * i + 1) as f64;
            let sinc = (std::f64::consts::PI * n / 2.0).sin() / (std::f64::consts::PI * n / 2.0);
            let w = bessel_i0(beta * (1.0 - (n / m) * (n / m)).max(0.0).sqrt()) / bessel_i0(beta);
            *t = (0.5 * sinc * w) as f32;
        }
        HalfBand {
            taps,
            hist: vec![Complex32::default(); 22],
        }
    }

    /// Appends the decimated samples of `x` to `out`.
    fn process(&mut self, x: &[Complex32], out: &mut Vec<Complex32>) {
        self.hist.extend_from_slice(x);
        let n = self.hist.len();
        // Output m is centred on hist[2m + 11].
        let mut c = 11;
        while c + 11 < n {
            let h = &self.hist;
            let mut acc = h[c] * 0.5;
            for (i, &t) in self.taps.iter().enumerate() {
                let k = 2 * i + 1;
                acc += (h[c - k] + h[c + k]) * t;
            }
            out.push(acc);
            c += 2;
        }
        // Keep what the next call needs: from c − 11.
        self.hist.drain(..c - 11);
    }
}

/// Per spreading factor: chirps, FFT and scratch, shared by the
/// detector and the frame receivers of a branch.
struct SfKit {
    sf: u8,
    n: usize,
    /// conj(up-chirp): multiplying by it dechirps an up-chirp.
    down: Vec<Complex32>,
    /// The up-chirp: dechirps a down-chirp.
    up: Vec<Complex32>,
    fft: Arc<dyn Fft<f32>>,
    buf: Vec<Complex32>,
    /// The dechirped samples before the FFT, kept when `keep` is set (for
    /// DFTs between bins).
    td: Vec<Complex32>,
    keep: bool,
    scratch: Vec<Complex32>,
    pow: Vec<f32>,
    /// Detection threshold, peak over mean bin.
    thresh: f32,
}

impl SfKit {
    fn new(sf: u8, planner: &mut FftPlanner<f32>) -> Self {
        let n = 1usize << sf;
        let up: Vec<Complex32> = (0..n)
            .map(|k| {
                let k = k as f64;
                let ph = 2.0 * std::f64::consts::PI * (k * k / (2.0 * n as f64) - 0.5 * k);
                Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let fft = planner.plan_fft_forward(n);
        let scratch = vec![Complex32::default(); fft.get_inplace_scratch_len()];
        SfKit {
            sf,
            n,
            down: up.iter().map(|c| c.conj()).collect(),
            up,
            fft,
            buf: vec![Complex32::default(); n],
            td: vec![Complex32::default(); n],
            keep: false,
            scratch,
            pow: vec![0.0; n],
            thresh: (n as f32).ln() + 3.5,
        }
    }

    /// Dechirps `n` samples of the ring from `pos` with `refc` (times a
    /// rotation of `-shift` bins), FFTs them into `buf` and powers into
    /// `pow`. Returns (peak bin, peak power, total power).
    fn spectrum(
        &mut self,
        ring: &[Complex32],
        pos: u64,
        refc: Rc,
        shift: f32,
    ) -> (usize, f32, f32) {
        self.spectrum_wrap(ring, pos, refc, shift, None)
    }

    /// As [`Self::spectrum`], and the samples from `wrap.0` on multiplied
    /// by `wrap.1` (undoing the phase step at an up-chirp's wrap).
    fn spectrum_wrap(
        &mut self,
        ring: &[Complex32],
        pos: u64,
        refc: Rc,
        shift: f32,
        wrap: Option<(usize, Complex32)>,
    ) -> (usize, f32, f32) {
        let r = match refc {
            Rc::Down => &self.down,
            Rc::Up => &self.up,
        };
        let start = (pos as usize) & (RING - 1);
        if shift == 0.0 {
            for k in 0..self.n {
                self.buf[k] = ring[(start + k) & (RING - 1)] * r[k];
            }
        } else {
            let step =
                Complex32::from_polar(1.0, -2.0 * std::f32::consts::PI * shift / self.n as f32);
            let mut rot = Complex32::new(1.0, 0.0);
            for k in 0..self.n {
                self.buf[k] = ring[(start + k) & (RING - 1)] * r[k] * rot;
                rot *= step;
                if k & 255 == 255 {
                    rot /= rot.norm();
                }
            }
        }
        if let Some((from, ph)) = wrap {
            for v in &mut self.buf[from.min(self.n)..] {
                *v *= ph;
            }
        }
        if self.keep {
            self.td.copy_from_slice(&self.buf);
        }
        self.fft
            .process_with_scratch(&mut self.buf, &mut self.scratch);
        let (mut best, mut peak, mut total) = (0, 0f32, 0f32);
        for (k, (p, c)) in self.pow.iter_mut().zip(&self.buf).enumerate() {
            *p = c.norm_sqr();
            total += *p;
            if *p > peak {
                peak = *p;
                best = k;
            }
        }
        (best, peak, total)
    }

    /// Peak over the mean of the other bins.
    fn ratio(&self, peak: f32, total: f32) -> f32 {
        peak / ((total - peak) / (self.n - 1) as f32).max(1e-30)
    }

    fn signed(&self, b: usize) -> i64 {
        let b = b as i64;
        if b > self.n as i64 / 2 {
            b - self.n as i64
        } else {
            b
        }
    }

    /// A fine DFT of the dechirped windows at `positions` around bin
    /// `centre` (±1 bin, 1/8 steps, then a parabola through the best
    /// three): the fractional peak and its summed power, the summed
    /// total power, and the windows' complex values at the peak.
    fn fine_peak(
        &mut self,
        ring: &[Complex32],
        positions: &[u64],
        refc: Rc,
        centre: f32,
    ) -> (f32, f32, f32) {
        let r = match refc {
            Rc::Down => &self.down,
            Rc::Up => &self.up,
        };
        let n = self.n;
        // Dechirped windows, then the DFT at each offset.
        let ys: Vec<Vec<Complex32>> = positions
            .iter()
            .map(|&pos| {
                let start = (pos as usize) & (RING - 1);
                (0..n)
                    .map(|k| ring[(start + k) & (RING - 1)] * r[k])
                    .collect()
            })
            .collect();
        let total: f32 = ys.iter().flatten().map(|c| c.norm_sqr()).sum::<f32>() * n as f32;
        let at = |f: f32| -> f32 {
            let step = Complex32::from_polar(1.0, -2.0 * std::f32::consts::PI * f / n as f32);
            ys.iter()
                .map(|y| {
                    let mut rot = Complex32::new(1.0, 0.0);
                    let mut acc = Complex32::default();
                    for (k, &v) in y.iter().enumerate() {
                        acc += v * rot;
                        rot *= step;
                        if k & 255 == 255 {
                            rot /= rot.norm();
                        }
                    }
                    acc.norm_sqr()
                })
                .sum()
        };
        let grid: Vec<(f32, f32)> = (-8..=8)
            .map(|i| centre + i as f32 / 8.0)
            .map(|f| (f, at(f)))
            .collect();
        let i = (0..grid.len())
            .max_by(|&a, &b| grid[a].1.total_cmp(&grid[b].1))
            .unwrap();
        let mut f = grid[i].0;
        if i > 0 && i + 1 < grid.len() {
            let (a, b, c) = (grid[i - 1].1, grid[i].1, grid[i + 1].1);
            let d = a - 2.0 * b + c;
            if d < 0.0 {
                f += 0.5 * (a - c) / d / 8.0;
            }
        }
        (f, at(f), total)
    }
}

#[derive(Clone, Copy)]
enum Rc {
    /// Multiply by conj(up-chirp): up-chirps become tones.
    Down,
    /// Multiply by the up-chirp: down-chirps become tones.
    Up,
}

/// One spreading factor's preamble watch in a branch.
struct Detector {
    kit: usize,
    /// Next window to look at (absolute branch sample).
    next: u64,
    last_peak: Option<usize>,
    hits: u32,
}

enum Phase {
    /// Walking through the preamble and sync word to the down-chirps.
    Pre {
        /// The last few windows: (position, up-chirp peak, was preamble).
        hist: Vec<(u64, i64, bool)>,
        windows: usize,
        weak: u32,
        /// Windows in a row that are neither preamble nor down-chirp (the
        /// sync word is two).
        others: u32,
        /// The first down-chirp's window, once seen (the second must be in
        /// before synchronising).
        down_at: Option<u64>,
    },
    Payload {
        start: u64,
        f_int: i64,
        delta: f32,
        /// How late the payload's first window is on its symbol, samples.
        tau: f64,
        /// How late the window is on the symbols, samples (= bins), and
        /// how much later each symbol: the clock error, followed.
        theta: f64,
        rate: f64,
        k: usize,
        dec: Decoder,
    },
}

/// One frame being received.
struct FrameRx {
    kit: usize,
    pos: u64,
    phase: Phase,
    out: RxFrame,
    done: bool,
}

struct Branch {
    bw: f64,
    on: bool,
    ring: Vec<Complex32>,
    written: u64,
    kits: Vec<SfKit>,
    dets: Vec<Detector>,
    rx: Vec<FrameRx>,
}

pub struct Receiver {
    pub config: Config,
    rate: f64,
    /// RF of the channel's centre (0 if unknown: no clock-drift tracking).
    pub centre_hz: f64,
    stages: Vec<HalfBand>,
    branches: Vec<Branch>,
    bufs: [Vec<Complex32>; 5],
    /// Frames finished since last taken.
    pub frames: Vec<RxFrame>,
    pub stats: Stats,
}

impl Receiver {
    /// `rate` must be [`INPUT_RATE`] (twice the widest bandwidth).
    pub fn new(rate: f64, config: Config) -> Self {
        assert!(
            (rate - INPUT_RATE).abs() < 1.0,
            "LoRa receiver wants {INPUT_RATE} S/s"
        );
        let mut planner = FftPlanner::new();
        let branches = BANDWIDTHS
            .iter()
            .enumerate()
            .map(|(i, &bw)| {
                let kits: Vec<SfKit> = SFS.map(|sf| SfKit::new(sf, &mut planner)).collect();
                let dets = (0..kits.len())
                    .filter(|&k| config.sf[k])
                    .map(|kit| Detector {
                        kit,
                        next: 0,
                        last_peak: None,
                        hits: 0,
                    })
                    .collect();
                Branch {
                    bw,
                    on: config.bw[i],
                    ring: vec![Complex32::default(); RING],
                    written: 0,
                    kits,
                    dets,
                    rx: Vec::new(),
                }
            })
            .collect();
        Receiver {
            config,
            rate,
            centre_hz: 0.0,
            stages: (0..4).map(|_| HalfBand::new()).collect(),
            branches,
            bufs: Default::default(),
            frames: Vec::new(),
            stats: Stats::default(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Forget everything in flight (a retune, a hole in the samples).
    pub fn reset(&mut self) {
        let cfg = self.config.clone();
        let centre = self.centre_hz;
        let stats = self.stats;
        *self = Receiver::new(self.rate, cfg);
        self.centre_hz = centre;
        self.stats = stats;
    }

    pub fn process(&mut self, iq: &[Complex32]) {
        // The half-band chain: input → 500k → 250k → 125k → 62.5k.
        self.bufs[0].clear();
        self.bufs[0].extend_from_slice(iq);
        // Stop halving once no branch below needs it.
        let lowest = (0..4).rev().find(|&i| self.branches[i].on).unwrap_or(0);
        for s in 0..=lowest {
            let (a, b) = self.bufs.split_at_mut(s + 1);
            b[0].clear();
            self.stages[s].process(&a[s], &mut b[0]);
        }
        for bi in 0..4 {
            if !self.branches[bi].on {
                continue;
            }
            let x = std::mem::take(&mut self.bufs[bi + 1]);
            self.branch(bi, &x);
            self.bufs[bi + 1] = x;
        }
    }

    fn branch(&mut self, bi: usize, x: &[Complex32]) {
        // Feed the ring in pieces no longer than the shortest look-back
        // allows, running detectors and receivers as the samples come.
        let chunk = 4096;
        for piece in x.chunks(chunk) {
            let b = &mut self.branches[bi];
            for &s in piece {
                b.ring[(b.written as usize) & (RING - 1)] = s;
                b.written += 1;
            }
            self.detect(bi);
            self.receive(bi);
        }
    }

    fn detect(&mut self, bi: usize) {
        let b = &mut self.branches[bi];
        for d in &mut b.dets {
            let n = b.kits[d.kit].n as u64;
            // While a frame of this SF is being received, rest.
            let busy = b.rx.iter().any(|r| r.kit == d.kit);
            while d.next + n <= b.written {
                let pos = d.next;
                d.next += 2 * n;
                if busy || b.written - pos > (RING as u64) / 2 {
                    d.hits = 0;
                    continue;
                }
                let kit = &mut b.kits[d.kit];
                let (peak, p, total) = kit.spectrum(&b.ring, pos, Rc::Down, 0.0);
                if kit.ratio(p, total) < kit.thresh {
                    d.hits = 0;
                    d.last_peak = None;
                    continue;
                }
                // Within two bins: the windows are not on the symbols, so
                // each holds a chirp's wrap, whose phase step splits the
                // peak in two a bin or so apart.
                let near = d.last_peak.is_some_and(|q| {
                    let diff = (peak as i64 - q as i64).rem_euclid(n as i64);
                    diff <= 2 || diff >= n as i64 - 2
                });
                d.hits = if near { d.hits + 1 } else { 1 };
                d.last_peak = Some(peak);
                if d.hits >= HITS {
                    d.hits = 0;
                    d.last_peak = None;
                    self.stats.detections += 1;
                    if debug() {
                        eprintln!(
                            "detect bw {} sf {} at {:.3} s peak {peak}",
                            b.bw,
                            kit.sf,
                            pos as f64 / b.bw
                        );
                    }
                    // Align so up-chirps peak in bin 0: start one window
                    // on, less the peak.
                    let u = kit.signed(peak);
                    let start = (pos as i64 + n as i64 - u) as u64;
                    let bw = b.bw;
                    b.rx.push(FrameRx {
                        kit: d.kit,
                        pos: start,
                        phase: Phase::Pre {
                            hist: Vec::new(),
                            windows: 0,
                            weak: 0,
                            others: 0,
                            down_at: None,
                        },
                        out: RxFrame {
                            at_s: 0.0,
                            bw_hz: bw,
                            sf: kit.sf,
                            sync: None,
                            ldro: codec::ldro(kit.sf, bw),
                            frame: None,
                            snr_db: 0.0,
                            cfo_hz: 0.0,
                            level_db: 0.0,
                            drift_ppm: 0.0,
                            symbols: Vec::new(),
                            rows: Vec::new(),
                        },
                        done: false,
                    });
                    break;
                }
            }
        }
    }

    fn receive(&mut self, bi: usize) {
        let centre = self.centre_hz;
        let b = &mut self.branches[bi];
        let bw = b.bw;
        for r in &mut b.rx {
            let kit = &mut b.kits[r.kit];
            let n = kit.n as u64;
            // Enough samples for this window (and drift's slack)?
            while !r.done && r.pos + n + 2 <= b.written {
                step(r, kit, &b.ring, bw, centre, &mut self.stats);
            }
        }
        let (done, keep): (Vec<FrameRx>, Vec<FrameRx>) = b.rx.drain(..).partition(|r| r.done);
        b.rx = keep;
        for r in done {
            if debug() {
                eprintln!(
                    "  done bw {} sf {} at {:.3} sync {:?} frame {:?}",
                    b.bw,
                    r.out.sf,
                    r.pos as f64 / b.bw,
                    r.out.sync,
                    r.out.frame.as_ref().map(|f| f.crc_ok)
                );
            }
            if r.out.frame.is_some() {
                self.frames.push(r.out);
            }
        }
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }
}

/// `LORA_DEBUG` set: the receiver says what it does on stderr (read once).
fn debug() -> bool {
    static ON: std::sync::LazyLock<bool> =
        std::sync::LazyLock::new(|| std::env::var_os("LORA_DEBUG").is_some());
    *ON
}

/// Loop gains for the symbol timing (decision-directed, on each symbol's
/// fractional peak): proportional, and integral (the clock error).
const TRACK_P: f64 = 0.08;
const TRACK_I: f64 = 0.002;

/// One window's worth of a frame receiver.
fn step(
    r: &mut FrameRx,
    kit: &mut SfKit,
    ring: &[Complex32],
    bw: f64,
    centre: f64,
    stats: &mut Stats,
) {
    let n = kit.n as i64;
    let mut sync_at = None;
    match &mut r.phase {
        Phase::Pre {
            hist,
            windows,
            weak,
            others,
            down_at,
        } => {
            if let Some(d) = *down_at {
                sync_at = Some(d);
            } else {
                let pos = r.pos;
                r.pos += n as u64;
                *windows += 1;
                let (pu, xu, tu) = kit.spectrum(ring, pos, Rc::Down, 0.0);
                let ru = kit.ratio(xu, tu);
                let (_, xd, td) = kit.spectrum(ring, pos, Rc::Up, 0.0);
                let rd = kit.ratio(xd, td);
                let up_peak = kit.signed(pu);
                if debug() {
                    eprintln!(
                        "sf{} pos {pos} up {up_peak} r {ru:.1} down r {rd:.1}",
                        kit.sf
                    );
                }
                if rd > kit.thresh && rd > ru {
                    // The first down-chirp; synchronise once the second is
                    // in too (the next window).
                    *down_at = Some(pos);
                    return;
                }
                if ru < kit.thresh && rd < kit.thresh {
                    *weak += 1;
                } else {
                    *weak = 0;
                }
                let preamble = ru > kit.thresh && up_peak.abs() <= 1;
                *others = if preamble { 0 } else { *others + 1 };
                hist.push((pos, up_peak, preamble));
                if hist.len() > 8 {
                    hist.remove(0);
                }
                // Not a frame after all (a false start lets this SF's
                // detector rest, so it must not linger).
                if *windows > MAX_PRE_WINDOWS || *weak >= 3 || *others >= 3 {
                    r.done = true;
                }
            }
        }
        Phase::Payload {
            start,
            f_int,
            delta,
            tau,
            theta,
            rate,
            k,
            dec,
        } => {
            // Symbol k's window: moved by the whole samples of the timing
            // error, the rest taken out as a frequency shift.
            let whole = theta.round();
            let pos = (*start as i64 + *k as i64 * n - whole as i64) as u64;
            let frac = *theta - whole;
            let shift = *delta + frac as f32;
            let reduced = dec.reduced();
            let f0 = *f_int;
            kit.keep = true;
            kit.spectrum(ring, pos, Rc::Down, shift);
            let mut v = pick(kit, f0, reduced);
            // A chirp late by a fraction τ of a sample is, from its wrap on,
            // e^(−j2πτ) out of phase with the rest (at one sample a chip the
            // wrap falls between samples): the peak spreads and moves by up to
            // a bin. Knowing the symbol, the step is put right and the
            // symbol decided again.
            // (τ is measured ~2.5 symbols after δ: see `sync`.)
            let lat = *tau - 2.5 * *rate + frac;
            let ph = (lat - lat.round()) as f32;
            if ph.abs() > 0.02 {
                let from = (kit.n as f64 - v as f64 - lat)
                    .ceil()
                    .clamp(0.0, kit.n as f64) as usize;
                let fix = Complex32::from_polar(1.0, 2.0 * std::f32::consts::PI * ph);
                kit.spectrum_wrap(ring, pos, Rc::Down, shift, Some((from, fix)));
                v = pick(kit, f0, reduced);
            }
            // Bin (f_int + v) mod N holds value v.
            let at = |v: usize| ((f0 + v as i64).rem_euclid(n)) as usize;
            let pow = &kit.pow;
            // The timing error left: before its wrap and after it, the
            // symbol's tone (at bin b) is 2πε out of phase for a window ε
            // late, less the πε a frequency error ε would turn between the
            // two halves' centres: arg(Z₂·Z₁*) = −πε, whatever the symbol.
            // (The peak's fraction of a bin cannot tell timing from
            // frequency.) Symbols with too short a part are skipped.
            let b = at(v) as f64;
            let n_w = ((kit.n as f64 - v as f64 - lat).ceil().max(0.0) as usize).min(kit.n);
            let rho = if n_w >= kit.n / 8 && n_w <= kit.n - kit.n / 8 {
                let step = Complex32::from_polar(
                    1.0,
                    (-2.0 * std::f64::consts::PI * b / kit.n as f64) as f32,
                );
                let mut rot = Complex32::new(1.0, 0.0);
                let (mut z1, mut z2) = (Complex32::default(), Complex32::default());
                for (i, &y) in kit.td.iter().enumerate() {
                    if i < n_w {
                        z1 += y * rot;
                    } else {
                        z2 += y * rot;
                    }
                    rot *= step;
                    if i & 255 == 255 {
                        rot /= rot.norm();
                    }
                }
                -((z2 * z1.conj()).arg() as f64) / std::f64::consts::PI
            } else {
                0.0
            };
            kit.keep = false;
            let e = rho.clamp(-0.5, 0.5);
            *rate += TRACK_I * e;
            *theta += *rate + TRACK_P * e;
            r.pos = (*start as i64 + (*k as i64 + 1) * n - theta.round() as i64).max(0) as u64;
            let bin = v as u16;
            r.out.symbols.push(bin);
            // The scope's row: the spectrum in symbol order, max-pooled.
            let per = kit.n / ROW_BINS;
            let row: Vec<f32> = (0..ROW_BINS)
                .map(|i| {
                    let m = (0..per).map(|j| pow[at(i * per + j)]).fold(0f32, f32::max);
                    10.0 * m.max(1e-20).log10()
                })
                .collect();
            r.out.rows.push(row);
            *k += 1;
            if dec.push(codec::symbol_value(bin, kit.sf, reduced)).is_err() {
                stats.header_fail += 1;
                r.done = true;
                return;
            }
            if let Some(total) = dec.total_symbols()
                && *k >= total
            {
                let f = dec.finish();
                if let Some(f) = &f {
                    if f.crc_ok {
                        stats.decoded += 1;
                    } else {
                        stats.crc_fail += 1;
                    }
                }
                r.out.frame = f;
                r.out.drift_ppm = (*rate / n as f64 * 1e6) as f32;
                r.done = true;
            }
        }
    }
    if let Some(d) = sync_at {
        sync(r, kit, ring, d, bw, centre, stats);
    }
}

/// Least-squares slope of y against x, and its standard error (from the
/// residuals; at least 3 points).
fn slope(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    let m = pts.len() as f64;
    if m < 3.0 {
        return None;
    }
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (mx, my) = (sx / m, sy / m);
    let (sxx, sxy) = pts.iter().fold((0.0, 0.0), |a, p| {
        (a.0 + (p.0 - mx).powi(2), a.1 + (p.0 - mx) * (p.1 - my))
    });
    if sxx <= 0.0 {
        return None;
    }
    let b = sxy / sxx;
    let res: f64 = pts
        .iter()
        .map(|p| (p.1 - my - b * (p.0 - mx)).powi(2))
        .sum();
    Some((b, (res / (m - 2.0) / sxx).sqrt()))
}

/// The value a symbol's spectrum (in `kit.pow`) says, bin (f0 + v) mod N
/// holding value v. Reduced rate: groups of four, (v − 1) / 4 = g so
/// v = 4g + 1 … 4g + 4, the group's power summed; then the strongest of
/// its four (for the timing).
fn pick(kit: &SfKit, f0: i64, reduced: bool) -> usize {
    let n = kit.n as i64;
    let at = |v: usize| ((f0 + v as i64).rem_euclid(n)) as usize;
    let pow = &kit.pow;
    if reduced {
        let mut best = (0usize, -1f32);
        for g in 0..kit.n / 4 {
            let e: f32 = (1..=4).map(|j| pow[at((4 * g + j) % kit.n)]).sum();
            if e > best.1 {
                best = (g, e);
            }
        }
        (1..=4)
            .map(|j| (4 * best.0 + j) % kit.n)
            .max_by(|&a, &b| pow[at(a)].total_cmp(&pow[at(b)]))
            .unwrap()
    } else {
        (0..kit.n)
            .max_by(|&a, &b| pow[at(a)].total_cmp(&pow[at(b)]))
            .unwrap()
    }
}

/// At the first down-chirp window `d` (the window grid is late on the
/// symbols by τ' ≈ −F, F the carrier offset in bins).
///
/// The down-chirps peak at C − τ' (C the whole carrier offset) ≈ 2F: that
/// gives F to a bin, and every window is moved by it onto the symbol
/// boundaries. That matters: a window off the boundary holds an up-chirp's
/// wrap, whose phase step (see the payload) biases the peak. On the moved
/// windows, up-chirps peak at τ + C and down-chirps at C − τ (τ now under
/// a sample): both measured to a fraction of a bin, they give C and τ. Then
/// the sync word, the SNR, the clock drift and the payload's start.
fn sync(
    r: &mut FrameRx,
    kit: &mut SfKit,
    ring: &[Complex32],
    d: u64,
    bw: f64,
    centre: f64,
    stats: &mut Stats,
) {
    let n = kit.n as i64;
    let Phase::Pre { hist, .. } = &r.phase else {
        return;
    };
    // The sync word's two windows are the two before the down-chirps; the
    // preamble windows are those before them.
    if hist.len() < 3 {
        r.done = true;
        return;
    }
    let (pre, syncs) = hist.split_at(hist.len() - 2);
    let syncs: Vec<u64> = syncs.iter().map(|w| w.0).collect();
    let all_pre: Vec<u64> = pre.iter().rev().take_while(|w| w.2).map(|w| w.0).collect();
    if all_pre.is_empty() {
        r.done = true;
        return;
    }
    // F, and the windows moved onto the symbols.
    let (pd, _, _) = kit.spectrum(ring, d, Rc::Up, 0.0);
    let f_int = (kit.signed(pd) as f64 / 2.0).round() as i64;
    let mv = |w: u64| (w as i64 + f_int) as u64;
    let pre: Vec<u64> = all_pre.iter().take(4).map(|&w| mv(w)).collect();
    let d = mv(d);
    // Up-chirps (preamble) and down-chirps, to a fraction of a bin.
    let (pu, _, _) = kit.spectrum(ring, pre[0], Rc::Down, 0.0);
    let (p_up, pk, total) = kit.fine_peak(ring, &pre, Rc::Down, kit.signed(pu) as f32);
    let (pd, _, _) = kit.spectrum(ring, d, Rc::Up, 0.0);
    let (p_dn, _, _) = kit.fine_peak(ring, &[d, d + n as u64], Rc::Up, kit.signed(pd) as f32);
    let c = (p_up + p_dn) as f64 / 2.0;
    let tau = (p_up - p_dn) as f64 / 2.0;
    // Payload symbol v peaks at v + p_up: a whole bin and a fraction.
    let f0 = p_up.round() as i64;
    let delta = p_up - f0 as f32;
    // Signal and noise per bin, from the preamble.
    let m = pre.len() as f32;
    let nb = ((total - pk) / (kit.n as f32 - 1.0)).max(1e-30);
    let snr = ((pk - nb) / (kit.n as f32 * nb)).max(1e-6);
    r.out.snr_db = 10.0 * snr.log10();
    r.out.level_db = 10.0
        * (total / (m * kit.n as f32 * kit.n as f32))
            .max(1e-20)
            .log10();
    // The sync word: 8 × each nibble.
    let mut nib = [0u8; 2];
    let mut ok = true;
    for (i, &w) in syncs.iter().enumerate() {
        let (p, _, _) = kit.spectrum(ring, mv(w), Rc::Down, delta);
        let v = (p as i64 - f0).rem_euclid(n);
        let v = if v > n - 4 { v - n } else { v } as f32 / 8.0;
        let q = v.round();
        ok &= (v - q).abs() <= 0.26 && (0.0..16.0).contains(&q);
        nib[i] = q.clamp(0.0, 15.0) as u8;
    }
    r.out.sync = ok.then_some((nib[0] << 4) | nib[1]);
    r.out.cfo_hz = c * bw / kit.n as f64;
    r.out.at_s = d as f64 / bw;
    // A transmitter fast by e sends its symbols N·e samples short, so the
    // window falls later on them by that each symbol, and their peaks move
    // up by as much: measured on the preamble windows (each
    // window's peak, to a fraction of a bin, against its time). If the
    // preamble is too weak for that, the carrier offset says the same if it
    // is all crystal error (not if the channel is off the transmitter's
    // nominal frequency); either way it is followed from there on.
    let mut pts: Vec<(f64, f64)> = Vec::new();
    // (Not the sync word's: their wraps fall mid-window.)
    for (w, q) in all_pre.iter().map(|&w| (w, 0u8)) {
        let w = mv(w);
        let (p, _, _) = kit.spectrum(ring, w, Rc::Down, 0.0);
        let (x, _, _) = kit.fine_peak(ring, &[w], Rc::Down, p as f32);
        let v = x as f64 - (f0 + 8 * q as i64) as f64;
        let v = v - (v / n as f64).round() * n as f64;
        pts.push(((w as i64 - d as i64) as f64 / n as f64, v));
    }
    // The two weighted by how far each can be trusted: the slope by its
    // own scatter, the carrier offset as if the channel could be ~2 kHz off
    // the transmitter's nominal frequency (10 ppm if the RF is unknown).
    let (prior, prior_sd) = if centre > 0.0 {
        (n as f64 * r.out.cfo_hz / centre, n as f64 * 2e3 / centre)
    } else {
        (0.0, n as f64 * 10e-6)
    };
    let rate = match slope(&pts) {
        Some((s, sd)) if pts.len() >= 3 => {
            let (wp, wm) = (1.0 / prior_sd.powi(2), 1.0 / sd.max(1e-6).powi(2));
            (prior * wp + s * wm) / (wp + wm)
        }
        _ => prior,
    };
    if debug() {
        eprintln!(
            "f_int {f_int} p_up {p_up:.3} p_dn {p_dn:.3} tau {tau:.3} rate {rate:.4} prior {prior:.4} pts {pts:.2?}"
        );
    }
    stats.synced += 1;
    // The payload starts 2.25 symbols after the first down-chirp.
    let start = d as i64 + 2 * n + n / 4;
    r.phase = Phase::Payload {
        start: start as u64,
        f_int: f0,
        delta,
        tau,
        // p_up was measured on the preamble, ~6.75 symbols before the
        // payload (τ, from the preamble and the down-chirps, ~4.25): the
        // drift since.
        theta: 6.75 * rate,
        rate,
        k: 0,
        dec: Decoder::new(kit.sf, r.out.ldro),
    };
    r.pos = start as u64;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{self, Noise, Tx};

    /// Frames of `tx` with `payload` at `snr_db` (in the signal's
    /// bandwidth), `freq` off centre and a crystal `ppm` off; returns what
    /// the receiver made of them.
    fn run(
        tx: &Tx,
        payload: &[u8],
        snr_db: f64,
        freq: f64,
        ppm: f64,
        seed: u64,
    ) -> (Vec<RxFrame>, Stats) {
        let fs = INPUT_RATE;
        let mut x = Vec::new();
        let gap = (0.05 * fs) as usize;
        let mut at = gap;
        for _ in 0..2 {
            sim::modulate(tx, payload, fs, freq, ppm, 1.0, at, &mut x);
            at = x.len() + gap;
        }
        x.resize(at + gap, Complex32::default());
        // Noise in the signal's bandwidth at the SNR: its density over fs.
        let p = 1.0 / 10f64.powf(snr_db / 10.0) * fs / tx.bw;
        Noise::new(seed).add(&mut x, p);
        let peak = x.iter().map(|c| c.norm()).fold(0f32, f32::max);
        for c in &mut x {
            *c /= peak;
        }
        let mut rx = Receiver::new(fs, Config::default());
        rx.centre_hz = 869.525e6;
        for b in x.chunks(10_000) {
            rx.process(b);
        }
        (std::mem::take(&mut rx.frames), rx.stats())
    }

    fn decoded(frames: &[RxFrame], payload: &[u8]) -> usize {
        frames
            .iter()
            .filter(|f| {
                f.frame
                    .as_ref()
                    .is_some_and(|f| f.crc_ok && f.payload == payload)
            })
            .count()
    }

    #[test]
    fn every_preset_decodes() {
        let payload: Vec<u8> = (0..40u8).map(|i| i.wrapping_mul(13) ^ 0x77).collect();
        for &(bw, sf) in &[
            (250e3, 11),
            (250e3, 7),
            (125e3, 12),
            (62.5e3, 8),
            (500e3, 9),
            (62.5e3, 12),
            (125e3, 10),
        ] {
            let tx = Tx {
                bw,
                sf,
                cr: 1,
                sync: 0x2b,
                preamble: 16,
            };
            let (f, st) = run(&tx, &payload, 10.0, 1500.0, 0.0, 1);
            assert_eq!(decoded(&f, &payload), 2, "bw {bw} sf {sf}: {st:?}");
            assert!(
                f.iter()
                    .all(|f| f.sync == Some(0x2b) && f.sf == sf && f.bw_hz == bw)
            );
        }
    }

    #[test]
    fn offsets_and_clock_error() {
        let payload: Vec<u8> = (0..200u8).collect();
        // LongFast, 10 ppm fast: 8.7 kHz and 0.02 chip a symbol.
        let tx = Tx::meshtastic("LongFast");
        let (f, st) = run(&tx, &payload, 0.0, 8_695.0, 10.0, 2);
        assert_eq!(decoded(&f, &payload), 2, "{st:?}");
        let cfo = f[0].cfo_hz;
        assert!((cfo - 8_695.0).abs() < 30.0, "cfo {cfo}");
        // Slow too, and a narrow preset whose symbols are long (LDRO).
        let tx = Tx::meshcore(62.5e3, 12, 8);
        let (f, st) = run(&tx, &payload[..60], 0.0, -4_000.0, -8.0, 3);
        assert_eq!(decoded(&f, &payload[..60]), 2, "{st:?}");
    }

    #[test]
    fn near_sensitivity() {
        // SF11 is specified to -17.5 dB; this receiver within ~2 dB.
        let payload: Vec<u8> = (0..30u8).collect();
        let tx = Tx::meshtastic("LongFast");
        let (f, st) = run(&tx, &payload, -15.0, 300.0, 1.0, 4);
        assert_eq!(decoded(&f, &payload), 2, "{st:?}");
    }

    #[test]
    fn noise_alone_finds_nothing() {
        let mut x = vec![Complex32::default(); 2_000_000];
        Noise::new(9).add(&mut x, 0.01);
        let mut rx = Receiver::new(INPUT_RATE, Config::default());
        for b in x.chunks(10_000) {
            rx.process(b);
        }
        assert!(rx.frames.is_empty(), "{:?}", rx.stats());
    }
}

#[cfg(test)]
mod debug {
    use super::*;
    use crate::sim::{self, Tx};

    /// `LORA_BW=125000 LORA_SF=12 LORA_PPM=0 cargo test --release -p
    /// declab-lora symbol_errors -- --ignored --nocapture`: where the
    /// demodulated symbols differ from those sent.
    #[test]
    #[ignore]
    fn symbol_errors() {
        let get = |k: &str, d: f64| {
            std::env::var(k)
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(d)
        };
        let tx = Tx {
            bw: get("LORA_BW", 125e3),
            sf: get("LORA_SF", 12.0) as u8,
            cr: get("LORA_CR", 1.0) as u8,
            sync: 0x2b,
            preamble: 16,
        };
        let ppm = get("LORA_PPM", 0.0);
        let payload: Vec<u8> = (0..get("LORA_LEN", 40.0) as u8)
            .map(|i| i.wrapping_mul(29) ^ 0x5c)
            .collect();
        let mut x = vec![Complex32::default(); 50_000];
        let at = 50_000 + get("LORA_AT", 0.0) as usize;
        sim::modulate(
            &tx,
            &payload,
            INPUT_RATE,
            get("LORA_FREQ", 1500.0),
            ppm,
            1.0,
            at,
            &mut x,
        );
        x.resize(x.len() + 50_000, Complex32::default());
        if let Ok(snr) = std::env::var("LORA_SNR") {
            let snr: f64 = snr.parse().unwrap();
            let p = 1.0 / 10f64.powf(snr / 10.0) * INPUT_RATE / tx.bw;
            sim::Noise::new(get("LORA_SEED", 1.0) as u64).add(&mut x, p);
        }
        let mut rx = Receiver::new(INPUT_RATE, Config::default());
        rx.centre_hz = 869.525e6;
        for b in x.chunks(10_000) {
            rx.process(b);
        }
        let want = codec::encode(&payload, tx.sf, tx.cr, true, codec::ldro(tx.sf, tx.bw));
        for f in &rx.frames {
            println!(
                "cfo {:.1} snr {:.1} sync {:?} frame {:?}",
                f.cfo_hz,
                f.snr_db,
                f.sync,
                f.frame.as_ref().map(|f| f.crc_ok)
            );
            for (k, (a, b)) in f.symbols.iter().zip(&want).enumerate() {
                if a != b {
                    println!("  symbol {k}: got {a} sent {b}");
                }
            }
        }
    }
}

#[cfg(test)]
mod rho_check {
    use super::*;

    #[test]
    #[ignore]
    fn rho_on_tones() {
        let n = 2048;
        let mut p = FftPlanner::new();
        let fft = p.plan_fft_forward(n);
        for rho in [-0.4f64, -0.2, 0.0, 0.2, 0.4] {
            let mut b: Vec<Complex32> = (0..n)
                .map(|k| {
                    Complex32::from_polar(
                        1.0,
                        (2.0 * std::f64::consts::PI * (300.0 + rho) * k as f64 / n as f64) as f32,
                    )
                })
                .collect();
            fft.process(&mut b);
            let (xm, x0, xp) = (b[299], b[300], b[301]);
            let est = ((xm - xp) / (x0 * 2.0 - xm - xp)).re;
            println!("rho {rho} est {est:.3}");
        }
    }
}
