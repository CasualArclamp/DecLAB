// made by CasualArclamp
//! The TETRA downlink receiver: channel IQ at 72 kS/s (four samples a
//! symbol) in, [`Event`]s out.
//!
//! - Front end: the AFC's mixer, then the receive root-raised-cosine
//!   (α = 0.35, clause 5.5) as the matched filter.
//! - Hunting: the 19-symbol synchronization training sequence (clause
//!   9.4.4.3.4) is looked for in the differential products z[n]·z*[n−4]
//!   with a normalised correlator (magnitude, so a frequency offset only
//!   turns it). A candidate whose fit passes the SNR gate and whose BSCH
//!   passes its CRC locks the receiver; the SYNC PDU gives the timeslot,
//!   frame and multiframe numbers and the colour code that scrambles
//!   everything else.
//! - Locked: the downlink is continuous, so every 255 symbols is a slot.
//!   Each is read at its symbol times (cubic interpolation), classified by
//!   its training sequences (normal 1, normal 2 or synchronization, plus
//!   training sequence 3 at its edges), and its fit gives the residual
//!   phase step a symbol (AFC), the timing error (early/late), the eye SNR
//!   and the soft-bit scale.
//! - Soft bits: differential detection of each symbol pair; for
//!   B(2k−1) the imaginary part of z_k·z*_{k−1}, for B(2k) the real part
//!   (table 5.1), scaled to LLRs by the fit.
//!
//! Logical channels follow clause 9 and the AACH: SCH/F or TCH in a burst
//! with training 1, two half-slot channels with training 2, the BSCH and a
//! BNCH or SCH/HD in a synchronization burst.

use num_complex::Complex32;

use super::coding::{self, Chan};
use super::mac::{self, AccessAssign, Address, ChanAlloc, Pdu, Signal, Sync, SysInfo};
use super::{ALPHA, Burst, SLOT_BITS, SLOT_SYMBOLS, field, freq_correction, transition};
use super::{train_n, train_p, train_q, train_y};
use crate::fsk4::{Iq, rrc};

pub const RATE: f64 = 72_000.0;
const SPS: usize = 4;
/// A slot in samples.
const SLOT: f64 = (SLOT_SYMBOLS * SPS) as f64;
/// Hunting: the normalised correlation a candidate must reach.
const ACQUIRE: f32 = 0.6;
/// Locked: a slot whose training fits worse than this is a miss.
const TRACK: f32 = 0.35;
/// Slots missed in a row before the receiver hunts again (two frames).
const LOST_AFTER: u32 = 8;
/// The SNR gate's default, dB: the eye SNR of the training fit in the
/// differential products. Measured (`tetra_sweep`, 2026-10-30): two minutes
/// of noise gave 53 000 candidates, the best fitting at 0.0 dB (one of them
/// passed the BSCH's CRC by chance, which the gate stops).
pub const SNR_GATE_DB: f32 = 3.0;
/// Locked slots are held to the gate less this: a weak carrier's slots dip
/// below the acquisition gate, noise fits far lower (about −12 dB).
const TRACK_MARGIN_DB: f32 = 6.0;
const AFC_GAIN: f64 = 0.2;
const TIMING_GAIN: f64 = 0.3;
/// A residual step (radians a symbol) above which coherent detection
/// takes the training's estimate out first.
const COARSE_STEP: f32 = 0.1;
/// Samples demodulated at a time: a quarter slot.
const AFC_SLICE: usize = 255;

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A SYNC PDU from a BSCH.
    Sync(Sync),
    SysInfo(SysInfo),
    /// A MAC-RESOURCE (or reassembled fragments) on timeslot `tn`: its
    /// address and the upper-layer PDU (`None` when encrypted or not
    /// followed).
    Signal {
        tn: u8,
        address: Address,
        encrypted: bool,
        chan: Option<ChanAlloc>,
        signal: Option<Signal>,
    },
    /// TCH/S: the descrambled type-4 soft bits of a full slot (432) or of
    /// block 2 alone (216, the first half stolen), for the speech decoder.
    Traffic {
        tn: u8,
        usage_marker: Option<u8>,
        half: bool,
        llr: Vec<f32>,
    },
}

#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub slots: u64,
    pub slots_bad: u64,
    /// BSCHs decoded.
    pub syncs: u64,
    /// Sync candidates while hunting (for the gate measurement).
    pub candidates: u64,
    pub gated: u64,
    pub crc_fail: u64,
    pub blocks_ok: u64,
    pub traffic_slots: u64,
    /// The latest slot's training fit, eye SNR (dB), and the offset.
    pub quality: f32,
    pub eye_snr_db: f32,
    /// The latest hunting candidate's eye SNR (dB), and the highest seen.
    pub sync_snr_db: f32,
    pub sync_snr_max: f32,
    pub offset_hz: f32,
    pub locked: bool,
    pub tn: u8,
    pub frame: u8,
    pub multiframe: u8,
    pub mcc: Option<u16>,
    pub mnc: u16,
    pub colour: u8,
}

/// The cell as learnt from its SYNC.
struct Cell {
    scramble: Vec<u8>,
}

struct Lock {
    /// Absolute sample position of the next slot's symbol 0.
    t0: f64,
    misses: u32,
    /// Timeslot (0–3), frame (1–18), multiframe (1–60) of the next slot.
    tn: u8,
    frame: u8,
    multiframe: u8,
}

/// The known part of a burst for the training fit: (symbol, transition in
/// units of π/4).
struct Pattern(Vec<(usize, i32)>);

impl Pattern {
    fn of(fields: &[(usize, &[u8])]) -> Pattern {
        let mut v = Vec::new();
        for &(bit0, bits) in fields {
            for (i, d) in bits.chunks(2).enumerate() {
                // Bit index (0-based) 2k−2 starts symbol k.
                v.push(((bit0 + 2 * i) / 2 + 1, transition(d[0], d[1])));
            }
        }
        Pattern(v)
    }
}

/// A TM-SDU being reassembled: the first piece's address, encryption flag
/// and channel allocation, and the bits so far.
type Fragments = (Address, bool, Option<ChanAlloc>, Vec<u8>);

pub struct Receiver {
    pub snr_gate_db: f32,
    afc_step: f64,
    afc_phase: f64,
    taps: Vec<f32>,
    ring: Vec<Iq>,
    ring_pos: usize,
    /// Matched-filtered samples; `z[0]` is absolute sample `base`.
    z: Vec<Iq>,
    /// z[n]·z*[n−4] and its magnitude, aligned with `z`.
    d: Vec<Iq>,
    dn: Vec<f32>,
    base: u64,
    scan: u64,
    lock: Option<Lock>,
    cell: Option<Cell>,
    /// Fragments being reassembled, per timeslot: the address and pieces.
    frag: [Option<Fragments>; 4],
    /// Each timeslot's latest traffic usage marker from the AACH.
    traffic: [Option<u8>; 4],
    pat_y: Pattern,
    pat_n: Pattern,
    pat_p: Pattern,
    /// Symbols 1..=19 of the synchronization training for hunting.
    hunt_y: Vec<Complex32>,
    bsch_scramble: Vec<u8>,
    /// Set by `signalling`: a MAC-RESOURCE said the second half slot is
    /// stolen too.
    second_stolen: bool,
    /// Coherent detection (the default) or differential (for comparison).
    pub coherent: bool,
    /// The latest coherent slot's carrier phase slope, radians a symbol.
    slope: Option<f32>,
    pub stats: Stats,
    events: Vec<Event>,
    tap: Option<Vec<Iq>>,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

fn rot(d: i32) -> Complex32 {
    Complex32::from_polar(1.0, d as f32 * std::f32::consts::FRAC_PI_4)
}

impl Receiver {
    pub fn new() -> Receiver {
        let q = train_q();
        let common: Vec<(usize, &[u8])> = vec![
            (field::Q_HEAD.start, &q[10..]),
            (field::Q_TAIL.start, &q[..10]),
        ];
        let mk = |extra: Vec<(usize, &[u8])>| {
            let mut f = common.clone();
            f.extend(extra);
            Pattern::of(&f)
        };
        let (n, p, y, fc) = (train_n(), train_p(), train_y(), freq_correction());
        let pat_y = mk(vec![
            (field::SDB_TRAIN.start, &y),
            (field::SDB_FREQ.start, &fc),
        ]);
        let pat_n = mk(vec![(field::NDB_TRAIN.start, &n)]);
        let pat_p = mk(vec![(field::NDB_TRAIN.start, &p)]);
        let hunt_y = y.chunks(2).map(|d| rot(-transition(d[0], d[1]))).collect();
        Receiver {
            snr_gate_db: SNR_GATE_DB,
            afc_step: 0.0,
            afc_phase: 0.0,
            taps: rrc(ALPHA, SPS, 8),
            ring: vec![Iq::new(0.0, 0.0); 2 * (16 * SPS + 1)],
            ring_pos: 0,
            z: Vec::new(),
            d: Vec::new(),
            dn: Vec::new(),
            base: 0,
            scan: 0,
            lock: None,
            cell: None,
            frag: Default::default(),
            traffic: [None; 4],
            pat_y,
            pat_n,
            pat_p,
            hunt_y,
            bsch_scramble: coding::scrambling(0, 120),
            second_stolen: false,
            coherent: true,
            slope: None,
            stats: Stats::default(),
            events: Vec::new(),
            tap: None,
        }
    }

    /// Keep each slot's symbols (for a constellation scope).
    pub fn set_tap(&mut self, on: bool) {
        self.tap = on.then(Vec::new);
    }

    pub fn take_tap(&mut self) -> Vec<Iq> {
        self.tap.as_mut().map(std::mem::take).unwrap_or_default()
    }

    pub fn reset(&mut self) {
        let (gate, coherent) = (self.snr_gate_db, self.coherent);
        let tap = self.tap.is_some();
        *self = Receiver::new();
        (self.snr_gate_db, self.coherent) = (gate, coherent);
        self.set_tap(tap);
    }

    pub fn process(&mut self, iq: &[Iq], out: &mut Vec<Event>) {
        // In short slices, so the AFC's corrections reach the mixer within
        // a slot (a whole host block late, the loop overshoots).
        for s in iq.chunks(AFC_SLICE) {
            self.slice(s);
        }
        out.append(&mut self.events);
    }

    fn slice(&mut self, iq: &[Iq]) {
        let n = self.taps.len();
        for &x in iq {
            // The AFC's mixer, then the matched filter (taps symmetric).
            let m = x * Complex32::from_polar(1.0, -self.afc_phase as f32);
            self.afc_phase = (self.afc_phase + self.afc_step) % std::f64::consts::TAU;
            self.ring[self.ring_pos] = m;
            self.ring[self.ring_pos + n] = m;
            self.ring_pos = (self.ring_pos + 1) % n;
            let w = &self.ring[self.ring_pos..self.ring_pos + n];
            let mut acc = Iq::new(0.0, 0.0);
            for (s, &h) in w.iter().zip(&self.taps) {
                acc += s * h;
            }
            self.z.push(acc);
            // The differential product with the sample a symbol back, and
            // its magnitude, for hunting.
            let n = self.z.len();
            let d = if n > SPS {
                acc * self.z[n - 1 - SPS].conj()
            } else {
                Iq::new(0.0, 0.0)
            };
            self.d.push(d);
            self.dn.push(d.norm());
        }
        loop {
            let progressed = if self.lock.is_some() {
                self.track()
            } else {
                self.hunt()
            };
            if !progressed {
                break;
            }
        }
        // Keep a little history behind whatever is needed next.
        let keep_from = match &self.lock {
            Some(l) => (l.t0 as u64).saturating_sub(16),
            None => self.scan.saturating_sub(SLOT as u64 + 64),
        };
        if keep_from > self.base + 4 * SLOT as u64 {
            let drop = (keep_from - self.base) as usize;
            self.z.drain(..drop);
            self.d.drain(..drop);
            self.dn.drain(..drop);
            self.base += drop as u64;
        }
    }

    fn end(&self) -> u64 {
        self.base + self.z.len() as u64
    }

    /// z at absolute (fractional) sample position `t`, by 4-point cubic
    /// (Catmull-Rom) interpolation.
    fn at(&self, t: f64) -> Iq {
        let i = t.floor();
        let mu = (t - i) as f32;
        let i = i as i64 - self.base as i64;
        let g = |k: i64| -> Iq {
            if k < 0 || k as usize >= self.z.len() {
                Iq::new(0.0, 0.0)
            } else {
                self.z[k as usize]
            }
        };
        let (p0, p1, p2, p3) = (g(i - 1), g(i), g(i + 1), g(i + 2));
        let a = p1 * 1.5 - p2 * 1.5 + p3 * 0.5 - p0 * 0.5;
        let b = p0 - p1 * 2.5 + p2 * 2.0 - p3 * 0.5;
        let c = (p2 - p0) * 0.5;
        ((a * mu + b) * mu + c) * mu + p1
    }

    /// Hunting for a synchronization burst. Returns whether to go on.
    fn hunt(&mut self) -> bool {
        // The 19 training symbols span symbols 107..126 of a slot; a slot
        // must fit after the candidate for the BSCH to be read.
        let need = 20 * SPS as u64 + (SLOT as u64) + 8;
        let start = self.scan.max(self.base + SPS as u64 + 2);
        if start + need > self.end() {
            self.scan = start;
            return false;
        }
        let last = self.end() - need;
        let mut best: Option<(u64, f32)> = None;
        let mut t = start;
        while t <= last {
            let c = self.hunt_corr(t as f64);
            if c > ACQUIRE {
                // Climb to the local peak within half a symbol.
                let (mut bt, mut bc) = (t, c);
                for u in t + 1..=(t + 2 * SPS as u64).min(last) {
                    let cu = self.hunt_corr(u as f64);
                    if cu > bc {
                        (bt, bc) = (u, cu);
                    }
                }
                best = Some((bt, bc));
                break;
            }
            t += 1;
        }
        let Some((tau, _)) = best else {
            self.scan = last + 1;
            return false;
        };
        self.stats.candidates += 1;
        // Parabolic refinement of the peak.
        let (cm, c0, cp) = (
            self.hunt_corr(tau as f64 - 1.0),
            self.hunt_corr(tau as f64),
            self.hunt_corr(tau as f64 + 1.0),
        );
        let den = cm - 2.0 * c0 + cp;
        let frac = if den.abs() > 1e-9 {
            (0.5 * (cm - cp) / den).clamp(-0.5, 0.5)
        } else {
            0.0
        };
        // τ is symbol 107's sample: the slot's symbol 0 is 107 symbols back.
        let t0 = tau as f64 + f64::from(frac) - 107.0 * SPS as f64;
        if t0 < self.base as f64 + 2.0 {
            self.scan = tau + 1;
            return true;
        }
        let fit = self.fit(t0, Burst::Sync);
        self.stats.sync_snr_db = fit.snr_db;
        if self.stats.candidates == 1 || fit.snr_db > self.stats.sync_snr_max {
            self.stats.sync_snr_max = fit.snr_db;
        }
        if fit.snr_db < self.snr_gate_db {
            self.stats.gated += 1;
            self.scan = tau + 1;
            return true;
        }
        // The frequency error the fit saw: correct it before the BSCH.
        let llr = self.soft(t0, &fit);
        let sb: Vec<f32> = llr[field::SDB_SB].to_vec();
        match coding::decode(Chan::Bsch, &sb, &self.bsch_scramble).and_then(|b| Sync::parse(&b)) {
            Some(s) => {
                self.stats.syncs += 1;
                // Acquisition: take the whole of the training's estimate.
                self.afc_by(fit.step, 1.0);
                self.lock = Some(Lock {
                    t0,
                    misses: 0,
                    tn: s.tn,
                    frame: s.frame,
                    multiframe: s.multiframe,
                });
                self.stats.locked = true;
                self.scan = tau + 1;
                true
            }
            None => {
                self.stats.crc_fail += 1;
                self.scan = tau + 1;
                true
            }
        }
    }

    /// The normalised correlation of the differential products with the
    /// synchronization training, symbol 107 at `t`.
    fn hunt_corr(&self, t: f64) -> f32 {
        let i0 = t as i64 - self.base as i64 + SPS as i64;
        let last = i0 + (self.hunt_y.len() as i64 - 1) * SPS as i64;
        if i0 < 0 || last as usize >= self.d.len() {
            return 0.0;
        }
        let mut acc = Complex32::new(0.0, 0.0);
        let mut mag = 0f32;
        for (k, r) in self.hunt_y.iter().enumerate() {
            let a = i0 as usize + k * SPS;
            acc += self.d[a] * r;
            mag += self.dn[a];
        }
        if mag > 0.0 { acc.norm() / mag } else { 0.0 }
    }

    /// Locked: decode the next slot if it is in. Returns whether one was.
    fn track(&mut self) -> bool {
        let Some(l) = &self.lock else { return false };
        let t0 = l.t0;
        if (t0 + SLOT + 2.0 * SPS as f64 + 4.0) as u64 >= self.end() {
            return false;
        }
        let (tn, frame) = (l.tn, l.frame);
        // Classify by the best of the three training patterns.
        let fits = [Burst::Full, Burst::Half, Burst::Sync].map(|b| self.fit(t0, b));
        let fit = fits
            .iter()
            .max_by(|a, b| a.quality.total_cmp(&b.quality))
            .copied()
            .expect("three fits");
        self.stats.slots += 1;
        self.stats.quality = fit.quality;
        self.stats.eye_snr_db = fit.snr_db;
        let mut t_next = t0 + SLOT;
        // Noise after a carrier ends can fit the training now and then: the
        // gate keeps it from steering the AFC and timing.
        let good = fit.quality >= TRACK && fit.snr_db >= self.snr_gate_db - TRACK_MARGIN_DB;
        if good {
            // Early/late on the fit's quality, a sample either side.
            let (qe, ql) = (
                self.fit(t0 - 1.0, fit.burst).quality,
                self.fit(t0 + 1.0, fit.burst).quality,
            );
            let den = qe - 2.0 * fit.quality + ql;
            if den < -1e-6 {
                let d = f64::from((0.5 * (qe - ql) / den).clamp(-0.5, 0.5));
                t_next += TIMING_GAIN * d;
            }
            let llr = self.soft(t0, &fit);
            self.afc(self.slope.unwrap_or(fit.step));
            self.slot(tn, frame, &fit, &llr);
        } else {
            self.stats.slots_bad += 1;
        }
        let lost = {
            let l = self.lock.as_mut().expect("locked");
            let (tn, frame, mn) = (l.tn, l.frame, l.multiframe);
            l.misses = if good { 0 } else { l.misses + 1 };
            l.t0 = t_next;
            l.tn = (tn + 1) % 4;
            if l.tn == 0 {
                l.frame = frame % 18 + 1;
                l.multiframe = if l.frame == 1 { mn % 60 + 1 } else { mn };
            }
            self.stats.tn = l.tn;
            self.stats.frame = l.frame;
            self.stats.multiframe = l.multiframe;
            l.misses > LOST_AFTER
        };
        if lost {
            self.lock = None;
            self.stats.locked = false;
            self.scan = t_next as u64;
            self.frag = Default::default();
        }
        true
    }

    fn afc(&mut self, step: f32) {
        self.afc_by(step, AFC_GAIN);
    }

    fn afc_by(&mut self, step: f32, gain: f64) {
        // A residual turn of `step` radians a symbol.
        let hz = f64::from(step) * super::SYMBOL_RATE / std::f64::consts::TAU;
        self.afc_step += gain * std::f64::consts::TAU * hz / RATE;
        self.stats.offset_hz = (self.afc_step * RATE / std::f64::consts::TAU) as f32;
    }

    /// The training fit of the slot whose symbol 0 is at `t0`.
    fn fit(&self, t0: f64, burst: Burst) -> Fit {
        let pat = match burst {
            Burst::Full => &self.pat_n,
            Burst::Half => &self.pat_p,
            Burst::Sync => &self.pat_y,
        };
        let mut acc = Complex32::new(0.0, 0.0);
        let mut mag = 0f32;
        let ds: Vec<Complex32> = pat
            .0
            .iter()
            .map(|&(k, d)| {
                let p =
                    self.at(t0 + (k * SPS) as f64) * self.at(t0 + ((k - 1) * SPS) as f64).conj();
                p * rot(-d)
            })
            .collect();
        for d in &ds {
            acc += d;
            mag += d.norm();
        }
        let n = ds.len() as f32;
        let quality = if mag > 0.0 { acc.norm() / mag } else { 0.0 };
        let step = acc.arg();
        let amp = acc.norm() / n;
        let r = Complex32::from_polar(1.0, -step);
        let var = ds.iter().map(|d| (d * r - amp).norm_sqr()).sum::<f32>() / n;
        let snr_db = 10.0 * (amp * amp / var.max(1e-12)).log10();
        Fit {
            burst,
            quality,
            step,
            amp,
            var: var.max(1e-12),
            snr_db,
        }
    }

    /// The slot's 510 LLRs, from its 256 symbols (the previous slot's last
    /// one first).
    fn soft(&mut self, t0: f64, fit: &Fit) -> Vec<f32> {
        let s: Vec<Iq> = (0..=SLOT_SYMBOLS)
            .map(|k| self.at(t0 + (k * SPS) as f64))
            .collect();
        if let Some(tap) = &mut self.tap {
            tap.extend_from_slice(&s[1..]);
        }
        if self.coherent {
            // The carrier phase is followed symbol by symbol; its slope over
            // the slot is the AFC's error (all 255 symbols, where the
            // training fit's step rests on about 30). Only a large step (the
            // AFC not yet in, about 300 Hz and more) is taken out first:
            // the training's estimate is too noisy to ramp across a slot.
            let pre = if fit.step.abs() > COARSE_STEP {
                fit.step
            } else {
                0.0
            };
            let s: Vec<Iq> = s
                .iter()
                .enumerate()
                .map(|(k, &z)| z * Complex32::from_polar(1.0, -pre * k as f32))
                .collect();
            let (llr, slope) = coherent(&s);
            self.slope = Some(slope + pre);
            llr
        } else {
            self.slope = None;
            differential(&s, fit)
        }
    }

    /// Decode a slot's logical channels; `tn`, `frame` number it (a SYNC
    /// in it renumbers it, and the lock with it).
    fn slot(&mut self, mut tn: u8, mut frame: u8, fit: &Fit, llr: &[f32]) {
        if fit.burst == Burst::Sync {
            let sb = &llr[field::SDB_SB];
            if let Some(s) =
                coding::decode(Chan::Bsch, sb, &self.bsch_scramble).and_then(|b| Sync::parse(&b))
            {
                self.stats.syncs += 1;
                let e = coding::extended_colour_code(s.mcc, s.mnc, s.colour);
                if self.stats.mcc != Some(s.mcc)
                    || self.stats.mnc != s.mnc
                    || self.stats.colour != s.colour
                {
                    self.cell = Some(Cell {
                        scramble: coding::scrambling(e, 432),
                    });
                    self.stats.mcc = Some(s.mcc);
                    self.stats.mnc = s.mnc;
                    self.stats.colour = s.colour;
                }
                (tn, frame) = (s.tn, s.frame);
                if let Some(l) = &mut self.lock {
                    l.tn = s.tn;
                    l.frame = s.frame;
                    l.multiframe = s.multiframe;
                }
                self.events.push(Event::Sync(s));
            }
        }
        let Some(scr) = self.cell.as_ref().map(|c| c.scramble.clone()) else {
            return;
        };
        let tn_i = tn as usize & 3;
        // The AACH.
        let bb: Vec<f32> = if fit.burst == Burst::Sync {
            llr[field::SDB_BB].to_vec()
        } else {
            llr[field::NDB_BB1]
                .iter()
                .chain(&llr[field::NDB_BB2])
                .copied()
                .collect()
        };
        let um = coding::decode_aach(&bb, &scr)
            .filter(|&(_, flips)| flips <= 4)
            .map(|(d, _)| AccessAssign::from_bits(d).traffic(frame));
        if let Some(u) = um {
            self.traffic[tn_i] = u;
        }
        let traffic = self.traffic[tn_i];
        let block = |r: std::ops::Range<usize>| -> Vec<f32> { llr[r].to_vec() };
        match fit.burst {
            Burst::Sync => {
                self.signalling(tn, Chan::Half, &block(field::BKN2), &scr);
            }
            Burst::Full => {
                let all: Vec<f32> = llr[field::NDB_BKN1]
                    .iter()
                    .chain(&llr[field::BKN2])
                    .copied()
                    .collect();
                if traffic.is_some() {
                    self.traffic_out(tn, traffic, false, &all, &scr);
                } else {
                    self.signalling(tn, Chan::Full, &all, &scr);
                }
            }
            Burst::Half => {
                let stolen2 = self.signalling(tn, Chan::Half, &block(field::NDB_BKN1), &scr)
                    && self.second_stolen;
                let b2 = block(field::BKN2);
                if traffic.is_some() && !stolen2 {
                    self.traffic_out(tn, traffic, true, &b2, &scr);
                } else {
                    self.signalling(tn, Chan::Half, &b2, &scr);
                }
            }
        }
    }

    fn traffic_out(&mut self, tn: u8, um: Option<u8>, half: bool, llr: &[f32], scr: &[u8]) {
        self.stats.traffic_slots += 1;
        let llr = llr
            .iter()
            .zip(scr)
            .map(|(&l, &p)| if p == 1 { -l } else { l })
            .collect();
        self.events.push(Event::Traffic {
            tn,
            usage_marker: um,
            half,
            llr,
        });
    }

    /// Decode a signalling block and report its PDUs. Returns whether its
    /// CRC passed.
    fn signalling(&mut self, tn: u8, chan: Chan, llr: &[f32], scr: &[u8]) -> bool {
        self.second_stolen = false;
        let Some(b1) = coding::decode(chan, llr, &scr[..chan.k5()]) else {
            self.stats.crc_fail += 1;
            return false;
        };
        self.stats.blocks_ok += 1;
        let ti = tn as usize & 3;
        for pdu in mac::parse_block(&b1) {
            match pdu {
                Pdu::Resource {
                    address,
                    encryption,
                    chan,
                    sdu,
                    frag,
                    second_half_stolen,
                } => {
                    self.second_stolen |= second_half_stolen;
                    let encrypted = encryption != 0;
                    if frag {
                        self.frag[ti] = Some((address, encrypted, chan, sdu));
                    } else {
                        self.report(tn, address, encrypted, chan, &sdu);
                    }
                }
                Pdu::Frag(piece) => {
                    if let Some(f) = &mut self.frag[ti] {
                        f.3.extend(piece);
                    }
                }
                Pdu::End { chan, sdu } => {
                    if let Some((address, enc, c0, mut all)) = self.frag[ti].take() {
                        all.extend(sdu);
                        self.report(tn, address, enc, chan.or(c0), &all);
                    }
                }
                Pdu::SysInfo(s) => self.events.push(Event::SysInfo(s)),
                Pdu::Null | Pdu::AccessDefine | Pdu::Other(_) => {}
            }
        }
        true
    }

    fn report(
        &mut self,
        tn: u8,
        address: Address,
        encrypted: bool,
        chan: Option<ChanAlloc>,
        sdu: &[u8],
    ) {
        // An empty TM-SDU is a bare grant or allocation: still worth a line
        // when it allocates a channel.
        if sdu.is_empty() && chan.is_none() {
            return;
        }
        let signal = if encrypted { None } else { mac::parse_sdu(sdu) };
        self.events.push(Event::Signal {
            tn,
            address,
            encrypted,
            chan,
            signal,
        });
    }
}

#[derive(Debug, Clone, Copy)]
struct Fit {
    burst: Burst,
    quality: f32,
    /// The residual phase step a symbol, radians.
    step: f32,
    amp: f32,
    var: f32,
    snr_db: f32,
}

/// Differential detection: z_k·z*_{k−1} ≈ A·e^{jD} + n, derotated by the
/// fit's residual step. Each bit's component is ±A/√2, so its LLR is
/// 2·(A/√2)·x/(σ²/2).
fn differential(s: &[Iq], fit: &Fit) -> Vec<f32> {
    let r = Complex32::from_polar(1.0, -fit.step);
    let c = 2.0 * std::f32::consts::SQRT_2 * fit.amp / fit.var;
    let mut out = Vec::with_capacity(SLOT_BITS);
    for k in 1..s.len() {
        let p = s[k] * s[k - 1].conj() * r;
        out.push((c * p.im).clamp(-30.0, 30.0));
        out.push((c * p.re).clamp(-30.0, 30.0));
    }
    out
}

/// The carrier phase along a slot of QPSK symbols `u`: the fourth power
/// (Viterbi & Viterbi, |u|²-weighted, u⁴/|u|²) summed over a sliding
/// window, unwrapped to the estimate nearest the previous one mod π/2.
fn phase_track(u: &[Iq]) -> Vec<f32> {
    let n = u.len();
    let mut pre = vec![Iq::new(0.0, 0.0); n + 1];
    for (k, &x) in u.iter().enumerate() {
        let m = x.norm_sqr();
        let w = if m > 0.0 {
            (x * x) * (x * x) / m
        } else {
            Iq::new(0.0, 0.0)
        };
        pre[k + 1] = pre[k] + w;
    }
    let quarter = std::f32::consts::FRAC_PI_2;
    let mut theta = vec![0f32; n];
    for k in 0..n {
        let a = k.saturating_sub(PHASE_WINDOW);
        let b = (k + PHASE_WINDOW + 1).min(n);
        let t = (pre[b] - pre[a]).arg() / 4.0;
        theta[k] = if k == 0 {
            t
        } else {
            t + quarter * ((theta[k - 1] - t) / quarter).round()
        };
    }
    theta
}

/// The least-squares slope of a phase track, radians a symbol.
fn slope(theta: &[f32]) -> f32 {
    let n = theta.len() as f32;
    let km = (n - 1.0) / 2.0;
    let tm = theta.iter().sum::<f32>() / n;
    let (mut sxy, mut sxx) = (0f32, 0f32);
    for (k, &t) in theta.iter().enumerate() {
        let dk = k as f32 - km;
        sxy += dk * (t - tm);
        sxx += dk * dk;
    }
    sxy / sxx
}

/// Half-width, in symbols, of the carrier phase estimator's window.
const PHASE_WINDOW: usize = 20;

/// Coherent detection of the differentially encoded symbols.
///
/// Taking out the π/4 step a symbol (u_k = z_k·e^{−jπk/4}) leaves QPSK on
/// the four quadrants j^q. The carrier phase comes from the fourth power
/// (Viterbi & Viterbi, |u|²-weighted) over a sliding window, unwrapped
/// along the slot; its π/2 ambiguity does not matter since the bits are in
/// the transitions. Then log-MAP detection over the differential encoding
/// (a transition of D = (2Δq + 1)·π/4, table 5.1) turns the coherent
/// symbol likelihoods into bit LLRs. Against differential detection this
/// avoids multiplying two noisy symbols: about 2 dB at Es/N0 6–10 dB
/// (`detectors_ber`).
fn coherent(s: &[Iq]) -> (Vec<f32>, f32) {
    let n = s.len();
    let u0: Vec<Iq> = s
        .iter()
        .enumerate()
        .map(|(k, &z)| z * rot(-((k % 8) as i32)))
        .collect();
    // Two passes: the first phase track's slope (a residual carrier offset
    // the window would smear) is taken out before the second.
    let slope1 = slope(&phase_track(&u0));
    let u: Vec<Iq> = u0
        .iter()
        .enumerate()
        .map(|(k, &x)| x * Complex32::from_polar(1.0, -slope1 * k as f32))
        .collect();
    let theta = phase_track(&u);
    let total_slope = slope1 + slope(&theta);
    // Amplitude and noise from the decisions.
    let mut amp = 0f32;
    let derot: Vec<Iq> = u
        .iter()
        .zip(&theta)
        .map(|(&x, &t)| x * Complex32::from_polar(1.0, -t))
        .collect();
    let nearest = |x: Iq| -> Iq {
        if x.re.abs() >= x.im.abs() {
            Iq::new(x.re.signum(), 0.0)
        } else {
            Iq::new(0.0, x.im.signum())
        }
    };
    for &x in &derot {
        amp += (x * nearest(x).conj()).re;
    }
    amp /= n as f32;
    let var = derot
        .iter()
        .map(|&x| (x - nearest(x) * amp).norm_sqr())
        .sum::<f32>()
        / n as f32;
    let c = 2.0 * amp / var.max(1e-12);
    // Symbol likelihoods per quadrant, exp(c·Re(x·conj(j^q))), scaled by
    // their largest (the scale cancels in each LLR).
    let like: Vec<[f32; 4]> = derot
        .iter()
        .map(|x| {
            let m = [c * x.re, c * x.im, -c * x.re, -c * x.im];
            let top = m.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            m.map(|v| (v - top).exp())
        })
        .collect();
    // The log-MAP BCJR over the differential trellis collapses here: with
    // no prior on the transitions, every quadrant may follow every other, so
    // the forward and backward terms are the same for each state and a
    // transition's posterior is P(q_{k−1} = p)·P(q_k = p + Δq). Each bit's
    // LLR is then a sum over the 16 pairs of two neighbouring symbols.
    // Bits of a transition Δq: 0 → 00, 1 → 01, 2 → 11, 3 → 10.
    let mut out = Vec::with_capacity(2 * (n - 1));
    for k in 1..n {
        let (a, b) = (&like[k - 1], &like[k]);
        // t[dq] = Σ_p a[p]·b[p+dq].
        let t: [f32; 4] = std::array::from_fn(|dq| (0..4).map(|p| a[p] * b[(p + dq) % 4]).sum());
        let llr =
            |num: f32, den: f32| (num.max(1e-30).ln() - den.max(1e-30).ln()).clamp(-30.0, 30.0);
        out.push(llr(t[0] + t[1], t[2] + t[3]));
        out.push(llr(t[0] + t[3], t[1] + t[2]));
    }
    (out, total_slope)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two detectors on symbols with white noise at Es/N0.
    #[test]
    fn detectors_ber() {
        let mut x = 0x1234_5678u64;
        let mut rnd = move || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            ((x >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        };
        for esn0 in [4.0f32, 6.0, 8.0, 10.0] {
            let sigma = (0.5 * 10f32.powf(-esn0 / 10.0)).sqrt();
            let (mut ec, mut ed, mut n) = (0, 0, 0);
            for slot in 0..200i32 {
                let bits: Vec<u8> = (0..SLOT_BITS).map(|_| u8::from(rnd() < 0.5)).collect();
                let mut ph = slot % 8;
                let mut s = vec![rot(ph)];
                for k in 0..SLOT_SYMBOLS {
                    ph += transition(bits[2 * k], bits[2 * k + 1]);
                    s.push(rot(ph));
                }
                let theta = rnd() as f32 * 6.0;
                let w = 0.05f32;
                for (k, v) in s.iter_mut().enumerate() {
                    *v *= Complex32::from_polar(1.0, w * k as f32);
                    let (u1, u2) = (rnd(), rnd());
                    let r = (-2.0 * u1.ln()).sqrt() as f32 * sigma;
                    let a = (std::f64::consts::TAU * u2) as f32;
                    *v = *v * Complex32::from_polar(1.0, theta) + Complex32::from_polar(r, a);
                }
                let s2: Vec<Iq> = s
                    .iter()
                    .enumerate()
                    .map(|(k, &z)| z * Complex32::from_polar(1.0, -w * k as f32))
                    .collect();
                let c = coherent(&s2).0;
                let fit = Fit {
                    burst: Burst::Full,
                    quality: 1.0,
                    step: w,
                    amp: 1.0,
                    var: 4.0 * sigma * sigma,
                    snr_db: 0.0,
                };
                let d = differential(&s, &fit);
                for i in 0..SLOT_BITS {
                    ec += usize::from(u8::from(c[i] < 0.0) != bits[i]);
                    ed += usize::from(u8::from(d[i] < 0.0) != bits[i]);
                    n += 1;
                }
            }
            eprintln!(
                "Es/N0 {esn0} dB: coherent {:.3} %, differential {:.3} %",
                100.0 * ec as f64 / n as f64,
                100.0 * ed as f64 / n as f64
            );
        }
    }
}
