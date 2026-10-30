// made by CasualArclamp
//! The DMR receiver: channel IQ in (24 kS/s, five samples a symbol),
//! [`Event`]s out.
//!
//! Built for weak signals:
//!
//! - **Acquisition** correlates the matched-filter output against the
//!   24-symbol SYNCs at every sample, normalised so the threshold does not
//!   depend on level, and takes the peak to a fraction of a sample. A
//!   candidate is only believed when its burst also decodes (slot type
//!   within Golay (20,8) distance, or a voice burst whose frames decode).
//! - **Tracking** then predicts every burst 30 ms after the last. At the
//!   predicted time it looks for SYNC in a ±3-sample window with a much
//!   lower threshold, and voice bursts B–F (which carry no SYNC at all) are
//!   decoded on the flywheel. A voice SYNC missed under noise is
//!   flywheeled through as well, so one bad burst does not end a call.
//! - **Timing** comes from the SYNC when there is one, else from the eye:
//!   the energy of the burst's 132 symbol samples is largest at the right
//!   instant, which a parabola through three offsets finds.
//! - **Levels** (frequency offset and deviation) are a least-squares fit to
//!   the SYNC's known symbols, else to the burst's own decisions; the fit's
//!   residual is the noise that scales the soft bits.
//! - **Soft bits** (max-log LLRs) feed every code: maximum likelihood for
//!   the slot type, EMB and TACT, Chase for the BPTC rows and the AMBE
//!   Golay words.

use super::ambe::{self, AmbeFrame};
use super::data::{self, Assembler, DataHeader, Packet, Rate};
use super::pdu::{Csbk, FullLc, ShortLc};
use super::{
    ALPHA, DataType, OUTER_DEV_HZ, SLOT_SYMBOLS, SYMBOL_RATE, Source, VOICE_SYNC, bptc, layout,
    mask, pattern_symbols,
};
use crate::fec;
use crate::fsk4::{Detector, Fsk4Demod, Iq};

/// The rate the receiver takes.
pub const RATE: f64 = 24_000.0;
const SPS: usize = 5;
const SLOT: f64 = (SLOT_SYMBOLS * SPS) as f64;
/// Symbol index (in the burst's 132) of the SYNC's last symbol.
const SYNC_END: f64 = 77.0;

/// The pre-detection low-pass's cut-off (the sweep's best).
pub const CHANNEL_HZ: f64 = 4_000.0;
/// The low-pass while hunting, before the AFC knows the offset (see the
/// NXDN receiver, where a narrow one turned an off-tune signal's first
/// frames away at the SNR gate).
const HUNT_CHANNEL_HZ: f64 = 5_500.0;
/// The SNR gate's default, dB of eye SNR. Measured (2026-10-29): over
/// 120 s of noise the best SYNC candidate fitted at 3.6 dB (median −0.2);
/// real SYNCs at 5 dB C/N, the sensitivity limit, have a median of 13 dB
/// and 95 % above 3.2 dB. Acquisition needs only one of many SYNCs, so 6 dB
/// costs it little and keeps noise out.
pub const SNR_GATE_DB: f32 = 6.0;
/// Once locked, the smoothed eye may sag this far below the gate before
/// bursts are refused (a fade should not drop a call at once).
const TRACK_MARGIN_DB: f32 = 6.0;
/// How much of each burst's measured offset the AFC takes.
const AFC_GAIN: f32 = 0.3;
/// Samples demodulated before the receiver looks for bursts again (20 ms).
const AFC_SLICE: usize = 480;
/// Correlation needed to start on a SYNC from cold, and at a predicted
/// burst (where a data SYNC is checked by its slot type anyway).
const ACQUIRE: f32 = 0.78;
const TRACK: f32 = 0.5;
/// Where a voice burst B–F is expected, SYNC must be this clear to win.
const OVERRIDE: f32 = 0.8;
/// Slot type and EMB: the most hard decisions ML decoding may overrule.
/// These short codes have small covering radii (noise lands within 4 of a
/// Golay (20,8) word, and within 3 of a QR (16,7) word, nearly always), so
/// they only choose what to try: a burst counts as real when its BPTC, its
/// CRC or its AMBE Golay words also check. The data types with no such
/// check need the tighter `SLOT_TYPE_ALONE`.
const SLOT_TYPE_DIST: u32 = 4;
const SLOT_TYPE_ALONE: u32 = 1;
const EMB_DIST: u32 = 2;
/// Golay corrections over a burst's three voice frames (of 138 coded bits)
/// above which it is not believed: noise averages about 17.
const VOICE_ERRORS: u32 = 12;
/// Bursts in a row that fail before the receiver hunts again.
const LOST_AFTER: u32 = 12;
/// Locked, a SYNC missing where one is due is looked for this many samples
/// either side (300 = 12.5 ms: less than half a burst, so the slot holds).
const RESYNC: i32 = 300;

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// The repeater's Short LC (from four CACHs).
    ShortLc(ShortLc),
    VoiceHeader {
        slot: u8,
        cc: u8,
        lc: FullLc,
    },
    Terminator {
        slot: u8,
        cc: u8,
        lc: FullLc,
    },
    /// The LC repeated in a voice superframe (bursts B–E).
    EmbeddedLc {
        slot: u8,
        lc: FullLc,
    },
    /// One voice burst: `letter` 0 for A … 5 for F.
    Voice {
        slot: u8,
        letter: u8,
        frames: [AmbeFrame; 3],
    },
    Csbk {
        slot: u8,
        cc: u8,
        csbk: Csbk,
    },
    /// A multi-block control: its header and last block, CRC good.
    Mbc {
        slot: u8,
        cc: u8,
        header: [u8; 10],
        last: [u8; 10],
    },
    Idle {
        slot: u8,
        cc: u8,
    },
    /// A data header (CRC good); its blocks follow.
    DataHeader {
        slot: u8,
        cc: u8,
        header: DataHeader,
    },
    /// A data packet put back together from its header and blocks.
    Packet {
        slot: u8,
        packet: Packet,
    },
    /// Other data bursts (headers and blocks of data calls).
    Data {
        slot: u8,
        cc: u8,
        data_type: DataType,
    },
}

/// Counters and the latest measurements, for the card.
#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub bursts: u64,
    pub bursts_bad: u64,
    pub syncs: u64,
    pub voice_frames: u64,
    /// Golay corrections in voice frames, and the coded bits they cover.
    pub voice_bit_errors: u64,
    pub voice_bits: u64,
    pub csbks: u64,
    pub crc_fail: u64,
    /// The latest SYNC correlation (1 = perfect) and eye SNR, dB.
    pub sync_quality: f32,
    pub eye_snr_db: f32,
    /// The frequency offset seen, Hz.
    pub offset_hz: f32,
    pub colour_code: Option<u8>,
    /// The eye SNR of the latest SYNC alone, dB.
    pub sync_snr_db: f32,
    /// SYNCs and bursts the SNR gate turned away.
    pub gated: u64,
    /// Bursts found again after a timing jump (samples lost).
    pub resyncs: u64,
    /// What the receiver is locked to.
    pub source: Option<Source>,
}

#[derive(Clone, Copy)]
struct Track {
    /// Sample time of the last burst's last SYNC symbol (whether or not it
    /// had a SYNC).
    tau: f64,
    source: Source,
    /// The slot that burst was (0 = TS1).
    slot: u8,
    misses: u32,
}

#[derive(Clone, Default)]
struct SlotState {
    /// The next voice burst expected (0 = A … 5 = F), while a call is on.
    voice_next: Option<u8>,
    /// Bad voice bursts in a row (the call ends after two).
    voice_bad: u8,
    emb: Vec<f32>,
    mbc: Option<[u8; 10]>,
    data: Assembler,
}

pub struct Receiver {
    /// The SNR gate, dB of eye SNR: a SYNC whose own fit is worse is not
    /// believed, and a locked channel whose eye falls this far (less
    /// `TRACK_MARGIN_DB`) is let go. [`SNR_GATE_DB`] by default.
    pub snr_gate_db: f32,
    chan_hz: f64,
    demod: Fsk4Demod,
    /// Matched-filter output; `f[0]` is sample `base`.
    f: Vec<f32>,
    base: u64,
    /// Next sample time the hunt correlates at.
    scan: u64,
    /// A SYNC peak being followed while hunting: (time, |ρ|, source, voice?).
    peak: Option<(u64, f32, usize, bool)>,
    /// A SYNC candidate waiting for its burst to arrive.
    candidate: Option<(f64, Source, bool, f32)>,
    track: Option<Track>,
    syncs: [[f32; 24]; 4],
    /// Levels: frequency offset and the size of one symbol unit, both in
    /// the demodulator's units, and the noise variance in symbol units².
    dc: f32,
    gain: f32,
    noise: f32,
    slots: [SlotState; 2],
    slc: Vec<f32>,
    pub stats: Stats,
    events: Vec<Event>,
    /// The latest bursts' symbol values, for the scope.
    tap: Option<Vec<f32>>,
    /// The slot the burst being decoded turned out to be in.
    decoded_slot: Option<u8>,
}

impl Receiver {
    pub fn new(detector: Detector) -> Receiver {
        Receiver::with_channel(detector, CHANNEL_HZ)
    }

    /// With the pre-detection low-pass at `chan_hz` (for the sweep).
    pub fn with_channel(detector: Detector, chan_hz: f64) -> Receiver {
        Receiver {
            snr_gate_db: SNR_GATE_DB,
            chan_hz,
            demod: Fsk4Demod::new(
                RATE,
                SYMBOL_RATE,
                OUTER_DEV_HZ,
                ALPHA,
                HUNT_CHANNEL_HZ.max(chan_hz),
                detector,
            ),
            f: Vec::with_capacity(16_384),
            base: 0,
            scan: (23 * SPS) as u64,
            peak: None,
            candidate: None,
            track: None,
            syncs: VOICE_SYNC.map(|(_, p)| pattern_symbols(p)),
            dc: 0.0,
            gain: 1.0,
            noise: 0.25,
            slots: Default::default(),
            slc: Vec::with_capacity(68),
            stats: Stats::default(),
            events: Vec::new(),
            tap: None,
            decoded_slot: None,
        }
    }

    /// Keep the symbol values of the bursts decoded, for a scope.
    pub fn enable_tap(&mut self, on: bool) {
        self.tap = on.then(Vec::new);
    }

    pub fn tap_mut(&mut self) -> Option<&mut Vec<f32>> {
        self.tap.as_mut()
    }

    pub fn locked(&self) -> Option<Source> {
        self.track.map(|t| t.source)
    }

    pub fn reset(&mut self) {
        let tap = self.tap.is_some();
        let gate = self.snr_gate_db;
        let stats = std::mem::take(&mut self.stats);
        *self = Receiver::with_channel(self.demod.detector(), self.chan_hz);
        self.snr_gate_db = gate;
        self.stats = Stats {
            bursts: stats.bursts,
            ..Stats::default()
        };
        self.enable_tap(tap);
    }

    /// Feed IQ at [`RATE`]; events found are appended to `out`.
    pub fn process(&mut self, iq: &[Iq], out: &mut Vec<Event>) {
        // A slice at a time, so the AFC's corrections reach the samples
        // still to be demodulated within a slot or two, however large the
        // host's blocks: demodulating a whole block first would have each
        // burst in it measure, and correct, the same offset again.
        for slice in iq.chunks(AFC_SLICE) {
            self.demod.process(slice, &mut self.f);
            loop {
                let progressed = if self.track.is_some() {
                    self.step_track()
                } else if self.candidate.is_some() {
                    self.step_candidate()
                } else {
                    self.hunt()
                };
                if !progressed {
                    break;
                }
            }
            self.trim();
        }
        out.append(&mut self.events);
    }

    fn end(&self) -> u64 {
        self.base + self.f.len() as u64
    }

    fn trim(&mut self) {
        // Keep two slots behind whatever is still to be looked at.
        let keep_from = match (self.track, self.candidate) {
            (Some(t), _) => t.tau - 2.0 * SLOT,
            (None, Some(c)) => c.0 - 2.0 * SLOT,
            _ => self.scan as f64 - 2.0 * SLOT,
        };
        let drop = (keep_from.max(0.0) as u64).saturating_sub(self.base) as usize;
        if drop > 8192 {
            self.f.drain(..drop);
            self.base += drop as u64;
        }
    }

    /// The matched-filter output at fractional time `t` (linear
    /// interpolation; the filter is oversampled five times).
    #[inline]
    fn at(&self, t: f64) -> f32 {
        let i = t.floor();
        let k = (i as i64 - self.base as i64).max(0) as usize;
        let a = self.f[k];
        let b = self.f[(k + 1).min(self.f.len() - 1)];
        a + (b - a) * (t - i) as f32
    }

    /// Normalised correlation ρ of SYNC `p` ending at `t`.
    fn correlate(&self, p: usize, t: f64) -> f32 {
        let mut c = 0f32;
        let mut s = 0f32;
        let mut s2 = 0f32;
        for (k, &sym) in self.syncs[p].iter().enumerate() {
            let x = self.at(t - ((23 - k) * SPS) as f64);
            c += sym * x;
            s += x;
            s2 += x * x;
        }
        let var = (s2 - s * s / 24.0).max(1e-9);
        c / (216.0 * var).sqrt()
    }

    /// Hunting: correlate at each new sample; follow a peak above ACQUIRE
    /// to its top, then hand it on as a candidate.
    fn hunt(&mut self) -> bool {
        let end = self.end();
        if self.scan + 1 >= end {
            return false;
        }
        while self.scan + 1 < end {
            let t = self.scan;
            self.scan += 1;
            if t < self.base + (23 * SPS) as u64 {
                continue;
            }
            let (mut best, mut which) = (0f32, 0usize);
            for p in 0..4 {
                let r = self.correlate(p, t as f64);
                if r.abs() > best.abs() {
                    best = r;
                    which = p;
                }
            }
            match self.peak {
                Some((pt, pr, pw, voice)) => {
                    if best.abs() > pr && which == pw && (best > 0.0) == voice {
                        self.peak = Some((t, best.abs(), which, voice));
                    } else if t - pt >= SPS as u64 {
                        self.peak = None;
                        let tf = self.refine(pw, pt as f64, voice);
                        self.candidate = Some((tf, VOICE_SYNC[pw].0, voice, pr));
                        return true;
                    }
                }
                None if best.abs() >= ACQUIRE => {
                    self.peak = Some((t, best.abs(), which, best > 0.0))
                }
                None => {}
            }
        }
        true
    }

    /// The SYNC peak to a fraction of a sample: a parabola through |ρ| at
    /// t − 1, t, t + 1.
    fn refine(&self, p: usize, t: f64, voice: bool) -> f64 {
        let sign = if voice { 1.0 } else { -1.0 };
        let r = |dt: f64| sign * self.correlate(p, t + dt);
        let (a, b, c) = (r(-1.0), r(0.0), r(1.0));
        let den = a - 2.0 * b + c;
        let d = if den < 0.0 {
            (0.5 * (a - c) / den).clamp(-0.5, 0.5)
        } else {
            0.0
        };
        t + f64::from(d)
    }

    /// Room for a whole burst (and the CACH) round `tau`?
    fn have(&self, tau: f64) -> bool {
        let first = tau - (SYNC_END + 12.0) * SPS as f64 - 2.0;
        let last = tau + (131.0 - SYNC_END) * SPS as f64 + 4.0;
        first >= self.base as f64 && last < self.end() as f64
    }

    fn step_candidate(&mut self) -> bool {
        let (tau, source, voice, rho) = self.candidate.expect("called with a candidate");
        if !self.have(tau) {
            return false;
        }
        self.candidate = None;
        let slot = match source {
            Source::Ts2 => 1,
            _ => 0,
        };
        if self.decode_burst(tau, Some((source, voice, rho)), source, slot) {
            self.afc_step(self.dc);
            self.demod.set_channel(self.chan_hz);
            self.track = Some(Track {
                tau,
                source,
                slot: self.last_slot(slot),
                misses: 0,
            });
            self.stats.source = Some(source);
        } else {
            // Not a burst after all: hunt on from past its peak (the hunt
            // has already looked that far).
            self.scan = self.scan.max(tau as u64 + SPS as u64);
        }
        true
    }

    /// The slot the last burst turned out to be in (the CACH may have said).
    fn last_slot(&self, guess: u8) -> u8 {
        self.decoded_slot.unwrap_or(guess)
    }

    fn step_track(&mut self) -> bool {
        let t = self.track.expect("called while tracking");
        let mut tau = t.tau + SLOT;
        if !self.have(tau + 3.0) {
            return false;
        }
        let slot = t.slot ^ 1;
        // SYNC near the prediction, from this source's patterns.
        let p = VOICE_SYNC
            .iter()
            .position(|(s, _)| *s == t.source)
            .unwrap_or(0);
        let cands: &[usize] = match t.source {
            Source::Ts1 | Source::Ts2 => &[2, 3],
            _ => std::slice::from_ref(&p),
        };
        let mut best = (0f32, 0usize, 0f64);
        for &c in cands {
            for d in -3..=3 {
                let r = self.correlate(c, tau.round() + f64::from(d));
                if r.abs() > best.0.abs() {
                    best = (r, c, tau.round() + f64::from(d));
                }
            }
        }
        let expect_voice = self.slots[slot as usize].voice_next.is_some_and(|l| l > 0);
        let need = if expect_voice { OVERRIDE } else { TRACK };
        // Samples lost (a radio's dropped USB transfer) move every burst
        // after them: a burst that should carry a SYNC and has none near
        // the prediction is looked for further out, at the acquisition
        // threshold. Seen on a real recording: a 3 ms jump cost 0.4 s.
        if best.0.abs() < need && !expect_voice {
            // Wait until the whole window is in.
            if !self.have(tau + f64::from(RESYNC) + 3.0) {
                return false;
            }
            for &c in cands {
                for d in -RESYNC..=RESYNC {
                    let r = self.correlate(c, tau.round() + f64::from(d));
                    if r.abs() >= ACQUIRE && r.abs() > best.0.abs() {
                        best = (r, c, tau.round() + f64::from(d));
                    }
                }
            }
            if best.0.abs() >= ACQUIRE {
                self.stats.resyncs += 1;
            }
        }
        let sync = if best.0.abs() >= need {
            let voice = best.0 > 0.0;
            tau = self.refine(best.1, best.2, voice);
            Some((VOICE_SYNC[best.1].0, voice, best.0.abs()))
        } else {
            None
        };
        let slot = match sync {
            Some((Source::Ts1, ..)) => 0,
            Some((Source::Ts2, ..)) => 1,
            _ => slot,
        };
        if sync.is_none() {
            tau += self.eye_timing(tau);
        }
        let ok = self.decode_burst(tau, sync, t.source, slot);
        let misses = if ok { 0 } else { t.misses + 1 };
        if misses >= LOST_AFTER {
            // Lost: hunt again from here.
            self.track = None;
            self.stats.source = None;
            self.demod.set_afc(0.0);
            self.demod.set_channel(HUNT_CHANNEL_HZ.max(self.chan_hz));
            self.dc = 0.0;
            self.slots = Default::default();
            self.scan = tau as u64;
        } else {
            self.track = Some(Track {
                tau,
                source: t.source,
                slot: self.last_slot(slot),
                misses,
            });
        }
        true
    }

    /// Move the AFC by `delta` symbol units (the offset the level fit
    /// sees). The samples already demodulated were made with the old
    /// setting; in a discriminator's output a frequency shift is only an
    /// offset, so they are shifted to match, and nothing straddles the
    /// change (a frame half before and half after a jump decoded badly).
    fn afc_step(&mut self, delta: f32) {
        let u = self.demod.units_per_hz();
        self.demod
            .set_afc(self.demod.afc_hz() + f64::from(delta / u));
        for x in &mut self.f {
            *x -= delta;
        }
        self.dc -= delta;
    }

    /// The timing correction the eye asks for: energy of the 132 symbol
    /// samples at −1, 0, +1 samples, the parabola's top (a third of it, so
    /// noise does not walk the clock).
    fn eye_timing(&self, tau: f64) -> f64 {
        let e = |d: f64| {
            (0..132)
                .map(|j| {
                    let x = self.at(tau + d + (j as f64 - SYNC_END) * SPS as f64) - self.dc;
                    x * x
                })
                .sum::<f32>()
        };
        let (a, b, c) = (e(-1.0), e(0.0), e(1.0));
        let den = a - 2.0 * b + c;
        if den < 0.0 {
            f64::from((0.5 * (a - c) / den).clamp(-0.5, 0.5)) / 3.0
        } else {
            0.0
        }
    }

    /// Fit `x = gain·s + dc` to known symbols; update the levels and noise.
    /// Returns this fit's own eye SNR, dB (mean symbol energy 5 over the
    /// residual), or `None` if nothing could be fitted.
    fn fit(&mut self, xs: &[f32], ss: &[f32], weight: f32) -> Option<f32> {
        let n = xs.len() as f32;
        let (sx, ss_, sxx, sss, sxs) = xs
            .iter()
            .zip(ss)
            .fold((0f32, 0f32, 0f32, 0f32, 0f32), |a, (&x, &s)| {
                (a.0 + x, a.1 + s, a.2 + x * x, a.3 + s * s, a.4 + x * s)
            });
        let den = n * sss - ss_ * ss_;
        if den.abs() < 1e-6 {
            return None;
        }
        let g = (n * sxs - sx * ss_) / den;
        if g <= 0.05 {
            return None;
        }
        let dc = (sx - g * ss_) / n;
        // Residual variance in symbol units².
        let res =
            (sxx - 2.0 * g * sxs - 2.0 * dc * sx + g * g * sss + 2.0 * g * dc * ss_ + n * dc * dc)
                / n;
        let nv = (res / (g * g)).max(0.01);
        self.gain += weight * (g - self.gain);
        self.dc += weight * (dc - self.dc);
        self.noise += weight * (nv - self.noise);
        Some(10.0 * (5.0 / nv).log10())
    }

    /// The 2 soft bits of symbol value `y` (symbol units).
    #[inline]
    fn llr(&self, y: f32, out: &mut [f32]) {
        let k = 1.0 / (2.0 * self.noise);
        let d = |s: f32| (y - s) * (y - s);
        let (p3, p1, m1, m3) = (d(3.0), d(1.0), d(-1.0), d(-3.0));
        // Bit 1: 0 for +3 (01) and +1 (00), 1 for −1 (10) and −3 (11).
        out[0] = (m1.min(m3) - p3.min(p1)) * k;
        // Bit 0: 1 for ±3, 0 for ±1.
        out[1] = (p3.min(m3) - p1.min(m1)) * k;
    }

    /// Decode the burst whose SYNC ends at `tau`. True if anything in it
    /// checked (which keeps the flywheel going).
    fn decode_burst(
        &mut self,
        tau: f64,
        sync: Option<(Source, bool, f32)>,
        source: Source,
        slot_guess: u8,
    ) -> bool {
        self.decoded_slot = None;
        let sp = SPS as f64;
        let raw: Vec<f32> = (0..132)
            .map(|j| self.at(tau + (j as f64 - SYNC_END) * sp))
            .collect();
        if let Some((s, voice, rho)) = sync {
            let p = VOICE_SYNC.iter().position(|(x, _)| *x == s).unwrap_or(0);
            let sign = if voice { 1.0 } else { -1.0 };
            let ss: Vec<f32> = self.syncs[p].iter().map(|v| v * sign).collect();
            let acquiring = self.track.is_none();
            let w = if acquiring { 1.0 } else { 0.5 };
            let snr = self.fit(&raw[54..78], &ss, w).unwrap_or(f32::MIN);
            self.stats.sync_quality = rho;
            self.stats.sync_snr_db = snr;
            self.stats.syncs += 1;
            // The SNR gate: a SYNC found in noise fits its 24 known symbols
            // badly, so its own eye SNR is low; a real one near the
            // sensitivity limit still fits well (see `SNR_GATE_DB`).
            if acquiring && snr < self.snr_gate_db {
                self.stats.gated += 1;
                return false;
            }
        }
        // While tracking, a channel whose eye has closed below the gate is
        // not decoded: the transmitter has gone, or noise has taken over.
        if self.track.is_some()
            && 10.0 * (5.0 / self.noise).log10() < self.snr_gate_db - TRACK_MARGIN_DB
        {
            self.stats.gated += 1;
            self.stats.bursts += 1;
            self.stats.bursts_bad += 1;
            return false;
        }
        let y: Vec<f32> = raw.iter().map(|x| (x - self.dc) / self.gain).collect();
        let mut llr = vec![0f32; 264];
        for (j, &v) in y.iter().enumerate() {
            self.llr(v, &mut llr[2 * j..2 * j + 2]);
        }

        // The CACH, on a base station's outbound.
        // TACT is Hamming (7,4), a perfect code: every word is within one bit
        // of a codeword, so only an exact one is trusted for the slot.
        let mut slot = slot_guess;
        if source == Source::Bs {
            let mut c = [0f32; 24];
            for (j, pair) in c.chunks_mut(2).enumerate() {
                let v = (self.at(tau + (j as f64 - 12.0 - SYNC_END) * sp) - self.dc) / self.gain;
                self.llr(v, pair);
            }
            let tl: Vec<f32> = layout::TACT.iter().map(|&p| c[p]).collect();
            let (tact, dist) = fec::hamming_7_4().ml(&tl);
            if dist == 0 {
                slot = ((tact >> 2) & 1) as u8;
            }
            if dist <= 1 {
                let lcss = (tact & 3) as u8;
                self.cach_payload(lcss, layout::CACH_PAYLOAD.iter().map(|&p| c[p]));
            }
        }
        self.decoded_slot = Some(slot);
        let s = slot as usize;

        let good = match sync {
            Some((_, false, _)) => self.data_burst(&llr, slot),
            Some((_, true, _)) => self.voice_burst(&llr, slot, true),
            None => match self.slots[s].voice_next {
                // A voice SYNC lost in noise: flywheel through it if the
                // frames decode cleanly.
                Some(0) => self.voice_burst(&llr, slot, false),
                Some(letter) => self.emb_burst(&llr, slot, letter),
                None => false,
            },
        };
        self.stats.bursts += 1;
        if !good {
            self.stats.bursts_bad += 1;
        }
        if good {
            let sym = [3.0f32, 1.0, -1.0, -3.0];
            let ss: Vec<f32> = y
                .iter()
                .map(|&v| {
                    *sym.iter()
                        .min_by(|a, b| (v - *a).abs().total_cmp(&(v - *b).abs()))
                        .unwrap()
                })
                .collect();
            if sync.is_none() {
                self.fit(&raw, &ss, 0.2);
            }
            self.stats.eye_snr_db = 10.0 * (5.0 / self.noise).log10();
            // AFC: move the mixer part of the way to the offset seen.
            self.afc_step(AFC_GAIN * self.dc);
            let u = self.demod.units_per_hz();
            self.stats.offset_hz = self.demod.afc_hz() as f32 + self.dc / u;
            if let Some(tap) = &mut self.tap {
                tap.extend_from_slice(&y);
                let over = tap.len().saturating_sub(4 * 132);
                tap.drain(..over);
            }
        }
        good
    }

    fn cach_payload(&mut self, lcss: u8, bits: impl Iterator<Item = f32>) {
        match lcss {
            0b01 => {
                self.slc.clear();
                self.slc.extend(bits);
            }
            0b11 if !self.slc.is_empty() && self.slc.len() < 51 => self.slc.extend(bits),
            0b10 if self.slc.len() == 51 => {
                self.slc.extend(bits);
                let llr: [f32; 68] = self.slc[..].try_into().expect("68 bits");
                if let Some(v) = bptc::decode_short_lc(&llr) {
                    self.events.push(Event::ShortLc(ShortLc::parse(v)));
                }
                self.slc.clear();
            }
            _ => self.slc.clear(),
        }
    }

    fn data_burst(&mut self, llr: &[f32], slot: u8) -> bool {
        let st: Vec<f32> = llr[layout::SLOT_A]
            .iter()
            .chain(&llr[layout::SLOT_B])
            .copied()
            .collect();
        let (w, dist) = fec::golay_20_8().ml(&st);
        if dist > SLOT_TYPE_DIST {
            return false;
        }
        let cc = (w >> 4) as u8;
        let dt = DataType::from_u8((w & 15) as u8);
        self.stats.colour_code = Some(cc);
        let s = slot as usize;
        let mut info = [0f32; 196];
        info[..98].copy_from_slice(&llr[layout::INFO_A]);
        info[98..].copy_from_slice(&llr[layout::INFO_B]);
        let bptc_types = matches!(
            dt,
            DataType::VoiceLcHeader
                | DataType::TerminatorLc
                | DataType::Csbk
                | DataType::MbcHeader
                | DataType::MbcContinuation
                | DataType::PiHeader
                | DataType::DataHeader
                | DataType::Rate12
                | DataType::Usbd
                | DataType::Idle
        );
        let decoded = if bptc_types {
            match bptc::decode_196(&info) {
                Some(d) => Some(d),
                None => return false,
            }
        } else if dist > SLOT_TYPE_ALONE {
            return false;
        } else {
            None
        };
        match (dt, decoded) {
            (DataType::VoiceLcHeader | DataType::TerminatorLc, Some((d, _))) => {
                let term = dt == DataType::TerminatorLc;
                let m = if term {
                    mask::TERMINATOR_LC
                } else {
                    mask::VOICE_LC_HEADER
                };
                let mut lc: [u8; 9] = d[..9].try_into().expect("9 bytes");
                let p = [d[9] ^ m[0], d[10] ^ m[1], d[11] ^ m[2]];
                if fec::rs129_check(&mut lc, &p).is_none() {
                    self.stats.crc_fail += 1;
                    return true;
                }
                let lc = FullLc::parse(&lc);
                if term {
                    self.slots[s].voice_next = None;
                    self.events.push(Event::Terminator { slot, cc, lc });
                } else {
                    self.slots[s].voice_next = Some(0);
                    self.slots[s].voice_bad = 0;
                    self.slots[s].emb.clear();
                    self.events.push(Event::VoiceHeader { slot, cc, lc });
                }
            }
            (DataType::Csbk, Some((d, _))) => {
                let b: [u8; 10] = d[..10].try_into().expect("10 bytes");
                if crc16(&b, mask::CSBK) == u16::from_be_bytes([d[10], d[11]]) {
                    self.stats.csbks += 1;
                    self.slots[s].voice_next = None;
                    self.events.push(Event::Csbk {
                        slot,
                        cc,
                        csbk: Csbk::parse(&b),
                    });
                } else {
                    self.stats.crc_fail += 1;
                }
            }
            (DataType::MbcHeader, Some((d, _))) => {
                let b: [u8; 10] = d[..10].try_into().expect("10 bytes");
                self.slots[s].mbc = (crc16(&b, mask::MBC_HEADER)
                    == u16::from_be_bytes([d[10], d[11]]))
                .then_some(b);
            }
            (DataType::MbcContinuation, Some((d, _))) => {
                // The last block's CRC covers the header and itself, with no
                // mask (table B.21 note 1).
                let last: [u8; 10] = d[..10].try_into().expect("10 bytes");
                if let Some(header) = self.slots[s].mbc.take()
                    && last[0] & 0x80 != 0
                {
                    let mut bits = Vec::with_capacity(160);
                    bptc::to_bits(&header, &mut bits);
                    bptc::to_bits(&last, &mut bits);
                    if fec::crc_ccitt(&bits) == u16::from_be_bytes([d[10], d[11]]) {
                        self.events.push(Event::Mbc {
                            slot,
                            cc,
                            header,
                            last,
                        });
                    } else {
                        self.stats.crc_fail += 1;
                    }
                }
            }
            (DataType::DataHeader, Some((d, corr))) => {
                self.slots[s].voice_next = None;
                let header = DataHeader::parse(d[..10].try_into().expect("10 bytes"));
                if DataHeader::crc_ok(&d) {
                    self.slots[s].data.header(header);
                    self.events.push(Event::DataHeader { slot, cc, header });
                } else if corr == 0 {
                    // Clean bits, failing CRC: see `Assembler::provisional`.
                    self.stats.crc_fail += 1;
                    self.slots[s].data.provisional(header);
                } else {
                    self.stats.crc_fail += 1;
                    self.slots[s].data.abandon();
                }
            }
            (DataType::Rate12, Some((d, _))) => {
                if let Some(packet) = self.slots[s].data.block(&d, Rate::Half) {
                    self.events.push(Event::Packet { slot, packet });
                }
            }
            (DataType::Rate34, _) => {
                let (d, _) = data::trellis_decode(&info);
                if let Some(packet) = self.slots[s].data.block(&d, Rate::ThreeQuarter) {
                    self.events.push(Event::Packet { slot, packet });
                }
            }
            (DataType::Idle, _) => {
                self.slots[s].voice_next = None;
                self.events.push(Event::Idle { slot, cc });
            }
            (dt, _) => {
                self.slots[s].voice_next = None;
                self.events.push(Event::Data {
                    slot,
                    cc,
                    data_type: dt,
                });
            }
        }
        true
    }

    fn frames(&mut self, llr: &[f32]) -> [AmbeFrame; 3] {
        let mut f2 = [0f32; 72];
        f2[..36].copy_from_slice(&llr[72..108]);
        f2[36..].copy_from_slice(&llr[156..192]);
        let frames = [
            ambe::decode(llr[..72].try_into().expect("72")),
            ambe::decode(&f2),
            ambe::decode(llr[192..264].try_into().expect("72")),
        ];
        for f in &frames {
            self.stats.voice_frames += 1;
            self.stats.voice_bit_errors += u64::from(f.errors);
            self.stats.voice_bits += 46;
        }
        frames
    }

    /// A voice burst with SYNC (A), or one flywheeled where A should be.
    /// Believed if the Golay words came out near; with SYNC the frames are
    /// passed on regardless.
    fn voice_burst(&mut self, llr: &[f32], slot: u8, synced: bool) -> bool {
        let s = slot as usize;
        let frames = self.frames(llr);
        let errors: u32 = frames.iter().map(|f| f.errors).sum();
        let good = errors <= VOICE_ERRORS;
        let letter = 0;
        if good || synced {
            self.slots[s].voice_next = Some(1);
            self.slots[s].voice_bad = 0;
            self.events.push(Event::Voice {
                slot,
                letter,
                frames,
            });
        } else {
            self.slots[s].voice_next = None;
        }
        good
    }

    /// Voice bursts B–F: EMB, embedded signalling, frames.
    fn emb_burst(&mut self, llr: &[f32], slot: u8, letter: u8) -> bool {
        let s = slot as usize;
        let e: Vec<f32> = llr[layout::EMB_A]
            .iter()
            .chain(&llr[layout::EMB_B])
            .copied()
            .collect();
        let (w, dist) = fec::qr_16_7().ml(&e);
        let cc = (w >> 3) as u8;
        let lcss = (w & 3) as u8;
        let cc_ok = self.stats.colour_code.is_none_or(|c| c == cc);
        if dist > EMB_DIST || !cc_ok {
            self.slots[s].voice_bad += 1;
            if self.slots[s].voice_bad >= 2 {
                self.slots[s].voice_next = None;
                return false;
            }
        } else {
            self.slots[s].voice_bad = 0;
            let emb = &llr[layout::EMBEDDED];
            let st = &mut self.slots[s];
            match lcss {
                0b01 => {
                    st.emb.clear();
                    st.emb.extend_from_slice(emb);
                }
                0b11 if !st.emb.is_empty() && st.emb.len() < 96 => st.emb.extend_from_slice(emb),
                0b10 if st.emb.len() == 96 => {
                    st.emb.extend_from_slice(emb);
                    let l: [f32; 128] = st.emb[..].try_into().expect("128 bits");
                    st.emb.clear();
                    if let Some(lc) = bptc::decode_embedded(&l) {
                        self.events.push(Event::EmbeddedLc {
                            slot,
                            lc: FullLc::parse(&lc),
                        });
                    }
                }
                _ => st.emb.clear(),
            }
        }
        let frames = self.frames(llr);
        let errors: u32 = frames.iter().map(|f| f.errors).sum();
        self.events.push(Event::Voice {
            slot,
            letter,
            frames,
        });
        self.slots[s].voice_next = Some(if letter >= 5 { 0 } else { letter + 1 });
        self.slots[s].voice_bad == 0 && errors <= VOICE_ERRORS + 2
    }
}

/// CRC-CCITT of 10 bytes under a mask.
fn crc16(b: &[u8; 10], m: u16) -> u16 {
    let mut bits = Vec::with_capacity(80);
    bptc::to_bits(b, &mut bits);
    fec::crc_ccitt(&bits) ^ m
}
