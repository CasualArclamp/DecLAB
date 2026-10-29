// made by CasualArclamp
//! The NXDN receiver (NXDN48 or NXDN96, chosen at construction): channel IQ
//! at 24 kS/s (ten or five samples a symbol) in,
//! [`Event`]s out. Built as the DMR receiver is (see `dmr::receiver` for
//! the reasoning): discriminator and matched filter with AFC, a normalised
//! correlator on the 10-symbol frame sync word, an SNR gate on each
//! candidate, an 80 ms flywheel once locked, max-log soft bits into soft
//! decoders (Viterbi for SACCH and FACCH1, Chase for the AMBE+2 Golay
//! words).
//!
//! The FSW is only 10 symbols, so a peak in noise is common; the LICH that
//! follows is 8 more symbols of known shape (all ±3), and both go into the
//! level fit whose eye SNR the gate tests. A candidate must then also pass
//! the LICH parity and either a SACCH or FACCH1 CRC or clean voice Golay
//! words.

use super::{
    ALPHA, FRAME_SYMBOLS, FSW, Lich, Message, Mode, Rfct, Steal, facch1_decode, layout,
    sacch_decode, scrambler_bit,
};
use crate::dmr::ambe::{self, AmbeFrame};
use crate::fsk4::{Detector, Fsk4Demod, Iq};

/// The rate the receiver takes.
pub const RATE: f64 = 24_000.0;
/// Symbol index of the FSW's last symbol.
const SYNC_END: f64 = 9.0;
/// The pre-detection low-pass once locked (the sweep's best), and while
/// hunting: before the AFC has the offset, a narrow filter clips an
/// off-tune signal's edge and the gate turns its first frames away.
pub const CHANNEL_HZ: f64 = 2_000.0;
const HUNT_CHANNEL_HZ: f64 = 3_000.0;
/// The same for NXDN96 (twice the symbol rate; see `nxdn_sweep`).
pub const CHANNEL_HZ_96: f64 = 4_000.0;
const HUNT_CHANNEL_HZ_96: f64 = 6_000.0;

/// The receiver's own low-pass for a mode: locked, and hunting.
pub fn channel_for(mode: Mode) -> (f64, f64) {
    match mode {
        Mode::Nxdn48 => (CHANNEL_HZ, HUNT_CHANNEL_HZ),
        Mode::Nxdn96 => (CHANNEL_HZ_96, HUNT_CHANNEL_HZ_96),
    }
}
/// FSW correlation to start from cold, and at a predicted frame.
const ACQUIRE: f32 = 0.8;
const TRACK: f32 = 0.45;
/// Frames in a row that fail before the receiver hunts again.
const LOST_AFTER: u32 = 5;
/// The SNR gate's default, dB of eye SNR over the FSW and LICH. Measured
/// (`nxdn_sweep`, 2026-10-29): in 120 s of noise, 1102 FSW candidates, the
/// best fitting at 7.4 dB (median 1.1); real frames near the sensitivity
/// limit (4–6 dB C/N) fit at a median of 11–14 dB. Gating on the 10 FSW
/// symbols alone was worse (noise reached 11.6 dB). Behind the gate a frame
/// must still pass the LICH parity and a CRC or clean voice.
pub const SNR_GATE_DB: f32 = 8.0;
const TRACK_MARGIN_DB: f32 = 6.0;
/// Golay corrections over a frame's four voice frames above which, without
/// a CRC to vouch for the frame, it is not believed.
const VOICE_ERRORS: u32 = 16;
const AFC_GAIN: f32 = 0.3;
const AFC_SLICE: usize = 960;

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A FACCH1 message (call set-up and release, mostly).
    Facch { msg: Message },
    /// A message completed by four SACCH superframe parts.
    Sacch { msg: Message },
    /// A frame's voice: four (or two, when FACCH1 stole a half) AMBE+2
    /// frames.
    Voice { frames: Vec<AmbeFrame> },
    /// A control channel frame (its CAC is not decoded yet).
    ControlChannel { lich: Lich },
}

#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub frames: u64,
    pub frames_bad: u64,
    pub syncs: u64,
    pub voice_frames: u64,
    pub voice_bit_errors: u64,
    pub voice_bits: u64,
    pub crc_fail: u64,
    pub sync_quality: f32,
    pub sync_snr_db: f32,
    pub eye_snr_db: f32,
    pub offset_hz: f32,
    pub gated: u64,
    /// The radio access number of the latest good SACCH.
    pub ran: Option<u8>,
    pub locked: bool,
}

pub struct Receiver {
    pub snr_gate_db: f32,
    mode: Mode,
    /// Samples a symbol and a frame.
    sps: usize,
    frame: f64,
    chan_hz: f64,
    hunt_hz: f64,
    demod: Fsk4Demod,
    f: Vec<f32>,
    base: u64,
    scan: u64,
    peak: Option<(u64, f32)>,
    candidate: Option<(f64, f32)>,
    /// Locked: the last frame's FSW end, and frames missed in a row.
    track: Option<(f64, u32)>,
    fsw: [f32; 10],
    dc: f32,
    gain: f32,
    noise: f32,
    /// SACCH superframe parts (18 bits each) gathered so far.
    sacch: [Option<[u8; 18]>; 4],
    pub stats: Stats,
    events: Vec<Event>,
    tap: Option<Vec<f32>>,
}

impl Receiver {
    /// NXDN48.
    pub fn new() -> Receiver {
        Receiver::for_mode(Mode::Nxdn48)
    }

    pub fn for_mode(mode: Mode) -> Receiver {
        Receiver::with_channel(mode, channel_for(mode).0)
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn with_channel(mode: Mode, chan_hz: f64) -> Receiver {
        let sps = (RATE / mode.symbol_rate()).round() as usize;
        let hunt_hz = channel_for(mode).1.max(chan_hz);
        let fsw = std::array::from_fn(|i| super::symbol(((FSW >> (18 - 2 * i)) & 3) as u8));
        Receiver {
            snr_gate_db: SNR_GATE_DB,
            mode,
            sps,
            frame: (FRAME_SYMBOLS * sps) as f64,
            chan_hz,
            hunt_hz,
            demod: Fsk4Demod::new(
                RATE,
                mode.symbol_rate(),
                mode.outer_dev_hz(),
                ALPHA,
                hunt_hz,
                Detector::Discriminator,
            ),
            f: Vec::with_capacity(32_768),
            base: 0,
            scan: (9 * sps) as u64,
            peak: None,
            candidate: None,
            track: None,
            fsw,
            dc: 0.0,
            gain: 1.0,
            noise: 0.25,
            sacch: [None; 4],
            stats: Stats::default(),
            events: Vec::new(),
            tap: None,
        }
    }

    pub fn enable_tap(&mut self, on: bool) {
        self.tap = on.then(Vec::new);
    }

    pub fn tap_mut(&mut self) -> Option<&mut Vec<f32>> {
        self.tap.as_mut()
    }

    pub fn locked(&self) -> bool {
        self.track.is_some()
    }

    pub fn reset(&mut self) {
        let (tap, gate) = (self.tap.is_some(), self.snr_gate_db);
        *self = Receiver::with_channel(self.mode, self.chan_hz);
        self.snr_gate_db = gate;
        self.enable_tap(tap);
    }

    pub fn process(&mut self, iq: &[Iq], out: &mut Vec<Event>) {
        // In slices, so the AFC reaches the samples still to come (see the
        // DMR receiver, where whole blocks once made it run away).
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
        self.stats.locked = self.track.is_some();
        out.append(&mut self.events);
    }

    fn end(&self) -> u64 {
        self.base + self.f.len() as u64
    }

    fn trim(&mut self) {
        let keep_from = match (self.track, self.candidate) {
            (Some((t, _)), _) => t - self.frame,
            (None, Some((c, _))) => c - self.frame,
            _ => self.scan as f64 - self.frame,
        };
        let drop = (keep_from.max(0.0) as u64).saturating_sub(self.base) as usize;
        if drop > 16_384 {
            self.f.drain(..drop);
            self.base += drop as u64;
        }
    }

    #[inline]
    fn at(&self, t: f64) -> f32 {
        let i = t.floor();
        let k = (i as i64 - self.base as i64).max(0) as usize;
        let a = self.f[k.min(self.f.len() - 1)];
        let b = self.f[(k + 1).min(self.f.len() - 1)];
        a + (b - a) * (t - i) as f32
    }

    /// Normalised correlation ρ of the FSW ending at `t`.
    fn correlate(&self, t: f64) -> f32 {
        let (mut c, mut s, mut s2) = (0f32, 0f32, 0f32);
        for (k, &sym) in self.fsw.iter().enumerate() {
            let x = self.at(t - ((9 - k) * self.sps) as f64);
            c += sym * x;
            s += x;
            s2 += x * x;
        }
        let e: f32 = self.fsw.iter().map(|v| v * v).sum();
        c / (e * (s2 - s * s / 10.0).max(1e-9)).sqrt()
    }

    fn refine(&self, t: f64) -> f64 {
        let (a, b, c) = (
            self.correlate(t - 1.0),
            self.correlate(t),
            self.correlate(t + 1.0),
        );
        let den = a - 2.0 * b + c;
        let d = if den < 0.0 {
            (0.5 * (a - c) / den).clamp(-0.5, 0.5)
        } else {
            0.0
        };
        t + f64::from(d)
    }

    fn hunt(&mut self) -> bool {
        let end = self.end();
        if self.scan + 1 >= end {
            return false;
        }
        while self.scan + 1 < end {
            let t = self.scan;
            self.scan += 1;
            if t < self.base + (9 * self.sps) as u64 {
                continue;
            }
            let r = self.correlate(t as f64);
            match self.peak {
                Some((pt, pr)) => {
                    if r > pr {
                        self.peak = Some((t, r));
                    } else if t - pt >= self.sps as u64 {
                        self.peak = None;
                        self.candidate = Some((self.refine(pt as f64), pr));
                        return true;
                    }
                }
                None if r >= ACQUIRE => self.peak = Some((t, r)),
                None => {}
            }
        }
        true
    }

    fn have(&self, tau: f64) -> bool {
        let first = tau - SYNC_END * self.sps as f64 - 2.0;
        let last = tau + (FRAME_SYMBOLS as f64 - 1.0 - SYNC_END) * self.sps as f64 + 4.0;
        first >= self.base as f64 && last < self.end() as f64
    }

    fn step_candidate(&mut self) -> bool {
        let (tau, rho) = self.candidate.expect("called with a candidate");
        if !self.have(tau) {
            return false;
        }
        self.candidate = None;
        if self.decode_frame(tau, Some(rho)) {
            self.track = Some((tau, 0));
            // Locked: take the whole offset now, then narrow the filter.
            self.afc_step(self.dc);
            self.demod.set_channel(self.chan_hz);
        } else {
            self.scan = self.scan.max(tau as u64 + self.sps as u64);
        }
        true
    }

    fn step_track(&mut self) -> bool {
        let (t, misses) = self.track.expect("called while tracking");
        let mut tau = t + self.frame;
        if !self.have(tau + 4.0) {
            return false;
        }
        let mut best = (f32::MIN, 0f64);
        for d in -4..=4 {
            let at = tau.round() + f64::from(d);
            let r = self.correlate(at);
            if r > best.0 {
                best = (r, at);
            }
        }
        let sync = if best.0 >= TRACK {
            tau = self.refine(best.1);
            Some(best.0)
        } else {
            tau += self.eye_timing(tau);
            None
        };
        let ok = self.decode_frame(tau, sync);
        let misses = if ok { 0 } else { misses + 1 };
        if misses >= LOST_AFTER {
            self.track = None;
            self.demod.set_afc(0.0);
            self.demod.set_channel(self.hunt_hz);
            self.dc = 0.0;
            self.sacch = [None; 4];
            self.scan = tau as u64;
        } else {
            self.track = Some((tau, misses));
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

    fn eye_timing(&self, tau: f64) -> f64 {
        let e = |d: f64| {
            (0..FRAME_SYMBOLS)
                .map(|j| {
                    let x = self.at(tau + d + (j as f64 - SYNC_END) * self.sps as f64) - self.dc;
                    x * x
                })
                .sum::<f32>()
        };
        let (a, b, c) = (e(-2.0), e(0.0), e(2.0));
        let den = a - 2.0 * b + c;
        if den < 0.0 {
            2.0 * f64::from((0.5 * (a - c) / den).clamp(-0.5, 0.5)) / 3.0
        } else {
            0.0
        }
    }

    /// Least-squares fit of `x = gain·s + dc`; updates the levels and
    /// returns this fit's eye SNR, dB (as in the DMR receiver).
    fn fit(&mut self, xs: &[f32], ss: &[f32], weight: f32) -> Option<f32> {
        let (g, dc, nv) = ls_fit(xs, ss)?;
        self.gain += weight * (g - self.gain);
        self.dc += weight * (dc - self.dc);
        self.noise += weight * (nv - self.noise);
        Some(10.0 * (5.0 / nv).log10())
    }
}

/// Least-squares `x = g·s + dc`: (g, dc, residual variance in symbol
/// units²), or `None` if nothing could be fitted.
fn ls_fit(xs: &[f32], ss: &[f32]) -> Option<(f32, f32, f32)> {
    {
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
        let res =
            (sxx - 2.0 * g * sxs - 2.0 * dc * sx + g * g * sss + 2.0 * g * dc * ss_ + n * dc * dc)
                / n;
        Some((g, dc, (res / (g * g)).max(0.01)))
    }
}

impl Receiver {
    #[inline]
    fn llr(&self, y: f32, out: &mut [f32]) {
        let k = 1.0 / (2.0 * self.noise);
        let d = |s: f32| (y - s) * (y - s);
        let (p3, p1, m1, m3) = (d(3.0), d(1.0), d(-1.0), d(-3.0));
        out[0] = (m1.min(m3) - p3.min(p1)) * k;
        out[1] = (p3.min(m3) - p1.min(m1)) * k;
    }

    /// Decode the frame whose FSW ends at `tau`. True if it checked.
    fn decode_frame(&mut self, tau: f64, sync: Option<f32>) -> bool {
        let raw: Vec<f32> = (0..FRAME_SYMBOLS)
            .map(|j| self.at(tau + (j as f64 - SYNC_END) * self.sps as f64))
            .collect();
        let acquiring = self.track.is_none();
        if let Some(rho) = sync {
            // FSW symbols are known; LICH symbols are ±3, their sign
            // decided against the present levels.
            let mut ss: Vec<f32> = self.fsw.to_vec();
            ss.extend(
                raw[10..18]
                    .iter()
                    .map(|&x| if x >= self.dc { 3.0 } else { -3.0 }),
            );
            let w = if acquiring { 1.0 } else { 0.5 };
            // The LICH's signs are decided from the data, which flatters
            // noise a little; the FSW alone (10 symbols) is too few to fit
            // cleanly. Both were measured: see `SNR_GATE_DB`.
            let snr = self.fit(&raw[..18], &ss, w).unwrap_or(f32::MIN);
            self.stats.sync_quality = rho;
            self.stats.sync_snr_db = snr;
            self.stats.syncs += 1;
            if acquiring && snr < self.snr_gate_db {
                self.stats.gated += 1;
                return false;
            }
        }
        if !acquiring && 10.0 * (5.0 / self.noise).log10() < self.snr_gate_db - TRACK_MARGIN_DB {
            self.stats.gated += 1;
            self.stats.frames += 1;
            self.stats.frames_bad += 1;
            return false;
        }
        let y: Vec<f32> = raw.iter().map(|x| (x - self.dc) / self.gain).collect();
        let mut llr = vec![0f32; 2 * FRAME_SYMBOLS];
        for (j, &v) in y.iter().enumerate() {
            self.llr(v, &mut llr[2 * j..2 * j + 2]);
        }
        // Descramble: a set scrambler bit flips that bit, so its LLR.
        for (i, l) in llr.iter_mut().enumerate() {
            if scrambler_bit(i) == 1 {
                *l = -*l;
            }
        }
        let lich = Lich((0..8).fold(0u8, |v, i| {
            (v << 1) | u8::from(llr[layout::LICH + 2 * i] < 0.0)
        }));
        let good = lich.parity_ok() && self.payload(lich, &llr);
        self.stats.frames += 1;
        if !good {
            self.stats.frames_bad += 1;
            return false;
        }
        if sync.is_none() {
            let sym = [3.0f32, 1.0, -1.0, -3.0];
            let ss: Vec<f32> = y
                .iter()
                .map(|&v| {
                    *sym.iter()
                        .min_by(|a, b| (v - *a).abs().total_cmp(&(v - *b).abs()))
                        .expect("four")
                })
                .collect();
            self.fit(&raw, &ss, 0.2);
        }
        self.stats.eye_snr_db = 10.0 * (5.0 / self.noise).log10();
        self.afc_step(AFC_GAIN * self.dc);
        let u = self.demod.units_per_hz();
        self.stats.offset_hz = self.demod.afc_hz() as f32 + self.dc / u;
        if let Some(tap) = &mut self.tap {
            tap.extend_from_slice(&y);
            let over = tap.len().saturating_sub(2 * FRAME_SYMBOLS);
            tap.drain(..over);
        }
        true
    }

    /// The SACCH and the payload. True if a CRC passed or the voice came out
    /// clean.
    fn payload(&mut self, lich: Lich, llr: &[f32]) -> bool {
        if lich.rfct() == Rfct::Rcch {
            // The control channel's CAC is not decoded yet: only the LICH
            // vouches for the frame.
            self.events.push(Event::ControlChannel { lich });
            return true;
        }
        let mut checked = false;
        match sacch_decode(&llr[layout::SACCH..layout::PAYLOAD]) {
            Some(b) => {
                checked = true;
                let structure = (b[0] << 1) | b[1];
                self.stats.ran = Some(b[2..8].iter().fold(0, |v, &x| (v << 1) | x));
                let part: [u8; 18] = b[8..26].try_into().expect("18 bits");
                if lich.fct() == 2 {
                    // Superframe: 1/4 has structure 3 … 4/4 has 0.
                    let idx = 3 - usize::from(structure);
                    if idx == 0 {
                        self.sacch = [None; 4];
                    }
                    self.sacch[idx] = Some(part);
                    if idx == 3 && self.sacch.iter().all(Option::is_some) {
                        let bits: Vec<u8> =
                            self.sacch.iter().flatten().flatten().copied().collect();
                        self.events.push(Event::Sacch {
                            msg: Message(pack(&bits)),
                        });
                        self.sacch = [None; 4];
                    }
                }
            }
            None => self.stats.crc_fail += 1,
        }
        let p = layout::PAYLOAD;
        let facch = |rx: &mut Receiver, at: usize| match facch1_decode(
            &llr[at..at + layout::FACCH1_BITS],
        ) {
            Some(b) => {
                rx.events.push(Event::Facch {
                    msg: Message(pack(&b)),
                });
                true
            }
            None => {
                rx.stats.crc_fail += 1;
                false
            }
        };
        let voice_at: &[usize] = match lich.steal() {
            Steal::None => &[p, p + 72, p + 144, p + 216],
            Steal::First => {
                checked |= facch(self, p);
                &[p + 144, p + 216]
            }
            Steal::Second => {
                checked |= facch(self, p + 144);
                &[p, p + 72]
            }
            Steal::Both => {
                // The same message twice: the second half is a spare.
                if !facch(self, p) {
                    checked |= facch(self, p + 144);
                } else {
                    checked = true;
                }
                &[]
            }
        };
        if !voice_at.is_empty()
            && matches!(lich.rfct(), Rfct::Rtch | Rfct::RtchC)
            && lich.fct() != 1
        {
            let frames: Vec<AmbeFrame> = voice_at
                .iter()
                .map(|&at| ambe::decode(llr[at..at + 72].try_into().expect("72")))
                .collect();
            let errors: u32 = frames.iter().map(|f| f.errors).sum();
            for f in &frames {
                self.stats.voice_frames += 1;
                self.stats.voice_bit_errors += u64::from(f.errors);
                self.stats.voice_bits += 46;
            }
            checked |= errors <= VOICE_ERRORS * frames.len() as u32 / 4;
            // Voice only from a frame that checked (a CRC passed here, or
            // the Golay words came out near): a flywheeled frame of noise
            // after the carrier drops must not be spoken.
            if checked {
                self.events.push(Event::Voice { frames });
            }
        }
        checked
    }
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

fn pack(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().fold(0u8, |v, &b| (v << 1) | b) << (8 - c.len()))
        .collect()
}
