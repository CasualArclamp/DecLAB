// made by CasualArclamp
//! The phase 2 receiver: a traffic channel's IQ at 24 kS/s (four samples a
//! symbol) in, [`Event`]s out.
//!
//! - **Front end:** AFC mixer, complex root-raised-cosine matched filter
//!   (α 0.2), then differential detection one symbol apart:
//!   arg(y[n]·y*[n−4]) × 4/π, so H-DQPSK's ±π/4, ±3π/4 steps come out as
//!   ±1, ±3, the units the phase 1 receiver works in (and a carrier offset
//!   as a level offset, 750 Hz a unit).
//! - **Hunt:** a normalised correlator on the 20-symbol S-ISCH (all outer
//!   symbols), the SNR gate on its fit, then the next timeslot's ISCH must
//!   decode too (an S-ISCH or one of the 128 I-ISCH codewords).
//! - **Locked:** every 180 symbols a timeslot: its ISCH decoded by ML over
//!   the 129 codewords keeps the timing (a parabola through the fit either
//!   side), the level and the AFC, and an I-ISCH says where in the
//!   superframe it is. The burst's DUID says what it carries.
//! - **Scrambling:** voice and the scrambled ACCHs are XORed with the
//!   superframe's sequence; until it is known (it is found from a clean
//!   ACCH, see [`super::scramble`]) voice is counted but not decoded.

use super::scramble::{Model, Scrambler};
use super::{
    CHANNEL_OF, DUID_AT, ESS_A_AT, ESS_B_AT, I_ISCH, MacPdu, S_ISCH, SLOT_SYMBOLS, SUPERFRAME,
    VOICE_AT, acch_dibits, acch_layout, crc12, duid, duid_codeword, has_s_isch, i_isch_timeslot,
    rs63, scramble, scrambled,
};
use crate::dmr::ambe::{self, AmbeFrame};
use crate::fsk4::{Iq, rrc};

pub const RATE: f64 = 24_000.0;
const SPS: usize = 4;
const SLOT: f64 = (SLOT_SYMBOLS * SPS) as f64;
const ACQUIRE: f32 = 0.78;
/// The SNR gate's default, dB: on the S-ISCH's fit when hunting.
pub const SNR_GATE_DB: f32 = 6.0;
/// ISCH bits (of 40) ML may overrule: on the check that confirms a lock,
/// and while locked (the code corrects 7).
const ISCH_DIST_VERIFY: u32 = 4;
const ISCH_DIST_TRACK: u32 = 7;
/// Timeslots in a row without an ISCH before the receiver hunts again.
const LOST_AFTER: u32 = 8;
const AFC_GAIN: f32 = 0.3;
const AFC_SLICE: usize = 960;
/// ACCH hexbits the second try erases (the least reliable).
const EXTRA_ERASURES: usize = 4;

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A voice burst's frames on logical channel `ch` (0 or 1).
    Voice { ch: u8, frames: Vec<AmbeFrame> },
    /// A MAC PDU that passed its CRC: from a FACCH (`fast`) or SACCH.
    Mac { ch: u8, fast: bool, pdu: MacPdu },
    /// The encryption sync from a 2V burst's ESS (ALGID 0x80 is clear).
    Encryption { ch: u8, algid: u8, kid: u16 },
    /// The scrambling's seed has been found (or was given).
    Scrambling { wacn: u32, system: u16, nac: u16 },
}

#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub timeslots: u64,
    /// Bursts whose DUID could not be read.
    pub bursts_bad: u64,
    pub voice_frames: u64,
    pub voice_bit_errors: u64,
    pub voice_bits: u64,
    /// Voice bursts heard before the scrambling was known.
    pub voice_held: u64,
    pub acch_ok: u64,
    pub crc_fail: u64,
    /// LCCH bursts: a phase 2 control channel (not decoded).
    pub lcch: u64,
    pub sync_quality: f32,
    pub sync_snr_db: f32,
    pub eye_snr_db: f32,
    pub offset_hz: f32,
    pub gated: u64,
    pub locked: bool,
    pub scrambling: Option<(u32, u16, u16)>,
    /// The superframe timeslot last decoded.
    pub timeslot: Option<usize>,
}

/// Seeds found so far in this process, tried first on any channel: a
/// trunked system's traffic channels share one (its WACN, system and NAC).
fn known_seeds() -> &'static std::sync::Mutex<Vec<u64>> {
    static S: std::sync::OnceLock<std::sync::Mutex<Vec<u64>>> = std::sync::OnceLock::new();
    S.get_or_init(Default::default)
}

/// Tell every phase 2 receiver in the process about a seed heard elsewhere
/// (a phase 1 control channel's network status, WACN and system, with its
/// NAC): tried first, so a followed call descrambles from its first burst.
pub fn remember_seed(seed: u64) {
    if let Ok(mut v) = known_seeds().lock()
        && !v.contains(&seed)
    {
        v.push(seed);
    }
}

/// Undo the scrambling of a burst's soft bits (from burst dibit 10, two a
/// dibit) in timeslot `ts`: flip the bits the sequence set. The DUID is
/// sent clear.
fn descramble(scr: &Scrambler, ts: usize, l: &mut [f32]) {
    for i in 11..170 {
        if DUID_AT.contains(&i) {
            continue;
        }
        let m = scr.dibit(ts, i);
        let at = 2 * (i - 10);
        if m & 2 != 0 {
            l[at] = -l[at];
        }
        if m & 1 != 0 {
            l[at + 1] = -l[at + 1];
        }
    }
}

/// The H-DQPSK front end (module notes).
struct Front {
    afc_step: f64,
    afc_phase: f64,
    taps: Vec<f32>,
    ring: Vec<Iq>,
    pos: usize,
    /// The last SPS filter outputs, for the one-symbol lag.
    lag: [Iq; SPS],
    lag_at: usize,
}

impl Front {
    fn new() -> Front {
        let taps = rrc(0.2, SPS, 6);
        let n = taps.len();
        Front {
            afc_step: 0.0,
            afc_phase: 0.0,
            taps,
            ring: vec![Iq::new(0.0, 0.0); 2 * n],
            pos: 0,
            lag: [Iq::new(0.0, 0.0); SPS],
            lag_at: 0,
        }
    }

    fn set_afc(&mut self, hz: f64) {
        self.afc_step = std::f64::consts::TAU * hz / RATE;
    }

    fn afc_hz(&self) -> f64 {
        self.afc_step * RATE / std::f64::consts::TAU
    }

    fn process(&mut self, iq: &[Iq], out: &mut Vec<f32>) {
        let n = self.taps.len();
        let k = 4.0 / std::f32::consts::PI;
        out.reserve(iq.len());
        for &x in iq {
            let x = if self.afc_step == 0.0 {
                x
            } else {
                self.afc_phase = (self.afc_phase - self.afc_step) % std::f64::consts::TAU;
                x * Iq::from_polar(1.0, self.afc_phase as f32)
            };
            // A ring kept twice over: each output one contiguous dot product.
            self.ring[self.pos] = x;
            self.ring[self.pos + n] = x;
            self.pos = (self.pos + 1) % n;
            let mut y = Iq::new(0.0, 0.0);
            for (s, &h) in self.ring[self.pos..self.pos + n].iter().zip(&self.taps) {
                y += s * h;
            }
            let prev = self.lag[self.lag_at];
            self.lag[self.lag_at] = y;
            self.lag_at = (self.lag_at + 1) % SPS;
            out.push((y * prev.conj()).arg() * k);
        }
    }
}

/// A timeslot boundary found by the hunt, waiting for its check.
#[derive(Clone, Copy)]
struct Candidate {
    tau: f64,
    rho: f32,
}

/// Locked: where the next timeslot starts (the sample of its first ISCH
/// symbol), which timeslot it is if known, and timeslots missed in a row.
#[derive(Clone, Copy)]
struct Track {
    tau: f64,
    ts: Option<usize>,
    misses: u32,
}

pub struct Receiver {
    pub snr_gate_db: f32,
    front: Front,
    f: Vec<f32>,
    base: u64,
    scan: u64,
    peak: Option<(u64, f32)>,
    cand: Option<Candidate>,
    track: Option<Track>,
    /// The ISCH codewords as symbols: S-ISCH first, then the 128 I-ISCH.
    isch: Vec<[f32; 20]>,
    /// The 16 DUID codewords.
    duids: [u8; 16],
    dc: f32,
    gain: f32,
    noise: f32,
    scr: Option<Scrambler>,
    /// Per channel: 4V bursts since the last 2V, and the ESS-B heard.
    vb: [usize; 2],
    ess_b: [[Option<u8>; 16]; 2],
    pub stats: Stats,
    events: Vec<Event>,
    tap: Option<Vec<f32>>,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

fn symbols(word: u64) -> [f32; 20] {
    std::array::from_fn(|k| crate::dmr::symbol(((word >> (38 - 2 * k)) & 3) as u8))
}

impl Receiver {
    pub fn new() -> Receiver {
        let mut isch = vec![symbols(S_ISCH)];
        isch.extend(I_ISCH.iter().map(|&w| symbols(w)));
        Receiver {
            snr_gate_db: SNR_GATE_DB,
            front: Front::new(),
            f: Vec::with_capacity(32_768),
            base: 0,
            scan: 0,
            peak: None,
            cand: None,
            track: None,
            isch,
            duids: std::array::from_fn(|d| duid_codeword(d as u8)),
            dc: 0.0,
            gain: 1.0,
            noise: 0.25,
            scr: None,
            vb: [0; 2],
            ess_b: [[None; 16]; 2],
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

    /// Use this seed (WACN · system · NAC, [`scramble::seed`]) rather than
    /// waiting to find it.
    pub fn set_seed(&mut self, seed: u64) {
        self.scr = Some(Scrambler::new(seed));
        let (wacn, system, nac) = scramble::split(seed);
        self.stats.scrambling = Some((wacn, system, nac));
    }

    pub fn reset(&mut self) {
        let (tap, gate) = (self.tap.is_some(), self.snr_gate_db);
        *self = Receiver::new();
        self.snr_gate_db = gate;
        self.enable_tap(tap);
    }

    pub fn process(&mut self, iq: &[Iq], out: &mut Vec<Event>) {
        for slice in iq.chunks(AFC_SLICE) {
            self.front.process(slice, &mut self.f);
            while self.step() {}
            self.trim();
        }
        self.stats.locked = self.track.is_some();
        out.append(&mut self.events);
    }

    fn end(&self) -> u64 {
        self.base + self.f.len() as u64
    }

    fn trim(&mut self) {
        let from = match (self.track, self.cand) {
            (Some(t), _) => t.tau,
            (None, Some(c)) => c.tau,
            _ => self.scan as f64,
        };
        let drop = ((from - 2000.0).max(0.0) as u64).saturating_sub(self.base) as usize;
        if drop > 16_384 {
            self.f.drain(..drop);
            self.base += drop as u64;
        }
    }

    #[inline]
    fn at(&self, t: f64) -> f32 {
        let i = t.floor();
        let k = ((i as i64 - self.base as i64).max(0) as usize).min(self.f.len() - 1);
        let a = self.f[k];
        let b = self.f[(k + 1).min(self.f.len() - 1)];
        a + (b - a) * (t - i) as f32
    }

    /// Normalised correlation of the 20 symbols from `t` with `s`.
    fn correlate(&self, t: f64, s: &[f32; 20]) -> f32 {
        let (mut c, mut sx, mut s2, mut ss) = (0f32, 0f32, 0f32, 0f32);
        for (k, &sym) in s.iter().enumerate() {
            let x = self.at(t + (k * SPS) as f64);
            c += sym * x;
            sx += x;
            s2 += x * x;
            ss += sym * sym;
        }
        c / (ss * (s2 - sx * sx / 20.0).max(1e-9)).sqrt()
    }

    /// The peak of the correlation with `s` near `t`, by a parabola through
    /// the samples either side.
    fn refine(&self, t: f64, s: &[f32; 20]) -> f64 {
        let (a, b, c) = (self.correlate(t - 1.0, s), self.correlate(t, s), self.correlate(t + 1.0, s));
        let den = a - 2.0 * b + c;
        let d = if den < 0.0 { (0.5 * (a - c) / den).clamp(-0.5, 0.5) } else { 0.0 };
        t + f64::from(d)
    }

    fn have(&self, tau: f64, symbols: usize) -> bool {
        tau - 2.0 >= self.base as f64 && tau + (symbols * SPS) as f64 + 4.0 < self.end() as f64
    }

    fn step(&mut self) -> bool {
        if let Some(t) = self.track {
            if !self.have(t.tau, SLOT_SYMBOLS) {
                return false;
            }
            self.timeslot(t);
            return true;
        }
        if let Some(c) = self.cand {
            // The candidate's timeslot and the next one's ISCH.
            if !self.have(c.tau, SLOT_SYMBOLS + 20) {
                return false;
            }
            self.cand = None;
            self.check(c);
            return true;
        }
        let s = self.isch[0];
        let end = self.end();
        while self.scan + (20 * SPS) as u64 + 2 < end {
            let t = self.scan;
            self.scan += 1;
            if (t as f64) < self.base as f64 + 3.0 {
                continue;
            }
            let r = self.correlate(t as f64, &s);
            match self.peak {
                Some((pt, pr)) => {
                    if r > pr {
                        self.peak = Some((t, r));
                    } else if t - pt >= SPS as u64 {
                        self.peak = None;
                        self.cand = Some(Candidate { tau: self.refine(pt as f64, &s), rho: pr });
                        return true;
                    }
                }
                None if r >= ACQUIRE => self.peak = Some((t, r)),
                None => {}
            }
        }
        false
    }

    /// A hunt's candidate: the gate on its fit, then the next ISCH.
    fn check(&mut self, c: Candidate) {
        let raw: Vec<f32> = (0..20).map(|k| self.at(c.tau + (k * SPS) as f64)).collect();
        let s = self.isch[0];
        let snr = self.fit(&raw, &s, 1.0).unwrap_or(f32::MIN);
        self.stats.sync_quality = c.rho;
        self.stats.sync_snr_db = snr;
        self.scan = self.scan.max(c.tau as u64 + SPS as u64);
        if snr < self.snr_gate_db {
            self.stats.gated += 1;
            return;
        }
        let Some((next, dist)) = self.isch_ml(c.tau + SLOT) else { return };
        if dist > ISCH_DIST_VERIFY {
            return;
        }
        // An I-ISCH after this S-ISCH names the timeslot after this one.
        let ts = (next > 0)
            .then(|| i_isch_timeslot(next - 1))
            .flatten()
            .map(|t1| (t1 + SUPERFRAME - 1) % SUPERFRAME)
            .filter(|&t0| has_s_isch(t0));
        if next > 0 && ts.is_none() {
            return;
        }
        // Centre the AFC on what the sync showed.
        self.afc_step(self.dc);
        self.track = Some(Track { tau: c.tau, ts, misses: 0 });
    }

    /// The ISCH from `t`: (codeword index, 0 = S-ISCH; bits ML overruled).
    fn isch_ml(&self, t: f64) -> Option<(usize, u32)> {
        let mut llr = [0f32; 40];
        for k in 0..20 {
            let mut p = [0f32; 2];
            self.llr((self.at(t + (k * SPS) as f64) - self.dc) / self.gain, &mut p);
            llr[2 * k] = p[0];
            llr[2 * k + 1] = p[1];
        }
        let words = std::iter::once(S_ISCH).chain(I_ISCH.iter().copied());
        let metric = |w: u64| -> f32 {
            llr.iter().enumerate().map(|(i, &l)| if (w >> (39 - i)) & 1 == 1 { -l } else { l }).sum()
        };
        let (best, w) = words.enumerate().max_by(|a, b| metric(a.1).total_cmp(&metric(b.1)))?;
        let hard = llr.iter().fold(0u64, |a, &l| (a << 1) | u64::from(l < 0.0));
        Some((best, (hard ^ w).count_ones()))
    }

    fn lose(&mut self) {
        self.track = None;
        self.front.set_afc(0.0);
        self.dc = 0.0;
        self.vb = [0; 2];
        self.ess_b = [[None; 16]; 2];
        self.stats.timeslot = None;
    }

    /// One locked timeslot.
    fn timeslot(&mut self, mut t: Track) {
        self.stats.timeslots += 1;
        let isch = self.isch_ml(t.tau).filter(|&(_, d)| d <= ISCH_DIST_TRACK);
        let prev = t.ts;
        let next = |p: Option<usize>| p.map(|p| (p + 1) % SUPERFRAME);
        match isch {
            Some((w, _)) => {
                t.misses = 0;
                let s = self.isch[w];
                t.tau = self.refine(t.tau, &s);
                let raw: Vec<f32> = (0..20).map(|k| self.at(t.tau + (k * SPS) as f64)).collect();
                if let Some(snr) = self.fit(&raw, &s, 0.3) {
                    self.stats.sync_snr_db = snr;
                }
                t.ts = if w == 0 {
                    next(prev).filter(|&x| has_s_isch(x))
                } else {
                    i_isch_timeslot(w - 1).or(next(prev))
                };
            }
            None => {
                t.misses += 1;
                if t.misses >= LOST_AFTER {
                    self.lose();
                    self.scan = (t.tau + SLOT) as u64;
                    return;
                }
                t.ts = next(prev);
            }
        }
        self.stats.timeslot = t.ts;
        // The timeslot's soft bits, two a symbol.
        let raw: Vec<f32> = (0..SLOT_SYMBOLS).map(|k| self.at(t.tau + (k * SPS) as f64)).collect();
        let y: Vec<f32> = raw.iter().map(|x| (x - self.dc) / self.gain).collect();
        let mut llr = vec![0f32; 2 * SLOT_SYMBOLS];
        for (k, &v) in y.iter().enumerate() {
            self.llr(v, &mut llr[2 * k..2 * k + 2]);
        }
        if isch.is_some() {
            // The eye over the whole timeslot (decisions as reference), and
            // the AFC from its level.
            let ss: Vec<f32> = y.iter().map(|&v| [3.0f32, 1.0, -1.0, -3.0].into_iter().min_by(|a, b| (v - a).abs().total_cmp(&(v - b).abs())).expect("four")).collect();
            self.fit(&raw, &ss, 0.2);
            self.stats.eye_snr_db = 10.0 * (5.0 / self.noise).log10();
            self.afc_step(AFC_GAIN * self.dc);
            self.stats.offset_hz = (self.front.afc_hz() + f64::from(self.dc) * 750.0) as f32;
            if let Some(tap) = &mut self.tap {
                tap.extend_from_slice(&y);
                let over = tap.len().saturating_sub(800);
                tap.drain(..over);
            }
        }
        // Burst dibit i (OP25's counting) is timeslot dibit i + 10. Only
        // behind an ISCH that decoded: a dead carrier's zeros read as a 4V
        // burst (that DUID's codeword is all zeros).
        if isch.is_some() {
            self.burst(t.ts, &llr[40..]);
        }
        t.tau += SLOT;
        self.track = Some(t);
    }

    /// One burst: `llr` from burst dibit 10 on (two a dibit).
    fn burst(&mut self, ts: Option<usize>, llr: &[f32]) {
        let b = |i: usize| 2 * (i - 10);
        // The DUID: ML over its 16 codewords, one bit's error allowed.
        let mut dl = [0f32; 8];
        for (k, &i) in DUID_AT.iter().enumerate() {
            dl[2 * k] = llr[b(i)];
            dl[2 * k + 1] = llr[b(i) + 1];
        }
        let metric = |w: u8| -> f32 { dl.iter().enumerate().map(|(i, &l)| if (w >> (7 - i)) & 1 == 1 { -l } else { l }).sum() };
        let (d, w) = self
            .duids
            .iter()
            .enumerate()
            .max_by(|a, b| metric(*a.1).total_cmp(&metric(*b.1)))
            .map(|(d, &w)| (d as u8, w))
            .expect("16");
        let hard = dl.iter().fold(0u8, |a, &l| (a << 1) | u8::from(l < 0.0));
        if (hard ^ w).count_ones() > 1 {
            self.stats.bursts_bad += 1;
            return;
        }
        let Some(ts) = ts else { return };
        let ch = CHANNEL_OF[ts];
        if d == duid::LCCH || d == duid::LCCH_S {
            self.stats.lcch += 1;
            return;
        }
        if !matches!(d, duid::V4 | duid::V2 | duid::SACCH_S | duid::FACCH_S | duid::SACCH | duid::FACCH) {
            return;
        }
        let fast = matches!(d, duid::FACCH | duid::FACCH_S);
        let mut l = llr.to_vec();
        if scrambled(d) {
            if self.scr.is_none() && !matches!(d, duid::V4 | duid::V2) {
                // A seed found before (another channel of the system, or
                // this one before a retune) if this ACCH decodes with it;
                // else try to find it from this ACCH's raw bits.
                let known = known_seeds().lock().map(|v| v.clone()).unwrap_or_default();
                let reuse = known.into_iter().map(Scrambler::new).find(|s| {
                    let mut l = llr.to_vec();
                    descramble(s, ts, &mut l);
                    self.acch_decode(fast, &l).is_some()
                });
                let bits: Vec<u8> = acch_dibits(fast)
                    .iter()
                    .flat_map(|&i| [u8::from(llr[b(i)] < 0.0), u8::from(llr[b(i) + 1] < 0.0)])
                    .collect();
                if let Some(s) = reuse.or_else(|| Model::get().recover(fast, ts, &bits)) {
                    if let Ok(mut v) = known_seeds().lock()
                        && !v.contains(&s.seed)
                    {
                        v.push(s.seed);
                    }
                    let (wacn, system, nac) = scramble::split(s.seed);
                    self.stats.scrambling = Some((wacn, system, nac));
                    self.events.push(Event::Scrambling { wacn, system, nac });
                    self.scr = Some(s);
                }
            }
            let Some(scr) = &self.scr else {
                if matches!(d, duid::V4 | duid::V2) {
                    self.stats.voice_held += 1;
                }
                return;
            };
            descramble(scr, ts, &mut l);
        }
        match d {
            duid::V4 | duid::V2 => self.voice(ch, d == duid::V4, &l),
            _ => self.acch(ch, fast, &l),
        }
    }

    fn voice(&mut self, ch: u8, four: bool, l: &[f32]) {
        let b = |i: usize| 2 * (i - 10);
        let n = if four { 4 } else { 2 };
        let frames: Vec<AmbeFrame> = VOICE_AT[..n]
            .iter()
            .map(|&at| ambe::decode(l[b(at)..b(at) + 72].try_into().expect("72")))
            .collect();
        for f in &frames {
            self.stats.voice_frames += 1;
            self.stats.voice_bit_errors += u64::from(f.errors);
            self.stats.voice_bits += 46;
        }
        let hex = |from: usize| -> u8 {
            (0..3).fold(0u8, |a, j| {
                let i = from + j;
                (a << 2) | (u8::from(l[b(i)] < 0.0) << 1) | u8::from(l[b(i) + 1] < 0.0)
            })
        };
        let c = usize::from(ch);
        if four {
            let part = self.vb[c].min(3);
            for k in 0..4 {
                self.ess_b[c][4 * part + k] = Some(hex(ESS_B_AT + 3 * k));
            }
            self.vb[c] += 1;
        } else {
            let mut word = [0u8; rs63::N];
            let mut erasures = Vec::new();
            for (k, h) in self.ess_b[c].iter().enumerate() {
                match h {
                    Some(v) => word[19 + k] = *v,
                    None => erasures.push(19 + k),
                }
            }
            let mut at = ESS_A_AT;
            for k in 0..28 {
                word[35 + k] = hex(at);
                at += if k == 15 { 4 } else { 3 };
            }
            if rs63::decode(&mut word, &erasures, 19).is_some() {
                let algid = (word[19] << 2) | (word[20] >> 4);
                let kid = (u16::from(word[20] & 15) << 12) | (u16::from(word[21]) << 6) | u16::from(word[22]);
                self.events.push(Event::Encryption { ch, algid, kid });
            }
            self.vb[c] = 0;
            self.ess_b[c] = [None; 16];
        }
        self.events.push(Event::Voice { ch, frames });
    }

    fn acch(&mut self, ch: u8, fast: bool, l: &[f32]) {
        match self.acch_decode(fast, l) {
            Some(pdu) => {
                self.stats.acch_ok += 1;
                self.events.push(Event::Mac { ch, fast, pdu });
            }
            None => self.stats.crc_fail += 1,
        }
    }

    /// An ACCH's MAC PDU, if its RS and CRC pass.
    fn acch_decode(&self, fast: bool, l: &[f32]) -> Option<MacPdu> {
        let b = |i: usize| 2 * (i - 10);
        let (j0, sent, len) = acch_layout(fast);
        let dib = acch_dibits(fast);
        let soft: Vec<f32> = dib.iter().flat_map(|&i| [l[b(i)], l[b(i) + 1]]).collect();
        let mut word = [0u8; rs63::N];
        for k in 0..sent {
            word[j0 + k] = soft[6 * k..6 * k + 6].iter().fold(0u8, |a, &x| (a << 1) | u8::from(x < 0.0));
        }
        let punctured: Vec<usize> = (j0 + sent..rs63::N).collect();
        let mut w = word;
        let mut ok = rs63::decode(&mut w, &punctured, j0).is_some();
        if !ok {
            // Again with the least reliable hexbits erased too.
            let mut rel: Vec<(f32, usize)> = (0..sent)
                .map(|k| (soft[6 * k..6 * k + 6].iter().fold(f32::MAX, |m, x| m.min(x.abs())), j0 + k))
                .collect();
            rel.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut er = punctured.clone();
            er.extend(rel.iter().take(EXTRA_ERASURES).map(|&(_, k)| k));
            w = word;
            ok = rs63::decode(&mut w, &er, j0).is_some();
        }
        let bits: Vec<u8> = w[j0..j0 + (len + 12) / 6]
            .iter()
            .flat_map(|&h| (0..6).rev().map(move |i| (h >> i) & 1))
            .collect();
        let crc = bits[len..len + 12].iter().fold(0u16, |a, &x| (a << 1) | u16::from(x));
        if !ok || crc12(&bits[..len]) != crc {
            return None;
        }
        let bytes: Vec<u8> = bits[..len].chunks(8).map(|c| c.iter().fold(0u8, |a, &x| (a << 1) | x)).collect();
        Some(MacPdu::parse(&bytes))
    }

    fn afc_step(&mut self, delta: f32) {
        self.front.set_afc(self.front.afc_hz() + f64::from(delta) * 750.0);
        for x in &mut self.f {
            *x -= delta;
        }
        self.dc -= delta;
    }

    /// Least-squares level and offset of `xs` against symbols `ss`: the
    /// fit's SNR, and (weighted) the receiver's gain, offset and noise.
    fn fit(&mut self, xs: &[f32], ss: &[f32], weight: f32) -> Option<f32> {
        let n = xs.len() as f32;
        let (sx, s_, sxx, sss, sxs) = xs.iter().zip(ss).fold((0f32, 0f32, 0f32, 0f32, 0f32), |a, (&x, &s)| {
            (a.0 + x, a.1 + s, a.2 + x * x, a.3 + s * s, a.4 + x * s)
        });
        let den = n * sss - s_ * s_;
        if den.abs() < 1e-6 {
            // All one symbol (the S-ISCH nearly is): fit the gain alone.
            let g = sxs / sss.max(1e-6);
            if g <= 0.05 {
                return None;
            }
            let res = (sxx - 2.0 * g * sxs + g * g * sss) / n;
            let nv = (res / (g * g)).max(0.01);
            self.gain += weight * (g - self.gain);
            self.noise += weight * (nv - self.noise);
            return Some(10.0 * (5.0 / nv).log10());
        }
        let g = (n * sxs - sx * s_) / den;
        if g <= 0.05 {
            return None;
        }
        let dc = (sx - g * s_) / n;
        let res = (sxx - 2.0 * g * sxs - 2.0 * dc * sx + g * g * sss + 2.0 * g * dc * s_ + n * dc * dc) / n;
        let nv = (res / (g * g)).max(0.01);
        self.gain += weight * (g - self.gain);
        self.dc += weight * (dc - self.dc);
        self.noise += weight * (nv - self.noise);
        Some(10.0 * (5.0 / nv).log10())
    }

    #[inline]
    fn llr(&self, y: f32, out: &mut [f32]) {
        let k = 1.0 / (2.0 * self.noise);
        let d = |s: f32| (y - s) * (y - s);
        let (p3, p1, m1, m3) = (d(3.0), d(1.0), d(-1.0), d(-3.0));
        out[0] = (m1.min(m3) - p3.min(p1)) * k;
        out[1] = (p3.min(m3) - p1.min(m1)) * k;
    }
}

#[cfg(test)]
mod tests {
    use super::super::sim::{self, Call};
    use super::super::{MacMsg, MacOp};
    use super::*;

    fn run(iq: &[Iq]) -> (Vec<Event>, Stats) {
        let mut rx = Receiver::new();
        let mut ev = Vec::new();
        for c in iq.chunks(2_400) {
            rx.process(c, &mut ev);
        }
        (ev, rx.stats)
    }

    #[test]
    fn decodes_two_calls_and_finds_the_seed() {
        let seed = scramble::seed(0xBEE00, 0x3A1, 0x293);
        let calls = [
            Some(Call { group: 4001, source: 1_234_567, algid: 0x80 }),
            Some(Call { group: 77, source: 7_654_321, algid: 0x80 }),
        ];
        let (dibits, frames) = sim::channel(seed, calls, 8);
        let mut iq = sim::modulate(&dibits, RATE, 0.1);
        sim::add_noise(&mut iq, RATE, 25.0, 300.0, 3);
        let (ev, st) = run(&iq);
        assert!(ev.contains(&Event::Scrambling { wacn: 0xBEE00, system: 0x3A1, nac: 0x293 }), "{st:?}");
        let users: Vec<(u8, u16, u32)> = ev
            .iter()
            .filter_map(|e| match e {
                Event::Mac { ch, pdu, .. } => pdu.messages.iter().find_map(|m| match m {
                    MacMsg::GroupVoice { group, source, .. } => Some((*ch, *group, *source)),
                    _ => None,
                }),
                _ => None,
            })
            .collect();
        assert!(users.contains(&(0, 4001, 1_234_567)), "{users:?}");
        assert!(users.contains(&(1, 77, 7_654_321)), "{users:?}");
        assert!(ev.iter().any(|e| matches!(e, Event::Mac { pdu, .. } if pdu.op == MacOp::Ptt && pdu.ptt.is_some_and(|p| p.1 == 4001))));
        assert!(ev.iter().any(|e| matches!(e, Event::Encryption { ch: 1, algid: 0x80, kid: 0x0101 })), "{st:?}");
        let voice: usize = ev.iter().map(|e| if let Event::Voice { frames, .. } = e { frames.len() } else { 0 }).sum();
        // Voice before the seed was found is held, not decoded; the PTT
        // superframe's ACCHs come first, so none should be lost.
        assert_eq!(voice as u32, frames, "{st:?}");
        assert_eq!(st.voice_bit_errors, 0, "{st:?}");
        let bits_ok = ev.iter().all(|e| match e {
            Event::Voice { frames, .. } => frames.iter().all(|f| f.errors == 0),
            _ => true,
        });
        assert!(bits_ok);
    }

    #[test]
    fn the_frames_are_the_ones_sent() {
        let seed = scramble::seed(1, 0x18, 0x293);
        let (dibits, _) = sim::channel(seed, [Some(Call { group: 5, source: 6, algid: 0x80 }), None], 2);
        let iq = sim::modulate(&dibits, RATE, 0.1);
        let (ev, _) = run(&iq);
        let got: Vec<[u8; 49]> = ev
            .iter()
            .filter_map(|e| if let Event::Voice { frames, .. } = e { Some(frames.iter().map(|f| f.bits).collect::<Vec<_>>()) } else { None })
            .flatten()
            .collect();
        let want: Vec<[u8; 49]> = (1..=got.len() as u32).map(crate::dmr::sim::test_frame).collect();
        assert!(!got.is_empty());
        let bad: Vec<usize> = (0..got.len()).filter(|&i| got[i] != want[i]).collect();
        assert!(bad.is_empty(), "{} frames, wrong: {bad:?}", got.len());
        assert_eq!(got.len(), 36);
    }

    #[test]
    fn stays_quiet_on_noise() {
        let mut iq = vec![Iq::new(0.0, 0.0); (RATE * 20.0) as usize];
        sim::add_noise(&mut iq, RATE, -100.0, 0.0, 11);
        // Unit-power noise in place of the carrier.
        let p = iq.iter().map(|z| z.norm_sqr()).sum::<f32>() / iq.len() as f32;
        iq.iter_mut().for_each(|z| *z /= p.sqrt());
        let (ev, st) = run(&iq);
        assert!(ev.is_empty(), "{ev:?}");
        assert_eq!(st.timeslots, 0, "{st:?}");
    }
}
