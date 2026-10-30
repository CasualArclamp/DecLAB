// made by CasualArclamp
//! The P25 phase 1 receiver: channel IQ at 24 kS/s (five samples a symbol)
//! in, [`Event`]s out. Built as the DMR and NXDN receivers are: the
//! discriminator and matched filter with AFC (hunting wide, locked narrow),
//! a normalised correlator on the 24-symbol frame sync, an SNR gate on each
//! sync's own fit (its 24 symbols are all ±3: a good fit), max-log soft bits
//! into soft decoders.
//!
//! Frames follow one another with no gap, their lengths set by the NID's
//! DUID, so once a frame is decoded the next sync is looked for just after
//! it (a TSDU's padding makes the end a little uncertain).

use super::fec::{
    RS_24_12, RS_24_16, RS_36_20, from_hex, golay18_decode, golay24_decode, hamming_10_6,
    nid_decode,
};
use super::imbe_frame::ImbeFrame;
use super::{
    ALPHA, FS, Header, IMBE_AT, LDU_LC_AT, Lc, OUTER_DEV_HZ, SYMBOL_RATE, Tsbk, duid, frame_bits,
    imbe_frame, stripped,
};
use crate::dmr::data::trellis12_decode;
use crate::fsk4::{Detector, Fsk4Demod, Iq};

pub const RATE: f64 = 24_000.0;
const SPS: usize = 5;
/// The pre-detection low-pass once locked (`p25_sweep`: 3.5 kHz beat 3.2,
/// 4 and 5 kHz at the sensitivity limit and held up at 1.5 kHz off
/// tune), and while hunting.
pub const CHANNEL_HZ: f64 = 3_500.0;
const HUNT_CHANNEL_HZ: f64 = 5_500.0;
const ACQUIRE: f32 = 0.78;
const TRACK: f32 = 0.5;
/// The NID's hard decisions ML may overrule: on acquisition (every NAC
/// tried) and while locked (the NAC known).
const NID_DIST_ACQUIRE: u32 = 8;
const NID_DIST_TRACK: u32 = 14;
/// Frames in a row that fail before the receiver hunts again.
const LOST_AFTER: u32 = 4;
/// The SNR gate's default, dB. Measured (`p25_sweep`, 2026-10-29): 120 s of
/// noise gave only 12 sync candidates, the best fitting at 0.5 dB; real
/// frames at 4–6 dB C/N fit at a median of 11–14 dB.
pub const SNR_GATE_DB: f32 = 6.0;
const TRACK_MARGIN_DB: f32 = 6.0;
const AFC_GAIN: f32 = 0.3;
const AFC_SLICE: usize = 960;
/// Symbols past a frame's nominal end the next sync may be found at.
const SEARCH_AFTER: usize = 48;

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Header {
        header: Header,
    },
    /// Link control from an LDU1 or a TDULC.
    Lc {
        lc: Lc,
    },
    /// Encryption sync from an LDU2: (ALGID, KID); 0x80 is clear.
    Encryption {
        algid: u8,
        kid: u16,
    },
    /// An LDU's nine voice frames.
    Voice {
        frames: Vec<ImbeFrame>,
    },
    /// The end of a transmission (TDU or TDULC).
    End,
    Tsbk {
        tsbk: Tsbk,
    },
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
    pub nac: Option<u16>,
    pub locked: bool,
    /// The last frame's DUID.
    pub duid: Option<u8>,
}

/// A frame whose sync has been found: where it ends, and its NID once read.
#[derive(Clone, Copy)]
struct Pending {
    tau: f64,
    rho: f32,
    nid: Option<(u16, u8, u32)>,
}

pub struct Receiver {
    pub snr_gate_db: f32,
    chan_hz: f64,
    demod: Fsk4Demod,
    f: Vec<f32>,
    base: u64,
    scan: u64,
    peak: Option<(u64, f32)>,
    pending: Option<Pending>,
    /// Locked: frames missed in a row; the next frame's earliest sync end.
    track: Option<(u32, f64)>,
    fs: [f32; 24],
    dc: f32,
    gain: f32,
    noise: f32,
    pub stats: Stats,
    events: Vec<Event>,
    tap: Option<Vec<f32>>,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver {
    pub fn new() -> Receiver {
        Receiver::with_channel(CHANNEL_HZ)
    }

    /// With the locked low-pass at `chan_hz` (for the sweep).
    pub fn with_channel(chan_hz: f64) -> Receiver {
        let fs = std::array::from_fn(|i| crate::dmr::symbol(((FS >> (46 - 2 * i)) & 3) as u8));
        Receiver {
            snr_gate_db: SNR_GATE_DB,
            chan_hz,
            demod: Fsk4Demod::new(
                RATE,
                SYMBOL_RATE,
                OUTER_DEV_HZ,
                ALPHA,
                HUNT_CHANNEL_HZ,
                Detector::Discriminator,
            ),
            f: Vec::with_capacity(32_768),
            base: 0,
            scan: (23 * SPS) as u64,
            peak: None,
            pending: None,
            track: None,
            fs,
            dc: 0.0,
            gain: 1.0,
            noise: 0.25,
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
        *self = Receiver::with_channel(self.chan_hz);
        self.snr_gate_db = gate;
        self.enable_tap(tap);
    }

    pub fn process(&mut self, iq: &[Iq], out: &mut Vec<Event>) {
        for slice in iq.chunks(AFC_SLICE) {
            self.demod.process(slice, &mut self.f);
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
        let keep_from = match self.pending {
            Some(p) => p.tau - 2000.0,
            None => self.scan as f64 - 2000.0,
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
        let k = ((i as i64 - self.base as i64).max(0) as usize).min(self.f.len() - 1);
        let a = self.f[k];
        let b = self.f[(k + 1).min(self.f.len() - 1)];
        a + (b - a) * (t - i) as f32
    }

    fn correlate(&self, t: f64) -> f32 {
        let (mut c, mut s, mut s2) = (0f32, 0f32, 0f32);
        for (k, &sym) in self.fs.iter().enumerate() {
            let x = self.at(t - ((23 - k) * SPS) as f64);
            c += sym * x;
            s += x;
            s2 += x * x;
        }
        c / (216.0 * (s2 - s * s / 24.0).max(1e-9)).sqrt()
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

    /// One step: a pending frame decoded if it has arrived, else the search
    /// for the next sync moved on. False when it must wait for samples.
    fn step(&mut self) -> bool {
        if let Some(p) = self.pending {
            return self.step_pending(p);
        }
        let end = self.end();
        // While locked, the sync is looked for in a window from the frame's
        // nominal end, at the lower threshold; else anywhere, at the higher.
        let (thr, limit) = match self.track {
            Some((_, from)) => (TRACK, Some(from + (SEARCH_AFTER * SPS) as f64)),
            None => (ACQUIRE, None),
        };
        while self.scan + 1 < end {
            let t = self.scan;
            if let Some(lim) = limit
                && t as f64 > lim
            {
                // Nothing where the next frame should be.
                self.miss(t as f64);
                return true;
            }
            self.scan += 1;
            if t < self.base + (23 * SPS) as u64 {
                continue;
            }
            let r = self.correlate(t as f64);
            match self.peak {
                Some((pt, pr)) => {
                    if r > pr {
                        self.peak = Some((t, r));
                    } else if t - pt >= SPS as u64 {
                        self.peak = None;
                        self.pending = Some(Pending {
                            tau: self.refine(pt as f64),
                            rho: pr,
                            nid: None,
                        });
                        return true;
                    }
                }
                None if r >= thr => self.peak = Some((t, r)),
                None => {}
            }
        }
        false
    }

    fn miss(&mut self, at: f64) {
        if let Some((misses, _)) = self.track {
            self.stats.frames_bad += 1;
            if misses + 1 >= LOST_AFTER {
                self.lose();
            } else {
                // Look again from here, as if a frame of the shortest kind
                // had gone by unheard.
                self.track = Some((misses + 1, at + (frame_bits(duid::TDU, 0) / 2 * SPS) as f64));
            }
        }
        self.peak = None;
    }

    fn lose(&mut self) {
        self.track = None;
        self.stats.nac = None;
        self.demod.set_afc(0.0);
        self.demod.set_channel(HUNT_CHANNEL_HZ);
        self.dc = 0.0;
    }

    fn have(&self, tau: f64, symbols: usize) -> bool {
        let first = tau - 23.0 * SPS as f64 - 2.0;
        let last = tau + ((symbols - 23) * SPS) as f64 + 4.0;
        first >= self.base as f64 && last < self.end() as f64
    }

    /// The frame's soft bits, status symbols dropped (`n` symbols as sent).
    fn frame_llr(&mut self, tau: f64, n: usize) -> (Vec<f32>, Vec<f32>) {
        let raw: Vec<f32> = (0..n)
            .map(|j| self.at(tau + (j as f64 - 23.0) * SPS as f64))
            .collect();
        let mut llr = Vec::with_capacity(2 * n);
        let mut pair = [0f32; 2];
        for (j, &x) in raw.iter().enumerate() {
            if (2 * j) % 72 == 70 {
                continue;
            }
            self.llr((x - self.dc) / self.gain, &mut pair);
            llr.extend_from_slice(&pair);
        }
        (raw, llr)
    }

    fn step_pending(&mut self, mut p: Pending) -> bool {
        let acquiring = self.track.is_none();
        // The sync and NID first: 57 symbols (one status among them).
        if p.nid.is_none() {
            if !self.have(p.tau, 57) {
                return false;
            }
            let raw: Vec<f32> = (0..24)
                .map(|j| self.at(p.tau + (j as f64 - 23.0) * SPS as f64))
                .collect();
            let fs = self.fs;
            let snr = self
                .fit(&raw, &fs, if acquiring { 1.0 } else { 0.5 })
                .unwrap_or(f32::MIN);
            self.stats.sync_quality = p.rho;
            self.stats.sync_snr_db = snr;
            self.stats.syncs += 1;
            let gate = if acquiring {
                self.snr_gate_db
            } else {
                self.snr_gate_db - TRACK_MARGIN_DB
            };
            if snr < gate {
                self.stats.gated += 1;
                self.reject(p);
                return true;
            }
            let (_, llr) = self.frame_llr(p.tau, 57);
            let (nac, d, dist) = nid_decode(&llr[48..112], self.stats.nac);
            let max = if acquiring {
                NID_DIST_ACQUIRE
            } else {
                NID_DIST_TRACK
            };
            if dist > max || d == duid::PDU {
                self.reject(p);
                return true;
            }
            p.nid = Some((nac, d, dist));
            self.pending = Some(p);
        }
        let (nac, d, _) = p.nid.expect("read above");
        // A TSDU's length depends on its TSBKs: read up to three's worth.
        let n = frame_bits(d, if d == duid::TSDU { 3 } else { 0 }) / 2;
        if !self.have(p.tau, n) {
            return false;
        }
        self.pending = None;
        let (raw, llr) = self.frame_llr(p.tau, n);
        let (good, used_bits) = self.decode(d, &llr);
        self.stats.frames += 1;
        self.stats.duid = Some(d);
        let end = p.tau + ((used_bits / 2 - 23) * SPS) as f64;
        if good {
            if acquiring {
                self.afc_step(self.dc);
                self.demod.set_channel(self.chan_hz);
            }
            self.stats.nac = Some(nac);
            self.track = Some((0, end));
            let sym = [3.0f32, 1.0, -1.0, -3.0];
            let y: Vec<f32> = raw[..used_bits / 2]
                .iter()
                .map(|x| (x - self.dc) / self.gain)
                .collect();
            let ss: Vec<f32> = y
                .iter()
                .map(|&v| {
                    *sym.iter()
                        .min_by(|a, b| (v - *a).abs().total_cmp(&(v - *b).abs()))
                        .expect("four")
                })
                .collect();
            let raw_used = raw[..used_bits / 2].to_vec();
            self.fit(&raw_used, &ss, 0.2);
            self.stats.eye_snr_db = 10.0 * (5.0 / self.noise).log10();
            self.afc_step(AFC_GAIN * self.dc);
            let u = self.demod.units_per_hz();
            self.stats.offset_hz = self.demod.afc_hz() as f32 + self.dc / u;
            if let Some(tap) = &mut self.tap {
                tap.extend_from_slice(&y[..y.len().min(400)]);
                let over = tap.len().saturating_sub(800);
                tap.drain(..over);
            }
            self.scan = end as u64;
        } else {
            self.stats.frames_bad += 1;
            if acquiring {
                self.scan = self.scan.max(p.tau as u64 + SPS as u64);
            } else {
                // The NID said what came; look for the next frame after it.
                self.scan = end as u64;
                self.miss(end);
            }
        }
        true
    }

    fn reject(&mut self, p: Pending) {
        self.pending = None;
        if self.track.is_some() {
            self.scan = p.tau as u64 + SPS as u64;
        } else {
            self.scan = self.scan.max(p.tau as u64 + SPS as u64);
        }
    }

    /// Decode a frame's payload: (checked, bits it took as sent).
    fn decode(&mut self, d: u8, llr: &[f32]) -> (bool, usize) {
        let s = |raw: usize| stripped(raw);
        match d {
            duid::HDU => {
                let mut hex: Vec<u8> = (0..36)
                    .map(|k| golay18_decode(&llr[112 + 18 * k..130 + 18 * k]).0)
                    .collect();
                let ok = RS_36_20.decode(&mut hex).is_some();
                if ok {
                    let mut bits = Vec::with_capacity(120);
                    from_hex(&hex[..20], &mut bits);
                    let b: [u8; 15] = pack(&bits).try_into().expect("15 bytes");
                    self.events.push(Event::Header {
                        header: Header::parse(&b),
                    });
                } else {
                    self.stats.crc_fail += 1;
                }
                (ok, frame_bits(d, 0))
            }
            duid::LDU1 | duid::LDU2 => {
                let ham = hamming_10_6();
                let mut hex = Vec::with_capacity(24);
                for &raw in &LDU_LC_AT {
                    let at = s(raw);
                    for w in 0..4 {
                        hex.push(ham.ml(&llr[at + 10 * w..at + 10 * w + 10]).0 as u8);
                    }
                }
                let lc_ok = if d == duid::LDU1 {
                    let ok = RS_24_12.decode(&mut hex).is_some();
                    if ok {
                        let mut bits = Vec::new();
                        from_hex(&hex[..12], &mut bits);
                        let b: [u8; 9] = pack(&bits).try_into().expect("9 bytes");
                        self.events.push(Event::Lc { lc: Lc::parse(&b) });
                    }
                    ok
                } else {
                    let ok = RS_24_16.decode(&mut hex).is_some();
                    if ok {
                        let mut bits = Vec::new();
                        from_hex(&hex[..16], &mut bits);
                        let b = pack(&bits);
                        self.events.push(Event::Encryption {
                            algid: b[9],
                            kid: u16::from_be_bytes([b[10], b[11]]),
                        });
                    }
                    ok
                };
                if !lc_ok {
                    self.stats.crc_fail += 1;
                }
                let frames: Vec<ImbeFrame> = IMBE_AT
                    .iter()
                    .map(|&raw| imbe_frame::decode(&llr[s(raw)..s(raw) + 144]))
                    .collect();
                let errors: u32 = frames.iter().map(|f| f.errors).sum();
                for f in &frames {
                    self.stats.voice_frames += 1;
                    self.stats.voice_bit_errors += u64::from(f.errors);
                    self.stats.voice_bits += 137;
                }
                // Clean voice vouches for the frame when the link control
                // did not (an average of under 8 corrections a frame).
                let ok = lc_ok || errors < 9 * 8;
                if ok {
                    self.events.push(Event::Voice { frames });
                }
                (ok, frame_bits(d, 0))
            }
            duid::TDU => {
                self.events.push(Event::End);
                (true, frame_bits(d, 0))
            }
            duid::TDULC => {
                let mut hex = Vec::with_capacity(24);
                for w in 0..12 {
                    let (v, _) = golay24_decode(&llr[112 + 24 * w..136 + 24 * w]);
                    hex.push((v >> 6) as u8);
                    hex.push((v & 63) as u8);
                }
                let ok = RS_24_12.decode(&mut hex).is_some();
                if ok {
                    let mut bits = Vec::new();
                    from_hex(&hex[..12], &mut bits);
                    let b: [u8; 9] = pack(&bits).try_into().expect("9 bytes");
                    self.events.push(Event::Lc { lc: Lc::parse(&b) });
                } else {
                    self.stats.crc_fail += 1;
                }
                self.events.push(Event::End);
                (ok, frame_bits(d, 0))
            }
            _ => {
                // TSDU: TSBKs until one says it is the last.
                let mut n = 0;
                let mut ok = false;
                while n < 3 {
                    let at = 112 + 196 * n;
                    let block: [f32; 196] = llr[at..at + 196].try_into().expect("196");
                    let (b, _) = trellis12_decode(&block);
                    n += 1;
                    let mut bits = Vec::with_capacity(80);
                    crate::dmr::bptc::to_bits(&b[..10], &mut bits);
                    if crate::fec::crc_ccitt(&bits) != u16::from_be_bytes([b[10], b[11]]) {
                        self.stats.crc_fail += 1;
                        break;
                    }
                    ok = true;
                    let t = Tsbk::parse(&b);
                    self.events.push(Event::Tsbk { tsbk: t });
                    if b[0] & 0x80 != 0 {
                        break;
                    }
                }
                (ok, frame_bits(d, n))
            }
        }
    }

    fn afc_step(&mut self, delta: f32) {
        let u = self.demod.units_per_hz();
        self.demod
            .set_afc(self.demod.afc_hz() + f64::from(delta / u));
        for x in &mut self.f {
            *x -= delta;
        }
        self.dc -= delta;
    }

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
        let res =
            (sxx - 2.0 * g * sxs - 2.0 * dc * sx + g * g * sss + 2.0 * g * dc * ss_ + n * dc * dc)
                / n;
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

fn pack(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().fold(0u8, |v, &b| (v << 1) | b) << (8 - c.len()))
        .collect()
}
