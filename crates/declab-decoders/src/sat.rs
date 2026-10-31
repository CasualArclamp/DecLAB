// made by CasualArclamp
//! Satellite: DVB-S, DVB-S2 and DVB-S2X, carrying a transport stream or GSE
//! (IP). DecSAT's receiver (`decsat-rx`, `decsat-frame`, `decsat-fec`,
//! `decsat-ts`, `decsat-gse`, copied from `..\DecDVB`) as a DecLAB decoder.
//!
//! What it does, end to end:
//!
//! ```text
//!  channel IQ ─► identify (symbol rate, roll-off, carrier, DVB-S2 or not)
//!       │            │ DVB-S2/S2X: PL demodulator ─► PLFRAMEs ──────────┐
//!       │            │ DVB-S:      QPSK demodulator ─► Viterbi + RS ───┐│
//!       ▼            ▼                                                  ││
//!   (re-identify    worker thread: LDPC, BCH, BBHEADER, every input     ││
//!    if lock lost)  stream (ISI) to its own stage:                      ││
//!                      transport packets ─► Sink::ts (+ PSI/SI)  ◄──────┘│
//!                      GSE / GSE-HEM ─► IP datagrams ─► Sink::ip  ◄──────┘
//! ```
//!
//! Like DecSAT's VFO the decoder finds everything itself; the settings
//! (symbol rate, scrambling code) override what it finds. The FEC runs on
//! a thread of its own so the demodulator keeps real time: from a radio a
//! full queue drops frames (counted), from a file the reader waits.

use std::collections::BTreeMap;
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::thread::JoinHandle;

use decsat_core::{Iq, Modulation, modcod_name, qef_es_n0_db};
use decsat_frame::{BBHEADER_LEN, PlsInfo};
use decsat_gse::{GseIp, HemDeframer};
use decsat_modem::dvbs::{DvbsRx, DvbsStats};
use decsat_rx::demod::{Demod, LockState, PlFrame};
use decsat_rx::fec::FecDecoder;
use decsat_rx::identify::{Identification, Verdict, identify_in};
use decsat_rx::psk::PskDemod;
use decsat_rx::streams::StreamContent;
use decsat_ts::{TS_LEN, TsAnalyser, TsDeframer, service_type_name};
use declab_api::{
    Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Lock, Service, Sink,
};

/// The card is sent this often (seconds of stream time).
const REPORT_S: f64 = 1.0 / 30.0;
/// The services list, this often.
const SERVICES_S: f64 = 1.0;
/// Constellation points kept for the scope.
const SCOPE_POINTS: usize = 4096;
/// Frames queued for the FEC thread before a live source's are dropped.
const QUEUE: usize = 64;
/// No frame for this long (stream seconds): look at the signal again.
const LOST_S: f64 = 4.0;
/// Rest between identifications that found nothing (seconds of stream).
const RETRY_S: f64 = 0.5;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "sat",
            name: "DVB-S/S2/S2X",
            family: Family::Satellite,
            standard: "EN 300 421 / EN 302 307-1/-2 / TS 102 606",
            summary: "Satellite TV and data: transport stream or GSE/IP, found blind",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // The channel filter's width is the user's to drag (carriers run from
        // 100 kS/s to the whole band); the host's rate for it is what the
        // demodulator gets.
        InputSpec { sample_rate: None, bandwidth_hz: 5.0e6, block_len: 65_536 }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Sat::default())
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(panel::SatPanel::default()))
    }
}

/// The settings panel: what the receiver otherwise finds blind.
#[cfg(feature = "ui")]
mod panel {
    use declab_api::{Command, Panel};

    #[derive(Default)]
    pub struct SatPanel {
        /// Symbol rate, kS/s as typed (empty: found blind).
        rate: String,
        /// DVB-S2 physical layer scrambling code, as typed.
        gold: String,
    }

    impl SatPanel {
        fn set(key: &str, value: &str) -> Command {
            Command::Set { key: key.into(), value: value.into() }
        }

        /// The symbol rate in Hz for the decoder ("" when not a number).
        fn rate_hz(&self) -> String {
            self.rate.trim().parse::<f64>().map_or(String::new(), |k| format!("{}", k * 1e3))
        }
    }

    impl Panel for SatPanel {
        fn settings(&self) -> Vec<Command> {
            vec![Self::set("symbol_rate", &self.rate_hz()), Self::set("gold", self.gold.trim())]
        }

        fn restore(&mut self, key: &str, value: &str) {
            match key {
                "symbol_rate" => {
                    self.rate = value.parse::<f64>().map_or(String::new(), |hz| format!("{}", hz / 1e3));
                }
                "gold" => self.gold = value.to_string(),
                _ => {}
            }
        }

        fn ui(&mut self, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
            ui.label(
                "Found blind from the signal. Set the symbol rate when the carrier is too weak for that                  (a DVB-S2 carrier down to about -2 dB Es/N0 decodes once the rate is given).",
            );
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Symbol rate, kS/s");
                if ui.add(egui::TextEdit::singleline(&mut self.rate).desired_width(90.0).hint_text("auto")).lost_focus() {
                    commands.push(Self::set("symbol_rate", &self.rate_hz()));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Scrambling code");
                if ui.add(egui::TextEdit::singleline(&mut self.gold).desired_width(90.0).hint_text("0")).lost_focus() {
                    commands.push(Self::set("gold", self.gold.trim()));
                }
            });
        }
    }
}

// ---------------------------------------------------------------------------
// The FEC thread

/// What the demodulator hands the worker.
enum Job {
    Frame(PlFrame),
    /// Carrier-locked QPSK symbols of a DVB-S carrier.
    Symbols(Vec<Iq>),
}

/// A snapshot of one input stream, for the card.
#[derive(Debug, Clone, Default, PartialEq)]
struct StreamInfo {
    isi: u8,
    content: String,
    frames: u64,
    ts_packets: u64,
    ts_crc_errors: u64,
    ip_packets: u64,
}

/// What the worker has done.
#[derive(Debug, Clone, Default, PartialEq)]
struct Stats {
    /// Data frames decoded (dummy frames excluded).
    frames: u64,
    ok: u64,
    bch_failed: u64,
    header_failed: u64,
    dropped: u64,
    pls: Option<PlsInfo>,
    es_n0_db: f32,
    ts_packets: u64,
    ip_packets: u64,
    streams: Vec<StreamInfo>,
    dvbs: Option<DvbsStats>,
    dvbs_rate: String,
}

/// What comes back from the worker.
#[derive(Default)]
struct Done {
    ts: Vec<[u8; TS_LEN]>,
    ip: Vec<Vec<u8>>,
    stats: Stats,
    services: Option<Vec<Service>>,
    text: Vec<(u32, String)>,
    log: Vec<String>,
}

struct Worker {
    tx: Option<SyncSender<Job>>,
    rx: Receiver<Done>,
    join: Option<JoinHandle<()>>,
    lossless: bool,
    dropped: u64,
}

impl Worker {
    fn spawn(lossless: bool) -> Worker {
        let (tx, jobs) = mpsc::sync_channel::<Job>(QUEUE);
        let (done_tx, rx) = mpsc::channel::<Done>();
        let join = std::thread::Builder::new()
            .name("sat-fec".into())
            .spawn(move || {
                let mut stage = Stage::new(!lossless);
                // Results go back when the queue is empty or a little work
                // has piled up, so the GUI is not flooded.
                let mut pending = Done::default();
                let mut last = std::time::Instant::now();
                loop {
                    let job = match jobs.try_recv() {
                        Ok(j) => j,
                        Err(mpsc::TryRecvError::Empty) => {
                            if stage.dirty {
                                stage.finish(&mut pending);
                                if done_tx.send(std::mem::take(&mut pending)).is_err() {
                                    return;
                                }
                                last = std::time::Instant::now();
                            }
                            match jobs.recv() {
                                Ok(j) => j,
                                Err(_) => return,
                            }
                        }
                        Err(mpsc::TryRecvError::Disconnected) => return,
                    };
                    match job {
                        Job::Frame(f) => stage.frame(&f, &mut pending),
                        Job::Symbols(s) => stage.dvbs_symbols(&s, &mut pending),
                    }
                    if last.elapsed().as_millis() > 100 {
                        stage.finish(&mut pending);
                        if done_tx.send(std::mem::take(&mut pending)).is_err() {
                            return;
                        }
                        last = std::time::Instant::now();
                    }
                }
            })
            .expect("spawn the FEC thread");
        Worker { tx: Some(tx), rx, join: Some(join), lossless, dropped: 0 }
    }

    fn offer(&mut self, job: Job) {
        let Some(tx) = &self.tx else { return };
        if self.lossless {
            let _ = tx.send(job);
        } else if let Err(TrySendError::Full(_)) = tx.try_send(job) {
            self.dropped += 1;
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        // Closing the queue ends the thread's loop.
        self.tx = None;
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// One input stream's stages.
struct StreamStage {
    content: StreamContent,
    frames: u64,
    deframer: TsDeframer,
    gse: GseIp,
    hem: HemDeframer,
    ip_packets: u64,
}

/// Everything after the demodulator, on the FEC thread.
struct Stage {
    fec: FecDecoder,
    streams: BTreeMap<u8, StreamStage>,
    /// The stream whose transport packets go out (the first met).
    primary: Option<u8>,
    analyser: TsAnalyser,
    dvbs: DvbsRx,
    dvbs_ts: Vec<[u8; TS_LEN]>,
    stats: Stats,
    dirty: bool,
    services_at: std::time::Instant,
    last_services: Vec<Service>,
    last_programme_text: BTreeMap<u32, String>,
    packets: Vec<[u8; TS_LEN]>,
    ip_scratch: Vec<decsat_gse::IpPacket>,
    whole: Vec<u8>,
}

impl Stage {
    fn new(hurry: bool) -> Stage {
        let mut fec = FecDecoder::new();
        fec.hurry = hurry;
        Stage {
            fec,
            streams: BTreeMap::new(),
            primary: None,
            analyser: TsAnalyser::new(),
            dvbs: DvbsRx::new(),
            dvbs_ts: Vec::new(),
            stats: Stats::default(),
            dirty: false,
            services_at: std::time::Instant::now(),
            last_services: Vec::new(),
            last_programme_text: BTreeMap::new(),
            packets: Vec::new(),
            ip_scratch: Vec::new(),
            whole: Vec::new(),
        }
    }

    fn reset(&mut self) {
        for s in self.streams.values_mut() {
            s.deframer.discontinuity();
            s.hem.discontinuity();
        }
        self.dvbs = DvbsRx::new();
        self.dirty = true;
    }

    fn frame(&mut self, f: &PlFrame, out: &mut Done) {
        if f.after_gap {
            self.reset();
        }
        let Some(b) = self.fec.decode(f) else { return };
        self.dirty = true;
        self.stats.frames += 1;
        self.stats.pls = Some(b.pls);
        self.stats.es_n0_db = b.es_n0_db;
        if b.bch.is_err() {
            self.stats.bch_failed += 1;
            return;
        }
        let Ok(h) = &b.header else {
            self.stats.header_failed += 1;
            return;
        };
        self.stats.ok += 1;
        let isi = if h.single_stream { 0 } else { h.isi };
        let end = (BBHEADER_LEN + h.dfl as usize / 8).min(b.bytes.len());
        let content = StreamContent::of(h);
        let st = self.streams.entry(isi).or_insert_with(|| StreamStage {
            content,
            frames: 0,
            deframer: TsDeframer::new(),
            gse: GseIp::new(),
            hem: HemDeframer::new(),
            ip_packets: 0,
        });
        st.content = content;
        st.frames += 1;
        if content.is_ts() {
            let primary = *self.primary.get_or_insert(isi);
            self.packets.clear();
            st.deframer.data_field(&b.bytes[BBHEADER_LEN..end], h.syncd, h.npd, h.issyi, &mut self.packets);
            if isi == primary {
                for p in &self.packets {
                    self.analyser.packet(p);
                }
                out.ts.extend_from_slice(&self.packets);
                self.stats.ts_packets += self.packets.len() as u64;
            }
        } else if content == StreamContent::Gse {
            self.ip_scratch.clear();
            st.gse.bbframe(&b.bytes[..end], &mut self.ip_scratch);
            st.ip_packets += self.ip_scratch.len() as u64;
            self.stats.ip_packets += self.ip_scratch.len() as u64;
            out.ip.extend(self.ip_scratch.drain(..).map(|p| p.data));
        } else if content == StreamContent::GseHem {
            self.whole.clear();
            st.hem.data_field(&b.bytes[BBHEADER_LEN..end], h.syncd, &mut self.whole);
            self.ip_scratch.clear();
            st.gse.data_field(&self.whole, &mut self.ip_scratch);
            st.ip_packets += self.ip_scratch.len() as u64;
            self.stats.ip_packets += self.ip_scratch.len() as u64;
            out.ip.extend(self.ip_scratch.drain(..).map(|p| p.data));
        }
    }

    fn dvbs_symbols(&mut self, s: &[Iq], out: &mut Done) {
        self.dvbs_ts.clear();
        self.dvbs.push(s, &mut self.dvbs_ts);
        for p in &self.dvbs_ts {
            self.analyser.packet(p);
        }
        self.stats.ts_packets += self.dvbs_ts.len() as u64;
        out.ts.append(&mut self.dvbs_ts);
        self.dirty = true;
    }

    /// Refresh the snapshot parts that are computed, not counted.
    fn finish(&mut self, out: &mut Done) {
        self.dirty = false;
        self.stats.streams = self
            .streams
            .iter()
            .map(|(&isi, s)| StreamInfo {
                isi,
                content: s.content.label(),
                frames: s.frames,
                ts_packets: s.deframer.stats.packets,
                ts_crc_errors: s.deframer.stats.crc_errors,
                ip_packets: s.ip_packets,
            })
            .collect();
        let d = &self.dvbs.stats;
        self.stats.dvbs = (d.rate.is_some() || d.packets > 0).then(|| d.clone());
        self.stats.dvbs_rate = d.rate
            .map(|r| {
                use decsat_modem::conv::Rate;
                match r {
                    Rate::R1_2 => "1/2",
                    Rate::R2_3 => "2/3",
                    Rate::R3_4 => "3/4",
                    Rate::R5_6 => "5/6",
                    Rate::R7_8 => "7/8",
                }
                .to_string()
            })
            .unwrap_or_default();
        out.stats = self.stats.clone();
        if self.services_at.elapsed().as_secs_f64() >= SERVICES_S && self.stats.ts_packets > 0 {
            self.services_at = std::time::Instant::now();
            let report = self.analyser.report();
            let mut services: Vec<Service> = report
                .programmes
                .iter()
                .filter(|p| p.number != 0)
                .map(|p| Service {
                    id: u32::from(p.number),
                    name: p.name.clone().unwrap_or_default(),
                    provider: p.provider.clone().unwrap_or_default(),
                    kind: p.service_type.map_or("", service_type_name).to_string(),
                })
                .collect();
            services.sort_by_key(|s| s.id);
            if services != self.last_services {
                out.services = Some(services.clone());
                self.last_services = services;
            }
            for p in &report.programmes {
                let mut line = String::new();
                if let Some(e) = &p.now {
                    line = format!("Now: {}", e.name);
                }
                if let Some(e) = &p.next {
                    line = format!("{line}{}Next: {}", if line.is_empty() { "" } else { " · " }, e.name);
                }
                let id = u32::from(p.number);
                if !line.is_empty() && self.last_programme_text.get(&id) != Some(&line) {
                    out.text.push((id, line.clone()));
                    self.last_programme_text.insert(id, line);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The decoder

enum Mode {
    /// Collecting samples for a look at the signal.
    Gather,
    /// DVB-S2/S2X: the PL demodulator feeds the worker.
    S2 { demod: Box<Demod> },
    /// DVB-S: QPSK symbols feed the worker.
    S1 { demod: Box<PskDemod> },
}

/// What the identification found, kept for the card.
#[derive(Default, Clone)]
struct Found {
    symbol_rate: f64,
    roll_off: f64,
    s2x: bool,
    hint: String,
    snr_db: f32,
}

struct Sat {
    /// Settings (None: found blind).
    symbol_rate: Option<f64>,
    gold_code: u32,
    mode: Mode,
    rate: f64,
    buf: Vec<Iq>,
    /// After a failure, the other of DVB-S2 and DVB-S is tried next.
    try_s1: bool,
    /// Samples to gather before the next look (0: the first look's).
    target: usize,
    /// The carrier's offset from the channel centre, removed by an
    /// oscillator running across blocks (phase in cycles).
    shift_hz: f64,
    shift_phase: f64,
    shifted: Vec<Iq>,
    worker: Option<Worker>,
    frames: Vec<PlFrame>,
    syms: Vec<(u8, Iq)>,
    found: Found,
    /// Stream seconds since start; next report, retry.
    t: f64,
    report_at: f64,
    retry_at: f64,
    last_frame_t: f64,
    stats: Stats,
    points: Vec<Iq>,
    live: bool,
    ts_total: u64,
    ts_rate: (f64, u64, f64),
    mbps: f64,
}

impl Default for Sat {
    fn default() -> Self {
        Sat {
            symbol_rate: None,
            gold_code: 0,
            mode: Mode::Gather,
            rate: 0.0,
            buf: Vec::new(),
            target: 0,
            try_s1: false,
            shift_hz: 0.0,
            shift_phase: 0.0,
            shifted: Vec::new(),
            worker: None,
            frames: Vec::new(),
            syms: Vec::new(),
            found: Found::default(),
            t: 0.0,
            report_at: 0.0,
            retry_at: 0.0,
            last_frame_t: 0.0,
            stats: Stats::default(),
            points: Vec::new(),
            live: false,
            ts_total: 0,
            ts_rate: (0.0, 0, 0.0),
            mbps: 0.0,
        }
    }
}

/// A first look: ~0.6 s of signal, but never more than 2 s, and enough for
/// a narrow carrier to give its symbol rate (as DecSAT's VFO).
fn first_look(rate: f64) -> usize {
    ((rate * 0.6) as usize).clamp((rate * 2.0).min(300_000.0) as usize, 2_000_000)
}

/// Samples holding two of the longest PLFRAMEs at `rs`: what it takes to see
/// a header confirmed on its frame grid wherever the listen starts.
fn enough_frames(rate: f64, rs: f64) -> usize {
    (2.2 * decsat_frame::MAX_PLFRAME_LEN as f64 * rate / rs) as usize
}

impl Sat {
    fn restart(&mut self) {
        self.mode = Mode::Gather;
        self.buf.clear();
        self.target = 0;
        self.worker = None;
        self.shift_hz = 0.0;
        self.shift_phase = 0.0;
        self.stats = Stats::default();
        self.found = Found::default();
        self.points.clear();
        self.ts_total = 0;
        self.ts_rate = (self.t, 0, 0.0);
        self.mbps = 0.0;
    }

    /// Take the carrier to DC (the demodulators expect it there).
    fn mix(&mut self, x: &[Iq]) -> Vec<Iq> {
        let mut shifted = std::mem::take(&mut self.shifted);
        shifted.clear();
        if self.shift_hz == 0.0 {
            shifted.extend_from_slice(x);
            return shifted;
        }
        let w = -self.shift_hz / self.rate;
        shifted.reserve(x.len());
        // An oscillator by rotation: one complex multiply per sample, not a
        // sin/cos; renormalised per block against drift.
        let step = Iq::new((std::f64::consts::TAU * w).cos() as f32, (std::f64::consts::TAU * w).sin() as f32);
        let ph = std::f64::consts::TAU * self.shift_phase;
        let mut osc = Iq::new(ph.cos() as f32, ph.sin() as f32);
        for &s in x {
            shifted.push(s * osc);
            osc *= step;
        }
        let n = osc.norm();
        if n > 0.0 {
            osc /= n;
        }
        self.shift_phase = (osc.im.atan2(osc.re) as f64) / std::f64::consts::TAU;
        shifted
    }

    fn look(&mut self, out: &mut Sink) {
        let mut id = identify_in(&self.buf, self.rate, None);
        // Too weak for Identify, but the symbol rate is given: the carrier is
        // where the channel is, and the PL demodulator finds the headers.
        let weak = self.symbol_rate.is_some() && matches!(id.verdict, Verdict::NoSignal);
        if weak {
            id.center_offset_hz = 0.0;
            id.carrier_offset_hz = None;
            id.verdict = Verdict::NotDvbS2 { hint: "weak signal".into() };
        }
        let usable = !matches!(id.verdict, Verdict::NoSignal);
        let rs = self.symbol_rate.or(id.symbol_rate).filter(|&r| usable && self.rate / r >= 2.0);
        self.target = 0;
        self.found = describe(&id, rs);
        let Some(rs) = rs else {
            self.buf.clear();
            self.retry_at = self.t + RETRY_S;
            return;
        };
        let alpha = id.roll_off.map_or(0.35, |r| r.as_f64());
        self.found.roll_off = alpha;
        // Not DVB-S2 yet, but the listen was too short to see two frames at
        // this rate: listen longer before judging.
        let needed = enough_frames(self.rate, rs).min(8_000_000);
        if !matches!(id.verdict, Verdict::DvbS2(_)) && self.buf.len() < needed {
            self.target = needed;
            return;
        }
        match &id.verdict {
            Verdict::DvbS2(_) => self.start_s2(rs, alpha, &id, out),
            Verdict::NotDvbS2 { hint } if !is_other(hint) && id.constellation.is_some_and(|c| c.modulation() == Modulation::Qpsk) && self.try_s1 => {
                // QPSK that Identify did not find S2 frames in: DVB-S if the
                // Viterbi search finds a rate (the worker says).
                let seed = id.carrier_offset_hz.unwrap_or(0.0) / rs;
                self.shift_hz = id.center_offset_hz;
                self.shift_phase = 0.0;
                self.worker = Some(Worker::spawn(!self.live));
                let buf = std::mem::take(&mut self.buf);
                let mut d = Box::new(PskDemod::new(self.rate, rs, alpha, Modulation::Qpsk, seed));
                let shifted = self.mix(&buf);
                self.syms.clear();
                d.process(&shifted, &mut self.syms);
                self.shifted = shifted;
                self.offer_symbols();
                self.mode = Mode::S1 { demod: d };
                out.log(format!("QPSK found: {:.3} MS/s, trying DVB-S", rs / 1e6));
            }
            Verdict::NotDvbS2 { hint } if !is_other(hint) => {
                // Not recognised as S2, but a carrier of a known symbol rate:
                // the PL demodulator searches for headers itself and gets
                // down to lower SNR than Identify's first look does.
                self.start_s2(rs, alpha, &id, out);
            }
            _ => {
                self.buf.clear();
                self.retry_at = self.t + RETRY_S;
                return;
            }
        }
        self.last_frame_t = self.t;
    }

    /// The PL demodulator on the carrier Identify centred.
    fn start_s2(&mut self, rs: f64, alpha: f64, id: &Identification, out: &mut Sink) {
        let mut d = Demod::new(self.rate, rs, alpha, self.gold_code);
        if let Some(f) = id.carrier_offset_hz {
            d = d.with_carrier_offset(f / rs);
        }
        self.shift_hz = id.center_offset_hz;
        self.shift_phase = 0.0;
        self.worker = Some(Worker::spawn(!self.live));
        let buf = std::mem::take(&mut self.buf);
        let mut d = Box::new(d);
        self.frames.clear();
        let shifted = self.mix(&buf);
        d.process(&shifted, &mut self.frames);
        self.shifted = shifted;
        self.mode = Mode::S2 { demod: d };
        self.offer_frames();
        out.log(format!("DVB-S2: {:.3} MS/s, roll-off {alpha:.2}", rs / 1e6));
    }

    fn offer_frames(&mut self) {
        let Some(w) = &mut self.worker else { return };
        for f in self.frames.drain(..) {
            w.offer(Job::Frame(f));
        }
    }

    fn offer_symbols(&mut self) {
        let Some(w) = &mut self.worker else { return };
        if !self.syms.is_empty() {
            w.offer(Job::Symbols(self.syms.iter().map(|s| s.1).collect()));
        }
    }

    /// Everything the worker has finished, into the sink.
    fn collect(&mut self, out: &mut Sink) {
        let Some(w) = &mut self.worker else { return };
        while let Ok(d) = w.rx.try_recv() {
            self.ts_total += d.ts.len() as u64;
            if !d.ts.is_empty() && matches!(self.mode, Mode::S1 { .. }) {
                self.last_frame_t = self.t;
            }
            out.ts.extend(d.ts);
            out.ip.extend(d.ip);
            if let Some(s) = d.services {
                out.services = Some(s);
            }
            out.text.extend(d.text);
            out.log.extend(d.log);
            // Progress: frames that decoded (a false lock on noise finds
            // headers that never do).
            if d.stats.ok > self.stats.ok && matches!(self.mode, Mode::S2 { .. }) {
                self.last_frame_t = self.t;
            }
            self.stats = d.stats;
        }
        self.stats.dropped = w.dropped;
    }

    fn report(&mut self, out: &mut Sink) {
        // The TS rate over the last second.
        let (t0, n0, _) = self.ts_rate;
        if self.t - t0 >= 1.0 {
            self.mbps = (self.ts_total - n0) as f64 * TS_LEN as f64 * 8.0 / (self.t - t0) / 1e6;
            self.ts_rate = (self.t, self.ts_total, self.mbps);
        }
        let s = &self.stats;
        let (lock, tracking) = match &self.mode {
            Mode::Gather => (Lock::Searching, false),
            Mode::S2 { demod } => {
                let l = demod.lock_state();
                let locked = s.ok > 0 && self.t - self.last_frame_t < 2.0;
                (
                    if locked {
                        Lock::Locked
                    } else if l != LockState::Searching {
                        Lock::Synced
                    } else {
                        Lock::Searching
                    },
                    l != LockState::Searching,
                )
            }
            Mode::S1 { .. } => {
                let locked = s.dvbs.as_ref().is_some_and(|d| d.packets > 0) && self.ts_total > 0;
                (if locked { Lock::Locked } else { Lock::Synced }, true)
            }
        };
        out.lock = lock;
        let _ = tracking;

        let f = &self.found;
        let mut card = Card::default();
        let mut details: Vec<(String, String)> = Vec::new();
        if f.symbol_rate > 0.0 {
            details.push(("Symbol rate".into(), format!("{:.3} MS/s", f.symbol_rate / 1e6)));
            details.push(("Roll-off".into(), format!("{:.2}", f.roll_off)));
        }
        match &self.mode {
            Mode::Gather => {
                card.title = "Satellite".into();
                card.headline = "Searching".into();
                if !f.hint.is_empty() {
                    card.text.push(("Seen".into(), f.hint.clone()));
                }
                if f.snr_db > 0.0 {
                    details.push(("Peak over floor".into(), format!("{:.0} dB", f.snr_db)));
                }
            }
            Mode::S2 { demod } => {
                let es = if s.es_n0_db.is_finite() { s.es_n0_db } else { 0.0 };
                let std = if f.s2x || s.pls.is_some_and(|p| p.modcod >= 32) { "DVB-S2X" } else { "DVB-S2" };
                card.title = format!("{std} · {:.2} MS/s", f.symbol_rate / 1e6);
                match s.pls {
                    Some(p) => {
                        card.headline = format!(
                            "{}{}",
                            modcod_name(p.modcod),
                            if p.short_fecframe { " short" } else { "" }
                        );
                        let need = qef_es_n0_db(p.modcod);
                        card.gauge = Some(Gauge {
                            label: "Es/N0".into(),
                            value_db: es,
                            need_db: need,
                            need_label: "needed".into(),
                            range: (-4.0, 24.0),
                        });
                        details.push(("Pilots".into(), if p.has_pilots { "yes" } else { "no" }.into()));
                    }
                    None => card.headline = "Waiting for frames".into(),
                }
                details.push(("Carrier".into(), format!("{:+.0} Hz", demod.carrier_offset_hz())));
                details.push(("Frames".into(), format!("{} ok of {}", s.ok, s.frames)));
                details.push((
                    "FEC failures".into(),
                    format!("{} BCH · {} header", s.bch_failed, s.header_failed),
                ));
                if s.dropped > 0 {
                    details.push(("Dropped".into(), format!("{} frames", s.dropped)));
                }
            }
            Mode::S1 { demod } => {
                card.title = format!("DVB-S · {:.2} MS/s", f.symbol_rate / 1e6);
                card.headline = if s.dvbs_rate.is_empty() {
                    "Looking for the code rate".into()
                } else {
                    format!("QPSK {}", s.dvbs_rate)
                };
                details.push(("MER".into(), format!("{:.1} dB", demod.mer_db())));
                if let Some(d) = &s.dvbs {
                    details.push(("RS".into(), format!("{} fixed · {} failed", d.rs_corrected, d.rs_failed)));
                    details.push(("Channel BER".into(), format!("{:.1e}", d.channel_ber)));
                }
            }
        }
        if s.ts_packets > 0 {
            details.push(("Transport".into(), format!("{:.2} Mb/s · {} packets", self.mbps, s.ts_packets)));
        }
        for st in &s.streams {
            let name = if s.streams.len() > 1 { format!("Stream {}", st.isi) } else { "Stream".into() };
            let mut v = st.content.clone();
            if st.ts_packets > 0 {
                v = format!("{v} · {} TS", st.ts_packets);
            }
            if st.ip_packets > 0 {
                v = format!("{v} · {} IP", st.ip_packets);
            }
            if st.ts_crc_errors > 0 {
                v = format!("{v} · {} CRC", st.ts_crc_errors);
            }
            card.text.push((name, v));
        }
        card.details = details;
        out.card = Some(card);

        if !self.points.is_empty() {
            out.constellation.extend_from_slice(&self.points);
        }
        if s.frames > 0 {
            out.measure("Es/N0", s.es_n0_db as f64, "dB");
        }
        if self.mbps > 0.0 {
            out.measure("TS rate", self.mbps, "Mb/s");
        }
    }

    fn keep_points(&mut self, syms: impl Iterator<Item = Iq>) {
        for s in syms {
            self.points.push(s);
        }
        if self.points.len() > SCOPE_POINTS {
            let cut = self.points.len() - SCOPE_POINTS;
            self.points.drain(..cut);
        }
    }
}

/// An identification that names something DecSAT reads with another
/// decoder: not DVB-S, so QPSK should not be tried as it.
fn is_other(hint: &str) -> bool {
    decsat_rx::identify::names_is19(hint)
        || decsat_rx::identify::names_is19_call(hint)
        || decsat_rx::identify::names_beacon(hint)
        || decsat_rx::identify::names_tpc(hint)
}

fn describe(id: &Identification, rs: Option<f64>) -> Found {
    let mut f = Found {
        symbol_rate: rs.unwrap_or(0.0),
        roll_off: id.roll_off.map_or(0.0, |r| r.as_f64()),
        snr_db: id.snr_db,
        ..Found::default()
    };
    match &id.verdict {
        Verdict::NoSignal => f.hint = "no signal".into(),
        Verdict::Carrier => f.hint = "an unmodulated carrier".into(),
        Verdict::DvbS2(d) => {
            f.s2x = d.is_s2x();
            f.hint = "DVB-S2".into();
        }
        Verdict::NotDvbS2 { hint } => f.hint = hint.clone(),
    }
    f
}

impl Decoder for Sat {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        let rate = block.tuning.sample_rate;
        if self.rate != rate {
            self.rate = rate;
            self.restart();
        }
        self.live = block.live;
        self.t = (block.first_sample + block.iq.len() as u64) as f64 / rate;

        match &mut self.mode {
            Mode::Gather => {
                if self.t >= self.retry_at {
                    self.buf.extend_from_slice(block.iq);
                    if self.buf.len() >= self.target.max(first_look(rate)) {
                        self.look(out);
                    }
                }
            }
            Mode::S2 { .. } => {
                let x = self.mix(block.iq);
                let Mode::S2 { demod } = &mut self.mode else { unreachable!() };
                self.frames.clear();
                demod.process(&x, &mut self.frames);
                let pts = demod.recent_symbols(SCOPE_POINTS);
                self.keep_points(pts.into_iter());
                self.offer_frames();
                self.shifted = x;
            }
            Mode::S1 { .. } => {
                let x = self.mix(block.iq);
                let Mode::S1 { demod } = &mut self.mode else { unreachable!() };
                self.syms.clear();
                demod.process(&x, &mut self.syms);
                let pts = demod.recent();
                self.keep_points(pts.into_iter());
                self.offer_symbols();
                self.shifted = x;
            }
        }
        self.collect(out);

        // Lock lost for good: look at the signal again.
        if !matches!(self.mode, Mode::Gather) && self.t - self.last_frame_t > LOST_S {
            out.log("lock lost: looking again");
            // Nothing ever decoded: the other standard next time.
            let nothing = self.stats.ok == 0 && self.ts_total == 0;
            let try_s1 = self.try_s1 != nothing;
            self.restart();
            self.try_s1 = try_s1;
            self.retry_at = self.t;
        }

        if self.t >= self.report_at {
            self.report_at = self.t + REPORT_S;
            self.report(out);
        }
        let _ = SERVICES_S;
    }

    fn command(&mut self, cmd: Command, _out: &mut Sink) {
        if let Command::Set { key, value } = cmd {
            match key.as_str() {
                "symbol_rate" => {
                    self.symbol_rate = value.trim().parse::<f64>().ok().filter(|&r| r > 0.0);
                    self.restart();
                }
                "gold" => {
                    self.gold_code = value.trim().parse().unwrap_or(0);
                    self.restart();
                }
                _ => {}
            }
        }
    }

    fn reset(&mut self) {
        self.restart();
        self.retry_at = 0.0;
    }

    fn wants_threads(&self) -> usize {
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use decsat_core::RollOff;
    use decsat_gse::Variant;
    use decsat_mod::{FrameSpec, GseBbFramer, PlFramer, Shaper};

    /// A carrier of `rs` S/s at 4 samples a symbol, `offset` Hz off centre.
    fn carrier(mut framer: PlFramer, spec: FrameSpec, rs: f64, seconds: f64, offset: f64) -> Vec<Iq> {
        framer.set_roll_off(RollOff::R20);
        let syms = framer.build_schedule(&[spec], (rs * seconds) as usize);
        let mut x = Vec::new();
        Shaper::new(4, 0.20, 16).process(&syms, &mut x);
        let w = std::f64::consts::TAU * offset / (rs * 4.0);
        x.iter()
            .enumerate()
            .map(|(n, s)| *s * Iq::new((w * n as f64).cos() as f32, (w * n as f64).sin() as f32))
            .collect()
    }

    /// Feed `x` through the decoder in blocks, as the host does from a file.
    fn run(x: &[Iq], rate: f64) -> Sink {
        let mut dec = Sat::default();
        let mut all = Sink::default();
        let mut sink = Sink::default();
        let mut at = 0u64;
        for chunk in x.chunks(65_536) {
            let block = Block {
                iq: chunk,
                tuning: Tuning { center_hz: 0.0, sample_rate: rate, generation: 0 },
                first_sample: at,
                dropped_before: 0,
                live: false,
            };
            at += chunk.len() as u64;
            dec.process(&block, &mut sink);
            all.ts.append(&mut sink.ts);
            all.ip.append(&mut sink.ip);
            sink.clear();
        }
        // The worker thread finishes its queue as the decoder drops.
        drop(dec.worker.take());
        all
    }

    #[test]
    fn decodes_a_transport_stream_off_centre() {
        let x = carrier(PlFramer::new(0, 3), FrameSpec::new(4, false, true), 1e6, 3.0, 90_000.0);
        let out = run(&x, 4e6);
        assert!(out.ts.len() > 1000, "{} packets", out.ts.len());
        assert!(out.ts.iter().all(|p| p[0] == 0x47));
    }

    #[test]
    fn decodes_gse_to_ip() {
        let framer = PlFramer::new(0, 1).with_source(Box::new(GseBbFramer::new(1, Variant::STANDARD)));
        let x = carrier(framer, FrameSpec::new(1, false, true), 1e6, 3.0, -50_000.0);
        let out = run(&x, 4e6);
        assert!(out.ip.len() > 50, "{} datagrams", out.ip.len());
        assert!(out.ip.iter().all(|p| p[0] >> 4 == 4));
    }
}
