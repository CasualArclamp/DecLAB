// made by CasualArclamp
//! The decoder host: everything around a plug-in decoder.
//!
//! Two threads, plus the waterfall's own:
//!
//! - **Source thread** (`declab-source`): reads the [`IqSource`], records if
//!   asked, takes the DC offset (and the image, if asked) off, feeds the
//!   waterfall, keeps the level readings, and hands each block to the
//!   decoder's worker. A file is paced to real time unless told otherwise.
//! - **Decoder worker** (`declab-decoder`): one per running decoder. Owns the
//!   down-converter (NCO mix, decimating FIR, then a rational resampler to the
//!   exact rate the decoder asked for), calls [`Decoder::process`], and
//!   empties the [`Sink`] into what the GUI reads and the sound card plays.
//!
//! Blocks go from one to the other as `Arc<Vec<Iq>>` from a small pool, so a
//! block is shared, not copied, and the source thread reuses the buffers the
//! worker has finished with. A live radio never waits on the worker: if it
//! falls behind, blocks are dropped and counted. A file does wait, so a file
//! decoded flat out (the CLI) loses nothing.

use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use declab_api::{
    Block, Command, Decoder, DecoderFactory, InputSpec, Lock, Measurement, Picture, Service, Sink,
    Tuning,
};
use declab_core::{Iq, SampleFormat};
use declab_dsp::{Ddc, Resampler};
use declab_io::{IqFileWriter, IqSource, format_from_path};

use crate::audio::AudioOut;
use crate::frontend::{FrontEnd, FrontEndReading, FrontEndSettings};
use crate::meter::{Stage, StageLoad, StageMeter};
use crate::waterfall::Waterfall;

/// Blocks queued for the decoder worker before a live source drops them.
const WORKER_QUEUE: usize = 16;
/// Log lines kept for the GUI.
const MAX_LOG: usize = 300;
/// Panel messages kept for a GUI that has not collected them.
const MAX_CUSTOM: usize = 64;

/// How the host runs.
#[derive(Debug, Clone, Copy)]
pub struct HostOptions {
    /// Waterfall FFT size.
    pub fft: usize,
    pub rows_per_s: f64,
    /// FFT windows averaged into each waterfall row.
    pub segments_per_row: u32,
    /// Read a file at the rate it was recorded (the GUI); off reads it as
    /// fast as the decoder keeps up (the CLI).
    pub pace_files: bool,
    /// Start a file again when it ends.
    pub loop_files: bool,
    /// Make waterfall rows at all (the CLI does not need them).
    pub waterfall: bool,
}

impl Default for HostOptions {
    fn default() -> Self {
        HostOptions {
            fft: 4096,
            rows_per_s: 25.0,
            segments_per_row: 16,
            pace_files: true,
            loop_files: true,
            waterfall: true,
        }
    }
}

/// One line of the waterfall.
#[derive(Debug, Clone)]
pub struct Row {
    /// dBFS per bin, the most negative frequency first.
    pub db: Vec<f32>,
    pub center_hz: f64,
    pub sample_rate: f64,
    /// Counts retunes: a row from before one does not line up with the
    /// rows after it.
    pub tuning: u32,
}

/// The source side, refreshed every second.
#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub describe: String,
    pub live: bool,
    pub sample_rate: f64,
    pub center_hz: f64,
    pub samples: u64,
    /// RMS level of the input, dBFS (a full-scale complex sample is 0).
    pub rms_dbfs: f32,
    /// Highest |I| or |Q| in the second, as a fraction of full scale.
    pub peak: f32,
    /// Samples at the ADC's limit, percent: the radio is overloaded.
    pub clip_percent: f32,
    /// Blocks the decoder worker was too busy to take.
    pub handoff_dropped: u64,
    pub front_end: FrontEndReading,
    pub stages: StageLoad,
    /// The file has ended (and is not looping), or the radio failed.
    pub ended: bool,
    pub error: Option<String>,
    /// Bytes written to the recording, if one is running.
    pub recording: Option<(PathBuf, u64)>,
}

/// How the decoder's channel is cut out of the wideband stream.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChannelInfo {
    /// Centre relative to the source's centre, Hz.
    pub offset_hz: f64,
    pub bandwidth_hz: f64,
    /// The rate the decoder is fed.
    pub out_rate: f64,
    /// The FIR's decimation (1: none).
    pub decimation: usize,
    /// The resampler's up/down (None: not needed).
    pub resample: Option<(usize, usize)>,
}

/// What the running decoder has said, for the GUI.
#[derive(Debug, Clone, Default)]
pub struct DecoderView {
    pub id: &'static str,
    pub name: &'static str,
    pub lock: Lock,
    /// The latest value of each measurement, in the order first seen.
    pub measurements: Vec<Measurement>,
    pub constellation: Vec<Iq>,
    /// The decoder's other scopes, the latest points of each.
    pub scopes: Vec<(&'static str, Vec<Iq>)>,
    pub log: VecDeque<String>,
    pub services: Vec<Service>,
    pub text: HashMap<u32, String>,
    /// The latest picture of each service.
    pub pictures: HashMap<u32, Picture>,
    pub channel: ChannelInfo,
    /// The worker thread's busy time (down-converter + decoder), percent of
    /// one core.
    pub cpu_percent: f32,
    /// Sound output in use, or why there is none.
    pub audio: Option<String>,
}

/// What the source thread is asked to do.
enum SourceCmd {
    Retune(f64),
    RetuneBy(f64),
    Record(Option<PathBuf>),
    FrontEnd(FrontEndSettings),
    Stop,
}

/// Where a block came from.
#[derive(Clone, Copy)]
struct BlockMeta {
    center_hz: f64,
    sample_rate: f64,
    generation: u32,
    dropped_before: u64,
}

/// Where the decoder's channel sits.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Target {
    /// This far from the source's centre, wherever the source is tuned.
    Offset(f64),
    /// At this RF frequency: the offset follows the source's retunes, so a
    /// station stays in the channel while the band is moved under it.
    At(f64),
}

impl Target {
    fn offset(self, center_hz: f64) -> f64 {
        match self {
            Target::Offset(o) => o,
            Target::At(f) => f - center_hz,
        }
    }
}

enum WorkerMsg {
    Block(Arc<Vec<Iq>>, BlockMeta),
    Command(Command),
    Channel { target: Target, bandwidth_hz: f64 },
    Stop,
}

/// State both threads and the GUI share.
struct Shared {
    rows: Arc<Mutex<VecDeque<Row>>>,
    stats: Mutex<Stats>,
    view: Mutex<DecoderView>,
    custom: Mutex<VecDeque<Box<dyn std::any::Any + Send>>>,
    /// The running worker's queue, if a decoder is running.
    worker_tx: Mutex<Option<SyncSender<WorkerMsg>>>,
    /// Blocks dropped because the worker's queue was full.
    handoff_dropped: Mutex<u64>,
    volume: AtomicU32,
    running: AtomicBool,
    /// Blocks handed to the worker and not yet decoded.
    in_flight: AtomicUsize,
}

struct Worker {
    tx: SyncSender<WorkerMsg>,
    join: JoinHandle<()>,
}

/// The running host. Dropping it stops both threads and the radio.
pub struct Host {
    shared: Arc<Shared>,
    cmd: Sender<SourceCmd>,
    source_join: Option<JoinHandle<()>>,
    worker: Option<Worker>,
    source_rate: f64,
    live: bool,
}

impl Host {
    /// Start reading `source`. No decoder runs until [`Host::set_decoder`].
    pub fn start(source: Box<dyn IqSource>, opts: HostOptions) -> Host {
        let source_rate = source.sample_rate();
        let live = source.is_live();
        let shared = Arc::new(Shared {
            rows: Arc::new(Mutex::new(VecDeque::new())),
            stats: Mutex::new(Stats::default()),
            view: Mutex::new(DecoderView::default()),
            custom: Mutex::new(VecDeque::new()),
            worker_tx: Mutex::new(None),
            handoff_dropped: Mutex::new(0),
            volume: AtomicU32::new(0.8f32.to_bits()),
            running: AtomicBool::new(true),
            in_flight: AtomicUsize::new(0),
        });
        let (cmd, cmd_rx) = mpsc::channel();
        let sh = shared.clone();
        let source_join = std::thread::Builder::new()
            .name("declab-source".into())
            .spawn(move || source_thread(source, opts, cmd_rx, sh))
            .expect("cannot start the source thread");
        Host {
            shared,
            cmd,
            source_join: Some(source_join),
            worker: None,
            source_rate,
            live,
        }
    }

    pub fn source_rate(&self) -> f64 {
        self.source_rate
    }

    /// A radio rather than a file.
    pub fn is_live(&self) -> bool {
        self.live
    }

    /// Run `factory`'s decoder (stopping the one running), or none. Its
    /// channel starts centred, as wide as it asks for.
    pub fn set_decoder(&mut self, factory: Option<&'static dyn DecoderFactory>) {
        if let Some(w) = self.worker.take() {
            *self.shared.worker_tx.lock().unwrap() = None;
            let _ = w.tx.send(WorkerMsg::Stop);
            let _ = w.join.join();
        }
        *self.shared.view.lock().unwrap() = DecoderView::default();
        self.shared.custom.lock().unwrap().clear();
        let Some(factory) = factory else { return };

        let info = factory.info();
        let spec = factory.input(self.source_rate);
        {
            let mut v = self.shared.view.lock().unwrap();
            v.id = info.id;
            v.name = info.name;
        }
        let decoder = factory.create();
        let (tx, rx) = mpsc::sync_channel(WORKER_QUEUE);
        let sh = self.shared.clone();
        let cmd = self.cmd.clone();
        let join = std::thread::Builder::new()
            .name("declab-decoder".into())
            .spawn(move || worker_thread(decoder, spec, rx, sh, cmd))
            .expect("cannot start the decoder thread");
        *self.shared.worker_tx.lock().unwrap() = Some(tx.clone());
        self.worker = Some(Worker { tx, join });
    }

    /// Move or resize the decoder's channel.
    pub fn set_channel(&self, offset_hz: f64, bandwidth_hz: f64) {
        if let Some(w) = &self.worker {
            let _ = w.tx.send(WorkerMsg::Channel { target: Target::Offset(offset_hz), bandwidth_hz });
        }
    }

    /// Put the decoder's channel at RF frequency `hz` and keep it there
    /// across retunes (the GUI's way: the channel stays on its station while
    /// the band is dragged).
    pub fn set_channel_at(&self, hz: f64, bandwidth_hz: f64) {
        if let Some(w) = &self.worker {
            let _ = w.tx.send(WorkerMsg::Channel { target: Target::At(hz), bandwidth_hz });
        }
    }

    /// Pass a command to the running decoder.
    pub fn command(&self, c: Command) {
        if let Some(w) = &self.worker {
            let _ = w.tx.send(WorkerMsg::Command(c));
        }
    }

    pub fn retune(&self, hz: f64) {
        let _ = self.cmd.send(SourceCmd::Retune(hz));
    }

    /// Start recording the raw samples to `path` (format from its extension,
    /// .cs8 if none), or stop.
    pub fn record(&self, path: Option<PathBuf>) {
        let _ = self.cmd.send(SourceCmd::Record(path));
    }

    pub fn set_front_end(&self, s: FrontEndSettings) {
        let _ = self.cmd.send(SourceCmd::FrontEnd(s));
    }

    pub fn set_volume(&self, v: f32) {
        self.shared.volume.store(v.clamp(0.0, 1.0).to_bits(), Ordering::Relaxed);
    }

    pub fn take_rows(&self) -> Vec<Row> {
        self.shared.rows.lock().unwrap().drain(..).collect()
    }

    pub fn stats(&self) -> Stats {
        self.shared.stats.lock().unwrap().clone()
    }

    pub fn view(&self) -> DecoderView {
        self.shared.view.lock().unwrap().clone()
    }

    /// Messages the decoder sent for its panel ([`Sink::custom`]).
    pub fn take_custom(&self) -> Vec<Box<dyn std::any::Any + Send>> {
        self.shared.custom.lock().unwrap().drain(..).collect()
    }

    /// Whether the source thread is still going (false once a file has ended
    /// without looping, or the radio failed).
    pub fn running(&self) -> bool {
        self.shared.running.load(Ordering::Relaxed)
    }

    /// The source has ended and the decoder has had every block of it.
    pub fn finished(&self) -> bool {
        !self.running() && self.shared.in_flight.load(Ordering::Acquire) == 0
    }
}

impl Drop for Host {
    fn drop(&mut self) {
        // The worker first: it may be blocked handing a retune to the source.
        self.set_decoder(None);
        let _ = self.cmd.send(SourceCmd::Stop);
        if let Some(j) = self.source_join.take() {
            let _ = j.join();
        }
    }
}

/// Level readings over one second.
#[derive(Default)]
struct Levels {
    energy: f64,
    peak: f32,
    clipped: u64,
    count: u64,
}

impl Levels {
    fn add(&mut self, iq: &[Iq]) {
        let mut e = 0.0f32;
        let mut peak = self.peak;
        let mut clipped = 0u64;
        for s in iq {
            e += s.norm_sqr();
            let m = s.re.abs().max(s.im.abs());
            peak = peak.max(m);
            clipped += (m >= 0.99) as u64;
        }
        self.energy += e as f64;
        self.peak = peak;
        self.clipped += clipped;
        self.count += iq.len() as u64;
    }
}

/// A buffer from the pool that no worker still holds, or a new one.
fn pooled(pool: &mut Vec<Arc<Vec<Iq>>>) -> Arc<Vec<Iq>> {
    // Rust note: `Arc::get_mut` succeeds only when this is the last handle,
    // i.e. the worker has dropped its copy, so the buffer is free to refill.
    if let Some(i) = pool.iter_mut().position(|a| Arc::get_mut(a).is_some()) {
        return pool.swap_remove(i);
    }
    Arc::new(Vec::new())
}

fn source_thread(
    mut source: Box<dyn IqSource>,
    opts: HostOptions,
    cmd: Receiver<SourceCmd>,
    sh: Arc<Shared>,
) {
    let rate = source.sample_rate();
    let live = source.is_live();
    let mut center = source.center_freq();
    let mut generation = 0u32;
    let mut front = FrontEnd::new(FrontEndSettings::default());
    let mut waterfall = opts
        .waterfall
        .then(|| Waterfall::start(opts.fft, rate, opts.rows_per_s, opts.segments_per_row, sh.rows.clone()));
    let mut meter = StageMeter::default();
    let mut levels = Levels::default();
    let mut recorder: Option<(PathBuf, IqFileWriter, u64)> = None;
    let mut pool: Vec<Arc<Vec<Iq>>> = Vec::new();
    let mut samples = 0u64;
    let mut dropped_before = 0u64;
    // Real-time pacing of a file: the wall time its samples should take.
    let mut pace_start = Instant::now();
    let mut paced_samples = 0u64;
    let mut second = Instant::now();
    let mut error: Option<String> = None;
    let mut ended = false;
    // Block size: about 6.5 ms of signal at 20 MS/s; a radio's own size wins.
    let block_len = 131_072usize;

    {
        let mut s = sh.stats.lock().unwrap();
        s.describe = source.describe();
        s.live = live;
        s.sample_rate = rate;
        s.center_hz = center;
    }

    'run: loop {
        // Commands first, without waiting.
        loop {
            match cmd.try_recv() {
                Ok(SourceCmd::Stop) | Err(mpsc::TryRecvError::Disconnected) => break 'run,
                Ok(SourceCmd::Retune(hz)) => retune(&mut *source, hz, &mut center, &mut generation, &mut waterfall, &mut error),
                Ok(SourceCmd::RetuneBy(d)) => {
                    let hz = center + d;
                    retune(&mut *source, hz, &mut center, &mut generation, &mut waterfall, &mut error)
                }
                Ok(SourceCmd::FrontEnd(s)) => front.settings = s,
                Ok(SourceCmd::Record(None)) => {
                    if let Some((_, w, _)) = recorder.take() {
                        let _ = w.finish();
                    }
                }
                Ok(SourceCmd::Record(Some(path))) => {
                    let fmt = format_from_path(&path).unwrap_or(SampleFormat::Cs8);
                    match IqFileWriter::create(&path, fmt) {
                        Ok(w) => recorder = Some((path, w, 0)),
                        Err(e) => error = Some(format!("recording: {e}")),
                    }
                }
                Err(mpsc::TryRecvError::Empty) => break,
            }
        }

        if ended {
            // Nothing more to read: wait for commands only.
            std::thread::sleep(Duration::from_millis(20));
            continue;
        }

        let mut buf = pooled(&mut pool);
        let n = {
            let v = Arc::get_mut(&mut buf).expect("pooled buffers are unshared");
            v.reserve(block_len.saturating_sub(v.capacity()));
            match meter.time(Stage::Source, || source.read(v)) {
                Ok(n) => n,
                Err(e) => {
                    error = Some(e.to_string());
                    0
                }
            }
        };
        if n == 0 {
            if error.is_none() && opts.loop_files && source.rewind().unwrap_or(false) {
                generation = generation.wrapping_add(1);
                pace_start = Instant::now();
                paced_samples = 0;
                continue;
            }
            ended = true;
            let mut s = sh.stats.lock().unwrap();
            s.samples = samples;
            s.ended = true;
            s.error = error.clone();
            drop(s);
            sh.running.store(false, Ordering::Release);
            continue;
        }

        if let Some((_, w, bytes)) = &mut recorder {
            let r = meter.time(Stage::Record, || w.write(&buf));
            match r {
                Ok(()) => *bytes += (n * 2) as u64,
                Err(e) => {
                    error = Some(format!("recording: {e}"));
                    recorder = None;
                }
            }
        }
        {
            let v = Arc::get_mut(&mut buf).expect("not yet shared");
            meter.time(Stage::FrontEnd, || {
                front.apply(v);
                levels.add(v);
            });
        }
        if let Some(w) = &mut waterfall {
            meter.time(Stage::Spectrum, || w.push(&buf, center, generation));
        }

        // Hand over. A radio never waits; a file does.
        let meta = BlockMeta { center_hz: center, sample_rate: rate, generation, dropped_before };
        let tx = sh.worker_tx.lock().unwrap().clone();
        if let Some(tx) = tx {
            meter.time(Stage::Handoff, || {
                let msg = WorkerMsg::Block(buf.clone(), meta);
                // Counted before sending, so the worker never sees it go
                // below zero; taken back if the block is dropped.
                sh.in_flight.fetch_add(1, Ordering::AcqRel);
                if live {
                    match tx.try_send(msg) {
                        Ok(()) => dropped_before = 0,
                        Err(TrySendError::Full(_)) => {
                            sh.in_flight.fetch_sub(1, Ordering::AcqRel);
                            dropped_before += n as u64;
                            *sh.handoff_dropped.lock().unwrap() += 1;
                        }
                        Err(TrySendError::Disconnected(_)) => {
                            sh.in_flight.fetch_sub(1, Ordering::AcqRel);
                        }
                    }
                } else if tx.send(msg).is_err() {
                    sh.in_flight.fetch_sub(1, Ordering::AcqRel);
                }
            });
        }
        pool.push(buf);
        samples += n as u64;

        if !live && opts.pace_files {
            paced_samples += n as u64;
            let due = Duration::from_secs_f64(paced_samples as f64 / rate);
            let el = pace_start.elapsed();
            if due > el {
                std::thread::sleep(due - el);
            } else if el - due > Duration::from_secs(1) {
                // Far behind (the machine was busy): do not race to catch up.
                pace_start = Instant::now();
                paced_samples = 0;
            }
        }

        if meter.tick() || second.elapsed() >= Duration::from_secs(1) {
            second = Instant::now();
            let mut s = sh.stats.lock().unwrap();
            s.center_hz = center;
            s.samples = samples;
            if levels.count > 0 {
                let mean = levels.energy / levels.count as f64;
                s.rms_dbfs = (10.0 * mean.max(1e-20).log10()) as f32;
                s.peak = levels.peak;
                s.clip_percent = (100.0 * levels.clipped as f64 / levels.count as f64) as f32;
            }
            levels = Levels::default();
            s.front_end = front.reading();
            s.stages = meter.last;
            s.handoff_dropped = *sh.handoff_dropped.lock().unwrap();
            s.error = error.clone();
            s.recording = recorder.as_ref().map(|(p, _, b)| (p.clone(), *b));
        }
    }
    if let Some((_, w, _)) = recorder.take() {
        let _ = w.finish();
    }
    sh.running.store(false, Ordering::Release);
}

fn retune(
    source: &mut dyn IqSource,
    hz: f64,
    center: &mut f64,
    generation: &mut u32,
    waterfall: &mut Option<Waterfall>,
    error: &mut Option<String>,
) {
    match source.retune(hz) {
        Ok(true) => {
            *center = hz;
            *generation = generation.wrapping_add(1);
            if let Some(w) = waterfall {
                w.clear();
            }
        }
        // A file: the label moves, the samples do not.
        Ok(false) => *center = hz,
        Err(e) => *error = Some(format!("retune: {e}")),
    }
}

/// The down-converter in front of a decoder: what [`InputSpec`] asks for,
/// cut out of the source's stream at the channel's offset.
struct Chain {
    ddc: Option<Ddc>,
    rs: Option<Resampler>,
    info: ChannelInfo,
    mid: Vec<Iq>,
    out: Vec<Iq>,
}

impl Chain {
    fn new(in_rate: f64, spec: &InputSpec, offset_hz: f64, bandwidth_hz: f64) -> (Chain, Option<String>) {
        let bw = bandwidth_hz.clamp(1.0, in_rate);
        let mut note = None;
        let want = spec.sample_rate;
        // Untouched: the decoder takes the source's own samples.
        let passthrough = want.is_none_or(|r| (r - in_rate).abs() < 1e-6 * in_rate)
            && offset_hz == 0.0
            && bw >= 0.8 * in_rate;
        let ddc = (!passthrough).then(|| Ddc::new(in_rate, offset_hz, bw));
        let mid_rate = ddc.as_ref().map_or(in_rate, |d| d.out_rate());
        let rs = match want {
            Some(r) if (r - mid_rate).abs() > 1e-6 * r => match Resampler::for_rates(mid_rate, r, bw) {
                Ok(rs) => Some(rs),
                Err(e) => {
                    note = Some(format!("cannot resample {mid_rate:.0} -> {r:.0} Hz ({e}); feeding {mid_rate:.0} Hz"));
                    None
                }
            },
            _ => None,
        };
        let out_rate = match (&rs, want) {
            (Some(_), Some(r)) => r,
            _ => mid_rate,
        };
        let info = ChannelInfo {
            offset_hz,
            bandwidth_hz: bw,
            out_rate,
            decimation: ddc.as_ref().map_or(1, |d| d.decimation()),
            resample: rs.as_ref().map(|r| r.ratio()),
        };
        (Chain { ddc, rs, info, mid: Vec::new(), out: Vec::new() }, note)
    }

    /// The channel's samples for `input`.
    fn run<'a>(&'a mut self, input: &'a [Iq]) -> &'a [Iq] {
        let staged: &[Iq] = match &mut self.ddc {
            Some(d) => {
                self.mid.clear();
                d.process(input, &mut self.mid);
                &self.mid
            }
            None => input,
        };
        match &mut self.rs {
            Some(r) => {
                self.out.clear();
                r.process(staged, &mut self.out);
                &self.out
            }
            None => staged,
        }
    }
}

fn worker_thread(
    mut decoder: Box<dyn Decoder>,
    spec: InputSpec,
    rx: Receiver<WorkerMsg>,
    sh: Arc<Shared>,
    cmd: Sender<SourceCmd>,
) {
    let mut sink = Sink::default();
    let mut target = Target::Offset(0.0);
    // The offset the chain was built for.
    let mut offset = 0.0f64;
    let mut bandwidth = spec.bandwidth_hz;
    let mut chain: Option<(f64, Chain)> = None;
    let mut generation: Option<u32> = None;
    let mut first_sample = 0u64;
    let mut audio: Option<AudioOut> = None;
    let mut busy = Duration::ZERO;
    let mut window = Instant::now();

    loop {
        let msg = match rx.recv_timeout(Duration::from_millis(200)) {
            Ok(m) => m,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => break,
        };
        let t0 = Instant::now();
        let mut in_flight_done = false;
        sink.clear();
        match msg {
            WorkerMsg::Stop => break,
            WorkerMsg::Command(c) => {
                if matches!(c, Command::SelectService(_))
                    && let Some(a) = &audio
                {
                    a.clear();
                }
                decoder.command(c, &mut sink);
            }
            WorkerMsg::Channel { target: t, bandwidth_hz } => {
                target = t;
                bandwidth = bandwidth_hz;
                chain = None;
                decoder.reset();
                first_sample = 0;
            }
            WorkerMsg::Block(buf, meta) => {
                if generation != Some(meta.generation) {
                    if generation.is_some() {
                        decoder.reset();
                        chain = None;
                    }
                    generation = Some(meta.generation);
                    first_sample = 0;
                }
                let want = target.offset(meta.center_hz);
                if want != offset {
                    offset = want;
                    chain = None;
                }
                if chain.as_ref().is_none_or(|(r, _)| *r != meta.sample_rate) {
                    let (c, note) = Chain::new(meta.sample_rate, &spec, offset, bandwidth);
                    if let Some(n) = note {
                        sink.log(n);
                    }
                    sh.view.lock().unwrap().channel = c.info;
                    chain = Some((meta.sample_rate, c));
                }
                let (_, ch) = chain.as_mut().expect("built above");
                let out_rate = ch.info.out_rate;
                let iq = ch.run(&buf);
                let block = Block {
                    iq,
                    tuning: Tuning {
                        center_hz: meta.center_hz + offset,
                        sample_rate: out_rate,
                        generation: meta.generation,
                    },
                    first_sample,
                    dropped_before: (meta.dropped_before as f64 * out_rate / meta.sample_rate) as u64,
                };
                first_sample += iq.len() as u64;
                decoder.process(&block, &mut sink);
                in_flight_done = true;
            }
        }
        busy += t0.elapsed();

        // Empty the sink.
        if let Some(d) = sink.retune_by_hz.take() {
            match target {
                Target::Offset(0.0) => {
                    let _ = cmd.send(SourceCmd::RetuneBy(d));
                }
                Target::Offset(o) => target = Target::Offset(o + d),
                Target::At(f) => target = Target::At(f + d),
            }
        }
        if !sink.audio.is_empty() {
            // The sound thread opens the device; nothing here waits for it.
            let a = audio.get_or_insert_with(AudioOut::spawn);
            a.set_volume(f32::from_bits(sh.volume.load(Ordering::Relaxed)));
            for chunk in sink.audio.drain(..) {
                a.push(chunk.sample_rate, chunk.channels, chunk.samples);
            }
        }
        let audio_status = audio.as_ref().and_then(AudioOut::status);
        if !sink.custom.is_empty() {
            let mut q = sh.custom.lock().unwrap();
            q.extend(sink.custom.drain(..));
            while q.len() > MAX_CUSTOM {
                q.pop_front();
            }
        }
        let mut v = sh.view.lock().unwrap();
        if audio_status.is_some() {
            v.audio = audio_status;
        }
        v.lock = sink.lock;
        for m in sink.measurements.drain(..) {
            match v.measurements.iter_mut().find(|x| x.name == m.name) {
                Some(x) => *x = m,
                None => v.measurements.push(m),
            }
        }
        if !sink.constellation.is_empty() {
            std::mem::swap(&mut v.constellation, &mut sink.constellation);
        }
        for (name, pts) in sink.scopes.drain(..) {
            if pts.is_empty() {
                continue;
            }
            match v.scopes.iter_mut().find(|(n, _)| *n == name) {
                Some((_, old)) => *old = pts,
                None => v.scopes.push((name, pts)),
            }
        }
        for line in sink.log.drain(..) {
            v.log.push_back(line);
            if v.log.len() > MAX_LOG {
                v.log.pop_front();
            }
        }
        for (id, t) in sink.text.drain(..) {
            v.text.insert(id, t);
        }
        if let Some(s) = sink.services.take() {
            v.services = s;
        }
        for p in sink.pictures.drain(..) {
            v.pictures.insert(p.service, p);
        }
        if let Some((_, c)) = &chain {
            v.channel = c.info;
        }
        let el = window.elapsed();
        if el >= Duration::from_secs(1) {
            v.cpu_percent = (100.0 * busy.as_secs_f64() / el.as_secs_f64()) as f32;
            busy = Duration::ZERO;
            window = Instant::now();
        }
        drop(v);
        // Only now, with its results in the view, does the block count as done.
        if in_flight_done {
            sh.in_flight.fetch_sub(1, Ordering::AcqRel);
        }
    }
    // Blocks never taken off the queue.
    for m in rx.try_iter() {
        if matches!(m, WorkerMsg::Block(..)) {
            sh.in_flight.fetch_sub(1, Ordering::AcqRel);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chain_reaches_the_asked_rate() {
        let spec = InputSpec { sample_rate: Some(2.048e6), bandwidth_hz: 1.536e6, block_len: 65536 };
        let (mut c, note) = Chain::new(20e6, &spec, 1e6, 1.536e6);
        assert!(note.is_none(), "{note:?}");
        assert_eq!(c.info.out_rate, 2.048e6);
        let input = vec![Iq::new(0.1, 0.0); 200_000];
        let n: usize = (0..10).map(|_| c.run(&input).len()).sum();
        let expect = 2_000_000.0 * 2.048e6 / 20e6;
        assert!((n as f64 - expect).abs() < 200.0, "{n} vs {expect}");
    }

    #[test]
    fn untouched_when_the_decoder_wants_the_source() {
        let spec = InputSpec { sample_rate: None, bandwidth_hz: 10e6, block_len: 65536 };
        let (mut c, _) = Chain::new(10e6, &spec, 0.0, 10e6);
        assert!(c.ddc.is_none() && c.rs.is_none());
        let input = vec![Iq::new(0.5, 0.5); 1000];
        assert_eq!(c.run(&input), &input[..]);
    }
}
