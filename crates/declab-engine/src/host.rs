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
    Block, Card, Command, Decoder, DecoderFactory, Heard, InputSpec, Lock, Measurement, Picture, Service,
    Sink, Tuning, WaterfallRow,
};
use declab_core::{Iq, SampleFormat};
use declab_dsp::{Ddc, Resampler};
use declab_io::{IqFileWriter, IqSource, format_from_path};

use crate::audio::AudioOut;
use crate::frontend::{FrontEnd, FrontEndReading, FrontEndSettings};
use crate::hub::{Feeder, Outputs, TsHub, TsView};
use crate::meter::{Stage, StageLoad, StageMeter};
use crate::waterfall::Waterfall;

/// Blocks queued for the decoder worker before a live source drops them.
const WORKER_QUEUE: usize = 16;
/// Log lines kept for the GUI.
const MAX_LOG: usize = 300;
/// Small-waterfall rows kept for a GUI that has not collected them.
const MAX_MINI_ROWS: usize = 256;
/// Panel messages kept for a GUI that has not collected them.
const MAX_CUSTOM: usize = 64;
/// VFOs at once. Each costs a down-converter and a thread; the radio's
/// bandwidth and the machine's cores are the real limits.
pub const MAX_VFOS: usize = 8;

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
    /// The decoder's latest card.
    pub card: Option<Card>,
    pub constellation: Vec<Iq>,
    /// The decoder's other scopes, the latest points of each.
    pub scopes: Vec<(&'static str, Vec<Iq>)>,
    pub log: VecDeque<String>,
    pub services: Vec<Service>,
    /// Talkgroups and radios heard (digital voice), the latest list.
    pub heard: Option<Heard>,
    pub text: HashMap<u32, String>,
    /// The latest picture of each service.
    pub pictures: HashMap<u32, Picture>,
    pub channel: ChannelInfo,
    /// The worker thread's busy time (down-converter + decoder), percent of
    /// one core.
    pub cpu_percent: f32,
    /// Sound output in use, or why there is none.
    pub audio: Option<String>,
    /// Transport-stream packets the decoder has made (`Sink::ts`).
    pub ts_packets: u64,
    /// IP datagrams (DVB-S2 GSE) and their bytes since the decoder started.
    pub ip_packets: u64,
    pub ip_bytes: u64,
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
    live: bool,
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

/// Names one VFO (one channel with one decoder on it) for as long as it
/// lives. Ids are never reused.
pub type VfoId = u32;

/// State both threads and the GUI share.
struct Shared {
    rows: Arc<Mutex<VecDeque<Row>>>,
    stats: Mutex<Stats>,
    /// The queue of every running VFO's worker, for the source thread.
    workers: Mutex<Vec<(VfoId, SyncSender<WorkerMsg>)>>,
    /// Blocks dropped because a worker's queue was full.
    handoff_dropped: Mutex<u64>,
    /// The master volume; each VFO's own gain multiplies it.
    volume: AtomicU32,
    running: AtomicBool,
    /// Blocks handed to workers and not yet decoded (summed over VFOs).
    in_flight: AtomicUsize,
}

/// What one VFO's worker and the GUI share.
struct VfoShared {
    view: Mutex<DecoderView>,
    custom: Mutex<VecDeque<Box<dyn std::any::Any + Send>>>,
    /// Rows of the decoder's small waterfalls not yet taken by the GUI.
    mini_rows: Mutex<VecDeque<(&'static str, WaterfallRow)>>,
    /// Where the decoder's transport stream is written, if anywhere.
    ts_file: Mutex<Option<std::io::BufWriter<std::fs::File>>>,
    /// Where the decoder's IP datagrams are written (a PCAP of raw IP).
    pcap_file: Mutex<Option<std::io::BufWriter<std::fs::File>>>,
    /// The decoder's sound to a WAV file (`Host::write_audio`).
    wav_file: Mutex<Option<WavOut>>,
    /// This VFO's volume against the master (0 mutes it), `f32` bits.
    gain: AtomicU32,
    /// The primary VFO may retune the radio when its decoder asks (a
    /// decoder following a drifting signal); the others only move their own
    /// channel, since the radio is theirs too.
    drives_source: AtomicBool,
}

impl VfoShared {
    fn new(drives_source: bool) -> VfoShared {
        VfoShared {
            view: Mutex::new(DecoderView::default()),
            custom: Mutex::new(VecDeque::new()),
            mini_rows: Mutex::new(VecDeque::new()),
            ts_file: Mutex::new(None),
            pcap_file: Mutex::new(None),
            wav_file: Mutex::new(None),
            gain: AtomicU32::new(1.0f32.to_bits()),
            drives_source: AtomicBool::new(drives_source),
        }
    }
}

/// One VFO: a channel cut out of the wideband stream, a decoder on it, and
/// everything that decoder produces.
struct Vfo {
    id: VfoId,
    vs: Arc<VfoShared>,
    tx: SyncSender<WorkerMsg>,
    join: JoinHandle<()>,
    /// Whatever transport stream the decoder makes goes to this hub.
    hub: TsHub,
    decoder_id: &'static str,
}

/// The running host. Dropping it stops all threads and the radio.
///
/// A host runs any number of VFOs at once (up to [`MAX_VFOS`]), each with its
/// own down-converter and decoder on its own worker thread, all fed from the
/// one radio. The methods without `_vfo` in their name act on the **focused**
/// VFO ([`Host::set_focus`]; the first one until told otherwise), so a caller
/// that wants one decoder never has to know about the others. Only the first
/// VFO, the **primary**, may retune the radio.
pub struct Host {
    shared: Arc<Shared>,
    cmd: Sender<SourceCmd>,
    source_join: Option<JoinHandle<()>>,
    vfos: Vec<Vfo>,
    next_id: VfoId,
    /// The VFO the methods without `_vfo` act on (the first when `None`).
    focus: Option<VfoId>,
    source_rate: f64,
    live: bool,
}

impl Host {
    /// Start reading `source`. No decoder runs until [`Host::set_decoder`]
    /// or [`Host::add_vfo`].
    pub fn start(source: Box<dyn IqSource>, opts: HostOptions) -> Host {
        let source_rate = source.sample_rate();
        let live = source.is_live();
        let shared = Arc::new(Shared {
            rows: Arc::new(Mutex::new(VecDeque::new())),
            stats: Mutex::new(Stats::default()),
            workers: Mutex::new(Vec::new()),
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
        Host { shared, cmd, source_join: Some(source_join), vfos: Vec::new(), next_id: 0, focus: None, source_rate, live }
    }

    pub fn source_rate(&self) -> f64 {
        self.source_rate
    }

    /// A radio rather than a file.
    pub fn is_live(&self) -> bool {
        self.live
    }

    // ---- VFOs ---------------------------------------------------------

    /// Add a VFO running `factory`'s decoder on the channel at RF frequency
    /// `hz`, `bandwidth_hz` wide (the channel then stays on that frequency
    /// across retunes). `None` when [`MAX_VFOS`] are running.
    pub fn add_vfo(&mut self, factory: &'static dyn DecoderFactory, hz: f64, bandwidth_hz: f64) -> Option<VfoId> {
        self.spawn_vfo(factory, Target::At(hz), bandwidth_hz)
    }

    fn spawn_vfo(&mut self, factory: &'static dyn DecoderFactory, target: Target, bandwidth_hz: f64) -> Option<VfoId> {
        if self.vfos.len() >= MAX_VFOS {
            return None;
        }
        let id = self.next_id;
        self.next_id += 1;
        let info = factory.info();
        let spec = factory.input(self.source_rate);
        let vs = Arc::new(VfoShared::new(self.vfos.is_empty()));
        {
            let mut v = vs.view.lock().unwrap();
            v.id = info.id;
            v.name = info.name;
        }
        let hub = TsHub::start();
        let feeder = hub.feeder();
        let (tx, rx) = mpsc::sync_channel(WORKER_QUEUE);
        // The channel is queued before the first block can arrive, so the
        // worker never builds a down-converter for the wrong place.
        let _ = tx.send(WorkerMsg::Channel { target, bandwidth_hz });
        let (sh, vsh, cmd, decoder) = (self.shared.clone(), vs.clone(), self.cmd.clone(), factory.create());
        let join = std::thread::Builder::new()
            .name(format!("declab-decoder-{id}"))
            .spawn(move || worker_thread(decoder, spec, rx, sh, vsh, cmd, feeder))
            .expect("cannot start the decoder thread");
        self.shared.workers.lock().unwrap().push((id, tx.clone()));
        self.vfos.push(Vfo { id, vs, tx, join, hub, decoder_id: info.id });
        Some(id)
    }

    /// Stop a VFO and everything it was producing. The next one in line
    /// becomes the primary if this was.
    pub fn remove_vfo(&mut self, id: VfoId) {
        let Some(i) = self.vfos.iter().position(|v| v.id == id) else { return };
        let v = self.vfos.remove(i);
        if self.focus == Some(id) {
            self.focus = None;
        }
        self.shared.workers.lock().unwrap().retain(|(w, _)| *w != id);
        let _ = v.tx.send(WorkerMsg::Stop);
        let _ = v.join.join();
        self.mark_primary();
    }

    /// Only the first VFO may retune the radio.
    fn mark_primary(&self) {
        for (i, v) in self.vfos.iter().enumerate() {
            v.vs.drives_source.store(i == 0, Ordering::Relaxed);
        }
    }

    /// The running VFOs, primary first.
    pub fn vfo_ids(&self) -> Vec<VfoId> {
        self.vfos.iter().map(|v| v.id).collect()
    }

    /// The decoder id (`"wfm"`, `"dmr"`...) a VFO runs.
    pub fn vfo_decoder(&self, id: VfoId) -> Option<&'static str> {
        self.vfo(id).map(|v| v.decoder_id)
    }

    fn vfo(&self, id: VfoId) -> Option<&Vfo> {
        self.vfos.iter().find(|v| v.id == id)
    }

    /// The focused VFO, or the first.
    fn focused(&self) -> Option<&Vfo> {
        self.focus.and_then(|id| self.vfo(id)).or_else(|| self.vfos.first())
    }

    /// Make the methods without `_vfo` act on this VFO (no change if it is
    /// not running).
    pub fn set_focus(&mut self, id: VfoId) {
        if self.vfo(id).is_some() {
            self.focus = Some(id);
        }
    }

    /// The VFO those methods act on, if any is running.
    pub fn focused_id(&self) -> Option<VfoId> {
        self.focused().map(|v| v.id)
    }

    /// Move or resize a VFO's channel, `offset_hz` from the radio's centre.
    pub fn set_vfo_channel(&self, id: VfoId, offset_hz: f64, bandwidth_hz: f64) {
        if let Some(v) = self.vfo(id) {
            let _ = v.tx.send(WorkerMsg::Channel { target: Target::Offset(offset_hz), bandwidth_hz });
        }
    }

    /// Put a VFO's channel at RF frequency `hz` and keep it there across
    /// retunes (the GUI's way: the channel stays on its station while the
    /// band is dragged).
    pub fn set_vfo_channel_at(&self, id: VfoId, hz: f64, bandwidth_hz: f64) {
        if let Some(v) = self.vfo(id) {
            let _ = v.tx.send(WorkerMsg::Channel { target: Target::At(hz), bandwidth_hz });
        }
    }

    /// Pass a command to a VFO's decoder.
    pub fn vfo_command(&self, id: VfoId, c: Command) {
        if let Some(v) = self.vfo(id) {
            let _ = v.tx.send(WorkerMsg::Command(c));
        }
    }

    /// A VFO's volume against the master: 1.0 as is, 0.0 muted.
    pub fn set_vfo_gain(&self, id: VfoId, gain: f32) {
        if let Some(v) = self.vfo(id) {
            v.vs.gain.store(gain.clamp(0.0, 1.0).to_bits(), Ordering::Relaxed);
        }
    }

    /// What a VFO's decoder has said.
    pub fn vfo_view(&self, id: VfoId) -> Option<DecoderView> {
        self.vfo(id).map(|v| v.vs.view.lock().unwrap().clone())
    }

    /// Messages a VFO's decoder sent for its panel ([`Sink::custom`]).
    pub fn take_vfo_custom(&self, id: VfoId) -> Vec<Box<dyn std::any::Any + Send>> {
        self.vfo(id).map_or_else(Vec::new, |v| v.vs.custom.lock().unwrap().drain(..).collect())
    }

    /// Rows of a VFO's small waterfalls since the last call, by name.
    pub fn take_vfo_waterfall_rows(&self, id: VfoId) -> Vec<(&'static str, WaterfallRow)> {
        self.vfo(id).map_or_else(Vec::new, |v| v.vs.mini_rows.lock().unwrap().drain(..).collect())
    }

    /// A VFO's transport stream as its hub sees it.
    pub fn vfo_ts_view(&self, id: VfoId) -> Option<TsView> {
        self.vfo(id).map(|v| v.hub.view())
    }

    /// The programme guide gathered from a VFO's stream.
    pub fn vfo_epg(&self, id: VfoId) -> Option<Arc<decdvbt_ts::Epg>> {
        self.vfo(id).map(|v| v.hub.epg())
    }

    /// Where a VFO's transport stream goes (file, UDP, HTTP, the player's
    /// tap, a Red Button carousel).
    pub fn set_vfo_outputs(&self, id: VfoId, o: Outputs) {
        if let Some(v) = self.vfo(id) {
            v.hub.set_outputs(o);
        }
    }

    /// Write a VFO's sound to a 16-bit WAV file (`None` closes it). A change
    /// of rate or channels starts the file over.
    pub fn write_vfo_audio(&self, id: VfoId, path: Option<PathBuf>) -> std::io::Result<()> {
        let Some(v) = self.vfo(id) else { return Ok(()) };
        let mut f = v.vs.wav_file.lock().unwrap();
        if let Some(old) = f.take() {
            old.finish()?;
        }
        if let Some(p) = path {
            *f = Some(WavOut { file: std::io::BufWriter::new(std::fs::File::create(p)?), shape: None, bytes: 0 });
        }
        Ok(())
    }

    /// Write a VFO's transport stream to `path` (a `.ts`), or stop.
    pub fn write_vfo_ts(&self, id: VfoId, path: Option<PathBuf>) -> std::io::Result<()> {
        let Some(v) = self.vfo(id) else { return Ok(()) };
        let mut f = v.vs.ts_file.lock().unwrap();
        if let Some(mut old) = f.take() {
            use std::io::Write;
            old.flush()?;
        }
        if let Some(p) = path {
            *f = Some(std::io::BufWriter::with_capacity(1 << 20, std::fs::File::create(p)?));
        }
        Ok(())
    }

    /// Write a VFO's IP datagrams to `path` (a `.pcap` of raw IP, link type
    /// 101), or stop.
    pub fn write_vfo_pcap(&self, id: VfoId, path: Option<PathBuf>) -> std::io::Result<()> {
        use std::io::Write;
        let Some(v) = self.vfo(id) else { return Ok(()) };
        let mut f = v.vs.pcap_file.lock().unwrap();
        if let Some(mut old) = f.take() {
            old.flush()?;
        }
        if let Some(p) = path {
            let mut w = std::io::BufWriter::new(std::fs::File::create(p)?);
            // Global header: magic, version 2.4, no zone, no sigfigs,
            // snaplen 65535, LINKTYPE_RAW (101).
            w.write_all(&0xA1B2_C3D4u32.to_le_bytes())?;
            w.write_all(&2u16.to_le_bytes())?;
            w.write_all(&4u16.to_le_bytes())?;
            w.write_all(&[0; 8])?;
            w.write_all(&65535u32.to_le_bytes())?;
            w.write_all(&101u32.to_le_bytes())?;
            *f = Some(w);
        }
        Ok(())
    }

    pub fn write_pcap(&self, path: Option<PathBuf>) -> std::io::Result<()> {
        self.focused().map_or(Ok(()), |v| self.write_vfo_pcap(v.id, path))
    }

    // ---- The focused VFO: the single-decoder API ------------------------

    /// Run `factory`'s decoder on the focused VFO's place (stopping the one
    /// running there), or none (stopping every VFO). The new VFO starts
    /// centred, as wide as the decoder asks for, and takes the focus.
    pub fn set_decoder(&mut self, factory: Option<&'static dyn DecoderFactory>) {
        let Some(f) = factory else {
            while let Some(id) = self.vfos.first().map(|v| v.id) {
                self.remove_vfo(id);
            }
            return;
        };
        let at = self.focused().and_then(|p| self.vfos.iter().position(|v| v.id == p.id));
        if let Some(i) = at {
            let id = self.vfos[i].id;
            self.remove_vfo(id);
        }
        let bw = f.input(self.source_rate).bandwidth_hz;
        if let Some(new) = self.spawn_vfo(f, Target::Offset(0.0), bw) {
            // Added last; put it where the old one was.
            let v = self.vfos.pop().expect("just added");
            self.vfos.insert(at.unwrap_or(0).min(self.vfos.len()), v);
            self.focus = Some(new);
            self.mark_primary();
        }
    }

    /// Move or resize the focused VFO's channel.
    pub fn set_channel(&self, offset_hz: f64, bandwidth_hz: f64) {
        if let Some(v) = self.focused() {
            self.set_vfo_channel(v.id, offset_hz, bandwidth_hz);
        }
    }

    pub fn set_channel_at(&self, hz: f64, bandwidth_hz: f64) {
        if let Some(v) = self.focused() {
            self.set_vfo_channel_at(v.id, hz, bandwidth_hz);
        }
    }

    /// Pass a command to the focused VFO's decoder.
    pub fn command(&self, c: Command) {
        if let Some(v) = self.focused() {
            self.vfo_command(v.id, c);
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

    pub fn write_audio(&self, path: Option<PathBuf>) -> std::io::Result<()> {
        self.focused().map_or(Ok(()), |v| self.write_vfo_audio(v.id, path))
    }

    pub fn write_ts(&self, path: Option<PathBuf>) -> std::io::Result<()> {
        self.focused().map_or(Ok(()), |v| self.write_vfo_ts(v.id, path))
    }

    pub fn set_outputs(&self, o: Outputs) {
        if let Some(v) = self.focused() {
            v.hub.set_outputs(o);
        }
    }

    /// The focused VFO's transport stream as the hub sees it.
    pub fn ts_view(&self) -> TsView {
        self.focused().map_or_else(TsView::default, |v| v.hub.view())
    }

    /// The programme guide gathered from the focused VFO's stream.
    pub fn epg(&self) -> Arc<decdvbt_ts::Epg> {
        self.focused().map_or_else(|| Arc::new(decdvbt_ts::Epg::new()), |v| v.hub.epg())
    }

    /// The master volume, over every VFO.
    pub fn set_volume(&self, v: f32) {
        self.shared.volume.store(v.clamp(0.0, 1.0).to_bits(), Ordering::Relaxed);
    }

    pub fn take_rows(&self) -> Vec<Row> {
        self.shared.rows.lock().unwrap().drain(..).collect()
    }

    pub fn stats(&self) -> Stats {
        self.shared.stats.lock().unwrap().clone()
    }

    /// What the focused VFO's decoder has said (empty with none running).
    pub fn view(&self) -> DecoderView {
        self.focused().map_or_else(DecoderView::default, |v| v.vs.view.lock().unwrap().clone())
    }

    /// Rows of the focused VFO's small waterfalls since the last call.
    pub fn take_waterfall_rows(&self) -> Vec<(&'static str, WaterfallRow)> {
        self.focused().map_or_else(Vec::new, |v| self.take_vfo_waterfall_rows(v.id))
    }

    /// Messages the focused VFO's decoder sent for its panel ([`Sink::custom`]).
    pub fn take_custom(&self) -> Vec<Box<dyn std::any::Any + Send>> {
        self.focused().map_or_else(Vec::new, |v| self.take_vfo_custom(v.id))
    }

    /// Whether the source thread is still going (false once a file has ended
    /// without looping, or the radio failed).
    pub fn running(&self) -> bool {
        self.shared.running.load(Ordering::Relaxed)
    }

    /// The source has ended and the decoders have had every block of it.
    pub fn finished(&self) -> bool {
        !self.running() && self.shared.in_flight.load(Ordering::Acquire) == 0
    }
}

impl Drop for Host {
    fn drop(&mut self) {
        // The workers first: one may be blocked handing a retune to the source.
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
    // Samples each VFO's queue refused since it last took a block.
    let mut dropped_before: HashMap<VfoId, u64> = HashMap::new();
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

        // Hand over to every VFO's worker. A radio never waits; a file does
        // (for the slowest worker).
        let workers: Vec<_> = sh.workers.lock().unwrap().clone();
        dropped_before.retain(|id, _| workers.iter().any(|(w, _)| w == id));
        meter.time(Stage::Handoff, || {
            for (id, tx) in &workers {
                let lost = dropped_before.entry(*id).or_insert(0);
                let meta = BlockMeta { center_hz: center, sample_rate: rate, generation, dropped_before: *lost, live };
                let msg = WorkerMsg::Block(buf.clone(), meta);
                // Counted before sending, so a worker never sees it go
                // below zero; taken back if the block is dropped.
                sh.in_flight.fetch_add(1, Ordering::AcqRel);
                if live {
                    match tx.try_send(msg) {
                        Ok(()) => *lost = 0,
                        Err(TrySendError::Full(_)) => {
                            sh.in_flight.fetch_sub(1, Ordering::AcqRel);
                            *lost += n as u64;
                            *sh.handoff_dropped.lock().unwrap() += 1;
                        }
                        Err(TrySendError::Disconnected(_)) => {
                            sh.in_flight.fetch_sub(1, Ordering::AcqRel);
                        }
                    }
                } else if tx.send(msg).is_err() {
                    sh.in_flight.fetch_sub(1, Ordering::AcqRel);
                }
            }
        });
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
        // Untouched: a decoder that asks for the source's own rate
        // (`sample_rate: None`) with its channel centred gets the samples as
        // they are (DVB-T's FFT runs on them directly: a full-rate filter
        // cost it 30 % more time and did nothing); one that names the source's
        // rate gets them untouched when its channel is most of the band.
        // A channel under half the band is cut out and decimated even then
        // (a satellite carrier of 2 MS/s on a 10 MS/s radio).
        let passthrough = offset_hz == 0.0
            && (want.is_none() && bw >= 0.5 * in_rate
                || want.is_some_and(|r| (r - in_rate).abs() < 1e-6 * in_rate) && bw >= 0.8 * in_rate);
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
    vs: Arc<VfoShared>,
    cmd: Sender<SourceCmd>,
    feeder: Feeder,
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
                // A new bandwidth, or a move within half of it (a channel
                // dragged), keeps the decoder going: resetting it at every
                // step of a drag would mute it until the drag ended.
                let jump = match (target, t) {
                    (Target::At(a), Target::At(b)) | (Target::Offset(a), Target::Offset(b)) => {
                        (a - b).abs() > bandwidth.min(bandwidth_hz) / 2.0
                    }
                    _ => true,
                };
                target = t;
                bandwidth = bandwidth_hz;
                chain = None;
                if jump {
                    decoder.reset();
                    first_sample = 0;
                    feeder.reset();
                }
            }
            WorkerMsg::Block(buf, meta) => {
                if generation != Some(meta.generation) {
                    if generation.is_some() {
                        decoder.reset();
                        chain = None;
                        feeder.reset();
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
                    vs.view.lock().unwrap().channel = c.info;
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
                    live: meta.live,
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
                Target::Offset(0.0) if vs.drives_source.load(Ordering::Relaxed) => {
                    let _ = cmd.send(SourceCmd::RetuneBy(d));
                }
                Target::Offset(o) => target = Target::Offset(o + d),
                Target::At(f) => target = Target::At(f + d),
            }
        }
        if !sink.audio.is_empty() {
            // The sound thread opens the device; nothing here waits for it.
            let a = audio.get_or_insert_with(AudioOut::spawn);
            a.set_volume(
                f32::from_bits(sh.volume.load(Ordering::Relaxed)) * f32::from_bits(vs.gain.load(Ordering::Relaxed)),
            );
            let mut wav = vs.wav_file.lock().unwrap();
            for chunk in sink.audio.drain(..) {
                if let Some(w) = wav.as_mut() {
                    w.push(chunk.sample_rate, chunk.channels, &chunk.samples);
                }
                a.push(chunk.sample_rate, chunk.channels, chunk.samples);
            }
        }
        let audio_status = audio.as_ref().and_then(AudioOut::status);
        if !sink.waterfalls.is_empty() {
            let mut q = vs.mini_rows.lock().unwrap();
            q.extend(sink.waterfalls.drain(..));
            // A GUI that is not collecting (minimised) keeps a few seconds.
            while q.len() > MAX_MINI_ROWS {
                q.pop_front();
            }
        }
        if !sink.custom.is_empty() {
            let mut q = vs.custom.lock().unwrap();
            q.extend(sink.custom.drain(..));
            while q.len() > MAX_CUSTOM {
                q.pop_front();
            }
        }
        let mut v = vs.view.lock().unwrap();
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
        if !sink.ts.is_empty() {
            v.ts_packets += sink.ts.len() as u64;
            feeder.feed(sink.ts.iter().flatten().copied().collect());
            if let Some(f) = vs.ts_file.lock().unwrap().as_mut() {
                use std::io::Write;
                for p in &sink.ts {
                    let _ = f.write_all(p);
                }
            }
        }
        if !sink.ip.is_empty() {
            v.ip_packets += sink.ip.len() as u64;
            v.ip_bytes += sink.ip.iter().map(|p| p.len() as u64).sum::<u64>();
            if let Some(f) = vs.pcap_file.lock().unwrap().as_mut() {
                use std::io::Write;
                let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
                for p in &sink.ip {
                    // Packet header: seconds, microseconds, captured, original.
                    let _ = f.write_all(&(now.as_secs() as u32).to_le_bytes());
                    let _ = f.write_all(&now.subsec_micros().to_le_bytes());
                    let _ = f.write_all(&(p.len() as u32).to_le_bytes());
                    let _ = f.write_all(&(p.len() as u32).to_le_bytes());
                    let _ = f.write_all(p);
                }
            }
        }
        if let Some(c) = sink.card.take() {
            v.card = Some(c);
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
        if let Some(h) = sink.heard.take() {
            v.heard = Some(h);
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

/// A WAV file being written: 16-bit PCM, its header's sizes filled in when
/// it is finished.
struct WavOut {
    file: std::io::BufWriter<std::fs::File>,
    /// Rate and channels, from the first chunk.
    shape: Option<(u32, u16)>,
    bytes: u32,
}

impl WavOut {
    fn header(rate: u32, ch: u16, bytes: u32) -> Vec<u8> {
        let mut h = Vec::with_capacity(44);
        h.extend_from_slice(b"RIFF");
        h.extend_from_slice(&(36 + bytes).to_le_bytes());
        h.extend_from_slice(b"WAVEfmt ");
        h.extend_from_slice(&16u32.to_le_bytes());
        h.extend_from_slice(&1u16.to_le_bytes());
        h.extend_from_slice(&ch.to_le_bytes());
        h.extend_from_slice(&rate.to_le_bytes());
        h.extend_from_slice(&(rate * u32::from(ch) * 2).to_le_bytes());
        h.extend_from_slice(&(ch * 2).to_le_bytes());
        h.extend_from_slice(&16u16.to_le_bytes());
        h.extend_from_slice(b"data");
        h.extend_from_slice(&bytes.to_le_bytes());
        h
    }

    fn push(&mut self, rate: u32, ch: u16, samples: &[f32]) {
        use std::io::Write;
        if samples.is_empty() || rate == 0 {
            return;
        }
        if self.shape != Some((rate, ch)) {
            // The first sound, or another service's: the file starts over
            // with this one (a service chosen after the first had played).
            use std::io::{Seek, SeekFrom};
            let _ = self.file.flush();
            let f = self.file.get_mut();
            let _ = f.set_len(0);
            let _ = f.seek(SeekFrom::Start(0));
            self.shape = Some((rate, ch));
            self.bytes = 0;
            let _ = self.file.write_all(&Self::header(rate, ch, 0));
        }
        for &x in samples {
            let v = (x.clamp(-1.0, 1.0) * 32767.0) as i16;
            let _ = self.file.write_all(&v.to_le_bytes());
        }
        self.bytes += 2 * samples.len() as u32;
    }

    fn finish(mut self) -> std::io::Result<()> {
        use std::io::{Seek, SeekFrom, Write};
        self.file.flush()?;
        if let Some((rate, ch)) = self.shape {
            let f = self.file.get_mut();
            f.seek(SeekFrom::Start(0))?;
            f.write_all(&Self::header(rate, ch, self.bytes))?;
        }
        Ok(())
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

