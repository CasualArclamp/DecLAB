// made by CasualArclamp
//! The DecLAB decoder plug-in API.
//!
//! A decoder is anything that turns IQ samples into something worth having:
//! a transport stream, IP packets, audio, text, pictures, measurements. The
//! host (the DecLAB app) owns everything around it — the radio or file, the
//! spectrum and waterfall, the down-converter, recording, the player, the
//! services list — so a decoder only has to decode.
//!
//! The shape, end to end:
//!
//! ```text
//!  IqSource ─► host: spectrum, waterfall, DDC to the decoder's InputSpec
//!                 │
//!                 ▼  Block (IQ at the rate asked for, plus the tuning)
//!          Box<dyn Decoder>::process   (one worker thread per decoder)
//!                 │
//!                 ▼  Sink: TS, IP, audio, services, measurements, scopes…
//!  host: TS hub, player, PCAP writer, GUI cards and plots
//! ```
//!
//! Writing a new decoder means one file: a type implementing [`Decoder`], a
//! [`DecoderFactory`] that describes and builds it, and one line adding the
//! factory to the list in `declab-decoders`. Big decoders (DVB-T2, DVB-S2)
//! keep their DSP in crates of their own and are wrapped by such a file.
//!
//! Rust note: `Box<dyn Decoder>` is a trait object, Rust's equivalent of a
//! pointer to an abstract base class, so the host can hold decoders it knows
//! nothing about. `Send` means the decoder may move to its worker thread.

pub use num_complex::Complex32;

/// One complex baseband sample, full scale = magnitude 1.0.
pub type Iq = Complex32;

/// Which part of the app a decoder lives in: the mode picker groups by it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Family {
    /// DVB-T, DVB-T2.
    Terrestrial,
    /// DVB-S, DVB-S2/S2X, DVB-RCS2 return.
    Satellite,
    /// DAB, DAB+.
    Radio,
    /// Anything else: tools, examples, third-party decoders.
    Other,
}

impl Family {
    pub const ALL: [Family; 4] = [Family::Terrestrial, Family::Satellite, Family::Radio, Family::Other];

    pub fn label(self) -> &'static str {
        match self {
            Family::Terrestrial => "Terrestrial TV",
            Family::Satellite => "Satellite",
            Family::Radio => "Digital radio",
            Family::Other => "Other",
        }
    }
}

/// What the mode picker shows about a decoder.
#[derive(Debug, Clone)]
pub struct DecoderInfo {
    /// Stable, lowercase, saved in prefs and used on the command line
    /// (`dvbt2`, `dvbs2-ts`, `dab`).
    pub id: &'static str,
    /// Shown in the picker ("DVB-T2").
    pub name: &'static str,
    pub family: Family,
    /// The standard it follows, for the about box ("ETSI EN 302 755").
    pub standard: &'static str,
    /// One line on what it does.
    pub summary: &'static str,
}

/// The IQ a decoder wants. The host down-converts and resamples to it, so a
/// decoder never sees the wideband stream unless it asks for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InputSpec {
    /// Samples per second the decoder is fed. `None` takes the source's own
    /// rate untouched (DecDVB-T's direct FFT on the radio's samples saved the
    /// most CPU of anything, so a decoder can refuse resampling).
    pub sample_rate: Option<f64>,
    /// Bandwidth the host's channel filter passes, Hz.
    pub bandwidth_hz: f64,
    /// Samples per [`Block`], roughly; the host may vary it.
    pub block_len: usize,
}

/// Where the samples in a block came from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// RF frequency at the block's 0 Hz, if known (0.0 for a plain file).
    pub center_hz: f64,
    /// The block's sample rate.
    pub sample_rate: f64,
    /// Counts retunes: a block from after a retune has a higher number, so
    /// a decoder holding lock knows to drop it.
    pub generation: u32,
}

/// One slice of IQ handed to [`Decoder::process`].
pub struct Block<'a> {
    pub iq: &'a [Iq],
    pub tuning: Tuning,
    /// Index of `iq[0]` since the stream started, at the block's rate.
    pub first_sample: u64,
    /// Samples the source dropped just before this block (the radio's USB
    /// fell behind): the decoder's timing has a hole there.
    pub dropped_before: u64,
}

/// How a decoder reports its own state on the card at the top of the app.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Lock {
    #[default]
    Searching,
    /// Found the signal, not yet error free (OFDM symbol sync, PL sync…).
    Synced,
    /// Decoding: the FEC is clean or nearly so.
    Locked,
}

/// A named reading for the card and the history plots: MER, BER, CNR,
/// LDPC iterations, packets per second.
#[derive(Debug, Clone, PartialEq)]
pub struct Measurement {
    pub name: &'static str,
    pub value: f64,
    pub unit: &'static str,
}

/// A programme in the services list: a TV or radio station, an IP stream.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Service {
    /// The decoder's own number for it (DVB service_id, DAB SId, a PID…);
    /// [`Command::SelectService`] carries it back.
    pub id: u32,
    pub name: String,
    pub provider: String,
    /// "TV", "HD TV", "Radio", "DAB+", "Data".
    pub kind: String,
}

/// Decoded audio, interleaved f32 at `sample_rate`.
#[derive(Debug, Clone, PartialEq)]
pub struct Audio {
    pub service: u32,
    pub sample_rate: u32,
    pub channels: u16,
    pub samples: Vec<f32>,
}

/// A picture to show (a DAB slideshow, a station logo): the encoded bytes.
#[derive(Debug, Clone, PartialEq)]
pub struct Picture {
    pub service: u32,
    /// MIME type ("image/jpeg", "image/png").
    pub mime: String,
    pub bytes: Vec<u8>,
}

/// Everything a decoder hands back from one [`Decoder::process`] call. The
/// host empties it after each call, so a decoder only pushes.
///
/// Plain `Vec`s rather than callbacks: the decoder never waits on the GUI or
/// the player, and the host can batch the hand-over to other threads.
#[derive(Default)]
pub struct Sink {
    pub lock: Lock,
    /// 188-byte MPEG transport-stream packets (DVB-T/T2, DVB-S/S2 TS mode).
    pub ts: Vec<[u8; 188]>,
    /// IPv4/IPv6 datagrams (DVB-S2 GSE, DVB-RCS2 return): the host writes PCAP.
    pub ip: Vec<Vec<u8>>,
    /// Decoded audio (DAB/DAB+). TV audio comes out of the TS instead.
    pub audio: Vec<Audio>,
    pub pictures: Vec<Picture>,
    /// Text for a service: DAB dynamic label, DVB now/next.
    pub text: Vec<(u32, String)>,
    /// The services list, sent whole whenever it changes.
    pub services: Option<Vec<Service>>,
    pub measurements: Vec<Measurement>,
    /// Equalised constellation points for the scope (a decimated handful,
    /// not every carrier).
    pub constellation: Vec<Iq>,
    /// One line for the log panel.
    pub log: Vec<String>,
    /// A frequency correction for the host's tuner, Hz (an OFDM receiver's
    /// whole-carrier offset, a satellite's Doppler): the host retunes and
    /// bumps [`Tuning::generation`].
    pub retune_by_hz: Option<f64>,
    /// Anything a decoder's own panel needs that the fields above do not
    /// cover (a DVB-T2 L1 dump, a DAB FIC table). The panel downcasts it.
    pub custom: Vec<Box<dyn std::any::Any + Send>>,
}

impl Sink {
    pub fn measure(&mut self, name: &'static str, value: f64, unit: &'static str) {
        self.measurements.push(Measurement { name, value, unit });
    }

    pub fn log(&mut self, line: impl Into<String>) {
        self.log.push(line.into());
    }

    /// Ready for the next call, keeping the vectors' capacity so the loop
    /// does not allocate.
    pub fn clear(&mut self) {
        self.ts.clear();
        self.ip.clear();
        self.audio.clear();
        self.pictures.clear();
        self.text.clear();
        self.services = None;
        self.measurements.clear();
        self.constellation.clear();
        self.log.clear();
        self.retune_by_hz = None;
        self.custom.clear();
    }
}

/// What the app asks of a running decoder.
pub enum Command {
    /// Decode this service (DAB decodes only the selected subchannel; DVB
    /// keeps the whole TS but the player follows it).
    SelectService(u32),
    /// A decoder-specific setting changed in its panel (`"plp"` → `"1"`).
    Set { key: String, value: String },
    /// Anything a panel sends that is not a string pair.
    Custom(Box<dyn std::any::Any + Send>),
}

/// A running decoder. It lives on a worker thread of its own.
pub trait Decoder: Send {
    /// Decode one block, pushing results into `out`. Called again and again
    /// with consecutive samples; never blocks for long, never allocates in
    /// steady state if it can help it (CPU is the main requirement).
    fn process(&mut self, block: &Block<'_>, out: &mut Sink);

    /// A command from the app, between two `process` calls.
    fn command(&mut self, _cmd: Command, _out: &mut Sink) {}

    /// The tuner moved or the file looped: forget lock and start again.
    fn reset(&mut self) {}

    /// The worker threads this decoder may spread its FEC over (DVB-T2's
    /// LDPC, DVB-S2's backlog). 1 keeps everything on its own thread.
    fn wants_threads(&self) -> usize {
        1
    }
}

/// A decoder's settings panel. It lives on the GUI thread, so it cannot
/// touch the [`Decoder`] (which is busy on its worker): it reads what the
/// decoder sent in [`Sink::custom`] and answers with [`Command`]s.
#[cfg(feature = "ui")]
pub trait Panel {
    /// Something from the decoder's [`Sink::custom`].
    fn receive(&mut self, _msg: &(dyn std::any::Any + Send)) {}

    /// Draw the panel; push any commands for the decoder into `commands`.
    fn ui(&mut self, ui: &mut egui::Ui, commands: &mut Vec<Command>);
}

/// Describes a decoder and builds it. One `static` of a type implementing
/// this goes in `declab-decoders`' list per decoder.
///
/// `Sync` because the list is shared by every thread.
pub trait DecoderFactory: Sync {
    fn info(&self) -> DecoderInfo;

    /// The IQ this decoder wants. `source_rate` is the radio's rate, for a
    /// decoder that adapts to it.
    fn input(&self, source_rate: f64) -> InputSpec;

    fn create(&self) -> Box<dyn Decoder>;

    /// The decoder's own panel, if it has settings beyond the services list.
    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn Panel>> {
        None
    }
}
