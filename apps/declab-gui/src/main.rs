// made by CasualArclamp
//! DecLAB's window: one receiver, any decoder.
//!
//! The top bar holds the frequency, the source and its gains, the level and
//! the decoder button. Under it the band (spectrum over waterfall): left
//! click tunes the decoder's channel, middle-drag moves the band, a middle
//! click opens the decoder wheel, scroll zooms. Below, the decoder's card,
//! its scope and readings, its own settings panel and its services; the
//! status bar last. Laid out from the agreed mockups (docs/DESIGN.md §1).
//!
//! ```text
//! declab-gui [CAPTURE] [--rate HZ] [--freq HZ] [--radio airspy|hackrf]
//!            [--decoder ID] [--chan HZ] [--zoom Z] [--wheel]
//!            [--screenshot OUT.png] [--after SECS]
//! ```
//!
//! A screenshot run opens, runs `--after` seconds (default 4), saves the
//! window and closes; it neither reads nor writes the preferences.

mod band;
mod freq_display;
mod panels;
mod prefs;
#[cfg(test)]
mod sim;
mod waterfall;
mod wheel;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use declab_api::{DecoderFactory, Panel};
use declab_engine::meter::CpuGauge;
use declab_engine::{DecoderView, Host, HostOptions, Stats};
use declab_io::{IqFileReader, IqSource, format_from_path, name_hint};
use eframe::egui::{self, Color32, RichText};
use waterfall::{History, RingImage};
use wheel::{Entry, Outcome, Wheel, family_colour};

/// Smallest view span, Hz: zooming stops here.
const MIN_SPAN_HZ: f64 = 20e3;
/// Retunes while the band is being dragged, at most this often.
const RETUNE_EVERY: Duration = Duration::from_millis(90);

#[derive(Default)]
struct Options {
    file: Option<PathBuf>,
    rate: Option<f64>,
    freq: Option<f64>,
    radio: Option<SourceKind>,
    decoder: Option<String>,
    chan: Option<f64>,
    zoom: Option<f64>,
    wheel: bool,
    screenshot: Option<PathBuf>,
    after: f64,
}

/// "98.8M", "2048k" or plain Hz.
fn parse_hz(v: &str) -> Result<f64, String> {
    let (num, scale) = match v.chars().last() {
        Some('k' | 'K') => (&v[..v.len() - 1], 1e3),
        Some('M' | 'm') => (&v[..v.len() - 1], 1e6),
        Some('G' | 'g') => (&v[..v.len() - 1], 1e9),
        _ => (v, 1.0),
    };
    num.parse::<f64>().map(|n| n * scale).map_err(|_| format!("{v:?} is not a frequency"))
}

impl Options {
    fn from_args() -> Result<Options, String> {
        let mut o = Options { after: 4.0, ..Options::default() };
        let mut args = std::env::args().skip(1);
        while let Some(a) = args.next() {
            let mut value = |what: &str| args.next().ok_or(format!("{a} needs {what}"));
            match a.as_str() {
                "--rate" => o.rate = Some(parse_hz(&value("a rate")?)?),
                "--freq" => o.freq = Some(parse_hz(&value("a frequency")?)?),
                "--chan" => o.chan = Some(parse_hz(&value("a frequency")?)?),
                "--zoom" => o.zoom = Some(value("a factor")?.parse().map_err(|_| "--zoom: a number")?),
                "--decoder" => o.decoder = Some(value("a decoder id")?),
                "--radio" => {
                    o.radio = Some(match value("airspy or hackrf")?.as_str() {
                        "airspy" => SourceKind::Airspy,
                        "hackrf" => SourceKind::HackRf,
                        r => return Err(format!("unknown radio {r:?}")),
                    })
                }
                "--wheel" => o.wheel = true,
                "--screenshot" => o.screenshot = Some(value("a .png path")?.into()),
                "--after" => o.after = value("seconds")?.parse().map_err(|_| "--after: seconds")?,
                "-h" | "--help" => {
                    return Err("declab-gui [CAPTURE] [--rate HZ] [--freq HZ] [--radio airspy|hackrf] \
                                [--decoder ID] [--chan HZ] [--zoom Z] [--wheel] [--screenshot OUT.png] [--after SECS]"
                        .into());
                }
                f if !f.starts_with("--") => o.file = Some(f.into()),
                other => return Err(format!("unknown option {other}")),
            }
        }
        Ok(o)
    }
}

fn main() -> eframe::Result {
    let opts = match Options::from_args() {
        Ok(o) => o,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("DecLAB")
            .with_inner_size([1440.0, 940.0])
            .with_min_inner_size([960.0, 640.0]),
        ..Default::default()
    };
    eframe::run_native(
        "DecLAB",
        options,
        Box::new(move |cc| {
            cc.egui_ctx.set_theme(egui::Theme::Dark);
            // One monospace face throughout, as in the other Dec apps.
            cc.egui_ctx.style_mut_of(egui::Theme::Dark, |s| {
                s.override_text_style = Some(egui::TextStyle::Monospace);
            });
            Ok(Box::new(App::new(opts)))
        }),
    )
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SourceKind {
    Airspy,
    HackRf,
    File,
}

impl SourceKind {
    fn label(self) -> &'static str {
        match self {
            SourceKind::Airspy => "Airspy R2",
            SourceKind::HackRf => "HackRF One",
            SourceKind::File => "IQ file",
        }
    }
    fn key(self) -> &'static str {
        match self {
            SourceKind::Airspy => "airspy",
            SourceKind::HackRf => "hackrf",
            SourceKind::File => "file",
        }
    }
}

/// Live controls of the open radio (gains, retune from the GUI thread).
enum Control {
    None,
    #[cfg(feature = "hackrf")]
    HackRf(declab_io::HackRfControl),
    #[cfg(feature = "airspy")]
    Airspy(declab_io::AirspyControl),
}

struct App {
    opts: Options,
    /// What is to be opened.
    kind: SourceKind,
    file: Option<PathBuf>,
    file_rate: f64,
    /// The radio's centre (what it is tuned to, or a file's label).
    center_hz: f64,
    hack_rate: f64,
    hack_amp: bool,
    hack_lna: u16,
    hack_vga: u16,
    air_lna: u8,
    air_mix: u8,
    air_vga: u8,

    host: Option<Host>,
    control: Control,
    live: bool,
    source_rate: f64,
    error: Option<String>,

    /// Index into `declab_decoders::ALL`.
    dec: usize,
    panel: Option<Box<dyn Panel>>,
    chan_hz: f64,
    bandwidth_hz: f64,
    selected_service: Option<u32>,
    volume: f32,

    /// The view: its centre, and how far it is zoomed into the source's span.
    view_hz: f64,
    zoom: f64,
    dragging: bool,
    last_retune: Instant,
    wheel: Option<Wheel>,

    history: History,
    ring: RingImage,
    smooth: Vec<f32>,
    row_center: f64,
    row_rate: f64,
    wf_rows: usize,

    stats: Stats,
    view: DecoderView,
    cpu: CpuGauge,

    started: Instant,
    shot_sent: bool,
    saving: bool,
}

impl App {
    fn new(opts: Options) -> App {
        // A screenshot run must come out the same each time: no preferences.
        let saving = opts.screenshot.is_none();
        let p = if saving { prefs::load() } else { Vec::new() };
        let kind = opts.radio.or(opts.file.as_ref().map(|_| SourceKind::File)).unwrap_or(match prefs::get(&p, "source") {
            Some("hackrf") => SourceKind::HackRf,
            Some("file") => SourceKind::File,
            _ => SourceKind::Airspy,
        });
        let file = opts.file.clone().or_else(|| prefs::get(&p, "file").map(PathBuf::from));
        let dec_id = opts.decoder.clone().or_else(|| prefs::get(&p, "decoder").map(String::from));
        let dec = dec_id
            .and_then(|id| declab_decoders::ALL.iter().position(|f| f.info().id == id))
            .unwrap_or(0);
        let center_hz = opts.freq.or(prefs::num(&p, "centre")).unwrap_or(98.0e6);
        let chan_hz = opts.chan.or(prefs::num(&p, "chan")).unwrap_or(center_hz);
        let mut app = App {
            kind,
            file,
            file_rate: opts.rate.unwrap_or(10e6),
            center_hz,
            hack_rate: opts.rate.filter(|_| kind == SourceKind::HackRf).or(prefs::num(&p, "hackrf_rate")).unwrap_or(10e6),
            hack_amp: prefs::get(&p, "hackrf_amp") == Some("1"),
            hack_lna: prefs::num(&p, "hackrf_lna").unwrap_or(24),
            hack_vga: prefs::num(&p, "hackrf_vga").unwrap_or(20),
            air_lna: prefs::num(&p, "airspy_lna").unwrap_or(12),
            air_mix: prefs::num(&p, "airspy_mix").unwrap_or(10),
            air_vga: prefs::num(&p, "airspy_vga").unwrap_or(8),
            host: None,
            control: Control::None,
            live: false,
            source_rate: 0.0,
            error: None,
            dec,
            panel: None,
            chan_hz,
            bandwidth_hz: 0.0,
            selected_service: None,
            volume: prefs::num(&p, "volume").unwrap_or(0.8),
            view_hz: center_hz,
            zoom: opts.zoom.or(prefs::num(&p, "zoom")).unwrap_or(1.0),
            dragging: false,
            last_retune: Instant::now(),
            wheel: None,
            history: History::default(),
            ring: RingImage::default(),
            smooth: Vec::new(),
            row_center: 0.0,
            row_rate: 0.0,
            wf_rows: 0,
            stats: Stats::default(),
            view: DecoderView::default(),
            cpu: CpuGauge::default(),
            started: Instant::now(),
            shot_sent: false,
            saving,
            opts,
        };
        app.choose_decoder(dec);
        // A file or radio named on the command line starts at once.
        if app.opts.file.is_some() || app.opts.radio.is_some() {
            app.start();
        }
        app
    }

    fn factory(&self) -> &'static dyn DecoderFactory {
        declab_decoders::ALL[self.dec]
    }

    /// The rate the source runs at, or would.
    fn rate(&self) -> f64 {
        if self.source_rate > 0.0 {
            return self.source_rate;
        }
        match self.kind {
            SourceKind::Airspy => 10e6,
            SourceKind::HackRf => self.hack_rate,
            SourceKind::File => self.file_rate,
        }
    }

    fn span(&self) -> f64 {
        (self.rate() / self.zoom).max(MIN_SPAN_HZ)
    }

    fn open_source(&mut self) -> Result<Box<dyn IqSource>, String> {
        self.control = Control::None;
        match self.kind {
            SourceKind::File => {
                let path = self.file.clone().ok_or("open a capture first")?;
                let hint = name_hint(&path);
                let fmt = format_from_path(&path).ok_or("unknown sample format (.cs8 .cs16 .cf32 .cu8 .wav)")?;
                let rate = self.opts.rate.or(hint.sample_rate).unwrap_or(self.file_rate);
                self.file_rate = rate;
                let mut r = IqFileReader::open(&path, fmt, rate, 131_072).map_err(|e| e.to_string())?;
                if let Some(c) = self.opts.freq.or(hint.center_hz) {
                    r = r.with_center_freq(c);
                }
                Ok(Box::new(r))
            }
            #[cfg(feature = "hackrf")]
            SourceKind::HackRf => {
                use declab_io::{HackRfGains, HackRfSettings, HackRfSource};
                let s = HackRfSettings {
                    center_hz: self.center_hz as u64,
                    sample_rate: self.hack_rate as u32,
                    gains: HackRfGains { amp: self.hack_amp, lna_db: self.hack_lna, vga_db: self.hack_vga },
                };
                let src = HackRfSource::open(s).map_err(|e| e.to_string())?;
                self.control = Control::HackRf(src.control());
                Ok(Box::new(src))
            }
            #[cfg(feature = "airspy")]
            SourceKind::Airspy => {
                use declab_io::{AirspyGains, AirspySettings, AirspySource};
                let s = AirspySettings {
                    center_hz: self.center_hz as u64,
                    gains: AirspyGains { lna: self.air_lna, mixer: self.air_mix, vga: self.air_vga }.clamped(),
                    ..AirspySettings::default()
                };
                let src = AirspySource::open(s).map_err(|e| e.to_string())?;
                self.control = Control::Airspy(src.control());
                Ok(Box::new(src))
            }
            #[allow(unreachable_patterns)]
            k => Err(format!("{} support was not built in", k.label())),
        }
    }

    fn start(&mut self) {
        self.stop();
        match self.open_source() {
            Ok(src) => {
                self.live = src.is_live();
                self.source_rate = src.sample_rate();
                if !self.live {
                    self.center_hz = src.center_freq();
                    if (self.chan_hz - self.center_hz).abs() > self.source_rate / 2.0 {
                        self.chan_hz = self.center_hz;
                    }
                }
                self.view_hz = self.center_hz;
                self.clamp_view();
                let opts = HostOptions { fft: 4096, rows_per_s: 30.0, segments_per_row: 12, ..HostOptions::default() };
                let mut host = Host::start(src, opts);
                host.set_volume(self.volume);
                host.set_decoder(Some(self.factory()));
                host.set_channel_at(self.chan_hz, self.bandwidth_hz);
                self.host = Some(host);
                self.history.reset();
                self.smooth.clear();
                self.row_rate = 0.0;
                self.error = None;
                self.save_prefs();
            }
            Err(e) => self.error = Some(e),
        }
    }

    fn stop(&mut self) {
        self.host = None;
        self.control = Control::None;
        self.source_rate = 0.0;
    }

    /// Run decoder `i` (on the running host too).
    fn choose_decoder(&mut self, i: usize) {
        self.dec = i.min(declab_decoders::ALL.len() - 1);
        let f = self.factory();
        self.panel = f.panel();
        self.bandwidth_hz = f.input(self.rate()).bandwidth_hz;
        self.selected_service = None;
        self.view = DecoderView::default();
        if let Some(h) = &mut self.host {
            h.set_decoder(Some(f));
            h.set_channel_at(self.chan_hz, self.bandwidth_hz);
        }
        self.save_prefs();
    }

    fn tune_channel(&mut self, hz: f64) {
        // Broadcast FM sits on a 100 kHz raster; anything else to 1 kHz.
        let snap = if self.factory().info().id == "wfm" { 100e3 } else { 1e3 };
        self.chan_hz = (hz / snap).round() * snap;
        // Off the edge of a radio's span: retune to it.
        if self.live && (self.chan_hz - self.center_hz).abs() > self.rate() / 2.0 - self.bandwidth_hz {
            self.retune(self.chan_hz);
            self.view_hz = self.chan_hz;
        }
        if let Some(h) = &self.host {
            h.set_channel_at(self.chan_hz, self.bandwidth_hz);
        }
        self.save_prefs();
    }

    fn retune(&mut self, hz: f64) {
        self.center_hz = hz.round();
        if let Some(h) = &self.host {
            h.retune(self.center_hz);
        }
        self.last_retune = Instant::now();
    }

    /// Keep the view inside what the source covers (a radio's view moves the
    /// radio, so only a file's is held in).
    fn clamp_view(&mut self) {
        let half = (self.rate() - self.span()).max(0.0) / 2.0;
        if !self.live {
            self.view_hz = self.view_hz.clamp(self.center_hz - half, self.center_hz + half);
        } else {
            let c = self.center_hz;
            self.view_hz = self.view_hz.clamp(c - half, c + half);
        }
    }

    fn apply_gains(&mut self) {
        match &self.control {
            #[cfg(feature = "hackrf")]
            Control::HackRf(c) => {
                let g = declab_io::HackRfGains { amp: self.hack_amp, lna_db: self.hack_lna, vga_db: self.hack_vga };
                if let Err(e) = c.set_gains(g.snapped()) {
                    self.error = Some(e.to_string());
                }
            }
            #[cfg(feature = "airspy")]
            Control::Airspy(c) => {
                let g = declab_io::AirspyGains { lna: self.air_lna, mixer: self.air_mix, vga: self.air_vga };
                if let Err(e) = c.set_gains(g.clamped()) {
                    self.error = Some(e.to_string());
                }
            }
            _ => {}
        }
        self.save_prefs();
    }

    fn save_prefs(&self) {
        if !self.saving {
            return;
        }
        prefs::set_many(&[
            ("source", self.kind.key().into()),
            ("file", self.file.as_ref().map(|p| p.display().to_string()).unwrap_or_default()),
            ("decoder", self.factory().info().id.into()),
            ("centre", format!("{:.0}", self.center_hz)),
            ("chan", format!("{:.0}", self.chan_hz)),
            ("zoom", format!("{:.3}", self.zoom)),
            ("volume", format!("{:.2}", self.volume)),
            ("hackrf_rate", format!("{:.0}", self.hack_rate)),
            ("hackrf_amp", if self.hack_amp { "1" } else { "0" }.into()),
            ("hackrf_lna", self.hack_lna.to_string()),
            ("hackrf_vga", self.hack_vga.to_string()),
            ("airspy_lna", self.air_lna.to_string()),
            ("airspy_mix", self.air_mix.to_string()),
            ("airspy_vga", self.air_vga.to_string()),
        ]);
    }

    /// Take what the host has made since the last frame.
    fn poll(&mut self) {
        let Some(h) = &self.host else { return };
        for row in h.take_rows() {
            if row.sample_rate != self.row_rate || row.center_hz != self.row_center {
                self.smooth.clear();
            }
            self.row_center = row.center_hz;
            self.row_rate = row.sample_rate;
            if self.smooth.len() != row.db.len() {
                self.smooth = row.db.clone();
            } else {
                for (s, &d) in self.smooth.iter_mut().zip(&row.db) {
                    *s += 0.3 * (d - *s);
                }
            }
            self.history.push_at(row.db, row.center_hz, row.sample_rate);
        }
        self.stats = h.stats();
        self.view = h.view();
        let msgs = h.take_custom();
        if let Some(p) = &mut self.panel {
            for m in &msgs {
                p.receive(m.as_ref());
            }
        }
        if !self.live && self.stats.center_hz != 0.0 {
            self.center_hz = self.stats.center_hz;
        }
    }

    fn entries(&self) -> Vec<Entry> {
        let rate = self.rate();
        declab_decoders::ALL
            .iter()
            .map(|f| {
                let i = f.info();
                let spec = f.input(rate);
                let r = spec.sample_rate.map_or("source rate".to_string(), |r| {
                    if r >= 1e6 { format!("{:.3} MS/s", r / 1e6) } else { format!("{:.0} kS/s", r / 1e3) }
                });
                let bw = if spec.bandwidth_hz >= 1e6 {
                    format!("{:.1} MHz", spec.bandwidth_hz / 1e6)
                } else {
                    format!("{:.1} kHz", spec.bandwidth_hz / 1e3)
                };
                Entry { id: i.id, label: i.name, family: i.family, input: format!("{r} · {bw}"), summary: i.summary }
            })
            .collect()
    }

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        let colour = family_colour(self.factory().info().family);
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let mut hz = self.chan_hz;
            if freq_display::show(ui, &mut hz, 0.0, 6e9, 40.0) {
                self.tune_channel(hz);
            }
            ui.add_space(14.0);
            ui.vertical(|ui| {
                ui.horizontal_wrapped(|ui| {
                    let running = self.host.is_some();
                    ui.label(RichText::new("source").color(Color32::GRAY));
                    let before = self.kind;
                    egui::ComboBox::from_id_salt("source").selected_text(self.kind.label()).show_ui(ui, |ui| {
                        for k in [SourceKind::Airspy, SourceKind::HackRf, SourceKind::File] {
                            ui.selectable_value(&mut self.kind, k, k.label());
                        }
                    });
                    if self.kind != before {
                        self.stop();
                        self.save_prefs();
                    }
                    match self.kind {
                        SourceKind::File => {
                            let name = self
                                .file
                                .as_ref()
                                .and_then(|p| p.file_name())
                                .map(|n| n.to_string_lossy().into_owned())
                                .unwrap_or_else(|| "no file".into());
                            ui.label(RichText::new(name).color(Color32::WHITE));
                            if ui.button("Open…").clicked()
                                && let Some(p) = rfd::FileDialog::new()
                                    .add_filter("IQ capture", &["cs8", "cs16", "cf32", "cu8", "wav", "iq"])
                                    .pick_file()
                            {
                                self.file = Some(p);
                                self.opts.rate = None;
                                self.opts.freq = None;
                                self.start();
                            }
                        }
                        SourceKind::HackRf => {
                            ui.label(RichText::new("rate").color(Color32::GRAY));
                            let before = self.hack_rate;
                            egui::ComboBox::from_id_salt("hackrf_rate")
                                .selected_text(format!("{:.2} MS/s", self.hack_rate / 1e6))
                                .show_ui(ui, |ui| {
                                    for r in [2e6, 4e6, 8e6, 10e6, 12.5e6, 16e6, 20e6] {
                                        ui.selectable_value(&mut self.hack_rate, r, format!("{:.2} MS/s", r / 1e6));
                                    }
                                });
                            if self.hack_rate != before && running {
                                self.start();
                            }
                            ui.menu_button(
                                format!("{}LNA {} · VGA {}", if self.hack_amp { "AMP · " } else { "" }, self.hack_lna, self.hack_vga),
                                |ui| {
                                    let mut ch = ui.checkbox(&mut self.hack_amp, "RF amp (+14 dB)").changed();
                                    ch |= ui.add(egui::Slider::new(&mut self.hack_lna, 0..=40).step_by(8.0).text("LNA dB")).changed();
                                    ch |= ui.add(egui::Slider::new(&mut self.hack_vga, 0..=62).step_by(2.0).text("VGA dB")).changed();
                                    if ch {
                                        self.apply_gains();
                                    }
                                },
                            );
                        }
                        SourceKind::Airspy => {
                            ui.label(RichText::new("rate").color(Color32::GRAY));
                            ui.label(RichText::new("10.00 MS/s").color(Color32::WHITE));
                            ui.menu_button(format!("LNA {} · MIX {} · VGA {}", self.air_lna, self.air_mix, self.air_vga), |ui| {
                                let mut ch = ui.add(egui::Slider::new(&mut self.air_lna, 0..=14).text("LNA")).changed();
                                ch |= ui.add(egui::Slider::new(&mut self.air_mix, 0..=15).text("mixer")).changed();
                                ch |= ui.add(egui::Slider::new(&mut self.air_vga, 0..=15).text("VGA")).changed();
                                if ch {
                                    self.apply_gains();
                                }
                            });
                        }
                    }
                    ui.label(RichText::new("centre").color(Color32::GRAY));
                    ui.label(RichText::new(format!("{:.3} MHz", self.center_hz / 1e6)).color(Color32::WHITE));
                    let rec = self.stats.recording.as_ref().map(|(_, b)| *b);
                    if running {
                        let label = match rec {
                            Some(b) => RichText::new(format!("● {:.0} MB", b as f64 / 1e6)).color(panels::RED),
                            None => RichText::new("● Record").color(panels::RED),
                        };
                        if ui.button(label).clicked()
                            && let Some(h) = &self.host
                        {
                            if rec.is_some() {
                                h.record(None);
                            } else {
                                let name = format!(
                                    "declab_{:.3}MHz_{}Msps.cs8",
                                    self.center_hz / 1e6,
                                    panels::value(self.rate() / 1e6)
                                );
                                if let Some(p) = rfd::FileDialog::new().set_file_name(name).save_file() {
                                    h.record(Some(p));
                                }
                            }
                        }
                        if ui.button("■ Stop").clicked() {
                            self.stop();
                        }
                    } else if ui.button(RichText::new("▶ Start").color(panels::GREEN)).clicked() {
                        self.start();
                    }
                });
                ui.horizontal(|ui| {
                    panels::level_bar(ui, self.stats.rms_dbfs, self.host.is_some() && self.stats.samples > 0);
                    let f = self.factory().info();
                    let b = egui::Button::new(RichText::new(format!("●  {}  ▾ decoder", f.name)).color(colour))
                        .stroke(egui::Stroke::new(1.0, colour));
                    if ui.add(b).clicked() {
                        let c = ui.ctx().content_rect().center();
                        self.wheel = Some(Wheel::new(c, self.dec, ui.ctx().cumulative_frame_nr()));
                    }
                    ui.label(RichText::new("or middle-click the band").size(12.0).color(Color32::GRAY));
                });
            });
        });
        ui.add_space(4.0);
    }

    fn handle_band(&mut self, actions: Vec<band::Action>, ctx: &egui::Context) {
        for a in actions {
            match a {
                band::Action::Tune(hz) => self.tune_channel(hz),
                band::Action::Pan(d) => {
                    self.dragging = true;
                    self.view_hz += d;
                    if self.live {
                        // The radio follows the view, a retune at a time.
                        if self.last_retune.elapsed() >= RETUNE_EVERY {
                            let target = self.view_hz;
                            self.retune(target);
                        }
                    } else {
                        self.clamp_view();
                    }
                }
                band::Action::DragEnd => {
                    self.dragging = false;
                    if self.live {
                        let target = self.view_hz;
                        self.retune(target);
                        self.clamp_view();
                    }
                    self.save_prefs();
                }
                band::Action::OpenWheel(p) => {
                    self.wheel = Some(Wheel::new(p, self.dec, ctx.cumulative_frame_nr()));
                }
                band::Action::Zoom { factor, anchor_hz, frac } => {
                    let max_zoom = (self.rate() / MIN_SPAN_HZ).max(1.0);
                    self.zoom = (self.zoom * factor).clamp(1.0, max_zoom);
                    let span = self.span();
                    self.view_hz = anchor_hz - (frac - 0.5) * span;
                    self.clamp_view();
                }
            }
        }
    }

    fn handle_screenshot(&mut self, ctx: &egui::Context) {
        let Some(path) = self.opts.screenshot.clone() else { return };
        if !self.shot_sent && self.started.elapsed().as_secs_f64() >= self.opts.after {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
            self.shot_sent = true;
        }
        let image = ctx.input(|i| {
            i.raw.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        let Some(image) = image else { return };
        let rgba: Vec<u8> = image.pixels.iter().flat_map(|c| c.to_array()).collect();
        let (w, h) = (image.size[0] as u32, image.size[1] as u32);
        match image::save_buffer(&path, &rgba, w, h, image::ColorType::Rgba8) {
            Ok(()) => eprintln!(
                "screenshot saved to {} (app CPU {:.0} %, decoder {:.0} %, {})",
                path.display(),
                self.cpu.percent,
                self.view.cpu_percent,
                self.stats.describe
            ),
            Err(e) => eprintln!("cannot save screenshot {}: {e}", path.display()),
        }
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll();
        self.cpu.poll();
        if self.opts.wheel && self.wheel.is_none() && self.started.elapsed() > Duration::from_millis(300) {
            self.opts.wheel = false;
            let c = ctx.content_rect().center() - egui::vec2(0.0, 120.0);
            self.wheel = Some(Wheel::new(c, self.dec, ctx.cumulative_frame_nr()));
        }

        let info = self.factory().info();
        let colour = family_colour(info.family);
        let running = self.host.as_ref().is_some_and(|h| h.running());

        egui::Panel::top("top").frame(egui::Frame::side_top_panel(&ctx.global_style()).fill(Color32::from_rgb(25, 25, 25))).show(ui, |ui| self.top_bar(ui));
        egui::Panel::bottom("status").show(ui, |ui| {
            panels::status(ui, &self.stats, &self.view, running, self.cpu.percent, self.error.as_deref());
        });
        egui::Panel::bottom("decoder").resizable(false).exact_size(280.0).show(ui, |ui| {
            ui.add_space(6.0);
            panels::card(ui, info.name, colour, &self.view, running);
            ui.add_space(6.0);
            let w = ui.available_width();
            ui.columns(3, |cols| {
                let _ = w;
                panels::boxed(&mut cols[0], |ui| panels::scope_and_readings(ui, &self.view, colour));
                panels::boxed(&mut cols[1], |ui| {
                    let (cmds, bw, vol) = panels::decoder_settings(
                        ui,
                        info.name,
                        self.panel.as_mut(),
                        &mut self.bandwidth_hz,
                        &mut self.volume,
                    );
                    if let Some(h) = &self.host {
                        for c in cmds {
                            h.command(c);
                        }
                        if bw {
                            h.set_channel_at(self.chan_hz, self.bandwidth_hz);
                        }
                        if vol {
                            h.set_volume(self.volume);
                        }
                    }
                    if vol {
                        self.save_prefs();
                    }
                });
                panels::boxed(&mut cols[2], |ui| {
                    if let Some(id) = panels::services(ui, &self.view, self.selected_service) {
                        self.selected_service = Some(id);
                        if let Some(h) = &self.host {
                            h.command(declab_api::Command::SelectService(id));
                        }
                    }
                });
            });
        });

        let mut actions = Vec::new();
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            self.wf_rows = band::waterfall_rows(ui.available_height(), ctx.pixels_per_point()).max(64);
            self.history.set_capacity(self.wf_rows);
            let tex = self.ring.update(&ctx, &self.history);
            let label = format!("{} {:.4}", info.name, self.chan_hz / 1e6);
            let idle = match (&self.host, &self.error) {
                (_, Some(e)) => e.clone(),
                (None, _) => "press Start, or open an IQ file".to_string(),
                (Some(_), _) => "waiting for samples…".to_string(),
            };
            let inp = band::BandInput {
                spectrum_db: &self.smooth,
                row_center_hz: self.row_center,
                row_rate: self.row_rate,
                view_center_hz: self.view_hz,
                view_span_hz: self.span(),
                levels: self.history.levels(),
                waterfall: tex,
                channel: Some((self.chan_hz, self.bandwidth_hz, colour, label)),
                moving: self.dragging.then(|| format!("moving the band · centre {:.3} MHz", self.view_hz / 1e6)),
                inert: self.wheel.is_some(),
                idle_text: &idle,
            };
            actions = band::show(ui, &inp);
        });
        self.handle_band(actions, &ctx);

        let entries = if self.wheel.is_some() { self.entries() } else { Vec::new() };
        if let Some(w) = &mut self.wheel {
            match w.show(&ctx, &entries) {
                Some(Outcome::Pick(i)) => {
                    self.wheel = None;
                    if i != self.dec {
                        self.choose_decoder(i);
                    }
                }
                Some(Outcome::Cancel) => self.wheel = None,
                None => {}
            }
        }

        self.handle_screenshot(&ctx);
        // Stopped, the window only redraws on input (it costs a fifth of a
        // core to draw it 30 times a second for nothing).
        if self.host.is_some() || self.opts.screenshot.is_some() {
            ctx.request_repaint_after(Duration::from_millis(33));
        }
    }
}
