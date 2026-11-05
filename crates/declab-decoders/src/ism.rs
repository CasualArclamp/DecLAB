// made by CasualArclamp
//! ISM devices: weather stations, temperature/humidity sensors, remotes and
//! door/window sensors, tyre-pressure sensors, energy monitors and meters
//! on 315, 345, 433, 868 and 915 MHz, as rtl_433 decodes them. The
//! protocols are `crates/declab-ism` (ported from rtl_433); this file wraps
//! them.
//!
//! The channel is wide (2 MHz by default, any width the user drags it to):
//! the receiver finds bursts anywhere in it, each at its own frequency.
//!
//! Scopes: "Pulses" is the last package's pulses (carrier on high, off
//! low; FSK: high tone high), "Bursts" puts each burst of the last minute
//! at its frequency (across) and age (down, newest at the top), with the
//! band's map under it. The card shows the latest device; the devices
//! heard are a table ([`Sink::devices`]); every message goes to the log.

use std::collections::{HashMap, VecDeque};

use declab_api::{
    Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Devices, Family, Gauge,
    HeardDevice, InputSpec, Iq, Lock, Sink, WaterfallRow,
};
use declab_ism::receiver::{Event, Unknown};
use declab_ism::{Group, Message, Receiver, ReceiverConfig, Value, device::field_label};

/// The channel rate: 10 MS/s ÷ 4, and 2 MHz of it passed (433.05–434.79
/// MHz, or 868–870 MHz, in one go).
const RATE: f64 = 2_500_000.0;
const BANDWIDTH: f64 = 2_000_000.0;
/// Messages kept on the card.
const RECENT: usize = 5;
/// The card says "locked" while messages came this recently.
const LOCKED_FOR_S: f64 = 60.0;
/// A device is "live" this long after its last message.
const LIVE_S: f64 = 30.0;
/// Bursts shown on the "Bursts" scope: the last minute's.
const BURSTS_FOR_S: f64 = 60.0;
/// Pulses shown on the "Pulses" scope.
const SCOPE_PULSES: usize = 16;
/// Flex specs kept (prefs keys `flex1` … `flex8`).
pub const FLEX_SLOTS: usize = 8;
/// Unknown packages kept for the panel.
const UNKNOWN_KEEP: usize = 12;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "ism",
            name: "ISM devices",
            family: Family::Other,
            standard: "rtl_433 protocols",
            summary: "Weather sensors, remotes, tyre pressure, meters (433/868/315/915 MHz), like rtl_433",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: BANDWIDTH,
            block_len: 65_536,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Ism::new(Settings::default()))
    }

    fn default_scope(&self) -> &'static str {
        "Pulses"
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(panel::IsmPanel::default()))
    }
}

/// What the panel can change, as [`Command::Set`] keys and values.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// Weather, remotes, TPMS, meters, flex (in [`Group::ALL`]'s order).
    pub groups: [bool; 5],
    /// rtl_433's off-by-default decoders too.
    pub all: bool,
    /// Unknown packages to the log.
    pub unknown: bool,
    pub flex: [String; FLEX_SLOTS],
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            groups: [true; 5],
            all: false,
            unknown: true,
            flex: Default::default(),
        }
    }
}

impl Settings {
    pub fn set(&mut self, key: &str, value: &str) -> Result<(), String> {
        let on = || match value {
            "on" => Ok(true),
            "off" => Ok(false),
            v => Err(format!("{key}: {v:?} is not on or off")),
        };
        if let Some(i) = Group::ALL.iter().position(|g| g.key() == key) {
            self.groups[i] = on()?;
            return Ok(());
        }
        if let Some(n) = key
            .strip_prefix("flex")
            .and_then(|n| n.parse::<usize>().ok())
            && (1..=FLEX_SLOTS).contains(&n)
        {
            self.flex[n - 1] = value.trim().to_string();
            return Ok(());
        }
        match key {
            "all" => self.all = on()?,
            "unknown" => self.unknown = on()?,
            k => return Err(format!("ism: no setting {k:?}")),
        }
        Ok(())
    }

    pub fn commands(&self) -> Vec<Command> {
        let onoff = |b: bool| if b { "on" } else { "off" }.to_string();
        let mut v: Vec<Command> = Group::ALL
            .iter()
            .zip(self.groups)
            .map(|(g, on)| Command::Set {
                key: g.key().into(),
                value: onoff(on),
            })
            .collect();
        v.push(Command::Set {
            key: "all".into(),
            value: onoff(self.all),
        });
        v.push(Command::Set {
            key: "unknown".into(),
            value: onoff(self.unknown),
        });
        for (i, f) in self.flex.iter().enumerate() {
            v.push(Command::Set {
                key: format!("flex{}", i + 1),
                value: f.clone(),
            });
        }
        v
    }

    fn config(&self) -> ReceiverConfig {
        ReceiverConfig {
            groups: Group::ALL
                .iter()
                .zip(self.groups)
                .filter(|(_, on)| *on)
                .map(|(g, _)| *g)
                .collect(),
            include_disabled: self.all,
            flex: self
                .flex
                .iter()
                .filter(|s| !s.is_empty())
                .cloned()
                .collect(),
        }
    }
}

/// What the decoder tells its panel (through [`Sink::custom`]).
#[derive(Debug, Clone, Default)]
pub struct PanelNews {
    /// Devices tried.
    pub devices: usize,
    /// Flex specs that did not parse.
    pub flex_errors: Vec<String>,
    /// The latest packages nothing decoded, newest first.
    pub unknown: Vec<Unknown>,
}

struct Heard {
    dev: HeardDevice,
    /// The last message, to tell a changed reading from a repeat.
    last: Message,
}

struct Ism {
    settings: Settings,
    rx: Receiver,
    rate: f64,
    centre_hz: f64,
    now_s: f64,
    since_s: f64,
    last_msg_s: Option<f64>,
    heard: HashMap<(String, String, String), Heard>,
    heard_dirty: bool,
    recent: VecDeque<(String, String)>,
    latest: Option<(f64, declab_ism::receiver::Decoded)>,
    bursts: VecDeque<(f64, f64, bool)>,
    unknown: VecDeque<Unknown>,
    news_dirty: bool,
    since_report: usize,
    reports: u64,
}

/// Seconds as m:ss (h:mm:ss past an hour).
fn clock(s: f64) -> String {
    let s = s.max(0.0) as u64;
    if s >= 3600 {
        format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{}:{:02}", s / 60, s % 60)
    }
}

/// A field as the card shows it: label, and value with its unit.
fn shown(key: &str, v: &Value) -> (String, String) {
    let (label, unit) = field_label(key);
    let value = match (key, v) {
        ("battery_ok", Value::Int(1)) => "ok".to_string(),
        ("battery_ok", Value::Int(0)) => "low".to_string(),
        _ if unit.is_empty() => v.to_string(),
        _ if unit == "%" => format!("{v} %"),
        _ => format!("{v} {unit}"),
    };
    (label.replace('_', " "), value)
}

impl Ism {
    fn new(settings: Settings) -> Self {
        let mut rx = Receiver::new(RATE, &settings.config());
        // The band's map: a row each ~20 ms (196 frames of 102 µs).
        rx.set_map(196);
        Ism {
            settings,
            rx,
            rate: RATE,
            centre_hz: 0.0,
            now_s: 0.0,
            since_s: 0.0,
            last_msg_s: None,
            heard: HashMap::new(),
            heard_dirty: true,
            recent: VecDeque::new(),
            latest: None,
            bursts: VecDeque::new(),
            unknown: VecDeque::new(),
            news_dirty: true,
            since_report: 0,
            reports: 0,
        }
    }

    fn rf(&self, offset_hz: f64) -> f64 {
        self.centre_hz + offset_hz
    }

    fn take_events(&mut self, out: &mut Sink) {
        for e in std::mem::take(&mut self.rx.events) {
            match e {
                Event::Decoded(d) => {
                    self.last_msg_s = Some(d.at_s);
                    self.bursts.push_back((d.at_s, d.freq_hz, true));
                    let m = &d.message;
                    let rf = self.rf(d.freq_hz);
                    let fields: Vec<(String, String)> = m
                        .fields
                        .iter()
                        .filter(|(k, _)| !matches!(*k, "id" | "channel"))
                        .map(|(k, v)| shown(k, v))
                        .collect();
                    let id = m.id_text();
                    let ch = m.channel_text();
                    // rtl_433-style log line.
                    let mut line = format!("{:.3} MHz  {}", rf / 1e6, m.model);
                    if !id.is_empty() {
                        line += &format!("  id {id}");
                    }
                    if !ch.is_empty() {
                        line += &format!("  ch {ch}");
                    }
                    for (l, v) in &fields {
                        line += &format!("  {l} {v}");
                    }
                    line += &format!("  ({:.0} dB{})", d.snr_db, if d.fsk { " FSK" } else { "" });
                    out.log(line);
                    // The card's recent list.
                    let mut label = format!("{} {}", clock(d.at_s), m.model);
                    if !id.is_empty() {
                        label += &format!(" id {id}");
                    }
                    if !ch.is_empty() {
                        label += &format!(" ch {ch}");
                    }
                    self.recent.push_front((label, m.readings()));
                    self.recent.truncate(RECENT);
                    // The heard list.
                    let key = (m.model.clone(), id.clone(), ch.clone());
                    let kind = d.group.label().to_string();
                    let h = self.heard.entry(key).or_insert_with(|| Heard {
                        dev: HeardDevice {
                            model: m.model.clone(),
                            kind,
                            id: id.clone(),
                            channel: ch.clone(),
                            first_s: d.at_s,
                            ..Default::default()
                        },
                        last: Message::default(),
                    });
                    h.dev.count += 1;
                    h.dev.last_s = d.at_s;
                    h.dev.readings = m.readings();
                    h.dev.fields = fields;
                    h.dev.freq_hz = rf;
                    h.dev.rssi_db = d.rssi_db;
                    h.dev.snr_db = d.snr_db;
                    h.dev.battery_low = matches!(m.get("battery_ok"), Some(Value::Int(0)));
                    h.last = m.clone();
                    self.heard_dirty = true;
                    self.latest = Some((d.at_s, d));
                }
                Event::Repeat { .. } => {}
                Event::Unknown(u) => {
                    self.bursts.push_back((u.at_s, u.freq_hz, false));
                    if self.settings.unknown {
                        out.log(format!(
                            "{:.3} MHz  unknown  {}  ({:.0} dB)",
                            self.rf(u.freq_hz) / 1e6,
                            u.summary,
                            u.snr_db
                        ));
                    }
                    self.unknown.push_front(u);
                    self.unknown.truncate(UNKNOWN_KEEP);
                    self.news_dirty = true;
                }
            }
        }
    }

    fn report(&mut self, out: &mut Sink) {
        let st = self.rx.stats;
        out.lock = match self.last_msg_s {
            Some(t) if self.now_s - t < LOCKED_FOR_S => Lock::Locked,
            _ if st.bursts > 0 => Lock::Synced,
            _ => Lock::Searching,
        };

        // "Pulses": the last package as levels across the plot.
        if let Some(last) = &self.rx.last
            && !last.pulses.is_empty()
        {
            let p = &last.pulses;
            // The first pulses of it (up to 16), so the levels can be read;
            // the final gap left out of the scale when long.
            let n = p.len().min(SCOPE_PULSES);
            let total: i64 = p.pulse[..n]
                .iter()
                .zip(&p.gap[..n])
                .map(|(a, b)| (*a + *b) as i64)
                .sum::<i64>()
                .max(1);
            let tail = p.gap[n - 1] as i64;
            let span = (total - tail + tail.min(total / 10)).max(1);
            // Levels along the time axis, and the edges drawn as columns
            // of points, so the trace reads as a square wave.
            let points = 1200;
            let mut pts = Vec::with_capacity(points + 16 * 2 * p.len());
            let (mut k, mut t0) = (0usize, 0i64);
            let mut last_on = false;
            for i in 0..points {
                let t = span * i as i64 / points as i64;
                while k < n && t >= t0 + (p.pulse[k] + p.gap[k]) as i64 {
                    t0 += (p.pulse[k] + p.gap[k]) as i64;
                    k += 1;
                }
                let on = k < n && t < t0 + p.pulse[k] as i64;
                let x = 2.0 * i as f32 / points as f32 - 1.0;
                if i > 0 && on != last_on {
                    for j in 1..16 {
                        pts.push(Iq::new(x, -0.6 + 1.2 * j as f32 / 16.0));
                    }
                }
                last_on = on;
                pts.push(Iq::new(x, if on { 0.6 } else { -0.6 }));
            }
            out.scopes.push(("Pulses", pts));
        }
        // "Bursts": frequency across (the channel's width), age down.
        while self
            .bursts
            .front()
            .is_some_and(|b| self.now_s - b.0 > BURSTS_FOR_S)
        {
            self.bursts.pop_front();
        }
        let half = BANDWIDTH.min(self.rate) / 2.0;
        let pts: Vec<Iq> = self
            .bursts
            .iter()
            .map(|&(t, f, _)| {
                Iq::new(
                    (f / half) as f32,
                    (1.0 - 2.0 * (self.now_s - t) / BURSTS_FOR_S) as f32,
                )
            })
            .collect();
        out.scopes.push(("Bursts", pts));
        let bin = self.rx.bin_hz() as f32;
        for row in self.rx.take_map_rows() {
            out.waterfalls.push((
                "Bursts",
                WaterfallRow {
                    low_hz: -(row.db.len() as f32) / 2.0 * bin,
                    span_hz: row.db.len() as f32 * bin,
                    // dB over the floor: 0 is the floor.
                    db: row.db.iter().map(|d| d - 60.0).collect(),
                    marks: Vec::new(),
                },
            ));
        }

        let groups: Vec<&str> = Group::ALL
            .iter()
            .zip(self.settings.groups)
            .filter(|(_, on)| *on)
            .map(|(g, _)| match g {
                Group::Weather => "weather",
                Group::Remote => "remotes",
                Group::Tpms => "TPMS",
                Group::Meter => "meters",
                Group::Flex => "flex",
            })
            .collect();
        out.measure("Decoded", st.decoded as f64, "");
        out.measure("Devices", self.heard.len() as f64, "");
        out.measure("Bursts", st.bursts as f64, "");
        let pair = |k: &str, v: String| (k.to_string(), v);
        let (headline, gauge, mut details) = match &self.latest {
            Some((_, d)) => {
                let m = &d.message;
                let mut details = Vec::new();
                let id = m.id_text();
                if !id.is_empty() {
                    details.push(pair("id", id));
                }
                let ch = m.channel_text();
                if !ch.is_empty() {
                    details.push(pair("channel", ch));
                }
                for (k, v) in &m.fields {
                    if !matches!(*k, "id" | "channel") {
                        details.push(shown(k, v));
                    }
                }
                details.push(pair(
                    "frequency",
                    format!("{:.3} MHz", self.rf(d.freq_hz) / 1e6),
                ));
                details.push(pair("modulation", if d.fsk { "FSK" } else { "OOK" }.into()));
                let gauge = Gauge {
                    label: "SNR".into(),
                    value_db: d.snr_db,
                    need_db: Some(10.0),
                    need_label: "to decode".into(),
                    range: (0.0, 40.0),
                };
                (m.model.clone(), Some(gauge), details)
            }
            None => ("listening".to_string(), None, Vec::new()),
        };
        details.push(pair("bursts", st.bursts.to_string()));
        details.push(pair(
            "decoded",
            format!("{} (+{} repeats)", st.decoded, st.repeats),
        ));
        details.push(pair("devices", self.heard.len().to_string()));
        details.push(pair("unknown", st.unknown.to_string()));
        out.card = Some(Card {
            title: if groups.len() == Group::ALL.len() {
                "ISM · all devices".to_string()
            } else {
                format!("ISM · {}", groups.join(", "))
            },
            headline,
            gauge,
            details,
            text: self.recent.iter().cloned().collect(),
        });

        // When it changes, and each second for the "live" marks.
        self.reports += 1;
        if self.heard_dirty || self.reports.is_multiple_of(10) {
            self.heard_dirty = false;
            let mut devices: Vec<HeardDevice> = self
                .heard
                .values()
                .map(|h| {
                    let mut d = h.dev.clone();
                    d.live = self.now_s - d.last_s < LIVE_S;
                    d
                })
                .collect();
            devices.sort_by(|a, b| b.last_s.total_cmp(&a.last_s));
            out.devices = Some(Devices {
                now_s: self.now_s,
                since_s: self.since_s,
                devices,
            });
        }
        if self.news_dirty {
            self.news_dirty = false;
            out.custom.push(Box::new(PanelNews {
                devices: self.rx.device_count(),
                flex_errors: self.rx.flex_errors.clone(),
                unknown: self.unknown.iter().cloned().collect(),
            }));
        }
    }

    fn clear_heard(&mut self) {
        self.heard.clear();
        self.recent.clear();
        self.latest = None;
        self.bursts.clear();
        self.unknown.clear();
        self.last_msg_s = None;
        self.heard_dirty = true;
        self.news_dirty = true;
        self.since_s = self.now_s;
    }
}

impl Decoder for Ism {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        if (block.tuning.sample_rate - self.rate).abs() > 1.0 {
            self.rate = block.tuning.sample_rate;
            let mut rx = Receiver::new(self.rate, &self.settings.config());
            rx.set_map(196);
            self.rx = rx;
        }
        self.centre_hz = block.tuning.center_hz;
        self.rx.process(block.iq);
        self.now_s = (block.first_sample + block.iq.len() as u64) as f64 / block.tuning.sample_rate;
        self.take_events(out);
        self.since_report += block.iq.len();
        if self.since_report as f64 >= block.tuning.sample_rate / 10.0 {
            self.since_report = 0;
            self.report(out);
        }
    }

    fn command(&mut self, cmd: Command, out: &mut Sink) {
        if let Command::Set { key, value } = cmd {
            let mut s = self.settings.clone();
            match s.set(&key, &value) {
                Ok(()) if s != self.settings => {
                    self.settings = s;
                    self.rx.configure(&self.settings.config());
                    for e in &self.rx.flex_errors {
                        out.log(e.clone());
                    }
                    self.news_dirty = true;
                }
                Ok(()) => {}
                Err(e) => out.log(e),
            }
        }
    }

    fn reset(&mut self) {
        self.rx.reset();
        self.clear_heard();
    }
}

/// The settings panel: device groups, rtl_433's off-by-default decoders,
/// flex specs, and the packages nothing decoded (with a flex spec to start
/// from). The GUI shows it in the Settings window and in the right-hand
/// box's "Flex" tab.
#[cfg(feature = "ui")]
pub mod panel {
    use declab_api::{Command, Panel};
    use declab_ism::Group;
    use egui::{Color32, RichText, Ui};

    use super::{FLEX_SLOTS, PanelNews, Settings};

    #[derive(Default)]
    pub struct IsmPanel {
        /// What has been sent, so the controls show it.
        pub s: Settings,
        /// The specs as being edited (sent on Apply).
        edit: Option<[String; FLEX_SLOTS]>,
        news: PanelNews,
    }

    fn onoff(b: bool) -> String {
        if b { "on" } else { "off" }.into()
    }

    impl Panel for IsmPanel {
        fn receive(&mut self, msg: &(dyn std::any::Any + Send)) {
            if let Some(n) = msg.downcast_ref::<PanelNews>() {
                self.news = n.clone();
            }
        }

        fn settings(&self) -> Vec<Command> {
            self.s.commands()
        }

        fn restore(&mut self, key: &str, value: &str) {
            let _ = self.s.set(key, value);
        }

        fn ui(&mut self, ui: &mut Ui, commands: &mut Vec<Command>) {
            let grey = Color32::from_gray(154);
            ui.horizontal_wrapped(|ui| {
                for (i, g) in Group::ALL.iter().enumerate() {
                    if ui.checkbox(&mut self.s.groups[i], g.label()).changed() {
                        commands.push(Command::Set {
                            key: g.key().into(),
                            value: onoff(self.s.groups[i]),
                        });
                    }
                }
            });
            ui.horizontal_wrapped(|ui| {
                if ui
                    .checkbox(&mut self.s.all, "rtl_433's off-by-default decoders")
                    .on_hover_text("Decoders rtl_433 leaves off because they give false positives")
                    .changed()
                {
                    commands.push(Command::Set {
                        key: "all".into(),
                        value: onoff(self.s.all),
                    });
                }
                if ui
                    .checkbox(&mut self.s.unknown, "unknown bursts to the log")
                    .changed()
                {
                    commands.push(Command::Set {
                        key: "unknown".into(),
                        value: onoff(self.s.unknown),
                    });
                }
            });
            if self.news.devices > 0 {
                ui.label(
                    RichText::new(format!("{} decoders on", self.news.devices))
                        .size(11.0)
                        .color(grey),
                );
            }
            ui.add_space(6.0);
            ui.label(RichText::new("Flex decoders").strong());
            ui.label(
                RichText::new(
                    "rtl_433's -X syntax, e.g. n=doorbell,m=OOK_PWM,s=400,l=1200,r=8000,bits>=24",
                )
                .size(11.0)
                .color(grey),
            );
            let edit = self.edit.get_or_insert_with(|| self.s.flex.clone());
            // To the box's visible edge (in a scroll area the available
            // width can run past it).
            let w = (ui
                .available_width()
                .min(ui.clip_rect().right() - ui.cursor().left())
                - 12.0)
                .max(120.0);
            for (i, spec) in edit.iter_mut().enumerate() {
                ui.add(
                    egui::TextEdit::singleline(spec)
                        .id_salt(("ism_flex", i))
                        .desired_width(w)
                        .font(egui::TextStyle::Monospace)
                        .hint_text(format!("flex {}", i + 1)),
                );
            }
            let changed = *edit != self.s.flex;
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(changed, egui::Button::new("Apply"))
                    .clicked()
                {
                    // Rust note: `zip` walks both arrays together, so no
                    // index is needed.
                    for (i, (e, f)) in edit.iter().zip(self.s.flex.iter_mut()).enumerate() {
                        if e != f {
                            *f = e.trim().to_string();
                            commands.push(Command::Set {
                                key: format!("flex{}", i + 1),
                                value: f.clone(),
                            });
                        }
                    }
                }
                if changed && ui.button("Revert").clicked() {
                    *edit = self.s.flex.clone();
                }
            });
            for e in &self.news.flex_errors {
                ui.label(
                    RichText::new(e)
                        .size(11.0)
                        .color(Color32::from_rgb(240, 120, 110)),
                );
            }
            if !self.news.unknown.is_empty() {
                ui.add_space(6.0);
                ui.label(RichText::new("Not decoded").strong());
                for u in &self.news.unknown {
                    ui.label(
                        RichText::new(format!(
                            "{:.1} s  {:+.0} kHz  {}",
                            u.at_s,
                            u.freq_hz / 1e3,
                            u.summary
                        ))
                        .size(11.0),
                    );
                    if !u.flex_hint.is_empty() {
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(&u.flex_hint)
                                    .monospace()
                                    .size(11.0)
                                    .color(grey),
                            );
                            if ui
                                .small_button("use")
                                .on_hover_text("Put this spec in the first free flex slot")
                                .clicked()
                                && let Some(slot) = edit.iter_mut().find(|s| s.is_empty())
                            {
                                *slot = u.flex_hint.clone();
                            }
                        });
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_ism::sim::{self, Noise, Rf};
    use num_complex::Complex32;

    #[test]
    fn decodes_a_simulated_sensor_and_lists_it() {
        let (dev, s) = declab_ism::devices::samples().next().unwrap();
        let lv = sim::levels(dev, s);
        let mut iq = vec![Complex32::default(); (0.8 * RATE) as usize];
        let mut noise = Noise::new(3);
        for x in iq.iter_mut() {
            *x = noise.gauss(1e-4);
        }
        let rf = Rf {
            offset_hz: 250e3,
            amplitude: 0.02,
            deviation_hz: 30e3,
        };
        sim::add_burst(
            &mut iq,
            RATE,
            (0.05 * RATE) as usize,
            &lv,
            dev.modulation.is_fsk(),
            rf,
        );
        let mut d = Ism::new(Settings::default());
        let mut sink = Sink::default();
        let mut logs = Vec::new();
        let mut devices = None;
        for (i, chunk) in iq.chunks(65_536).enumerate() {
            let block = Block {
                iq: chunk,
                tuning: Tuning {
                    center_hz: 433.92e6,
                    sample_rate: RATE,
                    generation: 0,
                },
                first_sample: (i * 65_536) as u64,
                dropped_before: 0,
                live: false,
            };
            d.process(&block, &mut sink);
            logs.append(&mut sink.log);
            if let Some(dv) = sink.devices.take() {
                devices = Some(dv);
            }
            sink.clear();
        }
        assert!(
            logs.iter()
                .any(|l| l.contains(s.model) && l.contains("434.170 MHz")),
            "{logs:?}"
        );
        let dv = devices.unwrap();
        assert_eq!(dv.devices.len(), 1);
        assert_eq!(dv.devices[0].model, s.model);
    }

    #[test]
    fn settings_round_trip() {
        let mut s = Settings::default();
        s.set("tpms", "off").unwrap();
        s.set("flex2", "n=bell,m=OOK_PWM,s=400,l=1200,r=8000")
            .unwrap();
        let mut t = Settings::default();
        for c in s.commands() {
            if let Command::Set { key, value } = c {
                t.set(&key, &value).unwrap();
            }
        }
        assert_eq!(s, t);
        assert!(s.set("nonsense", "on").is_err());
    }
}
