// made by CasualArclamp
//! LoRa: Meshtastic and MeshCore. The receiver (CSS demodulation, every
//! bandwidth and spreading factor at once, so the presets are found by
//! themselves) and the two meshes' packets are `crates/declab-lora`; this
//! file wraps them.
//!
//! The channel is 1 MS/s about the VFO: a LoRa signal is found within
//! about a quarter of its bandwidth of the VFO's frequency, so put the VFO
//! on the mesh's channel (869.525 MHz for Meshtastic's EU_868 LongFast and
//! MeshCore's 250 kHz preset; a second VFO for another).
//!
//! Card: the latest packet's preset, its SNR against what its SF needs,
//! offset, drift, sender. Scope "Dechirp": the latest frame's symbols (bin
//! up, time across), with each symbol's dechirped spectrum as a small
//! waterfall under it. The panel is the right-hand box's tabs (the agreed
//! mockup): Nodes, Messages, Packets, Keys. It gets its rows through
//! [`Sink::custom`] as [`News`].

use std::collections::HashMap;

use declab_api::{
    Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    Sink, WaterfallRow,
};
use declab_lora::meshtastic::{self, node_name};
use declab_lora::receiver::{BANDWIDTHS, Config, INPUT_RATE, ROW_BINS, RxFrame};
use declab_lora::{Keys, Payload, Receiver, decode_payload, meshcore};

/// The host filters to this much of the 1 MS/s channel. 400 kHz makes the
/// host's cut from a 10 MS/s radio a plain ÷10 (it decimates to at least
/// 2.5 × the bandwidth): at 900 kHz it went ÷4 and then a 2/5 resampler,
/// which cost the worker ~30 % of a core against ~4 %. The 500 kHz
/// presets (not allowed in the EU's 869.4–869.65 MHz sub-band) lose their
/// outer 50 kHz each side, which costs them sensitivity, not decoding.
const BANDWIDTH: f64 = 400e3;
/// The card says "locked" while packets came this recently.
const LOCKED_FOR_S: f64 = 120.0;
/// Rows of a frame's dechirped spectra sent to the scope's waterfall.
const SCOPE_ROWS: usize = 96;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "lora",
            name: "LoRa mesh",
            family: Family::Other,
            standard: "LoRa CSS; Meshtastic, MeshCore",
            summary: "LoRa (SF7–12, 62.5–500 kHz, found by itself): Meshtastic and MeshCore packets, default keys built in",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        InputSpec {
            sample_rate: Some(INPUT_RATE),
            bandwidth_hz: BANDWIDTH,
            block_len: 65_536,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Lora::new(Settings::default()))
    }

    fn default_scope(&self) -> &'static str {
        "Dechirp"
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(panel::LoraPanel::default()))
    }
}

/// What the panel can change, as [`Command::Set`] keys and values:
/// `bw` ("500,250,125,62.5", the bandwidths watched), `sf` ("7,8,…,12"),
/// `mt_channels` (Meshtastic channels besides the presets' defaults:
/// "Name=PSK;…", PSK in base64 as the apps show it), `mc_channels`
/// (MeshCore channels besides Public: "#hashtag;Name=PSK;…"), `bad_crc`
/// (frames that failed their CRC to the log: on/off).
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    pub bw: [bool; 4],
    pub sf: [bool; 6],
    pub mt_channels: Vec<(String, String)>,
    pub mc_channels: Vec<(String, String)>,
    pub bad_crc: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            bw: [true; 4],
            sf: [true; 6],
            mt_channels: Vec::new(),
            mc_channels: Vec::new(),
            bad_crc: true,
        }
    }
}

fn bw_label(bw: f64) -> String {
    if bw == 62.5e3 {
        "62.5".into()
    } else {
        format!("{:.0}", bw / 1e3)
    }
}

/// "Name=PSK;#tag" → pairs (a bare name has an empty PSK).
fn parse_channels(v: &str) -> Vec<(String, String)> {
    v.split(';')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(|s| match s.split_once('=') {
            Some((n, k)) => (n.trim().to_string(), k.trim().to_string()),
            None => (s.to_string(), String::new()),
        })
        .collect()
}

fn join_channels(c: &[(String, String)]) -> String {
    c.iter()
        .map(|(n, k)| {
            if k.is_empty() {
                n.clone()
            } else {
                format!("{n}={k}")
            }
        })
        .collect::<Vec<_>>()
        .join(";")
}

impl Settings {
    pub fn set(&mut self, key: &str, value: &str) -> Result<(), String> {
        match key {
            "bw" => {
                let v: Vec<&str> = value.split(',').map(str::trim).collect();
                self.bw = BANDWIDTHS.map(|b| value == "all" || v.contains(&bw_label(b).as_str()));
            }
            "sf" => {
                let v: Vec<&str> = value.split(',').map(str::trim).collect();
                self.sf = core::array::from_fn(|i| {
                    value == "all" || v.contains(&(i + 7).to_string().as_str())
                });
            }
            "mt_channels" => self.mt_channels = parse_channels(value),
            "mc_channels" => self.mc_channels = parse_channels(value),
            "bad_crc" => self.bad_crc = value == "on",
            // The panel's tab (`--set tab=packets`): not the decoder's.
            "tab" => {}
            k => return Err(format!("lora: no setting {k:?}")),
        }
        Ok(())
    }

    pub fn commands(&self) -> Vec<Command> {
        let set = |k: &str, v: String| Command::Set {
            key: k.into(),
            value: v,
        };
        let bws: Vec<String> = BANDWIDTHS
            .iter()
            .zip(self.bw)
            .filter(|(_, on)| *on)
            .map(|(b, _)| bw_label(*b))
            .collect();
        let sfs: Vec<String> = (7..=12)
            .zip(self.sf)
            .filter(|(_, on)| *on)
            .map(|(s, _)| s.to_string())
            .collect();
        vec![
            set("bw", bws.join(",")),
            set("sf", sfs.join(",")),
            set("mt_channels", join_channels(&self.mt_channels)),
            set("mc_channels", join_channels(&self.mc_channels)),
            set("bad_crc", if self.bad_crc { "on" } else { "off" }.into()),
        ]
    }

    fn config(&self) -> Config {
        Config {
            bw: self.bw,
            sf: self.sf,
        }
    }

    /// The keys, and what was wrong with any that did not parse.
    fn keys(&self) -> (Keys, Vec<String>) {
        let mut k = Keys::default();
        let mut errors = Vec::new();
        for (n, psk) in &self.mt_channels {
            match meshtastic::Channel::new(n, psk) {
                Ok(c) => k.meshtastic.insert(0, c),
                Err(e) => errors.push(format!("Meshtastic {n}: {e}")),
            }
        }
        for (n, psk) in &self.mc_channels {
            if psk.is_empty() {
                k.meshcore.push(meshcore::Channel::hashtag(n));
            } else {
                match meshcore::Channel::new(n, psk) {
                    Ok(c) => k.meshcore.push(c),
                    Err(e) => errors.push(format!("MeshCore {n}: {e}")),
                }
            }
        }
        (k, errors)
    }
}

/// Which mesh a row is from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Net {
    #[default]
    Lora,
    Meshtastic,
    MeshCore,
}

impl Net {
    pub fn short(self) -> &'static str {
        match self {
            Net::Meshtastic => "MT",
            Net::MeshCore => "MC",
            Net::Lora => "—",
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Net::Meshtastic => "Meshtastic",
            Net::MeshCore => "MeshCore",
            Net::Lora => "LoRa",
        }
    }
}

/// A node heard: a Meshtastic node by its number, a MeshCore node by its
/// advert's key.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NodeRow {
    pub net: Net,
    pub id: String,
    pub name: String,
    pub short: String,
    /// "TBEAM / ROUTER", "repeater".
    pub kind: String,
    pub snr_db: f32,
    /// Hops it came over ("0", "2"; MeshCore: "flood 2", "direct").
    pub hops: String,
    pub count: u32,
    pub first_s: f64,
    pub last_s: f64,
    /// Its latest news, short ("text", "batt 87 % 4.02 V", "45.48, 7.25").
    pub latest: String,
    /// Everything it has said about itself, latest of each.
    pub fields: Vec<(String, String)>,
}

/// A text message.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MsgRow {
    pub at_s: f64,
    pub net: Net,
    pub channel: String,
    pub from: String,
    pub to: String,
    pub text: String,
    /// Not readable (no key): `text` says what it is.
    pub encrypted: bool,
}

/// A frame off the air.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PacketRow {
    pub at_s: f64,
    /// "SF11/250".
    pub preset: String,
    pub net: Net,
    pub from_to: String,
    pub kind: String,
    pub hops: String,
    pub snr_db: f32,
    pub cfo_hz: f64,
    pub crc_ok: bool,
    pub summary: String,
    pub fields: Vec<(String, String)>,
    pub bytes: Vec<u8>,
    /// Meshtastic: the decrypted `Data` message.
    pub plain: Vec<u8>,
}

/// What the decoder tells its panel (through [`Sink::custom`]).
#[derive(Debug, Clone, Default)]
pub struct News {
    /// The whole nodes list, when it changed.
    pub nodes: Option<Vec<NodeRow>>,
    /// New since the last news.
    pub messages: Vec<MsgRow>,
    pub packets: Vec<PacketRow>,
    /// Forget what was shown (a retune).
    pub clear: bool,
    pub now_s: f64,
    pub since_s: f64,
    pub key_errors: Vec<String>,
}

/// Semtech's demodulator SNR limit per SF (SX1276 datasheet table 13).
fn snr_needed(sf: u8) -> f32 {
    -2.5 * (sf as f32 - 4.0)
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

fn preset_text(f: &RxFrame) -> String {
    format!("SF{}/{}", f.sf, bw_label(f.bw_hz))
}

struct Lora {
    settings: Settings,
    rx: Receiver,
    keys: Keys,
    key_errors: Vec<String>,
    now_s: f64,
    since_s: f64,
    last_packet_s: Option<f64>,
    nodes: HashMap<(Net, String), NodeRow>,
    nodes_dirty: bool,
    news: News,
    latest: Option<(RxFrame, PacketRow, Option<String>)>,
    last_text: Option<String>,
    packets: u64,
    crc_fail: u64,
    since_report: usize,
}

impl Lora {
    fn new(settings: Settings) -> Self {
        let (keys, key_errors) = settings.keys();
        Lora {
            rx: Receiver::new(INPUT_RATE, settings.config()),
            keys,
            key_errors,
            settings,
            now_s: 0.0,
            since_s: 0.0,
            last_packet_s: None,
            nodes: HashMap::new(),
            nodes_dirty: true,
            news: News::default(),
            latest: None,
            last_text: None,
            packets: 0,
            crc_fail: 0,
            since_report: 0,
        }
    }

    fn node(&mut self, net: Net, id: &str, at_s: f64) -> &mut NodeRow {
        self.nodes_dirty = true;
        self.nodes
            .entry((net, id.to_string()))
            .or_insert_with(|| NodeRow {
                net,
                id: id.to_string(),
                first_s: at_s,
                ..Default::default()
            })
    }

    fn take_frames(&mut self, out: &mut Sink) {
        for f in std::mem::take(&mut self.rx.frames) {
            let Some(fr) = &f.frame else { continue };
            let mut row = PacketRow {
                at_s: f.at_s,
                preset: preset_text(&f),
                snr_db: f.snr_db,
                cfo_hz: f.cfo_hz,
                crc_ok: fr.crc_ok,
                bytes: fr.payload.clone(),
                ..Default::default()
            };
            row.fields.push((
                "preset".into(),
                format!(
                    "SF{} {} kHz 4/{}",
                    f.sf,
                    bw_label(f.bw_hz),
                    fr.header.cr + 4
                ),
            ));
            row.fields.push((
                "sync word".into(),
                f.sync.map_or("?".into(), |s| format!("0x{s:02X}")),
            ));
            row.fields
                .push(("SNR".into(), format!("{:.1} dB", f.snr_db)));
            row.fields
                .push(("offset".into(), format!("{:+.0} Hz", f.cfo_hz)));
            row.fields
                .push(("drift".into(), format!("{:+.1} ppm", f.drift_ppm)));
            if !fr.crc_ok {
                self.crc_fail += 1;
                row.kind = "CRC failed".into();
                row.summary = format!("{} bytes, payload CRC failed", fr.payload.len());
                if self.settings.bad_crc {
                    out.log(format!(
                        "{} {}  {:.1} dB  CRC failed ({} bytes)",
                        clock(f.at_s),
                        row.preset,
                        f.snr_db,
                        fr.payload.len()
                    ));
                }
                self.news.packets.push(row);
                continue;
            }
            self.packets += 1;
            self.last_packet_s = Some(f.at_s);
            let mut text = None;
            match decode_payload(f.sync, &fr.payload, &self.keys) {
                Payload::Meshtastic(p) => {
                    row.net = Net::Meshtastic;
                    let h = p.header;
                    let from = node_name(h.from);
                    row.from_to = format!("{from} → {}", node_name(h.to));
                    row.kind = p.kind.clone();
                    let hops = h.hop_start().saturating_sub(h.hop_limit());
                    row.hops = format!("{}/{}", h.hop_limit(), h.hop_start());
                    row.summary = p.summary.clone();
                    row.fields.push(("from".into(), from.clone()));
                    row.fields.push(("to".into(), node_name(h.to)));
                    row.fields
                        .push(("packet id".into(), format!("{:08x}", h.id)));
                    row.fields.push((
                        "hop limit / start".into(),
                        format!("{} / {}", h.hop_limit(), h.hop_start()),
                    ));
                    if h.want_ack() {
                        row.fields.push(("want ack".into(), "yes".into()));
                    }
                    if h.via_mqtt() {
                        row.fields.push(("via MQTT".into(), "yes".into()));
                    }
                    if h.relay != 0 {
                        row.fields
                            .push(("relayed by".into(), format!("..{:02x}", h.relay)));
                    }
                    row.fields.push((
                        "channel".into(),
                        p.channel
                            .clone()
                            .unwrap_or_else(|| format!("hash 0x{:02x} (no key)", h.channel)),
                    ));
                    row.fields.extend(p.fields.iter().cloned());
                    row.plain = p.plain.clone();
                    // The node.
                    let snr = f.snr_db;
                    let at = f.at_s;
                    let n = self.node(Net::Meshtastic, &from, at);
                    n.count += 1;
                    n.last_s = at;
                    n.snr_db = snr;
                    n.hops = hops.to_string();
                    n.latest = if p.kind == "TEXT_MESSAGE" {
                        "text".into()
                    } else {
                        p.summary.clone()
                    };
                    let nn = &p.node;
                    if let Some(v) = &nn.long_name {
                        n.name = v.clone();
                    }
                    if let Some(v) = &nn.short_name {
                        n.short = v.clone();
                    }
                    if nn.hw_model.is_some() || nn.role.is_some() {
                        n.kind = format!(
                            "{} / {}",
                            nn.hw_model.clone().unwrap_or_default(),
                            nn.role.clone().unwrap_or_default()
                        );
                    }
                    if p.port.is_some() {
                        for (k, v) in &p.fields {
                            if k == "port" || k == "text" {
                                continue;
                            }
                            match n.fields.iter_mut().find(|(a, _)| a == k) {
                                Some(e) => e.1 = v.clone(),
                                None => n.fields.push((k.clone(), v.clone())),
                            }
                        }
                    }
                    // Messages: text, and direct messages we cannot read.
                    if let Some(t) = &p.text {
                        text = Some(t.clone());
                        self.news.messages.push(MsgRow {
                            at_s: f.at_s,
                            net: Net::Meshtastic,
                            channel: p.channel.clone().unwrap_or_default(),
                            from: self.nodes.get(&(Net::Meshtastic, from.clone())).map_or(
                                from.clone(),
                                |n| {
                                    if n.name.is_empty() {
                                        from.clone()
                                    } else {
                                        n.name.clone()
                                    }
                                },
                            ),
                            to: node_name(h.to),
                            text: t.clone(),
                            encrypted: false,
                        });
                    } else if p.port.is_none() && h.channel == 0 && h.to != meshtastic::BROADCAST {
                        self.news.messages.push(MsgRow {
                            at_s: f.at_s,
                            net: Net::Meshtastic,
                            channel: "PKI".into(),
                            from: from.clone(),
                            to: node_name(h.to),
                            text: "direct message (encrypted)".into(),
                            encrypted: true,
                        });
                    }
                }
                Payload::MeshCore(p) => {
                    row.net = Net::MeshCore;
                    let route = meshcore::ROUTES[p.route as usize];
                    let ty = meshcore::payload_type_name(p.payload_type);
                    row.kind = ty.into();
                    row.hops = match p.route {
                        0 | 1 => format!("flood {}", p.path.len()),
                        _ => "direct".into(),
                    };
                    row.from_to = route.to_string();
                    row.summary = p.summary.clone();
                    row.fields.extend(p.fields.iter().cloned());
                    if let Some(a) = &p.advert {
                        let id = meshcore::key_id(&a.public_key);
                        row.from_to = format!("{id} ({})", a.name.clone().unwrap_or_default());
                        let snr = f.snr_db;
                        let at = f.at_s;
                        let hops = row.hops.clone();
                        let n = self.node(Net::MeshCore, &id, at);
                        n.count += 1;
                        n.last_s = at;
                        n.snr_db = snr;
                        n.hops = hops;
                        n.kind = meshcore::node_type_name(a.node_type).into();
                        if let Some(name) = &a.name {
                            n.name = name.clone();
                        }
                        n.latest = match a.position {
                            Some((la, lo)) => format!("advert {la:.4}, {lo:.4}"),
                            None => "advert".into(),
                        };
                        n.fields = p.fields.clone();
                    }
                    match &p.group_text {
                        Some((ch, t)) => {
                            let (from, body) = t
                                .split_once(": ")
                                .map_or((String::new(), t.clone()), |(a, b)| (a.into(), b.into()));
                            text = Some(t.clone());
                            self.news.messages.push(MsgRow {
                                at_s: f.at_s,
                                net: Net::MeshCore,
                                channel: ch.clone(),
                                from,
                                to: "all".into(),
                                text: body,
                                encrypted: false,
                            });
                        }
                        None if p.payload_type == 2 => {
                            let (d, s) = (
                                p.payload.first().copied().unwrap_or(0),
                                p.payload.get(1).copied().unwrap_or(0),
                            );
                            self.news.messages.push(MsgRow {
                                at_s: f.at_s,
                                net: Net::MeshCore,
                                channel: "direct".into(),
                                from: format!("{s:02x}"),
                                to: format!("{d:02x}"),
                                text: "text message (encrypted)".into(),
                                encrypted: true,
                            });
                        }
                        None => {}
                    }
                }
                Payload::Other => {
                    row.kind = f.sync.map_or("LoRa".into(), |s| format!("sync 0x{s:02X}"));
                    row.summary = format!("{} bytes", fr.payload.len());
                }
            }
            out.log(format!(
                "{} {} {:>4.1} dB {:+6.0} Hz  {} {} {}  {}",
                clock(f.at_s),
                row.preset,
                f.snr_db,
                f.cfo_hz,
                row.net.name(),
                row.from_to,
                row.kind,
                row.summary
            ));
            if text.is_some() {
                self.last_text = text.clone();
            }
            self.news.packets.push(row.clone());
            self.latest = Some((f, row, text));
        }
    }

    fn report(&mut self, out: &mut Sink) {
        let st = self.rx.stats();
        out.lock = match self.last_packet_s {
            Some(t) if self.now_s - t < LOCKED_FOR_S => Lock::Locked,
            _ if st.synced > 0 => Lock::Synced,
            _ => Lock::Searching,
        };
        out.measure("Packets", self.packets as f64, "");
        out.measure("Nodes", self.nodes.len() as f64, "");
        out.measure("CRC failed", self.crc_fail as f64, "");
        let pair = |a: &str, b: String| (a.to_string(), b);
        let watching = {
            let bws: Vec<String> = BANDWIDTHS
                .iter()
                .zip(self.settings.bw)
                .filter(|(_, on)| *on)
                .map(|(b, _)| bw_label(*b))
                .collect();
            let n_sf = self.settings.sf.iter().filter(|x| **x).count();
            format!("{n_sf} SFs × {} kHz", bws.join("/"))
        };
        let card = match &self.latest {
            Some((f, row, _)) => {
                let fr = f.frame.as_ref().expect("decoded");
                let headline = match (row.net, meshtastic::preset_name(f.bw_hz, f.sf)) {
                    (Net::Meshtastic, Some(name)) => name.to_string(),
                    _ => format!("SF{} · {} kHz", f.sf, bw_label(f.bw_hz)),
                };
                let mut details = vec![
                    pair(
                        "mode",
                        format!("SF{} BW{} 4/{}", f.sf, bw_label(f.bw_hz), fr.header.cr + 4),
                    ),
                    pair("sync", f.sync.map_or("?".into(), |s| format!("0x{s:02X}"))),
                    pair("offset", format!("{:+.2} kHz", f.cfo_hz / 1e3)),
                    pair("drift", format!("{:+.1} ppm", f.drift_ppm)),
                ];
                if !row.from_to.is_empty() {
                    details.push(pair("from → to", row.from_to.clone()));
                }
                details.push(pair("type", row.kind.clone()));
                if !row.hops.is_empty() {
                    details.push(pair("hops", row.hops.clone()));
                }
                details.push(pair(
                    "ok / CRC",
                    format!("{} / {}", self.packets, self.crc_fail),
                ));
                details.push(pair("nodes", self.nodes.len().to_string()));
                details.push(pair("heard", clock(self.now_s - f.at_s) + " ago"));
                let mut text = vec![pair("latest", row.summary.clone())];
                if let Some(t) = &self.last_text {
                    text.push(pair("text", t.clone()));
                }
                Card {
                    title: format!("LoRa · {}", row.net.name()),
                    headline,
                    gauge: Some(Gauge {
                        label: "SNR".into(),
                        value_db: f.snr_db,
                        need_db: Some(snr_needed(f.sf)),
                        need_label: format!("SF{} needs", f.sf),
                        range: (-25.0, 20.0),
                    }),
                    details,
                    text,
                }
            }
            None => Card {
                title: "LoRa".into(),
                headline: "listening".into(),
                gauge: None,
                details: vec![
                    pair("watching", watching),
                    pair("preambles", st.detections.to_string()),
                    pair("synced", st.synced.to_string()),
                    pair("header failed", st.header_fail.to_string()),
                ],
                text: Vec::new(),
            },
        };
        out.card = Some(card);
        self.news.now_s = self.now_s;
        self.news.since_s = self.since_s;
        if self.nodes_dirty {
            let mut v: Vec<NodeRow> = self.nodes.values().cloned().collect();
            v.sort_by(|a, b| b.last_s.total_cmp(&a.last_s));
            self.news.nodes = Some(v);
            self.nodes_dirty = false;
        }
        self.news.key_errors = self.key_errors.clone();
        out.custom.push(Box::new(std::mem::take(&mut self.news)));
    }

    /// The scope: the latest frame's symbols (time across, bin up) and its
    /// dechirped spectra as rows.
    fn scope(f: &RxFrame, out: &mut Sink) {
        let n = (1u32 << f.sf) as f32;
        let len = f.symbols.len().max(1) as f32;
        let pts: Vec<Iq> = f
            .symbols
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                Iq::new(
                    2.0 * (i as f32 + 0.5) / len - 1.0,
                    2.0 * (s as f32 + 0.5) / n - 1.0,
                )
            })
            .collect();
        out.scopes.push(("Dechirp", pts));
        let step = f.rows.len().div_ceil(SCOPE_ROWS).max(1);
        for row in f.rows.iter().step_by(step) {
            // dB over the row's median.
            let mut s = row.clone();
            s.sort_by(f32::total_cmp);
            let med = s[s.len() / 2];
            out.waterfalls.push((
                "Dechirp",
                WaterfallRow {
                    low_hz: 0.0,
                    span_hz: ROW_BINS as f32,
                    db: row.iter().map(|d| d - med - 40.0).collect(),
                    marks: Vec::new(),
                },
            ));
        }
    }
}

impl Decoder for Lora {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        if block.dropped_before > 0 {
            self.rx.reset();
        }
        self.rx.centre_hz = block.tuning.center_hz;
        self.rx.process(block.iq);
        self.now_s = (block.first_sample + block.iq.len() as u64) as f64 / block.tuning.sample_rate;
        let before = self.latest.as_ref().map(|l| l.0.at_s);
        self.take_frames(out);
        if let Some((f, _, _)) = &self.latest
            && before != Some(f.at_s)
        {
            Self::scope(f, out);
        }
        self.since_report += block.iq.len();
        if self.since_report as f64 >= block.tuning.sample_rate / 4.0 {
            self.since_report = 0;
            self.report(out);
        }
    }

    fn command(&mut self, cmd: Command, out: &mut Sink) {
        if let Command::Set { key, value } = cmd {
            let mut s = self.settings.clone();
            match s.set(&key, &value) {
                Ok(()) if s != self.settings => {
                    if s.config() != self.settings.config() {
                        let centre = self.rx.centre_hz;
                        self.rx = Receiver::new(INPUT_RATE, s.config());
                        self.rx.centre_hz = centre;
                    }
                    (self.keys, self.key_errors) = s.keys();
                    for e in &self.key_errors {
                        out.log(e.clone());
                    }
                    self.settings = s;
                }
                Ok(()) => {}
                Err(e) => out.log(e),
            }
        }
    }

    fn reset(&mut self) {
        self.rx.reset();
        self.nodes.clear();
        self.nodes_dirty = true;
        self.latest = None;
        self.last_text = None;
        self.since_s = self.now_s;
        self.news = News {
            clear: true,
            ..Default::default()
        };
    }
}

#[cfg(feature = "ui")]
pub mod panel {
    //! The right-hand box's tabs: Nodes (sortable; click one for all it has
    //! said), Messages, Packets (click one for its fields and bytes) and
    //! Keys (what is watched, and channel keys).

    use std::collections::VecDeque;

    use declab_api::{Command, Panel};
    use egui::{Color32, RichText, Ui};

    use super::{BANDWIDTHS, MsgRow, Net, News, NodeRow, PacketRow, Settings, bw_label, clock};

    const GREY: Color32 = Color32::from_gray(154);
    const GREEN: Color32 = Color32::from_rgb(110, 210, 120);
    const AMBER: Color32 = Color32::from_rgb(240, 176, 64);
    const BLUE: Color32 = Color32::from_rgb(122, 184, 255);
    const KEEP_MESSAGES: usize = 500;
    const KEEP_PACKETS: usize = 1000;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub enum Tab {
        #[default]
        Nodes,
        Messages,
        Packets,
        Keys,
    }

    #[derive(Default)]
    pub struct LoraPanel {
        pub tab: Tab,
        pub s: Settings,
        nodes: Vec<NodeRow>,
        messages: VecDeque<MsgRow>,
        packets: VecDeque<PacketRow>,
        now_s: f64,
        since_s: f64,
        key_errors: Vec<String>,
        picked_node: Option<(Net, String)>,
        /// The packet picked, by its time.
        picked_packet: Option<f64>,
        /// Sort column of the nodes table, and reversed (None: last heard).
        sort: Option<(usize, bool)>,
        /// New channel being typed: (Meshtastic?, name, key).
        new_mt: (String, String),
        new_mc: (String, String),
    }

    fn net_text(n: Net) -> RichText {
        RichText::new(n.short()).color(match n {
            Net::Meshtastic => BLUE,
            Net::MeshCore => AMBER,
            Net::Lora => GREY,
        })
    }

    impl Panel for LoraPanel {
        fn receive(&mut self, msg: &(dyn std::any::Any + Send)) {
            let Some(n) = msg.downcast_ref::<News>() else {
                return;
            };
            if n.clear {
                self.nodes.clear();
                self.messages.clear();
                self.packets.clear();
            }
            if let Some(v) = &n.nodes {
                self.nodes = v.clone();
            }
            self.messages.extend(n.messages.iter().cloned());
            while self.messages.len() > KEEP_MESSAGES {
                self.messages.pop_front();
            }
            self.packets.extend(n.packets.iter().cloned());
            while self.packets.len() > KEEP_PACKETS {
                self.packets.pop_front();
            }
            self.now_s = n.now_s;
            self.since_s = n.since_s;
            self.key_errors = n.key_errors.clone();
        }

        fn settings(&self) -> Vec<Command> {
            self.s.commands()
        }

        fn restore(&mut self, key: &str, value: &str) {
            if key == "tab" {
                self.tab = match value {
                    "messages" => Tab::Messages,
                    "packets" => Tab::Packets,
                    "keys" => Tab::Keys,
                    _ => Tab::Nodes,
                };
                return;
            }
            let _ = self.s.set(key, value);
        }

        fn ui(&mut self, ui: &mut Ui, commands: &mut Vec<Command>) {
            ui.horizontal(|ui| {
                for (t, name) in [
                    (Tab::Nodes, "Nodes"),
                    (Tab::Messages, "Messages"),
                    (Tab::Packets, "Packets"),
                    (Tab::Keys, "Keys"),
                ] {
                    if ui.selectable_label(self.tab == t, name).clicked() {
                        self.tab = t;
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        RichText::new(format!(
                            "{} heard in {}",
                            self.nodes.len(),
                            clock(self.now_s - self.since_s)
                        ))
                        .color(GREY)
                        .size(12.0),
                    );
                });
            });
            ui.add_space(4.0);
            match self.tab {
                Tab::Nodes => self.nodes_tab(ui),
                Tab::Messages => self.messages_tab(ui),
                Tab::Packets => self.packets_tab(ui),
                Tab::Keys => self.keys_tab(ui, commands),
            }
        }
    }

    impl LoraPanel {
        fn nodes_tab(&mut self, ui: &mut Ui) {
            if self.nodes.is_empty() {
                ui.label(RichText::new("listening: no node heard yet").color(GREY));
                return;
            }
            let cols = [
                "net",
                "node",
                "name",
                "SNR",
                "hops",
                "last",
                "latest",
                "hw / role",
            ];
            let mut rows: Vec<&NodeRow> = self.nodes.iter().collect();
            let (col, rev) = self.sort.unwrap_or((5, false));
            rows.sort_by(|a, b| {
                let o = match col {
                    0 => a.net.short().cmp(b.net.short()),
                    1 => a.id.cmp(&b.id),
                    2 => a.name.to_lowercase().cmp(&b.name.to_lowercase()),
                    3 => b.snr_db.total_cmp(&a.snr_db),
                    4 => a.hops.cmp(&b.hops),
                    5 => b.last_s.total_cmp(&a.last_s),
                    6 => a.latest.cmp(&b.latest),
                    _ => a.kind.cmp(&b.kind),
                };
                if rev { o.reverse() } else { o }
            });
            let detail_h = if self.picked_node.is_some() {
                90.0
            } else {
                0.0
            };
            let mut clicked_col = None;
            let mut picked = None;
            egui::ScrollArea::both()
                .id_salt("lora_nodes")
                .max_height((ui.available_height() - detail_h).max(40.0))
                .show(ui, |ui| {
                    egui::Grid::new("lora_nodes_grid")
                        .striped(true)
                        .spacing([10.0, 3.0])
                        .show(ui, |ui| {
                            for (i, c) in cols.iter().enumerate() {
                                let sorted = i == col;
                                let t = if sorted {
                                    format!("{c} {}", if rev { "▲" } else { "▼" })
                                } else {
                                    c.to_string()
                                };
                                if ui
                                    .add(
                                        egui::Label::new(RichText::new(t).color(GREY))
                                            .sense(egui::Sense::click()),
                                    )
                                    .clicked()
                                {
                                    clicked_col = Some(i);
                                }
                            }
                            ui.end_row();
                            for n in rows {
                                let live = self.now_s - n.last_s < 60.0;
                                ui.label(net_text(n.net));
                                let id = ui.add(
                                    egui::Label::new(RichText::new(&n.id).color(if live {
                                        GREEN
                                    } else {
                                        Color32::from_gray(220)
                                    }))
                                    .sense(egui::Sense::click()),
                                );
                                if id.clicked() {
                                    picked = Some((n.net, n.id.clone()));
                                }
                                ui.label(if n.short.is_empty() || n.short == n.name {
                                    n.name.clone()
                                } else {
                                    format!("{} ({})", n.name, n.short)
                                });
                                ui.label(format!("{:.1}", n.snr_db));
                                ui.label(&n.hops);
                                ui.label(if self.now_s - n.last_s < 5.0 {
                                    "now".into()
                                } else {
                                    clock(self.now_s - n.last_s)
                                });
                                ui.label(&n.latest);
                                ui.label(&n.kind);
                                ui.end_row();
                            }
                        });
                });
            if let Some(c) = clicked_col {
                self.sort = Some(if col == c { (c, !rev) } else { (c, false) });
            }
            if picked.is_some() {
                self.picked_node = if self.picked_node == picked {
                    None
                } else {
                    picked
                };
            }
            if let Some(key) = &self.picked_node
                && let Some(n) = self.nodes.iter().find(|n| (n.net, n.id.clone()) == *key)
            {
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt("lora_node_detail")
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(format!(
                                "{} {} {}  first {}  ×{}",
                                n.net.name(),
                                n.id,
                                n.name,
                                clock(n.first_s),
                                n.count
                            ))
                            .strong(),
                        );
                        let line: Vec<String> =
                            n.fields.iter().map(|(k, v)| format!("{k} {v}")).collect();
                        ui.label(RichText::new(line.join(" · ")).size(12.0));
                    });
            }
        }

        fn messages_tab(&mut self, ui: &mut Ui) {
            if self.messages.is_empty() {
                ui.label(RichText::new("no messages yet").color(GREY));
                return;
            }
            egui::ScrollArea::both()
                .id_salt("lora_msgs")
                .show(ui, |ui| {
                    egui::Grid::new("lora_msgs_grid")
                        .striped(true)
                        .spacing([10.0, 3.0])
                        .show(ui, |ui| {
                            for c in ["time", "net", "channel", "from", "to", "text"] {
                                ui.label(RichText::new(c).color(GREY));
                            }
                            ui.end_row();
                            for m in self.messages.iter().rev() {
                                ui.label(clock(m.at_s));
                                ui.label(net_text(m.net));
                                ui.label(&m.channel);
                                ui.label(&m.from);
                                ui.label(&m.to);
                                if m.encrypted {
                                    ui.label(RichText::new(&m.text).color(GREY));
                                } else {
                                    ui.label(&m.text);
                                }
                                ui.end_row();
                            }
                        });
                });
        }

        fn packets_tab(&mut self, ui: &mut Ui) {
            if self.packets.is_empty() {
                ui.label(RichText::new("no packets yet").color(GREY));
                return;
            }
            let detail_h = if self.picked_packet.is_some() {
                120.0
            } else {
                0.0
            };
            let mut picked = None;
            egui::ScrollArea::both()
                .id_salt("lora_packets")
                .max_height((ui.available_height() - detail_h).max(40.0))
                .show(ui, |ui| {
                    egui::Grid::new("lora_packets_grid")
                        .striped(true)
                        .spacing([10.0, 3.0])
                        .show(ui, |ui| {
                            for c in [
                                "time",
                                "net",
                                "type",
                                "from → to",
                                "summary",
                                "SNR",
                                "preset",
                                "hops",
                                "CRC",
                            ] {
                                ui.label(RichText::new(c).color(GREY));
                            }
                            ui.end_row();
                            for p in self.packets.iter().rev() {
                                if ui
                                    .add(
                                        egui::Label::new(clock(p.at_s)).sense(egui::Sense::click()),
                                    )
                                    .clicked()
                                {
                                    picked = Some(p.at_s);
                                }
                                ui.label(net_text(p.net));
                                ui.label(&p.kind);
                                ui.label(&p.from_to);
                                ui.label(&p.summary);
                                ui.label(format!("{:.1}", p.snr_db));
                                ui.label(&p.preset);
                                ui.label(&p.hops);
                                ui.label(if p.crc_ok {
                                    RichText::new("ok").color(GREEN)
                                } else {
                                    RichText::new("bad").color(AMBER)
                                });
                                ui.end_row();
                            }
                        });
                });
            if picked.is_some() {
                self.picked_packet = if self.picked_packet == picked {
                    None
                } else {
                    picked
                };
            }
            if let Some(t) = self.picked_packet
                && let Some(p) = self.packets.iter().find(|p| p.at_s == t)
            {
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt("lora_packet_detail")
                    .show(ui, |ui| {
                        let line: Vec<String> =
                            p.fields.iter().map(|(k, v)| format!("{k} {v}")).collect();
                        ui.label(RichText::new(line.join(" · ")).size(12.0));
                        let hex = |b: &[u8]| {
                            b.iter()
                                .map(|x| format!("{x:02x}"))
                                .collect::<Vec<_>>()
                                .join(" ")
                        };
                        ui.label(
                            RichText::new(format!("bytes  {}", hex(&p.bytes)))
                                .monospace()
                                .size(11.0)
                                .color(GREY),
                        );
                        if !p.plain.is_empty() {
                            ui.label(
                                RichText::new(format!("decrypted  {}", hex(&p.plain)))
                                    .monospace()
                                    .size(11.0)
                                    .color(GREY),
                            );
                        }
                    });
            }
        }

        fn keys_tab(&mut self, ui: &mut Ui, commands: &mut Vec<Command>) {
            let before = self.s.clone();
            ui.label(
                RichText::new(
                    "Watched (all: the presets are found by themselves; fewer: less CPU)",
                )
                .color(GREY),
            );
            ui.horizontal(|ui| {
                ui.label("bandwidth");
                for (i, b) in BANDWIDTHS.iter().enumerate() {
                    ui.checkbox(&mut self.s.bw[i], format!("{} kHz", bw_label(*b)));
                }
            });
            ui.horizontal(|ui| {
                ui.label("SF");
                for i in 0..6 {
                    ui.checkbox(&mut self.s.sf[i], (i + 7).to_string());
                }
            });
            ui.add_space(6.0);
            ui.label(
                RichText::new("Meshtastic channels: every preset's default (key AQ==) is built in")
                    .color(GREY),
            );
            let mut drop = None;
            for (i, (n, k)) in self.s.mt_channels.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!(
                        "{n}  key {}",
                        if k.len() > 8 {
                            format!("{}…", &k[..8])
                        } else {
                            k.clone()
                        }
                    ));
                    if ui.small_button("remove").clicked() {
                        drop = Some(i);
                    }
                });
            }
            if let Some(i) = drop {
                self.s.mt_channels.remove(i);
            }
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.new_mt.0)
                        .hint_text("name")
                        .desired_width(110.0),
                );
                ui.add(
                    egui::TextEdit::singleline(&mut self.new_mt.1)
                        .hint_text("PSK (base64)")
                        .desired_width(260.0),
                );
                if ui.button("add").clicked() && !self.new_mt.0.trim().is_empty() {
                    self.s
                        .mt_channels
                        .push((self.new_mt.0.trim().into(), self.new_mt.1.trim().into()));
                    self.new_mt = Default::default();
                }
            });
            ui.add_space(6.0);
            ui.label(
                RichText::new(
                    "MeshCore channels: Public is built in; #hashtag channels need only their name",
                )
                .color(GREY),
            );
            let mut drop = None;
            for (i, (n, k)) in self.s.mc_channels.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(if k.is_empty() {
                        n.clone()
                    } else {
                        format!("{n}  key {}…", &k[..k.len().min(8)])
                    });
                    if ui.small_button("remove").clicked() {
                        drop = Some(i);
                    }
                });
            }
            if let Some(i) = drop {
                self.s.mc_channels.remove(i);
            }
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.new_mc.0)
                        .hint_text("#name or name")
                        .desired_width(110.0),
                );
                ui.add(
                    egui::TextEdit::singleline(&mut self.new_mc.1)
                        .hint_text("PSK (base64; empty for #hashtag)")
                        .desired_width(260.0),
                );
                if ui.button("add").clicked() && !self.new_mc.0.trim().is_empty() {
                    self.s
                        .mc_channels
                        .push((self.new_mc.0.trim().into(), self.new_mc.1.trim().into()));
                    self.new_mc = Default::default();
                }
            });
            ui.add_space(6.0);
            ui.checkbox(&mut self.s.bad_crc, "frames that fail their CRC to the log");
            for e in &self.key_errors {
                ui.label(RichText::new(e).color(AMBER));
            }
            if self.s != before {
                commands.extend(self.s.commands());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_lora::sim::{self, Noise};
    use num_complex::Complex32;

    #[test]
    fn decodes_both_meshes() {
        let fs = INPUT_RATE;
        let mut x = Vec::new();
        let mut at = 30_000;
        for s in sim::samples() {
            sim::modulate(&s.tx, &s.bytes, fs, 800.0, 0.5, 0.2, at, &mut x);
            at = x.len() + 30_000;
        }
        x.resize(at, Complex32::default());
        Noise::new(5).add(&mut x, 1e-3);
        let mut d = Lora::new(Settings::default());
        let mut sink = Sink::default();
        let mut news = Vec::new();
        let mut log = Vec::new();
        for (i, b) in x.chunks(65_536).enumerate() {
            let block = Block {
                iq: b,
                tuning: Tuning {
                    center_hz: 869.525e6,
                    sample_rate: fs,
                    generation: 0,
                },
                first_sample: (i * 65_536) as u64,
                dropped_before: 0,
                live: false,
            };
            sink.clear();
            d.process(&block, &mut sink);
            log.append(&mut sink.log);
            for c in sink.custom.drain(..) {
                if let Ok(n) = c.downcast::<News>() {
                    news.push(*n);
                }
            }
        }
        assert_eq!(d.packets, 11, "{log:#?}");
        let msgs: Vec<&MsgRow> = news.iter().flat_map(|n| &n.messages).collect();
        assert!(
            msgs.iter()
                .any(|m| m.text == "Good morning from the hill" && m.net == Net::Meshtastic)
        );
        assert!(msgs.iter().any(|m| m.text == "anyone on the north side?"
            && m.from == "Delta"
            && m.channel == "Public"));
        let nodes = news.iter().rev().find_map(|n| n.nodes.clone()).unwrap();
        let alfa = nodes
            .iter()
            .find(|n| n.id == format!("!{:08x}", sim::NODE_A))
            .unwrap();
        assert_eq!(alfa.name, "ALFA Base");
        assert!(
            nodes
                .iter()
                .any(|n| n.net == Net::MeshCore && n.name == "Echo Rptr" && n.kind == "repeater")
        );
        let tele = nodes
            .iter()
            .find(|n| n.id == format!("!{:08x}", sim::NODE_B))
            .unwrap();
        assert!(
            tele.fields
                .iter()
                .any(|(k, v)| k == "device.battery" && v == "87 %"),
            "{:?}",
            tele.fields
        );
    }

    #[test]
    fn settings_round_trip() {
        let mut s = Settings::default();
        s.set("bw", "250,62.5").unwrap();
        s.set("sf", "8,11").unwrap();
        s.set("mt_channels", "Mine=AQ==").unwrap();
        s.set("mc_channels", "#test;Club=izOH6cXN6mrJ5e26oRXNcg==")
            .unwrap();
        let mut t = Settings::default();
        for c in s.commands() {
            if let Command::Set { key, value } = c {
                t.set(&key, &value).unwrap();
            }
        }
        assert_eq!(s, t);
        assert_eq!(t.bw, [false, true, false, true]);
        assert_eq!(t.sf, [false, true, false, false, true, false]);
        let (k, e) = t.keys();
        assert!(e.is_empty() && k.meshcore.len() == 3 && k.meshtastic[0].name == "Mine");
    }
}
