// made by CasualArclamp
//! DAB and DAB+ (EN 300 401, TS 102 563) as a plug-in: DecDAB's receiver
//! (`crates/decdab-*`) behind the decoder API. The receive loop and the
//! service decoder follow DecDAB's engine (`decdab-engine/src/rx.rs`,
//! `service.rs`).
//!
//! The host hands over the block at 2.048 MS/s, centred; the OFDM
//! demodulator gives each transmission frame's soft bits, the FIC describes
//! the ensemble, and the chosen service's sub-channel is decoded to sound:
//! DAB+ superframes through libxaac (HE-AAC v2), DAB's Layer II through
//! DecDAB's own decoder. Its PAD carries the dynamic label (the service's
//! text) and the slideshow (its pictures).

use std::collections::VecDeque;

use declab_api::{
    Audio, Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec,
    Iq, Lock, Picture, Service, Sink,
};

use decdab_codecs::{AacDecoder, Mp2Decoder, Pcm};
use decdab_data::pad::{PadDecoder, PadEvent};
use decdab_fic::{Component, FicDecoder, SubChannel, Transport};
use decdab_msc::{AudioParams, SubChannelDecoder, SuperframeDecoder, au_pad};
use decdab_ofdm::Demod;
use decdab_ofdm::demod::FrameStats;
use decdab_ofdm::params::{CIF_BITS, CIFS, FIC_BITS};

/// DAB's sample rate (EN 300 401 §14.5: 2.048 MHz for mode I).
const RATE: f64 = 2.048e6;
/// The card, this often (s).
const REPORT_S: f64 = 0.1;
/// The MER where EEP 3-A (the usual protection) decodes without errors, as
/// DecDAB measured it on air (its DESIGN, the card's margin).
const MER_NEEDED_DB: f32 = 10.5;
/// Points kept for the scope.
const SCOPE_POINTS: usize = 3000;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "dab",
            name: "DAB/DAB+",
            family: Family::Radio,
            standard: "EN 300 401 / TS 102 563",
            summary: "Digital radio: the ensemble's services, DAB+ and DAB sound, text, slideshow",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // 1.536 MHz of carriers; a 96 ms frame's samples a block.
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: 1.6e6,
            block_len: 49_152,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Dab::new())
    }

    fn default_scope(&self) -> &'static str {
        "Constellation"
    }
}

/// The chosen service's sound decoder.
enum Sound {
    Plus {
        sf: Box<SuperframeDecoder>,
        aac: Option<(AudioParams, AacDecoder)>,
    },
    Mp2(Box<Mp2Decoder>),
}

/// One service: its sub-channel's CIFs to sound and PAD (DecDAB's
/// `ServiceDecoder`, trimmed to what the plug-in shows).
struct Playing {
    sid: u32,
    name: String,
    sub: SubChannelDecoder,
    sound: Sound,
    pad: PadDecoder,
    codec: String,
    au_ok: u64,
    au_bad: u64,
    rs_corrected: u64,
    rs_failed: u64,
    errors: u64,
    last_error: Option<String>,
}

impl Playing {
    fn new(sid: u32, name: String, comp: &Component, sub: SubChannel) -> Option<Playing> {
        let d = SubChannelDecoder::new(sub)?;
        let sound = if comp.is_dab_plus() {
            Sound::Plus {
                sf: Box::new(SuperframeDecoder::new(d.frame_bytes())),
                aac: None,
            }
        } else if matches!(comp.transport, Transport::Audio { .. }) {
            Sound::Mp2(Box::new(Mp2Decoder::new()))
        } else {
            return None;
        };
        Some(Playing {
            sid,
            name,
            sub: d,
            sound,
            pad: PadDecoder::new(),
            codec: String::new(),
            au_ok: 0,
            au_bad: 0,
            rs_corrected: 0,
            rs_failed: 0,
            errors: 0,
            last_error: None,
        })
    }

    fn push_cif(&mut self, cif: &[i8], pcm: &mut Vec<Pcm>, pad: &mut Vec<PadEvent>) {
        let Some(lf) = self.sub.push_cif(cif) else {
            return;
        };
        let pad_dec = &mut self.pad;
        let (codec, errors, last_error) = (&mut self.codec, &mut self.errors, &mut self.last_error);
        match &mut self.sound {
            Sound::Plus { sf, aac } => {
                sf.push(lf, |p, au| {
                    // Rust note: `is_none_or` is true for `None` or when the
                    // closure holds: a new stream or changed parameters.
                    if aac.as_ref().is_none_or(|(q, _)| q != p) {
                        match AacDecoder::new(&p.audio_specific_config()) {
                            Ok(d) => {
                                *aac = Some((*p, d));
                                *codec = format!(
                                    "{} · {} kHz{}",
                                    p.codec_name(),
                                    p.output_rate() / 1000,
                                    if p.stereo || p.ps { " stereo" } else { " mono" }
                                );
                            }
                            Err(e) => {
                                *errors += 1;
                                *last_error = Some(e.to_string());
                            }
                        }
                    }
                    if au.ok
                        && let Some(field) = au_pad(au.data)
                    {
                        pad_dec.push(field, pad);
                    }
                    if let Some((_, d)) = aac {
                        match d.decode(au.ok.then_some(au.data)) {
                            Ok(p) => pcm.push(p),
                            Err(e) => {
                                *errors += 1;
                                *last_error = Some(e.to_string());
                            }
                        }
                    }
                });
                let s = sf.stats;
                self.au_ok = s.au_ok;
                self.au_bad = s.au_bad;
                self.rs_corrected = s.rs_corrected;
                self.rs_failed = s.rs_failed;
            }
            Sound::Mp2(m) => {
                let mut frames: Vec<Vec<u8>> = Vec::new();
                m.push(lf, |f| frames.push(f.to_vec()));
                for f in frames {
                    if let Some(field) = decdab_codecs::mp2::pad_field(&f) {
                        pad_dec.push(&field, pad);
                    }
                    match m.decode(&f) {
                        Ok(p) => {
                            if codec.is_empty()
                                && let Some(h) = decdab_codecs::mp2::Header::parse(&f)
                            {
                                *codec = format!(
                                    "MPEG-{} Layer II · {} kHz{}",
                                    if h.mpeg1 { 1 } else { 2 },
                                    h.rate / 1000,
                                    if h.channels() == 2 { " stereo" } else { " mono" }
                                );
                            }
                            self.au_ok += 1;
                            pcm.push(p);
                        }
                        Err(e) => {
                            self.au_bad += 1;
                            *errors += 1;
                            *last_error = Some(e.to_string());
                        }
                    }
                }
            }
        }
    }
}

struct Dab {
    demod: Demod,
    fic: FicDecoder,
    fic_gen: u64,
    /// The service asked for (`None`: the first radio service).
    want: Option<u32>,
    playing: Option<Playing>,
    frames: Vec<Vec<i8>>,
    stats: Option<FrameStats>,
    points: VecDeque<Iq>,
    services: Vec<Service>,
    dls: Option<String>,
    slides: u64,
    pcm: Vec<Pcm>,
    pad: Vec<PadEvent>,
    since_report: usize,
}

impl Dab {
    fn new() -> Dab {
        let mut demod = Demod::new();
        demod.set_scopes(true);
        Dab {
            demod,
            fic: FicDecoder::new(),
            fic_gen: u64::MAX,
            want: None,
            playing: None,
            frames: Vec::new(),
            stats: None,
            points: VecDeque::new(),
            services: Vec::new(),
            dls: None,
            slides: 0,
            pcm: Vec::new(),
            pad: Vec::new(),
            since_report: 0,
        }
    }

    /// The services list, from the FIC (when it changed).
    fn services(&mut self, out: &mut Sink) {
        if self.fic.generation == self.fic_gen {
            return;
        }
        self.fic_gen = self.fic.generation;
        let e = &self.fic.ensemble;
        // The FIC fills in over a second or two: a service started on an
        // early, partial picture (seen: a service's SId mapped to another's
        // sub-channel) is started again when its description changes.
        if let Some(p) = &self.playing {
            let now = e.services.get(&p.sid).and_then(|svc| {
                let c = svc.primary()?;
                Some((e.subchannel_of(c)?.id, c.is_dab_plus()))
            });
            let was = (p.sub.subchannel().id, matches!(p.sound, Sound::Plus { .. }));
            if now != Some(was) {
                self.playing = None;
                self.dls = None;
            }
        }
        let ens = e.name();
        let list: Vec<Service> = e
            .services_by_name()
            .into_iter()
            .map(|s| Service {
                id: s.sid,
                name: s.name(),
                provider: ens.clone(),
                kind: match s.primary() {
                    Some(c) if c.is_dab_plus() => "DAB+",
                    Some(c) if matches!(c.transport, Transport::Audio { .. }) => "DAB",
                    _ => "Data",
                }
                .into(),
            })
            .collect();
        if list != self.services {
            out.services = Some(list.clone());
            self.services = list;
        }
        // Nothing picked: play what the list shows first (the GUI marks
        // the first service when none is chosen), which can change while
        // the labels come in.
        if self.want.is_none()
            && let Some(first) = self.services.iter().find(|s| s.kind != "Data")
            && self.playing.as_ref().is_some_and(|p| p.sid != first.id)
        {
            self.playing = None;
            self.dls = None;
        }
    }

    /// Start the wanted service (or the first radio one) once the FIC
    /// describes it.
    fn start(&mut self, out: &mut Sink) {
        if self.playing.is_some() {
            return;
        }
        let e = &self.fic.ensemble;
        let sid = match self.want {
            Some(s) => s,
            None => match self.services.iter().find(|s| s.kind != "Data") {
                Some(s) => s.id,
                None => return,
            },
        };
        if let Some(svc) = e.services.get(&sid)
            && let Some(comp) = svc.primary()
            && let Some(sc) = e.subchannel_of(comp)
            && let Some(p) = Playing::new(sid, svc.name(), comp, *sc)
        {
            self.demod.set_scope_subchannel(Some((sc.start, sc.size)));
            out.log(format!("playing {} ({})", p.name, sc.protection_name()));
            self.playing = Some(p);
            self.dls = None;
        }
    }

    fn report(&mut self, out: &mut Sink) {
        let locked = self.demod.is_locked();
        let sounding = self.playing.as_ref().is_some_and(|p| p.au_ok > 0);
        out.lock = match (locked, sounding) {
            (false, _) => Lock::Searching,
            (true, false) => Lock::Synced,
            (true, true) => Lock::Locked,
        };
        let mer = self.stats.map_or(0.0, |s| s.mer_db);
        if locked {
            out.measure("MER", f64::from(mer), "dB");
        }
        out.constellation = self.points.iter().copied().collect();
        let e = &self.fic.ensemble;
        let ens = e.name();
        let pair = |k: &str, v: String| (k.to_string(), v);
        let mut details = Vec::new();
        if !ens.is_empty() {
            let id = e.eid.map_or(String::new(), |id| format!(" (EId {id:04X})"));
            details.push(pair("ensemble", format!("{ens}{id}")));
        }
        details.push(pair("services", self.services.len().to_string()));
        if let Some(s) = &self.stats {
            details.push(pair("offset", format!("{:+.0} Hz", s.cfo_hz)));
            details.push(pair("frames", s.frame.to_string()));
        }
        let mut text = Vec::new();
        let headline = match &self.playing {
            Some(p) => {
                let sc = p.sub.subchannel();
                details.push(pair(
                    "audio",
                    if p.codec.is_empty() { "starting".into() } else { p.codec.clone() },
                ));
                details.push(pair(
                    "sub-channel",
                    format!("{} · {} kbit/s · {}", sc.id, sc.bitrate(), sc.protection_name()),
                ));
                details.push(pair("AUs", format!("{} ({} bad)", p.au_ok, p.au_bad)));
                if p.rs_corrected + p.rs_failed > 0 {
                    details.push(pair(
                        "RS",
                        format!("{} bytes fixed, {} lost", p.rs_corrected, p.rs_failed),
                    ));
                }
                if p.errors > 0 {
                    details.push(pair(
                        "errors",
                        format!("{} ({})", p.errors, p.last_error.clone().unwrap_or_default()),
                    ));
                }
                if let Some(d) = &self.dls {
                    text.push(pair("now", d.clone()));
                }
                // Its label may have come after it started.
                e.services.get(&p.sid).map_or(p.name.clone(), |s| s.name())
            }
            None if locked && !ens.is_empty() => ens.clone(),
            None if locked => "synchronised".into(),
            None => "searching".into(),
        };
        let kind = match &self.playing {
            Some(p) if matches!(p.sound, Sound::Plus { .. }) => "DAB+",
            Some(_) => "DAB",
            None => "DAB/DAB+",
        };
        out.card = Some(Card {
            title: if ens.is_empty() { kind.into() } else { format!("{kind} · {ens}") },
            headline,
            gauge: locked.then(|| Gauge {
                label: "MER".into(),
                value_db: mer,
                need_db: Some(MER_NEEDED_DB),
                need_label: "EEP 3-A".into(),
                range: (0.0, 30.0),
            }),
            details,
            text,
        });
    }
}

impl Decoder for Dab {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        let (frames, stats, points) = (&mut self.frames, &mut self.stats, &mut self.points);
        self.demod.push(block.iq, |f| {
            frames.push(f.soft.to_vec());
            *stats = Some(f.stats);
            if let Some(s) = f.scope {
                let pts = if s.msc.is_empty() { &s.fic } else { &s.msc };
                // The differential symbols sit on the diagonals at about
                // ±1: scaled into the scope's ±1 square.
                points.extend(pts.iter().map(|z| z * 0.7));
                let over = points.len().saturating_sub(SCOPE_POINTS);
                points.drain(..over);
            }
        });
        for soft in std::mem::take(&mut self.frames) {
            self.fic.push_frame(&soft[..FIC_BITS]);
            self.services(out);
            self.start(out);
            if let Some(p) = &mut self.playing {
                for c in 0..CIFS {
                    let a = FIC_BITS + c * CIF_BITS;
                    p.push_cif(&soft[a..a + CIF_BITS], &mut self.pcm, &mut self.pad);
                }
                let sid = p.sid;
                for pcm in self.pcm.drain(..) {
                    // Concealment before the first frame has no shape yet.
                    if pcm.samples.is_empty() || pcm.rate == 0 {
                        continue;
                    }
                    out.audio.push(Audio {
                        service: sid,
                        sample_rate: pcm.rate,
                        channels: pcm.channels,
                        samples: pcm.samples.iter().map(|&s| f32::from(s) / 32768.0).collect(),
                    });
                }
                for ev in self.pad.drain(..) {
                    match ev {
                        PadEvent::Dls(l) => {
                            if self.dls.as_deref() != Some(l.text.as_str()) {
                                out.text.push((sid, l.text.clone()));
                                self.dls = Some(l.text);
                            }
                        }
                        PadEvent::DlsClear => self.dls = None,
                        PadEvent::Mot { mime, data, .. } if mime.starts_with("image/") => {
                            self.slides += 1;
                            out.pictures.push(Picture {
                                service: sid,
                                mime,
                                bytes: data,
                            });
                        }
                        PadEvent::Mot { .. } => {}
                    }
                }
            }
        }
        self.since_report += block.iq.len();
        if self.since_report as f64 >= RATE * REPORT_S {
            self.since_report = 0;
            self.report(out);
        }
    }

    fn command(&mut self, cmd: Command, out: &mut Sink) {
        match cmd {
            Command::SelectService(sid) => {
                if self.playing.as_ref().is_none_or(|p| p.sid != sid) {
                    self.want = Some(sid);
                    self.playing = None;
                    self.dls = None;
                    self.demod.set_scope_subchannel(None);
                    self.start(out);
                }
            }
            Command::Set { key, value } => out.log(format!("dab: no setting {key:?} = {value:?}")),
            Command::Custom(_) => {}
        }
    }

    fn reset(&mut self) {
        self.demod.reset();
        self.fic = FicDecoder::new();
        self.fic_gen = u64::MAX;
        self.playing = None;
        self.services.clear();
        self.points.clear();
        self.stats = None;
        self.dls = None;
    }
}
