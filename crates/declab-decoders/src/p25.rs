// made by CasualArclamp
//! P25 phase 1, from `crates/declab-pmr`: calls (who, to which talkgroup,
//! how long, the voice BER), the NAC, encryption, voice through the IMBE
//! vocoder at 48 kHz, and a trunking control channel's TSBKs, the grants'
//! channels turned into frequencies once the identifier updates have been
//! heard.
//!
//! Encrypted calls are logged but not played. Phase 2 (TDMA) is not
//! decoded.

use std::collections::{HashMap, VecDeque};

use declab_api::{
    Audio, Block, Card, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    Sink,
};
use declab_pmr::p25::imbe_frame::ImbeFrame;
use declab_pmr::p25::receiver::{RATE, SNR_GATE_DB};
use declab_pmr::p25::{Band, Event, Lc, Receiver, Tsbk};
use declab_pmr::vocoder::ambe2::{self, FrameKind};
use declab_pmr::vocoder::{ImbeDecoder, Upsample6};

const RECENT: usize = 8;
const SYMBOL_POINTS: usize = 400;
/// A call with no voice for this long has ended (its terminator was lost).
const CALL_TIMEOUT_S: f64 = 1.0;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "p25",
            name: "P25",
            family: Family::Voice,
            standard: "TIA-102 phase 1 (layouts after MMDVMHost, DSD, SDRTrunk)",
            summary: "P25 phase 1: calls, talkgroups, IMBE speech, trunking control channel",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: 12_500.0,
            block_len: 2_400,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(P25::new())
    }

    fn default_scope(&self) -> &'static str {
        "Symbols"
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(crate::gate_panel::GatePanel {
            gates: vec![gate()],
        }))
    }
}

/// P25's SNR gate for the settings panel, with its measured figures.
#[cfg(feature = "ui")]
pub(crate) fn gate() -> crate::gate_panel::Gate {
    crate::gate_panel::Gate {
        label: "P25",
        key: "snr_gate",
        db: SNR_GATE_DB,
        noise_max: 0.5,
        real_median: 11.0,
    }
}

struct Call {
    lc: Lc,
    start_s: f64,
    last_s: f64,
    frames: u64,
    errors: u64,
    encrypted: bool,
}

struct P25 {
    rx: Receiver,
    events: Vec<Event>,
    call: Option<Call>,
    /// The talkgroup the last header announced, until the link control says
    /// who is talking.
    header_tg: Option<(u16, bool)>,
    /// Encrypted (from the header or the LDU2), for the call in progress.
    encrypted: bool,
    /// Channel identifier bands, by their 4-bit id.
    bands: HashMap<u8, Band>,
    last_tsbk: Vec<Tsbk>,
    system: Option<String>,
    recent: VecDeque<(String, String)>,
    now_s: f64,
    since_report: usize,
    sym_hist: VecDeque<f32>,
    vocoder: ImbeDecoder,
    up: Upsample6,
    frames_out: u64,
    pcm8: Vec<f32>,
    pcm48: Vec<f32>,
}

impl P25 {
    fn new() -> Self {
        let mut rx = Receiver::new();
        rx.enable_tap(true);
        P25 {
            rx,
            events: Vec::new(),
            call: None,
            header_tg: None,
            encrypted: false,
            bands: HashMap::new(),
            last_tsbk: Vec::new(),
            system: None,
            recent: VecDeque::new(),
            now_s: 0.0,
            since_report: 0,
            sym_hist: VecDeque::new(),
            vocoder: ImbeDecoder::new(),
            up: Upsample6::new(),
            frames_out: 0,
            pcm8: Vec::with_capacity(9 * ambe2::FRAME),
            pcm48: Vec::with_capacity(54 * ambe2::FRAME),
        }
    }

    fn note(&mut self, value: String, out: &mut Sink) {
        out.log(value.clone());
        self.recent.push_front(("P25".into(), value));
        self.recent.truncate(RECENT);
    }

    fn start_call(&mut self, lc: Lc, out: &mut Sink) {
        if lc.call().is_none() || self.call.as_ref().is_some_and(|c| c.lc.call() == lc.call()) {
            return;
        }
        self.end_call(out);
        let encrypted = self.encrypted || lc.call().is_some_and(|c| c.3.encrypted());
        let tag = if encrypted { " (encrypted)" } else { "" };
        self.note(format!("{lc}{tag}"), out);
        self.vocoder.reset();
        self.up.reset();
        self.call = Some(Call {
            lc,
            start_s: self.now_s,
            last_s: self.now_s,
            frames: 0,
            errors: 0,
            encrypted,
        });
    }

    fn end_call(&mut self, out: &mut Sink) {
        if let Some(c) = self.call.take() {
            let ber = 100.0 * c.errors as f64 / (c.frames * 137).max(1) as f64;
            out.log(format!(
                "end of {} after {:.1} s, {} voice frames, BER {ber:.2} %",
                c.lc,
                c.last_s - c.start_s,
                c.frames
            ));
        }
        self.encrypted = false;
        self.header_tg = None;
    }

    fn voice(&mut self, frames: &[ImbeFrame], out: &mut Sink) {
        let encrypted = self
            .call
            .as_ref()
            .map_or(self.encrypted, |c| c.encrypted || self.encrypted);
        if let Some(c) = &mut self.call {
            c.frames += frames.len() as u64;
            c.errors += frames.iter().map(|f| u64::from(f.errors)).sum::<u64>();
            c.last_s = self.now_s;
            c.encrypted = encrypted;
        }
        if encrypted {
            return;
        }
        self.pcm8.clear();
        let mut buf = [0f32; ambe2::FRAME];
        for f in frames {
            if self.vocoder.decode(&f.bits, f.data_errors, &mut buf) != FrameKind::Silence {
                self.frames_out += 1;
            }
            self.pcm8.extend_from_slice(&buf);
        }
        self.pcm48.clear();
        self.up.process(&self.pcm8, &mut self.pcm48);
        out.audio.push(Audio {
            service: 0,
            sample_rate: 48_000,
            channels: 1,
            samples: self.pcm48.clone(),
        });
    }

    /// A grant's channel as text, with its frequency once its band is known.
    fn channel(&self, band: u8, ch: u16) -> String {
        match self.bands.get(&band) {
            Some(b) => format!("{:.5} MHz", b.downlink_hz(ch) / 1e6),
            None => format!("{band}-{ch}"),
        }
    }

    fn tsbk(&mut self, t: Tsbk, out: &mut Sink) {
        if let Tsbk::Iden { id, band } = t {
            self.bands.insert(id, band);
        }
        let text = match t {
            Tsbk::GroupGrant {
                band,
                channel,
                group,
                source,
                ..
            } => {
                format!(
                    "grant TG {group} ← {source} on {}",
                    self.channel(band, channel)
                )
            }
            Tsbk::UnitGrant {
                band,
                channel,
                target,
                source,
            } => {
                format!(
                    "grant {source} → {target} on {}",
                    self.channel(band, channel)
                )
            }
            Tsbk::GroupGrantUpdate { a, b } => {
                let mut s = format!("TG {} on {}", a.2, self.channel(a.0, a.1));
                if let Some(b) = b {
                    s += &format!(", TG {} on {}", b.2, self.channel(b.0, b.1));
                }
                format!("grant update {s}")
            }
            Tsbk::RfssStatus {
                system,
                rfss,
                site,
                band,
                channel,
            } => {
                let sys = format!(
                    "system {system:03X} RFSS {rfss} site {site}, control channel {}",
                    self.channel(band, channel)
                );
                if self.system.as_deref() != Some(&sys) {
                    out.log(format!("system: {sys}"));
                    self.system = Some(sys);
                }
                return;
            }
            t => t.to_string(),
        };
        let grant = matches!(t, Tsbk::GroupGrant { .. } | Tsbk::UnitGrant { .. });
        if !grant {
            if self.last_tsbk.contains(&t) {
                return;
            }
            let same = |a: &Tsbk| std::mem::discriminant(a) == std::mem::discriminant(&t);
            self.last_tsbk.retain(|a| !same(a));
            self.last_tsbk.push(t);
        }
        self.note(text, out);
    }

    fn take_events(&mut self, out: &mut Sink) {
        for e in std::mem::take(&mut self.events) {
            match e {
                Event::Header { header } => {
                    self.encrypted = header.encrypted();
                    self.header_tg = Some((header.tgid, header.encrypted()));
                }
                Event::Lc { lc } => self.start_call(lc, out),
                Event::Encryption { algid, .. } => self.encrypted = algid != 0x80,
                Event::Voice { frames } => self.voice(&frames, out),
                Event::End => self.end_call(out),
                Event::Tsbk { tsbk } => self.tsbk(tsbk, out),
            }
        }
        if self
            .call
            .as_ref()
            .is_some_and(|c| self.now_s - c.last_s > CALL_TIMEOUT_S)
        {
            self.end_call(out);
        }
    }

    fn report(&mut self, out: &mut Sink) {
        let st = self.rx.stats.clone();
        out.lock = match (st.locked, &self.call) {
            (false, _) => Lock::Searching,
            (true, Some(_)) => Lock::Locked,
            (true, None) => Lock::Synced,
        };
        if let Some(tap) = self.rx.tap_mut() {
            self.sym_hist.extend(tap.drain(..));
        }
        let over = self.sym_hist.len().saturating_sub(SYMBOL_POINTS);
        self.sym_hist.drain(..over);
        let n = self.sym_hist.len().max(2) as f32 - 1.0;
        let pts: Vec<Iq> = self
            .sym_hist
            .iter()
            .enumerate()
            .map(|(i, &v)| Iq::new(2.0 * i as f32 / n - 1.0, (v / 3.0).clamp(-1.5, 1.5)))
            .collect();
        out.scopes.push(("Symbols", pts));
        let ber = 100.0 * st.voice_bit_errors as f64 / st.voice_bits.max(1) as f64;
        out.measure("Eye SNR", f64::from(st.eye_snr_db), "dB");
        out.measure("Voice BER", ber, "%");
        let headline = match (&self.call, st.locked) {
            (Some(c), _) => {
                let enc = if c.encrypted { " 🔒" } else { "" };
                match c.lc.call() {
                    Some((s, d, true, _)) => format!("TG {d} ← {s}{enc}"),
                    Some((s, d, false, _)) => format!("{s} → {d}{enc}"),
                    None => c.lc.to_string(),
                }
            }
            (None, true) if st.duid == Some(7) => "control channel".into(),
            (None, true) => match self.header_tg {
                Some((tg, _)) => format!("TG {tg}"),
                None => "carrier".into(),
            },
            (None, false) => "searching".into(),
        };
        let pair = |k: &str, v: String| (k.to_string(), v);
        let mut text: Vec<(String, String)> = Vec::new();
        if let Some(s) = &self.system {
            text.push(("system".into(), s.clone()));
        }
        text.extend(self.recent.iter().cloned());
        out.card = Some(Card {
            title: "P25 phase 1".into(),
            headline,
            gauge: st.locked.then(|| Gauge {
                label: "eye SNR".into(),
                value_db: st.eye_snr_db,
                need_db: Some(8.0),
                need_label: "voice".into(),
                range: (0.0, 30.0),
            }),
            details: vec![
                pair("NAC", st.nac.map_or("–".into(), |n| format!("{n:03X}"))),
                pair("offset", format!("{:+.0} Hz", st.offset_hz)),
                pair(
                    "voice frames",
                    format!("{} (BER {ber:.2} %)", st.voice_frames),
                ),
                pair(
                    "frames",
                    format!(
                        "{} ({} bad, {} CRC fails)",
                        st.frames, st.frames_bad, st.crc_fail
                    ),
                ),
                pair(
                    "sync",
                    format!(
                        "{:.2} ({:.0} dB)",
                        st.sync_quality,
                        st.sync_snr_db.max(-99.0)
                    ),
                ),
                pair(
                    "SNR gate",
                    format!("{:.0} dB ({} turned away)", self.rx.snr_gate_db, st.gated),
                ),
                pair("spoken", format!("{} frames", self.frames_out)),
                pair("bands known", self.bands.len().to_string()),
            ],
            text,
        });
    }
}

impl Decoder for P25 {
    fn command(&mut self, cmd: declab_api::Command, out: &mut Sink) {
        if let declab_api::Command::Set { key, value } = cmd {
            match (key.as_str(), value.parse::<f32>()) {
                ("snr_gate", Ok(db)) => self.rx.snr_gate_db = db.clamp(0.0, 20.0),
                _ => out.log(format!("p25: no setting {key:?} = {value:?}")),
            }
        }
    }

    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        self.rx.process(block.iq, &mut self.events);
        self.now_s = (block.first_sample + block.iq.len() as u64) as f64 / block.tuning.sample_rate;
        self.take_events(out);
        self.since_report += block.iq.len();
        if self.since_report as f64 >= block.tuning.sample_rate / 10.0 {
            self.since_report = 0;
            self.report(out);
        }
    }

    fn reset(&mut self) {
        self.rx.reset();
        self.call = None;
        self.sym_hist.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_pmr::p25::{ServiceOptions, sim};

    #[test]
    fn logs_a_call_speaks_and_reads_a_control_channel() {
        let lc = Lc::GroupVoice {
            opts: ServiceOptions(0),
            group: 101,
            source: 1_234_567,
        };
        let (mut bits, frames) = sim::call(0x293, &lc, 2);
        let iden = Tsbk::Iden {
            id: 1,
            band: Band {
                base_hz: 851_006_250.0,
                spacing_hz: 6250.0,
                slots: 1,
            },
        };
        let grant = Tsbk::GroupGrant {
            opts: ServiceOptions(0),
            band: 1,
            channel: 0x10,
            group: 101,
            source: 77,
        };
        for _ in 0..4 {
            bits.extend(sim::tsdu(0x293, &[iden, grant]));
        }
        let mut iq = sim::modulate(&bits, RATE, 0.1);
        sim::add_noise(&mut iq, RATE, 20.0, 200.0, 4);
        let mut d = Factory.create();
        let (mut log, mut sound) = (Vec::new(), 0usize);
        let tuning = Tuning {
            center_hz: 0.0,
            sample_rate: RATE,
            generation: 0,
        };
        for (n, chunk) in iq.chunks(2_400).enumerate() {
            let block = Block {
                iq: chunk,
                tuning,
                first_sample: n as u64 * 2_400,
                dropped_before: 0,
                live: false,
            };
            let mut sink = Sink::default();
            d.process(&block, &mut sink);
            log.append(&mut sink.log);
            sound += sink.audio.iter().map(|a| a.samples.len()).sum::<usize>();
        }
        assert!(
            log.iter().any(|l| l == "group call TG 101 from 1234567"),
            "{log:#?}"
        );
        assert!(
            log.iter().any(|l| l.starts_with("end of group call TG 101")
                && l.contains(&format!("{frames} voice frames"))),
            "{log:#?}"
        );
        assert_eq!(sound, frames as usize * 960);
        // 851.00625 + 16 × 6.25 kHz.
        assert!(
            log.iter()
                .any(|l| l == "grant TG 101 ← 77 on 851.10625 MHz"),
            "{log:#?}"
        );
    }
}
