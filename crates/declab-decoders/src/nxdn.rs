// made by CasualArclamp
//! NXDN48 (6.25 kHz) and NXDN96 (12.5 kHz) conventional voice, from
//! `crates/declab-pmr`: calls (who, to which talkgroup, how long, the
//! voice BER) from FACCH1 and the SACCH superframe, the RAN, and voice
//! through the AMBE+2 vocoder at 48 kHz.
//!
//! Control channels (trunking) are recognised but their CAC is not decoded
//! yet; nor are data calls or NXDN96.
//!
//! Scope "Symbols": the soft symbols of the latest frames, four bands.

use std::collections::VecDeque;

use declab_api::{
    Audio, Block, Card, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    Sink,
};
use declab_pmr::nxdn::receiver::{RATE, SNR_GATE_DB};
use declab_pmr::nxdn::{Event, Message, Mode, Receiver};
use declab_pmr::vocoder::ambe2::{self, FrameKind};
use declab_pmr::vocoder::{Ambe2Decoder, Upsample6};

const RECENT: usize = 8;
const SYMBOL_POINTS: usize = 384;
/// A call with no voice for this long has ended (its release was lost).
const CALL_TIMEOUT_S: f64 = 1.0;

/// NXDN48 or NXDN96.
pub struct Factory(pub Mode);

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        let (id, summary) = match self.0 {
            Mode::Nxdn48 => (
                "nxdn",
                "NXDN48 (6.25 kHz) conventional voice: calls, talkgroups, AMBE+2 speech",
            ),
            Mode::Nxdn96 => (
                "nxdn96",
                "NXDN96 (12.5 kHz) conventional voice: calls, talkgroups, AMBE+2 speech",
            ),
        };
        DecoderInfo {
            id,
            name: self.0.name(),
            family: Family::Voice,
            standard: "NXDN (layouts after MMDVMHost and SDRTrunk)",
            summary,
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // The receiver filters to its own narrower low-pass; the host keeps
        // the neighbours out.
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: self.0.channel_hz(),
            block_len: 2_400,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Nxdn::new(self.0))
    }

    fn default_scope(&self) -> &'static str {
        "Symbols"
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(crate::gate_panel::GatePanel {
            gates: vec![gate(self.0)],
        }))
    }
}

/// NXDN's SNR gate for the settings panel, with its measured figures.
#[cfg(feature = "ui")]
pub(crate) fn gate(mode: Mode) -> crate::gate_panel::Gate {
    // The best fit two minutes of noise made (`nxdn_sweep`).
    let noise_max = if mode == Mode::Nxdn48 { 7.4 } else { 8.0 };
    crate::gate_panel::Gate {
        label: mode.name(),
        key: "snr_gate",
        db: SNR_GATE_DB,
        noise_max,
        real_median: 11.0,
    }
}

struct Call {
    msg: Message,
    start_s: f64,
    last_s: f64,
    frames: u64,
    errors: u64,
}

struct Nxdn {
    mode: Mode,
    rx: Receiver,
    events: Vec<Event>,
    call: Option<Call>,
    recent: VecDeque<(String, String)>,
    now_s: f64,
    since_report: usize,
    sym_hist: VecDeque<f32>,
    vocoder: Ambe2Decoder,
    up: Upsample6,
    frames_out: u64,
    pcm8: Vec<f32>,
    pcm48: Vec<f32>,
    control_frames: u64,
}

impl Nxdn {
    fn new(mode: Mode) -> Self {
        let mut rx = Receiver::for_mode(mode);
        rx.enable_tap(true);
        Nxdn {
            mode,
            rx,
            events: Vec::new(),
            call: None,
            recent: VecDeque::new(),
            now_s: 0.0,
            since_report: 0,
            sym_hist: VecDeque::new(),
            vocoder: Ambe2Decoder::new(),
            up: Upsample6::new(),
            frames_out: 0,
            pcm8: Vec::with_capacity(4 * ambe2::FRAME),
            pcm48: Vec::with_capacity(24 * ambe2::FRAME),
            control_frames: 0,
        }
    }

    fn note(&mut self, value: String, out: &mut Sink) {
        out.log(value.clone());
        self.recent.push_front((self.mode.name().into(), value));
        self.recent.truncate(RECENT);
    }

    fn start_call(&mut self, msg: Message, out: &mut Sink) {
        if self
            .call
            .as_ref()
            .is_some_and(|c| c.msg.call() == msg.call())
        {
            return;
        }
        self.end_call(out);
        self.note(msg.to_string(), out);
        self.vocoder.reset();
        self.up.reset();
        self.call = Some(Call {
            msg,
            start_s: self.now_s,
            last_s: self.now_s,
            frames: 0,
            errors: 0,
        });
    }

    fn end_call(&mut self, out: &mut Sink) {
        if let Some(c) = self.call.take() {
            let ber = 100.0 * c.errors as f64 / (c.frames * 46).max(1) as f64;
            out.log(format!(
                "end of {} after {:.1} s, {} voice frames, BER {ber:.2} %",
                c.msg,
                c.last_s - c.start_s,
                c.frames
            ));
        }
    }

    fn voice(&mut self, frames: &[declab_pmr::dmr::ambe::AmbeFrame], out: &mut Sink) {
        if let Some(c) = &mut self.call {
            c.frames += frames.len() as u64;
            c.errors += frames.iter().map(|f| u64::from(f.errors)).sum::<u64>();
            c.last_s = self.now_s;
        }
        self.pcm8.clear();
        let mut buf = [0f32; ambe2::FRAME];
        for f in frames {
            if self.vocoder.decode(&f.bits, f.data_errors, &mut buf) != FrameKind::Erasure {
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

    fn take_events(&mut self, out: &mut Sink) {
        for e in std::mem::take(&mut self.events) {
            match e {
                Event::Facch { msg } | Event::Sacch { msg } => match msg.kind() {
                    0x01 => self.start_call(msg, out),
                    0x08 => self.end_call(out),
                    0x10 => {}
                    _ => self.note(msg.to_string(), out),
                },
                Event::Voice { frames } => self.voice(&frames, out),
                Event::ControlChannel { .. } => self.control_frames += 1,
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
            (Some(c), _) => match c.msg.call() {
                Some((s, d, true)) => format!("TG {d} ← {s}"),
                Some((s, d, false)) => format!("{s} → {d}"),
                None => c.msg.to_string(),
            },
            (None, true) if self.control_frames > 0 => "control channel".into(),
            (None, true) => "carrier".into(),
            (None, false) => "searching".into(),
        };
        let pair = |k: &str, v: String| (k.to_string(), v);
        out.card = Some(Card {
            title: self.mode.name().into(),
            headline,
            gauge: st.locked.then(|| Gauge {
                label: "eye SNR".into(),
                value_db: st.eye_snr_db,
                need_db: Some(8.0),
                need_label: "voice".into(),
                range: (0.0, 30.0),
            }),
            details: vec![
                pair("RAN", st.ran.map_or("–".into(), |r| r.to_string())),
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
                pair("control frames", self.control_frames.to_string()),
            ],
            text: self.recent.iter().cloned().collect(),
        });
    }
}

impl Decoder for Nxdn {
    fn command(&mut self, cmd: declab_api::Command, out: &mut Sink) {
        if let declab_api::Command::Set { key, value } = cmd {
            match (key.as_str(), value.parse::<f32>()) {
                ("snr_gate", Ok(db)) => self.rx.snr_gate_db = db.clamp(0.0, 20.0),
                _ => out.log(format!("nxdn: no setting {key:?} = {value:?}")),
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
    use declab_pmr::nxdn::sim;

    #[test]
    fn logs_a_call_and_speaks() {
        let msg = Message::vcall(0x01, 1234, 77, true);
        let (bits, frames) = sim::call(5, &msg, 2);
        let mut iq = sim::modulate(&bits, RATE, 0.1);
        sim::add_noise(&mut iq, RATE, 20.0, 200.0, 4);
        let mut d = Factory(Mode::Nxdn48).create();
        let (mut log, mut sound, mut card) = (Vec::new(), 0usize, None);
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
            card = sink.card.or(card);
            sound += sink.audio.iter().map(|a| a.samples.len()).sum::<usize>();
        }
        assert!(
            log.iter().any(|l| l == "group call TG 77 from 1234"),
            "{log:#?}"
        );
        assert!(
            log.iter()
                .any(|l| l.starts_with("end of group call TG 77 from 1234")
                    && l.contains(&format!("{frames} voice frames"))),
            "{log:#?}"
        );
        assert_eq!(sound, frames as usize * 960);
        assert!(card.is_some());
    }
}
