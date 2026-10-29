// made by CasualArclamp
//! DMR (ETSI TS 102 361) Tier I, II and III, from `crates/declab-pmr`.
//!
//! One channel, both time slots: calls (who, to which talkgroup, how long,
//! the voice channel's bit error rate), the Tier III control channel
//! (Aloha, grants, broadcasts, clears) and the repeater's Short LC, on the
//! card and in the log, and data packets, with Motorola LRRP location
//! reports decoded to latitude and longitude.
//!
//! Sound: one slot at a time, the first to carry voice, held until its call
//! ends. AMBE+2 frames go through the vocoder (ported from mbelib) to 8 kHz,
//! then six times up to 48 kHz. Calls marked private (encrypted) are not
//! played: they would only be noise.
//!
//! Scope "Symbols": the soft symbols of the latest bursts, four bands.

use std::collections::VecDeque;

use declab_api::{
    Audio, Block, Card, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    Sink,
};
use declab_pmr::dmr::data::Packet;
use declab_pmr::dmr::lrrp::{self, Lrrp};
use declab_pmr::dmr::pdu::{Csbk, FullLc, ShortLc};
use declab_pmr::dmr::receiver::RATE;
use declab_pmr::dmr::{Event, Receiver, Source};
use declab_pmr::fsk4::Detector;
use declab_pmr::vocoder::ambe2::{self, FrameKind};
use declab_pmr::vocoder::{Ambe2Decoder, Upsample6};

/// Lines kept on the card.
const RECENT: usize = 8;
const SYMBOL_POINTS: usize = 528;
/// A call with no voice for this long has ended (its terminator was lost).
const CALL_TIMEOUT_S: f64 = 1.0;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "dmr",
            name: "DMR",
            family: Family::Voice,
            standard: "ETSI TS 102 361-1/-2/-4",
            summary: "DMR Tier I/II/III: calls, talkgroups, trunking control channel",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // The receiver filters to ±4 kHz itself; the host only needs to keep
        // the neighbours out.
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: 12_500.0,
            block_len: 2_400,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Dmr::new())
    }

    fn default_scope(&self) -> &'static str {
        "Symbols"
    }
}

/// A call in progress on one slot.
struct Call {
    lc: FullLc,
    start_s: f64,
    last_s: f64,
    frames: u64,
    errors: u64,
}

struct Dmr {
    rx: Receiver,
    events: Vec<Event>,
    calls: [Option<Call>; 2],
    /// The last Tier III PDU of each kind logged, so a control channel's
    /// steady repeats are logged once.
    last_csbk: Vec<Csbk>,
    system: Option<String>,
    /// LRRP location reports decoded.
    locations: u64,
    recent: VecDeque<(String, String)>,
    now_s: f64,
    since_report: usize,
    sym_hist: VecDeque<f32>,
    /// One vocoder per slot (each keeps its own frame history).
    vocoders: [Ambe2Decoder; 2],
    up: Upsample6,
    /// The slot being played, and when its last voice came.
    playing: Option<(u8, f64)>,
    frames_out: u64,
    pcm8: Vec<f32>,
    pcm48: Vec<f32>,
}

/// Silence after which the slot being played lets go of the speaker.
const PLAY_HOLD_S: f64 = 1.0;

/// A call in a few characters, for the headline: "TG 2350 ← 2341234".
fn short(lc: &FullLc) -> String {
    match lc.call() {
        Some((src, dst, true, _)) => format!("TG {dst} ← {src}"),
        Some((src, dst, false, _)) => format!("{src} → {dst}"),
        None => lc.to_string(),
    }
}

fn slot_name(s: u8) -> &'static str {
    if s == 0 { "TS1" } else { "TS2" }
}

impl Dmr {
    fn new() -> Self {
        let mut rx = Receiver::new(Detector::Discriminator);
        rx.enable_tap(true);
        Dmr {
            rx,
            events: Vec::new(),
            calls: [None, None],
            last_csbk: Vec::new(),
            system: None,
            locations: 0,
            recent: VecDeque::new(),
            now_s: 0.0,
            since_report: 0,
            sym_hist: VecDeque::new(),
            vocoders: [Ambe2Decoder::new(), Ambe2Decoder::new()],
            up: Upsample6::new(),
            playing: None,
            frames_out: 0,
            pcm8: Vec::with_capacity(3 * ambe2::FRAME),
            pcm48: Vec::with_capacity(18 * ambe2::FRAME),
        }
    }

    /// One voice burst's three frames: to the speaker if this slot is the
    /// one being played (or nothing is).
    fn play(&mut self, slot: u8, frames: &[declab_pmr::dmr::ambe::AmbeFrame; 3], out: &mut Sink) {
        let s = slot as usize;
        let encrypted = self.calls[s]
            .as_ref()
            .is_some_and(|c| c.lc.call().is_some_and(|x| x.3.privacy()));
        match self.playing {
            Some((p, t)) if p != slot && self.now_s - t < PLAY_HOLD_S => return,
            Some((p, _)) if p == slot => {}
            _ => {
                self.vocoders[s].reset();
                self.up.reset();
            }
        }
        if encrypted {
            return;
        }
        self.playing = Some((slot, self.now_s));
        self.pcm8.clear();
        let mut buf = [0f32; ambe2::FRAME];
        for f in frames {
            let kind = self.vocoders[s].decode(&f.bits, f.data_errors, &mut buf);
            if kind != FrameKind::Erasure {
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

    fn note(&mut self, label: String, value: String, out: &mut Sink) {
        out.log(format!("{label}: {value}"));
        self.recent.push_front((label, value));
        self.recent.truncate(RECENT);
    }

    fn start_call(&mut self, slot: u8, lc: FullLc, out: &mut Sink) {
        let s = slot as usize;
        if self.calls[s]
            .as_ref()
            .is_some_and(|c| c.lc.call() == lc.call())
        {
            return;
        }
        if lc.call().is_none() {
            return;
        }
        self.end_call(slot, out);
        self.note(slot_name(slot).into(), lc.to_string(), out);
        self.calls[s] = Some(Call {
            lc,
            start_s: self.now_s,
            last_s: self.now_s,
            frames: 0,
            errors: 0,
        });
    }

    fn end_call(&mut self, slot: u8, out: &mut Sink) {
        if self.playing.is_some_and(|(p, _)| p == slot) {
            self.playing = None;
        }
        if let Some(c) = self.calls[slot as usize].take() {
            let ber = 100.0 * c.errors as f64 / (c.frames * 46).max(1) as f64;
            out.log(format!(
                "{}: end of {} after {:.1} s, {} voice frames, BER {ber:.2} %",
                slot_name(slot),
                c.lc,
                c.last_s - c.start_s,
                c.frames
            ));
        }
    }

    fn csbk(&mut self, slot: u8, csbk: Csbk, out: &mut Sink) {
        // Grants are news every time; for the rest, only a change is.
        let same_kind = |a: &Csbk, b: &Csbk| std::mem::discriminant(a) == std::mem::discriminant(b);
        let grant = matches!(csbk, Csbk::Grant { .. });
        if !grant {
            if self.last_csbk.contains(&csbk) {
                return;
            }
            self.last_csbk.retain(|c| !same_kind(c, &csbk));
            self.last_csbk.push(csbk);
        }
        self.note(slot_name(slot).into(), csbk.to_string(), out);
    }

    /// A data packet: LRRP locations to the card and log, the rest a line.
    fn packet(&mut self, slot: u8, p: &Packet, out: &mut Sink) {
        let h = &p.header;
        let to = if h.group {
            format!("TG {}", h.dst)
        } else {
            h.dst.to_string()
        };
        let crc = if p.crc_ok { "" } else { " (CRC failed)" };
        match lrrp::udp(p) {
            Some(u) if u.sport == lrrp::LRRP_PORT || u.dport == lrrp::LRRP_PORT => {
                match Lrrp::parse(&u.payload) {
                    Some(l) => {
                        self.locations += 1;
                        self.note(
                            format!("{} {}", slot_name(slot), h.src),
                            format!("{l}{crc}"),
                            out,
                        );
                    }
                    None => out.log(format!(
                        "{}: LRRP from {} unreadable{crc}",
                        slot_name(slot),
                        h.src
                    )),
                }
            }
            Some(u) => {
                let ip = |a: [u8; 4]| format!("{}.{}.{}.{}", a[0], a[1], a[2], a[3]);
                out.log(format!(
                    "{}: UDP {}:{} → {}:{}, {} bytes, from {} to {to}{crc}",
                    slot_name(slot),
                    ip(u.src),
                    u.sport,
                    ip(u.dst),
                    u.dport,
                    u.payload.len(),
                    h.src
                ));
            }
            None => out.log(format!(
                "{}: data packet SAP {} {:?}, {} bytes, from {} to {to}{crc}",
                slot_name(slot),
                h.sap,
                h.format,
                p.data.len(),
                h.src
            )),
        }
    }

    fn take_events(&mut self, out: &mut Sink) {
        let events = std::mem::take(&mut self.events);
        for e in events {
            match e {
                Event::VoiceHeader { slot, lc, .. } | Event::EmbeddedLc { slot, lc } => {
                    self.start_call(slot, lc, out)
                }
                Event::Terminator { slot, .. } => self.end_call(slot, out),
                Event::Voice { slot, frames, .. } => {
                    self.play(slot, &frames, out);
                    if let Some(c) = &mut self.calls[slot as usize] {
                        c.frames += 3;
                        c.errors += frames.iter().map(|f| u64::from(f.errors)).sum::<u64>();
                        c.last_s = self.now_s;
                    }
                }
                Event::Csbk { slot, csbk, .. } => self.csbk(slot, csbk, out),
                Event::ShortLc(slc) => {
                    if matches!(
                        slc,
                        ShortLc::ControlParms { .. } | ShortLc::PayloadParms { .. }
                    ) {
                        // The slot counter ticks; the system does not.
                        let text = slc.to_string();
                        let text = text
                            .split(", slot counter")
                            .next()
                            .unwrap_or("")
                            .to_string();
                        if self.system.as_deref() != Some(&text) {
                            out.log(format!("system: {text}"));
                            self.system = Some(text);
                        }
                    }
                }
                Event::Data {
                    slot, data_type, ..
                } => out.log(format!("{}: data burst {data_type:?}", slot_name(slot))),
                Event::Mbc { slot, header, .. } => {
                    out.log(format!("{}: MBC {}", slot_name(slot), Csbk::parse(&header)));
                }
                Event::Idle { .. } | Event::DataHeader { .. } => {}
                Event::Packet { slot, packet } => self.packet(slot, &packet, out),
            }
        }
        for slot in 0..2u8 {
            if self.calls[slot as usize]
                .as_ref()
                .is_some_and(|c| self.now_s - c.last_s > CALL_TIMEOUT_S)
            {
                self.end_call(slot, out);
            }
        }
    }

    fn report(&mut self, out: &mut Sink) {
        // Rust note: a copy of the stats, so `self.rx` is free to lend its
        // tap mutably below.
        let st = self.rx.stats.clone();
        let source = self.rx.locked();
        out.lock = match source {
            None => Lock::Searching,
            Some(_) if self.calls.iter().any(Option::is_some) || st.csbks > 0 => Lock::Locked,
            Some(_) => Lock::Synced,
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
        let src = match source {
            Some(Source::Bs) => "repeater",
            Some(Source::Ms) => "radio → repeater",
            Some(Source::Ts1 | Source::Ts2) => "direct mode",
            None => "searching",
        };
        let headline = match &self.calls {
            [None, None] if st.csbks > 0 && source.is_some() => "control channel".to_string(),
            [None, None] => src.to_string(),
            calls => calls
                .iter()
                .enumerate()
                .filter_map(|(i, c)| {
                    c.as_ref()
                        .map(|c| format!("{} {}", slot_name(i as u8), short(&c.lc)))
                })
                .collect::<Vec<_>>()
                .join(" · "),
        };
        let pair = |k: &str, v: String| (k.to_string(), v);
        let mut text: Vec<(String, String)> = Vec::new();
        if let Some(s) = &self.system {
            text.push(("system".into(), s.clone()));
        }
        text.extend(self.recent.iter().cloned());
        out.card = Some(Card {
            title: format!("DMR · {src}"),
            headline,
            gauge: source.map(|_| Gauge {
                label: "eye SNR".into(),
                value_db: st.eye_snr_db,
                // Voice holds to about 5 dB C/N, an eye SNR near 8 dB.
                need_db: Some(8.0),
                need_label: "voice".into(),
                range: (0.0, 30.0),
            }),
            details: vec![
                pair(
                    "colour code",
                    st.colour_code.map_or("–".into(), |c| c.to_string()),
                ),
                pair("offset", format!("{:+.0} Hz", st.offset_hz)),
                pair(
                    "voice frames",
                    format!("{} (BER {ber:.2} %)", st.voice_frames),
                ),
                pair("CSBKs", format!("{} ({} CRC fails)", st.csbks, st.crc_fail)),
                pair("LRRP locations", self.locations.to_string()),
                pair("bursts", format!("{} ({} bad)", st.bursts, st.bursts_bad)),
                pair("sync", format!("{:.2}", st.sync_quality)),
                pair(
                    "playing",
                    match self.playing {
                        Some((p, _)) => {
                            format!("{} ({} frames spoken)", slot_name(p), self.frames_out)
                        }
                        None => format!("– ({} frames spoken)", self.frames_out),
                    },
                ),
            ],
            text,
        });
    }
}

impl Decoder for Dmr {
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
        self.calls = [None, None];
        self.playing = None;
        self.sym_hist.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_pmr::dmr::pdu::{GrantKind, ServiceOptions, SysCode};
    use declab_pmr::dmr::sim::{self, Repeater};

    #[test]
    fn logs_a_call_and_a_grant() {
        let sys = SysCode((0b01 << 14) | (12 << 7) | (4 << 2));
        let mut r = Repeater::new(
            1,
            ShortLc::ControlParms {
                syscode14: sys.0 >> 2,
                reg: false,
                counter: 5,
            },
        );
        let lc = FullLc::GroupVoice {
            opts: ServiceOptions(0),
            group: 91,
            source: 2_345_678,
        };
        let grant = Csbk::Grant {
            kind: GrantKind::TalkgroupVoice,
            lpcn: 4,
            slot: 1,
            late_entry: false,
            emergency: false,
            offset: false,
            target: 91,
            source: 2_345_678,
        };
        let idle = sim::idle_burst(Source::Bs, 1);
        r.push(&idle);
        r.push(&idle);
        r.voice_call(&lc, 1, |rep| sim::csbk_burst(Source::Bs, rep.cc, &grant));
        let mut iq = sim::modulate(&r.bits, RATE, 0.05);
        sim::add_noise(&mut iq, RATE, 20.0, 0.0, 3);
        let mut d = Factory.create();
        let mut log = Vec::new();
        let tuning = Tuning {
            center_hz: 0.0,
            sample_rate: RATE,
            generation: 0,
        };
        let mut card = None;
        let mut sound = Vec::new();
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
            for a in sink.audio {
                assert_eq!((a.sample_rate, a.channels), (48_000, 1));
                sound.extend(a.samples);
            }
        }
        assert!(
            log.iter()
                .any(|l| l == "TS1: group call TG 91 from 2345678"),
            "{log:#?}"
        );
        assert!(
            log.iter()
                .any(|l| l.starts_with("TS1: end of group call TG 91")
                    && l.contains("18 voice frames")),
            "{log:#?}"
        );
        assert!(
            log.iter()
                .any(|l| l.starts_with("TS2: TV_GRANT 2345678 → TG 91 on LPCN 4 TS2")),
            "{log:#?}"
        );
        assert!(
            log.iter()
                .any(|l| l.starts_with("system: C_SYS_Parms Small net 12 site 5")),
            "{log:#?}"
        );
        assert!(card.is_some());
        // 18 voice frames of 20 ms, at 48 kHz, and not silent.
        assert_eq!(sound.len(), 18 * 960);
        assert!(sound.iter().all(|x| x.is_finite()));
        assert!(sound.iter().any(|x| x.abs() > 1e-3));
    }
}
