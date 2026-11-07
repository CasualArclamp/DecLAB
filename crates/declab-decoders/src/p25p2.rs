// made by CasualArclamp
//! P25 phase 2, from `crates/declab-pmr`: a TDMA traffic channel's two
//! slots — calls (who, to which talkgroup, how long, the voice BER) from
//! the MAC's PTT and channel-user messages, encryption from MAC_PTT and the
//! ESS, voice through the AMBE+2 vocoder at 48 kHz.
//!
//! The scrambling is found from the channel itself (the seed's WACN,
//! system and NAC are shown once it is); until then voice is counted, not
//! played. The seed can also be given: `p25p2_seed` = `WACN-SYSTEM-NAC` in
//! hex, as the phase 1 control channel's network status shows them.
//!
//! Sound: one slot at a time, the first to carry voice, held until its call
//! has been quiet a second (as DMR's two slots). Encrypted calls are logged
//! but not played.

use std::collections::VecDeque;

use crate::heard::HeardList;
use declab_api::{
    Audio, Block, Card, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    Sink,
};
use declab_pmr::dmr::ambe::AmbeFrame;
use declab_pmr::p25p2::receiver::{RATE, SNR_GATE_DB};
use declab_pmr::p25p2::{Event, MacMsg, MacOp, MacPdu, Receiver, scramble};
use declab_pmr::vocoder::ambe2::{self, Ambe2Decoder, FrameKind};
use declab_pmr::vocoder::Upsample6;

const RECENT: usize = 8;
const SYMBOL_POINTS: usize = 400;
/// A call with no voice or signalling for this long has ended.
const CALL_TIMEOUT_S: f64 = 1.5;
/// Silence after which the slot being played lets go of the speaker.
const PLAY_HOLD_S: f64 = 1.0;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "p25p2",
            name: "P25 phase 2",
            family: Family::Voice,
            standard: "TIA-102.BBAB/BBAC (layouts after OP25 and DSD-FME)",
            summary: "P25 phase 2 TDMA traffic channel: both slots, AMBE+2 speech",
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
        Box::new(P25p2::new())
    }

    fn default_scope(&self) -> &'static str {
        "Symbols"
    }
}

/// Phase 2's SNR gate for the settings panel, with its measured figures
/// (`p25p2_sweep`).
#[cfg(feature = "ui")]
pub(crate) fn gate() -> crate::gate_panel::Gate {
    crate::gate_panel::Gate {
        label: "P25 phase 2",
        key: "snr_gate",
        db: SNR_GATE_DB,
        noise_max: 5.1,
        real_median: 15.0,
    }
}

fn slot_name(ch: u8) -> &'static str {
    if ch == 0 { "TS1" } else { "TS2" }
}

#[derive(Clone)]
struct Call {
    source: u32,
    dest: u32,
    group: bool,
    encrypted: bool,
    start_s: f64,
    last_s: f64,
    frames: u64,
    errors: u64,
}

impl Call {
    fn name(&self) -> String {
        if self.group { format!("TG {}", self.dest) } else { format!("private {}", self.dest) }
    }

    fn describe(&self) -> String {
        if self.group {
            format!("group call TG {} from {}", self.dest, self.source)
        } else {
            format!("private call {} → {}", self.source, self.dest)
        }
    }

    fn short(&self) -> String {
        let lock = if self.encrypted { " 🔒" } else { "" };
        if self.group {
            format!("TG {} ← {}{lock}", self.dest, self.source)
        } else {
            format!("{} → {}{lock}", self.source, self.dest)
        }
    }
}

struct P25p2 {
    rx: Receiver,
    events: Vec<Event>,
    calls: [Option<Call>; 2],
    /// The last non-call MAC messages logged, so repeats are logged once.
    last_msgs: Vec<MacMsg>,
    recent: VecDeque<(String, String)>,
    heard: HeardList,
    now_s: f64,
    since_report: usize,
    sym_hist: VecDeque<f32>,
    vocoders: [Ambe2Decoder; 2],
    up: Upsample6,
    playing: Option<(u8, f64)>,
    frames_out: u64,
    pcm8: Vec<f32>,
    pcm48: Vec<f32>,
}

impl P25p2 {
    fn new() -> Self {
        let mut rx = Receiver::new();
        rx.enable_tap(true);
        P25p2 {
            rx,
            events: Vec::new(),
            calls: [None, None],
            last_msgs: Vec::new(),
            recent: VecDeque::new(),
            heard: HeardList::default(),
            now_s: 0.0,
            since_report: 0,
            sym_hist: VecDeque::new(),
            vocoders: [Ambe2Decoder::new(), Ambe2Decoder::new()],
            up: Upsample6::new(),
            playing: None,
            frames_out: 0,
            pcm8: Vec::with_capacity(4 * ambe2::FRAME),
            pcm48: Vec::with_capacity(24 * ambe2::FRAME),
        }
    }

    fn note(&mut self, label: String, value: String, out: &mut Sink) {
        out.log(format!("{label}: {value}"));
        self.recent.push_front((label, value));
        self.recent.truncate(RECENT);
    }

    /// A call named on slot `ch`: a new one starts (ending what was there)
    /// unless it is the call already in progress.
    fn start_call(&mut self, ch: u8, source: u32, dest: u32, group: bool, encrypted: bool, out: &mut Sink) {
        let c = ch as usize;
        if let Some(cur) = &mut self.calls[c]
            && cur.dest == dest
            && cur.group == group
            && (cur.source == source || source == 0)
        {
            cur.last_s = self.now_s;
            cur.encrypted |= encrypted;
            return;
        }
        self.end_call(ch, out);
        let call = Call {
            source,
            dest,
            group,
            encrypted,
            start_s: self.now_s,
            last_s: self.now_s,
            frames: 0,
            errors: 0,
        };
        let tag = if encrypted { " (encrypted)" } else { "" };
        self.note(slot_name(ch).into(), format!("{}{tag}", call.describe()), out);
        self.heard
            .start(self.now_s, &call.name(), slot_name(ch), Some(&source.to_string()), encrypted);
        self.calls[c] = Some(call);
    }

    fn end_call(&mut self, ch: u8, out: &mut Sink) {
        if self.playing.is_some_and(|(p, _)| p == ch) {
            self.playing = None;
        }
        if let Some(c) = self.calls[ch as usize].take() {
            self.heard.end(&c.name());
            let ber = 100.0 * c.errors as f64 / (c.frames * 46).max(1) as f64;
            out.log(format!(
                "{}: end of {} after {:.1} s, {} voice frames, BER {ber:.2} %",
                slot_name(ch),
                c.describe(),
                c.last_s - c.start_s,
                c.frames
            ));
        }
    }

    fn mac(&mut self, ch: u8, pdu: MacPdu, out: &mut Sink) {
        match pdu.op {
            MacOp::Ptt => {
                if let Some((src, grp, alg, _)) = pdu.ptt {
                    self.start_call(ch, src, u32::from(grp), true, alg != 0x80, out);
                }
            }
            MacOp::EndPtt => self.end_call(ch, out),
            MacOp::Idle => self.end_call(ch, out),
            _ => {}
        }
        for m in pdu.messages {
            match m {
                MacMsg::GroupVoice { opts, group, source } => {
                    if pdu.op != MacOp::Idle {
                        self.start_call(ch, source, u32::from(group), true, opts & 0x40 != 0, out);
                    }
                }
                MacMsg::UnitVoice { opts, target, source } => {
                    if pdu.op != MacOp::Idle {
                        self.start_call(ch, source, target, false, opts & 0x40 != 0, out);
                    }
                }
                m => {
                    if self.last_msgs.contains(&m) {
                        continue;
                    }
                    self.last_msgs.push(m.clone());
                    if self.last_msgs.len() > 32 {
                        self.last_msgs.remove(0);
                    }
                    self.note(slot_name(ch).into(), m.to_string(), out);
                }
            }
        }
    }

    fn voice(&mut self, ch: u8, frames: &[AmbeFrame], out: &mut Sink) {
        let c = ch as usize;
        let encrypted = self.calls[c].as_ref().is_some_and(|x| x.encrypted);
        if let Some(call) = &mut self.calls[c] {
            call.frames += frames.len() as u64;
            call.errors += frames.iter().map(|f| u64::from(f.errors)).sum::<u64>();
            call.last_s = self.now_s;
            let (name, src) = (call.name(), call.source.to_string());
            self.heard
                .talk(self.now_s, &name, slot_name(ch), Some(&src), encrypted, 0.02 * frames.len() as f64);
        }
        match self.playing {
            Some((p, t)) if p != ch && self.now_s - t < PLAY_HOLD_S => return,
            Some((p, _)) if p == ch => {}
            _ => {
                self.vocoders[c].reset();
                self.up.reset();
            }
        }
        if encrypted {
            return;
        }
        self.playing = Some((ch, self.now_s));
        self.pcm8.clear();
        let mut buf = [0f32; ambe2::FRAME];
        for f in frames {
            if self.vocoders[c].decode(&f.bits, f.data_errors, &mut buf) != FrameKind::Erasure {
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
                Event::Mac { ch, pdu, .. } => self.mac(ch, pdu, out),
                Event::Voice { ch, frames } => self.voice(ch, &frames, out),
                Event::Encryption { ch, algid, .. } => {
                    if let Some(c) = &mut self.calls[ch as usize]
                        && algid != 0x80
                        && !c.encrypted
                    {
                        c.encrypted = true;
                        out.log(format!("{}: encrypted (ALGID 0x{algid:02X})", slot_name(ch)));
                    }
                }
                Event::Scrambling { wacn, system, nac } => out.log(format!(
                    "scrambling found: WACN {wacn:05X}, system {system:03X}, NAC {nac:03X}"
                )),
            }
        }
        for ch in 0..2u8 {
            if self.calls[ch as usize]
                .as_ref()
                .is_some_and(|c| self.now_s - c.last_s > CALL_TIMEOUT_S)
            {
                self.end_call(ch, out);
            }
        }
    }

    fn report(&mut self, out: &mut Sink) {
        let st = self.rx.stats.clone();
        let calling = self.calls.iter().any(Option::is_some);
        out.lock = match (st.locked, calling) {
            (false, _) => Lock::Searching,
            (true, true) => Lock::Locked,
            (true, false) => Lock::Synced,
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
        let headline = if calling {
            self.calls
                .iter()
                .enumerate()
                .filter_map(|(i, c)| c.as_ref().map(|c| format!("{} {}", slot_name(i as u8), c.short())))
                .collect::<Vec<_>>()
                .join(" · ")
        } else if st.locked && st.lcch > 0 && st.acch_ok == 0 {
            "TDMA control channel".into()
        } else if st.locked {
            "traffic channel".into()
        } else {
            "searching".into()
        };
        let pair = |k: &str, v: String| (k.to_string(), v);
        let scrambling = match st.scrambling {
            Some((w, s, n)) => format!("WACN {w:05X}, system {s:03X}, NAC {n:03X}"),
            None if st.voice_held > 0 => format!("not found yet ({} voice bursts held)", st.voice_held),
            None => "not found yet".into(),
        };
        out.card = Some(Card {
            title: "P25 phase 2".into(),
            headline,
            gauge: st.locked.then(|| Gauge {
                label: "eye SNR".into(),
                value_db: st.eye_snr_db,
                need_db: Some(8.0),
                need_label: "voice".into(),
                range: (0.0, 30.0),
            }),
            details: vec![
                pair("system", scrambling),
                pair("offset", format!("{:+.0} Hz", st.offset_hz)),
                pair("voice frames", format!("{} (BER {ber:.2} %)", st.voice_frames)),
                pair(
                    "timeslots",
                    format!("{} ({} unreadable, {} CRC fails)", st.timeslots, st.bursts_bad, st.crc_fail),
                ),
                pair(
                    "sync",
                    format!("{:.2} ({:.0} dB)", st.sync_quality, st.sync_snr_db.max(-99.0)),
                ),
                pair("SNR gate", format!("{:.0} dB ({} turned away)", self.rx.snr_gate_db, st.gated)),
                pair("spoken", format!("{} frames", self.frames_out)),
            ],
            text: self.recent.iter().cloned().collect(),
        });
    }
}

/// `WACN-SYSTEM-NAC` in hex → the packed seed.
fn parse_seed(s: &str) -> Option<u64> {
    let p: Vec<&str> = s.trim().split(['-', ':', ' ', '.']).filter(|x| !x.is_empty()).collect();
    let [w, sy, n] = p.as_slice() else { return None };
    let h = |x: &str| u32::from_str_radix(x.trim_start_matches("0x"), 16).ok();
    Some(scramble::seed(h(w)?, h(sy)? as u16, h(n)? as u16))
}

impl Decoder for P25p2 {
    fn command(&mut self, cmd: declab_api::Command, out: &mut Sink) {
        if let declab_api::Command::Set { key, value } = cmd {
            match key.as_str() {
                "snr_gate" => match value.parse::<f32>() {
                    Ok(db) => self.rx.snr_gate_db = db.clamp(0.0, 20.0),
                    Err(_) => out.log(format!("p25p2: bad gate {value:?}")),
                },
                "p25p2_seed" => match parse_seed(&value) {
                    Some(s) => self.rx.set_seed(s),
                    None => out.log(format!("p25p2: seed {value:?} is not WACN-SYSTEM-NAC in hex")),
                },
                _ => out.log(format!("p25p2: no setting {key:?}")),
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
            self.heard.report(self.now_s, out);
        }
    }

    fn reset(&mut self) {
        self.rx.reset();
        self.heard.clear();
        self.calls = [None, None];
        self.playing = None;
        self.sym_hist.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_pmr::p25p2::sim::{self, Call as SimCall};

    #[test]
    fn logs_both_slots_and_plays_one() {
        let seed = scramble::seed(0xBEE00, 0x3A1, 0x293);
        let calls = [
            Some(SimCall { group: 4001, source: 1_234_567, algid: 0x80 }),
            Some(SimCall { group: 77, source: 7_654_321, algid: 0x84 }),
        ];
        let (dibits, frames) = sim::channel(seed, calls, 6);
        let mut iq = sim::modulate(&dibits, RATE, 0.1);
        sim::add_noise(&mut iq, RATE, 20.0, -200.0, 5);
        let mut d = Factory.create();
        let tuning = Tuning { center_hz: 0.0, sample_rate: RATE, generation: 0 };
        let (mut log, mut sound) = (Vec::new(), 0usize);
        for (n, chunk) in iq.chunks(2_400).enumerate() {
            let mut sink = Sink::default();
            let block = Block { iq: chunk, tuning, first_sample: n as u64 * 2_400, dropped_before: 0, live: false };
            d.process(&block, &mut sink);
            log.append(&mut sink.log);
            sound += sink.audio.iter().map(|a| a.samples.len()).sum::<usize>();
        }
        assert!(log.iter().any(|l| l == "scrambling found: WACN BEE00, system 3A1, NAC 293"), "{log:#?}");
        assert!(log.iter().any(|l| l == "TS1: group call TG 4001 from 1234567"), "{log:#?}");
        assert!(log.iter().any(|l| l == "TS2: group call TG 77 from 7654321 (encrypted)"), "{log:#?}");
        assert!(log.iter().any(|l| l.starts_with("TS1: end of group call TG 4001")), "{log:#?}");
        // TS1's voice only (TS2 is encrypted): half the frames, 20 ms each.
        assert_eq!(sound, frames as usize / 2 * 960, "{log:#?}");
    }

    #[test]
    fn seeds_parse() {
        assert_eq!(parse_seed("BEE00-3A1-293"), Some(scramble::seed(0xBEE00, 0x3A1, 0x293)));
        assert_eq!(parse_seed("nope"), None);
    }
}
