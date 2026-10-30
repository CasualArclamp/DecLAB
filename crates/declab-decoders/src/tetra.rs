// made by CasualArclamp
//! TETRA (ETSI EN 300 392-2) downlink, from `crates/declab-pmr`: the cell
//! (MCC, MNC, colour code, location area, main carrier), group and
//! individual calls (who, to whom, on which timeslot), transmit grants,
//! status and short data, and speech through the ACELP codec when its
//! library is installed (see `declab_pmr::vocoder::acelp`).
//!
//! Encrypted signalling is shown as such; speech is muted when the call's
//! signalling was encrypted or end-to-end encryption was granted, and when
//! traffic turns up on a cell offering air-interface encryption with no
//! clear call set-up heard for it.

use std::collections::VecDeque;
use crate::heard::HeardList;

use declab_api::{
    Audio, Block, Card, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    NearbyCell, Sink,
};
use declab_pmr::tetra::mac::{self, Address, ChanAlloc, Neighbour, Signal, Sync, SysInfo};
use declab_pmr::tetra::receiver::{RATE, SNR_GATE_DB};
use declab_pmr::tetra::{Event, Receiver};
use declab_pmr::vocoder::Upsample6;
use declab_pmr::vocoder::acelp::{Acelp, SLOT_SAMPLES};

const RECENT: usize = 8;
const POINTS: usize = 600;
/// A call with no traffic or signalling for this long has ended.
const CALL_TIMEOUT_S: f64 = 2.0;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "tetra",
            name: "TETRA",
            family: Family::Voice,
            standard: "ETSI EN 300 392-2 (π/4-DQPSK downlink), EN 300 395-2 (ACELP)",
            summary: "TETRA cell, calls, status and SDS, ACELP speech",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: 25_000.0,
            block_len: 7_200,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Tetra::new())
    }

    fn default_scope(&self) -> &'static str {
        // The same name as the other voice modes' scope, so the Digital
        // voice decoder's pick shows TETRA's constellation too.
        "Symbols"
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(crate::gate_panel::GatePanel {
            gates: vec![gate()],
        }))
    }
}

/// TETRA's SNR gate for the settings panel, with its measured figures.
#[cfg(feature = "ui")]
pub(crate) fn gate() -> crate::gate_panel::Gate {
    crate::gate_panel::Gate {
        label: "TETRA",
        key: "snr_gate",
        db: SNR_GATE_DB,
        noise_max: 0.0,
        real_median: 9.0,
    }
}

struct Call {
    id: u16,
    /// For the heard list: "TG 9001", "private 4000123".
    group: String,
    to: Option<u32>,
    from: Option<u32>,
    usage_marker: Option<u8>,
    timeslot: Option<u8>,
    encrypted: bool,
    start_s: f64,
    last_s: f64,
    slots: u64,
}

struct Tetra {
    rx: Receiver,
    events: Vec<Event>,
    sync: Option<Sync>,
    sysinfo: Option<SysInfo>,
    calls: Vec<Call>,
    /// Neighbour cells from D-NWRK-BROADCAST, by cell identifier.
    neighbours: Vec<Neighbour>,
    /// The timeslot whose speech is being played, and when it last spoke.
    playing: Option<(u8, f64)>,
    codec: Result<Acelp, String>,
    codec_noted: bool,
    recent: VecDeque<(String, String)>,
    last_other: Vec<String>,
    now_s: f64,
    heard: HeardList,
    since_report: usize,
    points: VecDeque<Iq>,
    up: Upsample6,
    pcm8: [f32; SLOT_SAMPLES],
    pcm48: Vec<f32>,
    slots_spoken: u64,
}

fn ssi(a: Option<u32>) -> String {
    a.map_or("?".into(), |s| s.to_string())
}

impl Tetra {
    fn new() -> Self {
        let mut rx = Receiver::new();
        rx.set_tap(true);
        Tetra {
            rx,
            events: Vec::new(),
            sync: None,
            sysinfo: None,
            calls: Vec::new(),
            neighbours: Vec::new(),
            playing: None,
            codec: Acelp::load(),
            codec_noted: false,
            recent: VecDeque::new(),
            last_other: Vec::new(),
            now_s: 0.0,
            heard: HeardList::default(),
            since_report: 0,
            points: VecDeque::new(),
            up: Upsample6::new(),
            pcm8: [0.0; SLOT_SAMPLES],
            pcm48: Vec::with_capacity(6 * SLOT_SAMPLES),
            slots_spoken: 0,
        }
    }

    fn note(&mut self, value: String, out: &mut Sink) {
        out.log(value.clone());
        self.recent.push_front(("TETRA".into(), value));
        self.recent.truncate(RECENT);
    }

    /// A call's speech to the heard list: its group, from whoever has the
    /// floor.
    fn heard_talk(&mut self, id: u16, speech_s: f64) {
        let Some(c) = self.calls.iter().find(|c| c.id == id) else {
            return;
        };
        let slot = c.timeslot.map_or(String::new(), |t| format!("TS{}", t + 1));
        let from = c.from.map(|f| f.to_string());
        self.heard.talk(
            self.now_s,
            &c.group,
            &slot,
            from.as_deref(),
            c.encrypted,
            speech_s,
        );
    }

    /// The neighbours to the heard list, once SYSINFO gives the band.
    fn nearby(&mut self) {
        let Some(si) = &self.sysinfo else { return };
        let sync = &self.sync;
        let cells = self
            .neighbours
            .iter()
            .map(|n| {
                let mut detail = n.la.map_or(String::new(), |la| format!("LA {la}"));
                let mut add = |s: String| {
                    if !detail.is_empty() {
                        detail.push_str(" · ");
                    }
                    detail.push_str(&s);
                };
                // Another network's cell (the same is left out, table 18.64).
                if let Some(mcc) = n.mcc.filter(|&m| sync.as_ref().is_none_or(|s| s.mcc != m)) {
                    add(format!("MCC {mcc}"));
                }
                if let Some(mnc) = n.mnc.filter(|&m| sync.as_ref().is_none_or(|s| s.mnc != m)) {
                    add(format!("MNC {mnc}"));
                }
                if !n.synchronized {
                    add("not synchronised".into());
                }
                if let Some(l) = ["", "load low", "load medium", "load high"].get(n.load as usize)
                    && !l.is_empty()
                {
                    add((*l).into());
                }
                NearbyCell {
                    name: format!("cell {}", n.cell),
                    freq_hz: n.carrier_hz(si),
                    detail,
                }
            })
            .collect();
        self.heard.set_cells(cells);
    }

    fn call_mut(&mut self, id: u16) -> Option<&mut Call> {
        self.calls.iter_mut().find(|c| c.id == id)
    }

    fn end_call(&mut self, id: u16, why: &str, out: &mut Sink) {
        if let Some(i) = self.calls.iter().position(|c| c.id == id) {
            let c = self.calls.remove(i);
            self.heard.end(&c.group);
            out.log(format!(
                "end of call {} to {} ({why}) after {:.1} s, {} traffic slots",
                c.id,
                ssi(c.to),
                c.last_s - c.start_s,
                c.slots
            ));
            if self.playing.is_some_and(|p| Some(p.0) == c.timeslot) {
                self.playing = None;
            }
        }
    }

    fn signal(
        &mut self,
        tn: u8,
        address: Address,
        encrypted: bool,
        chan: Option<ChanAlloc>,
        signal: Option<Signal>,
        out: &mut Sink,
    ) {
        let to = address.ssi();
        let Some(signal) = signal else {
            if encrypted {
                if let Some(t) = to {
                    self.heard
                        .seen(self.now_s, &t.to_string(), "encrypted signalling");
                }
                let s = format!("encrypted signalling to {}", ssi(to));
                if !self.last_other.contains(&s) {
                    self.last_other.push(s.clone());
                    self.last_other.truncate(16);
                    self.note(s, out);
                }
            }
            return;
        };
        let now = self.now_s;
        match signal {
            Signal::Setup {
                call,
                calling,
                service,
                ..
            } => {
                let usage_marker = match address {
                    Address::SsiUsageMarker(_, u) => Some(u),
                    _ => None,
                };
                // The allocated timeslot: the lowest set in the bitmap
                // (bit 3 = timeslot 1), else the one this came on.
                let timeslot = chan
                    .filter(|c| c.timeslots != 0)
                    .map(|c| {
                        (0..4u8)
                            .find(|t| c.timeslots >> (3 - t) & 1 == 1)
                            .unwrap_or(tn)
                    })
                    .or(Some(tn));
                if let Some(c) = self.call_mut(call) {
                    c.last_s = now;
                    c.usage_marker = usage_marker.or(c.usage_marker);
                    c.timeslot = timeslot;
                    return;
                }
                let ts = timeslot.map_or(String::new(), |t| format!(" on TS{}", t + 1));
                self.note(
                    format!("call {call} to {} from {}{ts}", ssi(to), ssi(calling)),
                    out,
                );
                // Basic service information (clause 14.8.2): bits 3–2 the
                // communication type, 00 point-to-point.
                let group = match (service >> 2 & 3, to) {
                    (0, Some(t)) => format!("private {t}"),
                    (_, Some(t)) => format!("TG {t}"),
                    (_, None) => format!("call {call}"),
                };
                let slot = timeslot.map_or(String::new(), |t| format!("TS{}", t + 1));
                self.heard.start(
                    now,
                    &group,
                    &slot,
                    calling.map(|c| c.to_string()).as_deref(),
                    encrypted,
                );
                self.calls.push(Call {
                    id: call,
                    group,
                    to,
                    from: calling,
                    usage_marker,
                    timeslot,
                    encrypted,
                    start_s: now,
                    last_s: now,
                    slots: 0,
                });
            }
            Signal::TxGranted {
                call,
                encrypted: e2e,
                party,
                ..
            } => {
                // A new talker takes the floor: a transmission of its own.
                let mut started = None;
                if let Some(c) = self.call_mut(call) {
                    c.last_s = now;
                    c.encrypted |= e2e || encrypted;
                    if party.is_some() && party != c.from {
                        c.from = party;
                        started = Some((c.group.clone(), c.timeslot, c.encrypted));
                    }
                }
                if let (Some((group, ts, enc)), Some(p)) = (started, party) {
                    let slot = ts.map_or(String::new(), |t| format!("TS{}", t + 1));
                    self.heard.start(now, &group, &slot, Some(&p.to_string()), enc);
                }
                let tag = if e2e { " (end-to-end encrypted)" } else { "" };
                self.note(format!("call {call}: {} talking{tag}", ssi(party)), out);
            }
            Signal::Connect { call } => {
                if let Some(c) = self.call_mut(call) {
                    c.last_s = now;
                }
            }
            Signal::TxCeased { call } => {
                if let Some(c) = self.call_mut(call) {
                    c.last_s = now;
                }
            }
            Signal::NwrkBroadcast { neighbours } => {
                // A cell sends its list in parts: each neighbour replaces
                // the one with its identifier.
                for n in neighbours {
                    match self.neighbours.iter_mut().find(|m| m.cell == n.cell) {
                        Some(m) => *m = n,
                        None => self.neighbours.push(n),
                    }
                }
                self.neighbours.sort_by_key(|n| n.cell);
                self.nearby();
            }
            Signal::Release { call, cause } => {
                self.end_call(call, &format!("released, cause {cause}"), out);
            }
            Signal::Status { from, status } => {
                if let Some(f) = from {
                    self.heard.seen(now, &f.to_string(), "status");
                }
                self.note(
                    format!("status {status:#06x} from {} to {}", ssi(from), ssi(to)),
                    out,
                );
            }
            Signal::Sds {
                from,
                kind,
                data,
                bits,
            } => {
                if let Some(f) = from {
                    self.heard.seen(now, &f.to_string(), "SDS");
                }
                let body = sds_text(kind, &data, bits);
                self.note(
                    format!("SDS from {} to {}: {body}", ssi(from), ssi(to)),
                    out,
                );
            }
            other => {
                let s = format!("{} to {}", mac::signal_name(&other), ssi(to));
                if !self.last_other.contains(&s) {
                    self.last_other.push(s.clone());
                    self.last_other.truncate(16);
                    out.log(s);
                }
            }
        }
    }

    fn traffic(&mut self, tn: u8, usage_marker: Option<u8>, llr: &[f32], out: &mut Sink) {
        let now = self.now_s;
        let call_i = self.calls.iter().position(|c| {
            c.timeslot == Some(tn) || (usage_marker.is_some() && c.usage_marker == usage_marker)
        });
        if let Some(i) = call_i {
            self.calls[i].slots += 1;
            self.calls[i].last_s = now;
            // A slot carries two 30 ms frames.
            self.heard_talk(self.calls[i].id, 0.06);
        }
        // One timeslot is played: the first to speak, until it falls quiet.
        match self.playing {
            Some((p, _)) if p != tn => return,
            _ => self.playing = Some((tn, now)),
        }
        let cell_aie = self.sysinfo.is_some_and(|s| s.encryption());
        let muted = match call_i {
            Some(i) => self.calls[i].encrypted,
            None => cell_aie,
        };
        if muted {
            return;
        }
        let codec = match &mut self.codec {
            Ok(c) => c,
            Err(e) => {
                if !self.codec_noted {
                    self.codec_noted = true;
                    let msg = format!("speech not played: {e}");
                    out.log(msg);
                }
                return;
            }
        };
        codec.decode(llr, &mut self.pcm8);
        self.slots_spoken += 1;
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
                Event::Sync(s) => {
                    let new_cell = self
                        .sync
                        .is_none_or(|o| (o.mcc, o.mnc, o.colour) != (s.mcc, s.mnc, s.colour));
                    if new_cell {
                        self.note(
                            format!("cell MCC {} MNC {} colour code {}", s.mcc, s.mnc, s.colour),
                            out,
                        );
                    }
                    self.sync = Some(s);
                }
                Event::SysInfo(si) => {
                    if self
                        .sysinfo
                        .is_none_or(|o| o.la != si.la || o.main_carrier != si.main_carrier)
                    {
                        let enc = if si.encryption() {
                            ", air-interface encryption available"
                        } else {
                            ""
                        };
                        self.note(
                            format!(
                                "LA {}, main carrier {:.4} MHz{enc}",
                                si.la,
                                si.main_carrier_hz() / 1e6
                            ),
                            out,
                        );
                    }
                    let first = self.sysinfo.is_none();
                    self.sysinfo = Some(si);
                    if first {
                        self.nearby();
                    }
                }
                Event::Signal {
                    tn,
                    address,
                    encrypted,
                    chan,
                    signal,
                } => self.signal(tn, address, encrypted, chan, signal, out),
                Event::Traffic {
                    tn,
                    usage_marker,
                    llr,
                    ..
                } => self.traffic(tn, usage_marker, &llr, out),
            }
        }
        if self.playing.is_some_and(|(_, t)| self.now_s - t > 0.5) {
            self.playing = None;
        }
        let stale: Vec<u16> = self
            .calls
            .iter()
            .filter(|c| self.now_s - c.last_s > CALL_TIMEOUT_S)
            .map(|c| c.id)
            .collect();
        for id in stale {
            self.end_call(id, "timed out", out);
        }
    }

    fn report(&mut self, out: &mut Sink) {
        let st = self.rx.stats.clone();
        out.lock = match (st.locked, self.calls.is_empty()) {
            (false, _) => Lock::Searching,
            (true, false) => Lock::Locked,
            (true, true) => Lock::Synced,
        };
        // The symbols after the phase track and π/4 removal, scaled to their
        // RMS: four QPSK points.
        self.points.extend(self.rx.take_tap());
        let over = self.points.len().saturating_sub(POINTS);
        self.points.drain(..over);
        let rms = (self.points.iter().map(|p| p.norm_sqr()).sum::<f32>()
            / self.points.len().max(1) as f32)
            .sqrt()
            .max(1e-12);
        let pts: Vec<Iq> = self.points.iter().map(|p| p / (rms * 1.3)).collect();
        out.scopes.push(("Symbols", pts));
        out.measure("Eye SNR", f64::from(st.eye_snr_db), "dB");
        let headline = match (self.calls.first(), &self.sync, st.locked) {
            (Some(c), _, _) => {
                let enc = if c.encrypted { " 🔒" } else { "" };
                format!("{} ← {}{enc}", ssi(c.to), ssi(c.from))
            }
            (None, Some(s), true) => format!("cell {} {}", s.mcc, s.mnc),
            (None, _, true) => "synchronised".into(),
            (None, _, false) => "searching".into(),
        };
        let pair = |k: &str, v: String| (k.to_string(), v);
        let mut details = Vec::new();
        if let Some(s) = &self.sync {
            details.push(pair(
                "cell",
                format!("MCC {} MNC {} CC {}", s.mcc, s.mnc, s.colour),
            ));
        }
        if let Some(si) = &self.sysinfo {
            details.push(pair("LA", si.la.to_string()));
            details.push(pair(
                "main carrier",
                format!("{:.4} MHz", si.main_carrier_hz() / 1e6),
            ));
            details.push(pair(
                "encryption",
                if si.encryption() {
                    "available on cell"
                } else {
                    "none"
                }
                .into(),
            ));
        }
        details.extend([
            pair("offset", format!("{:+.0} Hz", st.offset_hz)),
            pair(
                "slot",
                format!(
                    "TS{} frame {} multiframe {}",
                    st.tn + 1,
                    st.frame,
                    st.multiframe
                ),
            ),
            pair(
                "slots",
                format!(
                    "{} ({} bad, {} re-aligned)",
                    st.slots, st.slots_bad, st.resyncs
                ),
            ),
            pair(
                "signalling",
                format!("{} blocks ({} CRC fails)", st.blocks_ok, st.crc_fail),
            ),
            pair(
                "SNR gate",
                format!("{:.0} dB ({} turned away)", self.rx.snr_gate_db, st.gated),
            ),
            pair(
                "speech",
                match &self.codec {
                    Ok(_) => format!("{} slots played", self.slots_spoken),
                    Err(_) => "codec not installed".into(),
                },
            ),
        ]);
        let mut text: Vec<(String, String)> = self
            .calls
            .iter()
            .map(|c| {
                let ts = c
                    .timeslot
                    .map_or(String::new(), |t| format!(" TS{}", t + 1));
                (
                    format!("call {}", c.id),
                    format!("{} ← {}{ts}", ssi(c.to), ssi(c.from)),
                )
            })
            .collect();
        text.extend(self.recent.iter().cloned());
        out.card = Some(Card {
            title: "TETRA".into(),
            headline,
            gauge: st.locked.then(|| Gauge {
                label: "eye SNR".into(),
                value_db: st.eye_snr_db,
                need_db: Some(6.0),
                need_label: "speech".into(),
                range: (-5.0, 30.0),
            }),
            details,
            text,
        });
    }
}

/// An SDS's user data as text where it looks like a text message (protocol
/// identifier 0x82 with a printable body), else hex.
fn sds_text(kind: u8, data: &[u8], bits: usize) -> String {
    let n = bits.div_ceil(8).min(data.len());
    let d = &data[..n];
    if kind == 3 && d.len() > 1 {
        let body = &d[1..];
        let printable = body.iter().filter(|b| (0x20..0x7f).contains(*b)).count();
        if d[0] == 0x82 && printable * 3 >= body.len() * 2 {
            let t: String = body
                .iter()
                .filter(|b| (0x20..0x7f).contains(*b))
                .map(|&b| b as char)
                .collect();
            return format!("\"{t}\"");
        }
    }
    let hex: Vec<String> = d.iter().map(|b| format!("{b:02x}")).collect();
    hex.join(" ")
}

impl Decoder for Tetra {
    fn command(&mut self, cmd: declab_api::Command, out: &mut Sink) {
        if let declab_api::Command::Set { key, value } = cmd {
            match (key.as_str(), value.parse::<f32>()) {
                ("snr_gate", Ok(db)) => self.rx.snr_gate_db = db.clamp(0.0, 20.0),
                _ => out.log(format!("tetra: no setting {key:?} = {value:?}")),
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
        self.calls.clear();
        self.neighbours.clear();
        self.sysinfo = None;
        self.playing = None;
        self.points.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_pmr::tetra::sim::{self, Cell};

    #[test]
    fn logs_the_cell_a_call_and_short_data() {
        let mut cell = Cell::new(234, 1234, 17);
        let (bits, traffic) = sim::call(&mut cell, 9001, 4_000_123, 20);
        let mut iq = sim::modulate(&bits, RATE, 0.1);
        sim::add_noise(&mut iq, RATE, 15.0, 300.0, 4);
        let mut d = Factory.create();
        let tuning = Tuning {
            center_hz: 0.0,
            sample_rate: RATE,
            generation: 0,
        };
        let (mut log, mut sound) = (Vec::new(), 0usize);
        for (n, chunk) in iq.chunks(7_200).enumerate() {
            let block = Block {
                iq: chunk,
                tuning,
                first_sample: n as u64 * 7_200,
                dropped_before: 0,
                live: false,
            };
            let mut sink = Sink::default();
            d.process(&block, &mut sink);
            log.append(&mut sink.log);
            sound += sink.audio.iter().map(|a| a.samples.len()).sum::<usize>();
        }
        // Speech when the codec library is there (DECLAB_ACELP or next to
        // the test executable): every traffic slot, 60 ms at 48 kHz.
        match Acelp::load() {
            Ok(_) => assert_eq!(sound, traffic as usize * 2_880, "{log:#?}"),
            Err(_) => assert!(
                log.iter().any(|l| l.starts_with("speech not played")),
                "{log:#?}"
            ),
        }
        let has = |s: &str| log.iter().any(|l| l == s);
        assert!(has("cell MCC 234 MNC 1234 colour code 17"), "{log:#?}");
        assert!(has("LA 1001, main carrier 430.8500 MHz"), "{log:#?}");
        assert!(has("call 77 to 9001 from 4000123 on TS2"), "{log:#?}");
        assert!(has("call 77: 4000123 talking"), "{log:#?}");
        assert!(
            log.iter()
                .any(|l| l.starts_with("end of call 77 to 9001 (released, cause 1)")),
            "{log:#?}"
        );
        assert!(has("SDS from 9001 to 4000123: \"ok\""), "{log:#?}");
        assert!(has("status 0x8002 from 2000 to 4000123"), "{log:#?}");
    }
}
