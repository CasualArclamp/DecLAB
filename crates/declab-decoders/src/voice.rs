// made by CasualArclamp
//! Digital voice: one decoder that finds out which mode the channel carries.
//!
//! DMR, NXDN48, NXDN96, P25 phase 1 and TETRA are decoded side by side
//! from the same channel: TETRA (25 kHz wide) takes it at 72 kS/s, the
//! 4FSK modes at 24 kS/s after one shared decimation by 3 (each receiver
//! then filters to its own band). Their SNR gates keep
//! the modes that are not there quiet, so whichever one locks is the one
//! on the air: it drives the card, the scope and the sound. The others
//! keep listening, so a change of mode is followed. Every mode's log lines
//! reach the log, tagged with the mode.
//!
//! Costs about 2–3 % of a core while all three hunt (measured per mode in
//! `dmr_sweep` and `nxdn_sweep`).

use declab_api::{
    Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Family, InputSpec, Lock, Sink,
};
use declab_pmr::fsk4::Iq;
use declab_pmr::nxdn::Mode;

use crate::{dmr, nxdn, p25, tetra};

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "voice",
            name: "Digital voice",
            family: Family::Voice,
            standard: "DMR (ETSI TS 102 361), NXDN48, NXDN96, P25 phase 1, TETRA (EN 300 392-2)",
            summary: "DMR Tier I/II/III, NXDN48, NXDN96, P25 and TETRA, the mode detected by itself",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // TETRA's: the widest (25 kHz).
        InputSpec {
            sample_rate: Some(declab_pmr::tetra::receiver::RATE),
            bandwidth_hz: 25_000.0,
            block_len: 7_200,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Voice::new())
    }

    fn default_scope(&self) -> &'static str {
        "Symbols"
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        let mut gates = vec![
            dmr::gate(),
            nxdn::gate(Mode::Nxdn48),
            nxdn::gate(Mode::Nxdn96),
            p25::gate(),
            tetra::gate(),
        ];
        for (g, key) in gates.iter_mut().zip(KEYS) {
            g.key = key;
        }
        Some(Box::new(crate::gate_panel::GatePanel { gates }))
    }
}

/// Each mode's gate setting, in the order of `Voice::modes`.
const KEYS: [&str; 5] = [
    "snr_gate_dmr",
    "snr_gate_nxdn48",
    "snr_gate_nxdn96",
    "snr_gate_p25",
    "snr_gate_tetra",
];

/// The modes, for the log.
const LISTENING: &str = "DMR, NXDN48, NXDN96, P25 and TETRA";

/// 72 kS/s to 24 kS/s for the 4FSK modes: a 9 kHz low-pass (the widest of
/// them, DMR and NXDN96, occupy about ±6.25 kHz), every third output kept.
struct Decimate3 {
    taps: Vec<f32>,
    buf: Vec<Iq>,
}

impl Decimate3 {
    fn new() -> Self {
        Decimate3 {
            taps: declab_pmr::fsk4::lowpass(9_000.0, 72_000.0, 49),
            buf: Vec::new(),
        }
    }

    fn process(&mut self, x: &[Iq], out: &mut Vec<Iq>) {
        self.buf.extend_from_slice(x);
        let n = self.taps.len();
        let mut at = 0;
        while at + n <= self.buf.len() {
            let mut acc = Iq::new(0.0, 0.0);
            for (s, &h) in self.buf[at..at + n].iter().zip(&self.taps) {
                acc += s * h;
            }
            out.push(acc);
            at += 3;
        }
        // Rust note: `drain(..at)` drops the consumed front in place; what
        // is left (under n samples) is the next output's history.
        self.buf.drain(..at);
    }

    fn reset(&mut self) {
        self.buf.clear();
    }
}

struct ModeDecoder {
    name: &'static str,
    dec: Box<dyn Decoder>,
    /// What it handed back from the latest call (reused, so nothing is
    /// allocated per block in steady state).
    sink: Sink,
    /// Its lock as of its latest report.
    lock: Lock,
    /// Takes the full 72 kS/s channel (TETRA); the rest take 24 kS/s.
    wide: bool,
}

struct Voice {
    modes: Vec<ModeDecoder>,
    /// The mode on the air (index into `modes`).
    active: Option<usize>,
    decimate: Decimate3,
    narrow: Vec<Iq>,
    /// The card of the last mode on the air, and when it went: kept up
    /// after the signal drops, since a bursting repeater's call (or beacon)
    /// was otherwise on screen for half a second.
    last_card: Option<(Card, f64)>,
    now_s: f64,
}

impl Voice {
    fn new() -> Voice {
        let mk = |name, dec| ModeDecoder {
            name,
            dec,
            sink: Sink::default(),
            lock: Lock::Searching,
            wide: name == "TETRA",
        };
        Voice {
            modes: vec![
                mk("DMR", dmr::Factory.create()),
                mk("NXDN48", nxdn::Factory(Mode::Nxdn48).create()),
                mk("NXDN96", nxdn::Factory(Mode::Nxdn96).create()),
                mk("P25", p25::Factory.create()),
                mk("TETRA", tetra::Factory.create()),
            ],
            active: None,
            decimate: Decimate3::new(),
            narrow: Vec::new(),
            last_card: None,
            now_s: 0.0,
        }
    }
}

impl Decoder for Voice {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        self.now_s = (block.first_sample + block.iq.len() as u64) as f64 / block.tuning.sample_rate;
        self.narrow.clear();
        self.decimate.process(block.iq, &mut self.narrow);
        let mut tuning = block.tuning;
        tuning.sample_rate = block.tuning.sample_rate / 3.0;
        let narrow = Block {
            iq: &self.narrow,
            tuning,
            first_sample: block.first_sample / 3,
            dropped_before: block.dropped_before / 3,
            live: block.live,
        };
        let mut reported = false;
        for m in &mut self.modes {
            m.sink.clear();
            m.dec
                .process(if m.wide { block } else { &narrow }, &mut m.sink);
            // A decoder sets its lock with its report (a card): keep it.
            if m.sink.card.is_some() {
                m.lock = m.sink.lock;
                reported = true;
            }
            for line in m.sink.log.drain(..) {
                out.log(format!("{} {line}", m.name));
            }
            // Data from any mode: it passed its own CRC (or the mode's
            // decode), so it is on the air even before the card says so.
            for mut p in m.sink.packets.drain(..) {
                p.channel = format!("{} {}", m.name, p.channel);
                out.packets.push(p);
            }
            for mut l in m.sink.locations.drain(..) {
                l.channel = format!("{} {}", m.name, l.channel);
                out.locations.push(l);
            }
            out.ip.append(&mut m.sink.ip);
        }
        // Keep the mode that is on the air while it stays locked; else take
        // whichever has locked.
        let locked = |i: usize| self.modes[i].lock != Lock::Searching;
        let next = match self.active {
            Some(a) if locked(a) => Some(a),
            _ => (0..self.modes.len()).find(|&i| locked(i)),
        };
        if next != self.active {
            match next {
                Some(i) => out.log(format!("detected {}", self.modes[i].name)),
                None => out.log(format!("signal lost, listening for {LISTENING}")),
            }
            self.active = next;
        }
        match self.active {
            Some(a) => {
                // Rust note: one mutable borrow of the mode, then its parts.
                let m = &mut self.modes[a];
                out.lock = m.lock;
                let s = &mut m.sink;
                out.audio.append(&mut s.audio);
                out.measurements.append(&mut s.measurements);
                out.scopes.append(&mut s.scopes);
                out.card = s.card.take();
                out.heard = s.heard.take();
                if let Some(c) = &out.card {
                    self.last_card = Some((c.clone(), self.now_s));
                }
            }
            None if reported => {
                out.lock = Lock::Searching;
                out.card = Some(match &self.last_card {
                    // The last mode's card, as it was, with its age.
                    Some((c, at)) => {
                        let mut c = c.clone();
                        c.title = format!("{} · last heard {} ago", c.title, crate::heard::clock(self.now_s - at));
                        c.gauge = None;
                        c.details.insert(0, ("listening for".into(), LISTENING.into()));
                        c
                    }
                    None => Card {
                        title: "Digital voice".into(),
                        headline: "searching".into(),
                        details: vec![("listening for".into(), LISTENING.into())],
                        ..Card::default()
                    },
                });
                // DMR's symbols, so the scope shows what the channel holds.
                out.scopes.append(&mut self.modes[0].sink.scopes);
            }
            None => {}
        }
        if self.active.is_none() {
            // Sound comes only from a mode with a call, and a call can begin
            // a moment before its mode's report says it has locked.
            for m in &mut self.modes {
                out.audio.append(&mut m.sink.audio);
            }
        }
    }

    fn command(&mut self, cmd: Command, out: &mut Sink) {
        if let Command::Set { key, value } = &cmd
            && let Some(i) = KEYS.iter().position(|k| k == key)
        {
            let set = Command::Set {
                key: "snr_gate".into(),
                value: value.clone(),
            };
            self.modes[i].dec.command(set, out);
            return;
        }
        out.log(format!(
            "voice: no such setting ({})",
            match cmd {
                Command::Set { key, .. } => key,
                _ => "?".into(),
            }
        ));
    }

    fn reset(&mut self) {
        for m in &mut self.modes {
            m.dec.reset();
            m.lock = Lock::Searching;
        }
        self.active = None;
        self.last_card = None;
        self.decimate.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;
    use declab_pmr::dmr::Source;
    use declab_pmr::dmr::pdu::{FullLc, ServiceOptions, ShortLc};
    use declab_pmr::dmr::sim::{self as dsim, Repeater};
    use declab_pmr::nxdn::{Message, sim as nsim};

    const RATE: f64 = 72_000.0;

    /// A DMR repeater (colour code `cc`, a call to `tg`) for about `secs`.
    fn dmr_carrier(cc: u8, tg: u32, secs: f64) -> Vec<declab_pmr::fsk4::Iq> {
        let mut r = Repeater::new(cc, ShortLc::Null);
        let idle = dsim::idle_burst(Source::Bs, cc);
        r.push(&idle);
        r.push(&idle);
        let lc = FullLc::GroupVoice {
            opts: ServiceOptions(0),
            group: tg,
            source: 2_345_678,
        };
        // A superframe is 360 ms.
        r.voice_call(&lc, (secs / 0.36) as usize + 1, |rep| dsim::idle_burst(Source::Bs, rep.cc));
        dsim::modulate(&r.bits, RATE, 0.0)
    }

    /// A TETRA cell (colour code `cc`) with a call, for about `secs`.
    fn tetra_carrier(cc: u8, secs: f64) -> Vec<declab_pmr::fsk4::Iq> {
        let mut cell = declab_pmr::tetra::sim::Cell::new(234, 1234, cc);
        let (b, _) = declab_pmr::tetra::sim::call(&mut cell, 9001, 4_000_123, (secs / 0.0567) as usize);
        declab_pmr::tetra::sim::modulate(&b, RATE, 0.0)
    }

    /// Carrier A then, at once, carrier B `step_hz` away (a retune that the
    /// host may or may not reset the decoder for): seconds until B is
    /// recognised (`want` in the log), or `None` within 6 s.
    fn relock(
        a: Vec<declab_pmr::fsk4::Iq>,
        b: Vec<declab_pmr::fsk4::Iq>,
        step_hz: f64,
        reset: bool,
        want: &str,
        mode: &str,
        drag_hz: f64,
    ) -> (Option<f64>, Option<f64>) {
        let switch = 3.0;
        let mut iq: Vec<_> = a.into_iter().take((switch * RATE) as usize).collect();
        // A drag: over the last 1.5 s on A the channel moves, so A seems to
        // slide by `drag_hz` (the host does not reset for small moves).
        let n_drag = (1.5 * RATE) as usize;
        let start = iq.len().saturating_sub(n_drag);
        let mut ph = 0f64;
        for (k, z) in iq[start..].iter_mut().enumerate() {
            let f = drag_hz * k as f64 / n_drag as f64;
            ph += 2.0 * std::f64::consts::PI * f / RATE;
            *z *= num_complex::Complex32::from_polar(1.0, ph as f32);
        }
        let n0 = iq.len();
        iq.extend(b.into_iter().take((6.0 * RATE) as usize).enumerate().map(|(k, z)| {
            z * num_complex::Complex32::from_polar(
                1.0,
                (2.0 * std::f64::consts::PI * step_hz * k as f64 / RATE) as f32,
            )
        }));
        declab_pmr::fsk4::add_noise(&mut iq, RATE, 30.0, 12_500.0, 150.0, 9);
        let mut d = Factory.create();
        let tuning = Tuning { center_hz: 0.0, sample_rate: RATE, generation: 0 };
        // When the new carrier's identity is logged (by whichever mode),
        // and when the card shows its mode, not searching (what the GUI
        // shows).
        let (mut heard, mut shown) = (None, None);
        for (n, chunk) in iq.chunks(7_200).enumerate() {
            let first = n * 7_200;
            if reset && first == n0 {
                d.reset();
            }
            let mut sink = Sink::default();
            d.process(
                &Block { iq: chunk, tuning, first_sample: first as u64, dropped_before: 0, live: false },
                &mut sink,
            );
            if first < n0 {
                continue;
            }
            let t = (first + chunk.len() - n0) as f64 / RATE;
            if heard.is_none() && sink.log.iter().any(|l| l.contains(want)) {
                heard = Some(t);
            }
            if let Some(c) = &sink.card
                && shown.is_none()
                && c.title.starts_with(mode)
                && sink.lock != Lock::Searching
            {
                shown = Some(t);
            }
            if heard.is_some() && shown.is_some() {
                break;
            }
        }
        (heard, shown)
    }

    /// A TETRA channel dragged off its carrier (the AFC follows it 8 kHz
    /// away) and onto another cell: the receiver must hunt afresh. Its AFC
    /// once stayed where the drag left it, and the new cell never locked.
    #[test]
    fn tetra_relocks_after_a_drag() {
        let (heard, _) = relock(
            tetra_carrier(17, 4.0),
            tetra_carrier(5, 4.0),
            0.0,
            false,
            "colour code 5",
            "TETRA",
            -8_000.0,
        );
        assert!(heard.is_some_and(|t| t < 1.5), "{heard:?}");
    }

    /// How long a retune to another carrier takes to lock, with and without
    /// the decoder being reset (the host resets only on a jump of more than
    /// half the channel). Printed, for looking into relock problems:
    /// `cargo test -p declab-decoders --release relock_after_retune -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn relock_after_retune() {
        type Make = Box<dyn Fn() -> (Vec<declab_pmr::fsk4::Iq>, Vec<declab_pmr::fsk4::Iq>)>;
        let cases: Vec<(&str, Make, &str, &str)> = vec![
            ("DMR -> DMR", Box::new(|| (dmr_carrier(1, 91, 4.0), dmr_carrier(7, 2350, 7.0))), "group call TG 2350", "DMR"),
            ("TETRA -> TETRA", Box::new(|| (tetra_carrier(17, 4.0), tetra_carrier(5, 7.0))), "colour code 5", "TETRA"),
            ("DMR -> TETRA", Box::new(|| (dmr_carrier(1, 91, 4.0), tetra_carrier(5, 7.0))), "colour code 5", "TETRA"),
            ("TETRA -> DMR", Box::new(|| (tetra_carrier(17, 4.0), dmr_carrier(7, 2350, 7.0))), "group call TG 2350", "DMR"),
        ];
        let fmt = |t: Option<f64>| t.map_or("never".to_string(), |t| format!("{t:.2} s"));
        for (name, make, want, mode) in &cases {
            for step in [0.0, 1500.0, -3000.0] {
                for reset in [true, false] {
                    let (a, b) = make();
                    let (heard, shown) = relock(a, b, step, reset, want, mode, 0.0);
                    println!(
                        "{name:16} step {step:+6.0} Hz  reset {reset:5}  decoded {:8} card {}",
                        fmt(heard),
                        fmt(shown)
                    );
                }
            }
            // A drag onto the new carrier: no reset.
            for drag in [8_000.0, -8_000.0] {
                let (a, b) = make();
                let (heard, shown) = relock(a, b, 0.0, false, want, mode, drag);
                println!(
                    "{name:16} drag {drag:+6.0} Hz  no reset    decoded {:8} card {}",
                    fmt(heard),
                    fmt(shown)
                );
            }
        }
    }

    #[test]
    fn detects_each_mode_in_turn() {
        // DMR, then NXDN48, then NXDN96, with gaps of noise between.
        let mut r = Repeater::new(1, ShortLc::Null);
        let idle = dsim::idle_burst(Source::Bs, 1);
        r.push(&idle);
        r.push(&idle);
        let lc = FullLc::GroupVoice {
            opts: ServiceOptions(0),
            group: 91,
            source: 2_345_678,
        };
        r.voice_call(&lc, 1, |rep| dsim::idle_burst(Source::Bs, rep.cc));
        let mut iq = dsim::modulate(&r.bits, RATE, 0.1);
        let gap = vec![declab_pmr::fsk4::Iq::new(0.0, 0.0); 72_000 * 2];
        iq.extend_from_slice(&gap);
        let (b48, _) = nsim::call(5, &Message::vcall(1, 1234, 77, true), 2);
        iq.extend(nsim::modulate(&b48, RATE, 0.1));
        iq.extend_from_slice(&gap);
        let (b96, _) = nsim::call(3, &Message::vcall(1, 4321, 9, true), 3);
        iq.extend(nsim::modulate_mode(Mode::Nxdn96, &b96, RATE, 0.1));
        iq.extend_from_slice(&gap);
        let p25lc = declab_pmr::p25::Lc::GroupVoice {
            opts: declab_pmr::p25::ServiceOptions(0),
            group: 101,
            source: 1_234_567,
        };
        let (bp, _) = declab_pmr::p25::sim::call(0x293, &p25lc, 2);
        iq.extend(declab_pmr::p25::sim::modulate(&bp, RATE, 0.1));
        iq.extend_from_slice(&gap);
        let mut cell = declab_pmr::tetra::sim::Cell::new(234, 1234, 17);
        let (bt, _) = declab_pmr::tetra::sim::call(&mut cell, 9001, 4_000_123, 10);
        iq.extend(declab_pmr::tetra::sim::modulate(&bt, RATE, 0.1));
        iq.extend_from_slice(&gap);
        // One noise floor over it all: 18 dB C/N in 12.5 kHz.
        declab_pmr::fsk4::add_noise(&mut iq, RATE, 18.0, 12_500.0, 150.0, 6);

        let mut d = Factory.create();
        let tuning = Tuning {
            center_hz: 0.0,
            sample_rate: RATE,
            generation: 0,
        };
        let (mut log, mut sound) = (Vec::new(), 0usize);
        // Every (group, radio) pair the heard lists showed.
        let mut heard: Vec<(String, String)> = Vec::new();
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
            if let Some(h) = sink.heard {
                for r in h.radios {
                    if !heard.contains(&(r.group.clone(), r.id.clone())) {
                        heard.push((r.group, r.id));
                    }
                }
            }
        }
        for (g, r) in [
            ("TG 91", "2345678"),
            ("TG 77", "1234"),
            ("TG 9", "4321"),
            ("TG 101", "1234567"),
            ("TG 9001", "4000123"),
        ] {
            assert!(heard.contains(&(g.into(), r.into())), "{g} {r}: {heard:?}");
        }
        let pos = |s: &str| {
            log.iter()
                .position(|l| l == s)
                .unwrap_or_else(|| panic!("no {s:?} in {log:#?}"))
        };
        let (a, b, c) = (
            pos("detected DMR"),
            pos("detected NXDN48"),
            pos("detected NXDN96"),
        );
        assert!(a < b && b < c && c < pos("detected P25"), "{log:#?}");
        assert!(pos("detected P25") < pos("detected TETRA"), "{log:#?}");
        assert!(
            log.iter()
                .any(|l| l == "TETRA call 77 to 9001 from 4000123 on TS2"),
            "{log:#?}"
        );
        assert!(
            log.iter()
                .any(|l| l == "P25 group call TG 101 from 1234567"),
            "{log:#?}"
        );
        assert!(
            log.iter()
                .any(|l| l == "DMR TS1: group call TG 91 from 2345678"),
            "{log:#?}"
        );
        assert!(
            log.iter().any(|l| l == "NXDN48 group call TG 77 from 1234"),
            "{log:#?}"
        );
        assert!(
            log.iter().any(|l| l == "NXDN96 group call TG 9 from 4321"),
            "{log:#?}"
        );
        // Every voice frame spoken, once: 18 (DMR) + 32 + 48 (NXDN) + 36 (P25)
        // frames of
        // 20 ms at 48 kHz.
        assert_eq!(sound, (18 + 32 + 48 + 36) * 960, "{log:#?}");
        // No mode other than the one on the air logs anything.
        assert!(
            !log.iter()
                .any(|l| l.starts_with("NXDN") && l.contains("TG 91")),
            "{log:#?}"
        );
    }
}
