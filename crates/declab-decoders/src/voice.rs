// made by CasualArclamp
//! Digital voice: one decoder that finds out which mode the channel carries.
//!
//! DMR, NXDN48, NXDN96 and P25 phase 1 are decoded side by side from the same 24 kS/s
//! channel (each receiver filters to its own band). Their SNR gates keep
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
use declab_pmr::nxdn::Mode;

use crate::{dmr, nxdn, p25};

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "voice",
            name: "Digital voice",
            family: Family::Voice,
            standard: "DMR (ETSI TS 102 361), NXDN48, NXDN96, P25 phase 1",
            summary: "DMR Tier I/II/III, NXDN48, NXDN96 and P25, the mode detected by itself",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // The widest of the three (DMR and NXDN96 are 12.5 kHz channels).
        InputSpec {
            sample_rate: Some(declab_pmr::dmr::receiver::RATE),
            bandwidth_hz: 12_500.0,
            block_len: 2_400,
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
        ];
        for (g, key) in gates.iter_mut().zip(KEYS) {
            g.key = key;
        }
        Some(Box::new(crate::gate_panel::GatePanel { gates }))
    }
}

/// Each mode's gate setting, in the order of `Voice::modes`.
const KEYS: [&str; 4] = [
    "snr_gate_dmr",
    "snr_gate_nxdn48",
    "snr_gate_nxdn96",
    "snr_gate_p25",
];

struct ModeDecoder {
    name: &'static str,
    dec: Box<dyn Decoder>,
    /// What it handed back from the latest call (reused, so nothing is
    /// allocated per block in steady state).
    sink: Sink,
    /// Its lock as of its latest report.
    lock: Lock,
}

struct Voice {
    modes: Vec<ModeDecoder>,
    /// The mode on the air (index into `modes`).
    active: Option<usize>,
}

impl Voice {
    fn new() -> Voice {
        let mk = |name, dec| ModeDecoder {
            name,
            dec,
            sink: Sink::default(),
            lock: Lock::Searching,
        };
        Voice {
            modes: vec![
                mk("DMR", dmr::Factory.create()),
                mk("NXDN48", nxdn::Factory(Mode::Nxdn48).create()),
                mk("NXDN96", nxdn::Factory(Mode::Nxdn96).create()),
                mk("P25", p25::Factory.create()),
            ],
            active: None,
        }
    }
}

impl Decoder for Voice {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        let mut reported = false;
        for m in &mut self.modes {
            m.sink.clear();
            m.dec.process(block, &mut m.sink);
            // A decoder sets its lock with its report (a card): keep it.
            if m.sink.card.is_some() {
                m.lock = m.sink.lock;
                reported = true;
            }
            for line in m.sink.log.drain(..) {
                out.log(format!("{} {line}", m.name));
            }
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
                None => out.log("signal lost, listening for DMR, NXDN48, NXDN96 and P25"),
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
            }
            None if reported => {
                out.lock = Lock::Searching;
                out.card = Some(Card {
                    title: "Digital voice".into(),
                    headline: "searching".into(),
                    details: vec![("listening for".into(), "DMR, NXDN48, NXDN96, P25".into())],
                    ..Card::default()
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

    const RATE: f64 = 24_000.0;

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
        let gap = vec![declab_pmr::fsk4::Iq::new(0.0, 0.0); 24_000 * 2];
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
        // One noise floor over it all: 18 dB C/N in 12.5 kHz.
        declab_pmr::fsk4::add_noise(&mut iq, RATE, 18.0, 12_500.0, 150.0, 6);

        let mut d = Factory.create();
        let tuning = Tuning {
            center_hz: 0.0,
            sample_rate: RATE,
            generation: 0,
        };
        let (mut log, mut sound) = (Vec::new(), 0usize);
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
