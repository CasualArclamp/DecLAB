// made by CasualArclamp
//! NXDN48 end to end: a simulated conventional call through the receiver.

use declab_pmr::fsk4::Iq;
use declab_pmr::nxdn::receiver::RATE;
use declab_pmr::nxdn::{Event, Message, Receiver, sim};

fn run(iq: &[Iq], chunk: usize) -> (Vec<Event>, Receiver) {
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in iq.chunks(chunk) {
        rx.process(c, &mut ev);
    }
    (ev, rx)
}

#[test]
fn decodes_a_call() {
    let msg = Message::vcall(0x01, 1234, 77, true);
    let (bits, frames) = sim::call(5, &msg, 3);
    let mut iq = sim::modulate(&bits, RATE, 0.1);
    sim::add_noise(&mut iq, RATE, 25.0, 250.0, 3);
    for chunk in [2400, 40_000] {
        let (ev, rx) = run(&iq, chunk);
        let facch: Vec<String> = ev
            .iter()
            .filter_map(|e| match e {
                Event::Facch { msg } => Some(msg.to_string()),
                _ => None,
            })
            .collect();
        assert!(
            facch.iter().any(|m| m == "group call TG 77 from 1234"),
            "{facch:?}"
        );
        assert!(
            facch.iter().any(|m| m.starts_with("TX_REL 1234 → TG 77")),
            "{facch:?}"
        );
        let sacch = ev
            .iter()
            .filter(|e| matches!(e, Event::Sacch { msg: m } if m.call() == msg.call()))
            .count();
        assert_eq!(sacch, 3, "{ev:?}");
        let voice: Vec<_> = ev
            .iter()
            .filter_map(|e| match e {
                Event::Voice { frames } => Some(frames.clone()),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(voice.len() as u32, frames);
        for (n, f) in voice.iter().enumerate() {
            assert_eq!(
                f.bits,
                declab_pmr::dmr::sim::test_frame(n as u32),
                "frame {n}"
            );
        }
        assert_eq!(rx.stats.ran, Some(5));
        assert!(
            (rx.stats.offset_hz - 250.0).abs() < 40.0,
            "offset {}",
            rx.stats.offset_hz
        );
    }
}

#[test]
fn quiet_on_noise() {
    let mut noise = vec![Iq::new(0.0, 0.0); 24_000 * 20];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 41);
    let (ev, rx) = run(&noise, 2400);
    assert!(ev.is_empty(), "events from noise: {ev:?}");
    assert!(!rx.locked());
}
