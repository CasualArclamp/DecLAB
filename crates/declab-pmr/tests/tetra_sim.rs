// made by CasualArclamp
//! The TETRA receiver against the simulated base station.

use declab_pmr::tetra::mac::{Address, Signal};
use declab_pmr::tetra::receiver::RATE;
use declab_pmr::tetra::sim::{self, Cell};
use declab_pmr::tetra::{Event, Receiver};

fn run(iq: &[declab_pmr::fsk4::Iq], chunk: usize) -> (Receiver, Vec<Event>) {
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in iq.chunks(chunk) {
        rx.process(c, &mut ev);
    }
    (rx, ev)
}

#[test]
fn cell_and_group_call() {
    let mut cell = Cell::new(234, 1234, 17);
    let (bits, traffic) = sim::call(&mut cell, 9001, 4_000_123, 30);
    let mut iq = sim::modulate(&bits, RATE, 0.1);
    sim::add_noise(&mut iq, RATE, 20.0, 600.0, 3);
    let (rx, ev) = run(&iq, 7200);

    let sync = ev.iter().find_map(|e| match e {
        Event::Sync(s) => Some(*s),
        _ => None,
    });
    let s = sync.expect("a SYNC");
    assert_eq!((s.mcc, s.mnc, s.colour), (234, 1234, 17));
    assert!(
        ev.iter()
            .any(|e| matches!(e, Event::SysInfo(si) if si.la == 1001))
    );

    let signals: Vec<(Address, Option<Signal>)> = ev
        .iter()
        .filter_map(|e| match e {
            Event::Signal {
                address, signal, ..
            } => Some((*address, signal.clone())),
            _ => None,
        })
        .collect();
    assert!(
        signals
            .iter()
            .any(|(a, s)| *a == Address::SsiUsageMarker(9001, 9)
                && matches!(
                    s,
                    Some(Signal::Setup {
                        call: 77,
                        calling: Some(4_000_123),
                        ..
                    })
                )),
        "{signals:?}"
    );
    assert!(signals.iter().any(|(_, s)| matches!(
        s,
        Some(Signal::TxGranted {
            party: Some(4_000_123),
            ..
        })
    )));
    assert!(
        signals
            .iter()
            .any(|(_, s)| matches!(s, Some(Signal::Release { call: 77, .. })))
    );
    assert!(
        signals
            .iter()
            .any(|(_, s)| matches!(s, Some(Signal::Sds { bits: 32, .. })))
    );
    assert!(
        signals
            .iter()
            .any(|(_, s)| matches!(s, Some(Signal::Status { status: 0x8002, .. })))
    );

    // Every traffic slot came out, on timeslot 2, with its bits right.
    let tr: Vec<&Vec<f32>> = ev
        .iter()
        .filter_map(|e| match e {
            Event::Traffic {
                tn: 1,
                llr,
                half: false,
                ..
            } => Some(llr),
            _ => None,
        })
        .collect();
    assert_eq!(tr.len() as u32, traffic, "traffic slots");
    let mut errors = 0;
    for (n, llr) in tr.iter().enumerate() {
        let want = sim::test_traffic(n as u32);
        errors += llr
            .iter()
            .zip(&want)
            .filter(|&(&l, &b)| u8::from(l < 0.0) != b)
            .count();
    }
    assert_eq!(errors, 0);
    assert!(
        rx.stats.offset_hz > 500.0 && rx.stats.offset_hz < 700.0,
        "{}",
        rx.stats.offset_hz
    );
}

#[test]
fn quiet_on_noise() {
    let mut iq = vec![declab_pmr::fsk4::Iq::new(0.0, 0.0); RATE as usize * 20];
    sim::add_noise(&mut iq, RATE, 0.0, 0.0, 5);
    let (rx, ev) = run(&iq, 7200);
    assert!(ev.is_empty(), "{:?}", &ev[..ev.len().min(4)]);
    assert!(!rx.stats.locked);
}
