// made by CasualArclamp
//! P25 phase 1 end to end: a conventional call, a control channel, noise.

use declab_pmr::fsk4::Iq;
use declab_pmr::p25::receiver::RATE;
use declab_pmr::p25::{Band, Event, Lc, Receiver, ServiceOptions, Tsbk, sim};

fn run(iq: &[Iq], chunk: usize) -> (Vec<Event>, Receiver, Option<u16>) {
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    // The NAC while locked (the receiver forgets it once the call is over).
    let mut nac = None;
    for c in iq.chunks(chunk) {
        rx.process(c, &mut ev);
        nac = rx.stats.nac.or(nac);
    }
    (ev, rx, nac)
}

#[test]
fn decodes_a_conventional_call() {
    let lc = Lc::GroupVoice {
        opts: ServiceOptions(0),
        group: 101,
        source: 1_234_567,
    };
    let (bits, frames) = sim::call(0x293, &lc, 3);
    let mut iq = sim::modulate(&bits, RATE, 0.1);
    sim::add_noise(&mut iq, RATE, 20.0, 350.0, 2);
    for chunk in [2400, 50_000] {
        let (ev, rx, nac) = run(&iq, chunk);
        assert!(
            ev.iter()
                .any(|e| matches!(e, Event::Header { header } if header.tgid == 101)),
            "{ev:?}"
        );
        let lcs = ev
            .iter()
            .filter(|e| matches!(e, Event::Lc { lc: l } if *l == lc))
            .count();
        assert_eq!(lcs, 4, "three LDU1s and the TDULC: {ev:?}");
        assert!(
            ev.iter()
                .any(|e| matches!(e, Event::Encryption { algid: 0x80, .. }))
        );
        assert!(ev.contains(&Event::End));
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
            assert_eq!(f.bits[..81], sim::test_frame(n as u32)[..81], "frame {n}");
        }
        // With large blocks the call can begin and end within one.
        if chunk == 2400 {
            assert_eq!(nac, Some(0x293));
        }
        assert!(
            (rx.stats.offset_hz - 350.0).abs() < 50.0,
            "offset {}",
            rx.stats.offset_hz
        );
    }
}

#[test]
fn decodes_a_control_channel() {
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
    let rfss = Tsbk::RfssStatus {
        system: 0x3a1,
        rfss: 2,
        site: 9,
        band: 1,
        channel: 0x21,
    };
    let mut bits = Vec::new();
    for k in 0..12 {
        let t: Vec<Tsbk> = match k % 3 {
            0 => vec![iden],
            1 => vec![grant, rfss],
            _ => vec![rfss, iden, grant],
        };
        bits.extend(sim::tsdu(0x3a1, &t));
    }
    let mut iq = sim::modulate(&bits, RATE, 0.1);
    sim::add_noise(&mut iq, RATE, 18.0, -200.0, 3);
    let (ev, _, _) = run(&iq, 2400);
    let got: Vec<Tsbk> = ev
        .iter()
        .filter_map(|e| match e {
            Event::Tsbk { tsbk } => Some(*tsbk),
            _ => None,
        })
        .collect();
    // Every TSBK after the first frame (acquisition may take the first).
    assert!(got.len() >= 23, "{} TSBKs: {got:?}", got.len());
    assert!(got.contains(&grant) && got.contains(&iden) && got.contains(&rfss));
}

#[test]
fn quiet_on_noise() {
    let mut noise = vec![Iq::new(0.0, 0.0); 24_000 * 20];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 51);
    let (ev, rx, _) = run(&noise, 2400);
    assert!(ev.is_empty(), "events from noise: {ev:?}");
    assert!(!rx.locked());
}
