// made by CasualArclamp
//! A simulated Tier III site end to end: TS1 carries a group call, TS2 the
//! control channel (Aloha, a grant, a broadcast), the CACHs a C_SYS_Parms.

use declab_pmr::dmr::pdu::{Csbk, FullLc, GrantKind, ServiceOptions, ShortLc, SysCode};
use declab_pmr::dmr::receiver::RATE;
use declab_pmr::dmr::sim::{self, Repeater};
use declab_pmr::dmr::{Event, Receiver, Source};
use declab_pmr::fsk4::Detector;

pub fn site() -> (Vec<u8>, FullLc, Vec<Csbk>, ShortLc, u32) {
    let sys = SysCode((0b01 << 14) | (12 << 7) | (4 << 2));
    let slc = ShortLc::ControlParms {
        syscode14: sys.0 >> 2,
        reg: true,
        counter: 77,
    };
    let mut r = Repeater::new(1, slc);
    let lc = FullLc::GroupVoice {
        opts: ServiceOptions(0),
        group: 2350,
        source: 2_341_234,
    };
    let csbks = vec![
        Csbk::Aloha {
            syscode: sys,
            ms: 0,
            reg: true,
            backoff: 2,
            mask: 0,
            version: 1,
        },
        Csbk::Grant {
            kind: GrantKind::TalkgroupVoice,
            lpcn: 3,
            slot: 0,
            late_entry: false,
            emergency: false,
            offset: false,
            target: 2350,
            source: 2_341_234,
        },
        Csbk::Bcast {
            ann: 6,
            parms1: 0x0042,
            reg: true,
            backoff: 2,
            syscode: sys,
            parms2: 0x00_1234,
        },
    ];
    // TS2 idles first so TS1 starts the call.
    let idle = sim::idle_burst(Source::Bs, 1);
    r.push(&idle);
    r.push(&idle);
    let mut k = 0;
    let c2 = csbks.clone();
    r.voice_call(&lc, 2, |rep| {
        k += 1;
        sim::csbk_burst(Source::Bs, rep.cc, &c2[k % c2.len()])
    });
    for _ in 0..4 {
        r.push(&idle);
    }
    let frames = r.frames;
    (r.bits, lc, csbks, slc, frames)
}

pub fn run(iq: &[declab_pmr::fsk4::Iq], detector: Detector) -> Vec<Event> {
    let mut rx = Receiver::new(detector);
    let mut ev = Vec::new();
    // Large blocks, as the host may hand over (the AFC once ran away on them).
    for chunk in iq.chunks(30_000) {
        rx.process(chunk, &mut ev);
    }
    ev
}

#[test]
fn decodes_lrrp_in_both_data_formats() {
    use declab_pmr::dmr::data::{DataHeader, Format, Rate, sap};
    use declab_pmr::dmr::lrrp::{self, Lrrp};
    let mut r = Repeater::new(1, ShortLc::Null);
    let idle = sim::idle_burst(Source::Bs, 1);
    r.push(&idle);
    r.push(&idle);
    let msg = Lrrp::location_response(9, 37.7749, -122.4194, (2026, 10, 29, 14, 5, 9));
    // Confirmed, rate 3/4, a whole IPv4/UDP header (as MOTOTRBO sends).
    let ip = sim::ipv4_udp([12, 0, 4, 210], [13, 0, 0, 100], 4001, 4001, &msg);
    let h = DataHeader {
        group: false,
        format: Format::Confirmed,
        sap: sap::IP,
        pad: 0,
        dst: 100,
        src: 1234,
        blocks: 0,
    };
    // Unconfirmed, rate 1/2, the compressed header with both ports extended.
    let mut hc = vec![0x00, 0x01, 0x00, 0x00, 0x00];
    hc.extend_from_slice(&4001u16.to_be_bytes());
    hc.extend_from_slice(&4001u16.to_be_bytes());
    hc.extend_from_slice(&msg);
    let h2 = DataHeader {
        format: Format::Unconfirmed,
        sap: sap::UDP_HC,
        src: 5678,
        ..h
    };
    let mut sent = 0;
    for (h, payload, rate) in [(h, ip, Rate::ThreeQuarter), (h2, hc, Rate::Half)] {
        for b in sim::data_packet(Source::Bs, 1, h, &payload, rate) {
            r.push(&b);
            r.push(&idle);
            sent += 1;
        }
    }
    assert!(sent > 4);
    for _ in 0..4 {
        r.push(&idle);
    }
    let mut iq = sim::modulate(&r.bits, RATE, 0.05);
    sim::add_noise(&mut iq, RATE, 12.0, 400.0, 11);
    let mut rx = Receiver::new(Detector::Discriminator);
    let mut ev = Vec::new();
    for chunk in iq.chunks(2400) {
        rx.process(chunk, &mut ev);
    }
    assert!((rx.stats.offset_hz - 400.0).abs() < 60.0, "offset {}", rx.stats.offset_hz);
    let packets: Vec<_> = ev
        .iter()
        .filter_map(|e| match e {
            Event::Packet { packet, .. } => Some(packet),
            _ => None,
        })
        .collect();
    assert_eq!(packets.len(), 2, "{ev:#?}");
    for (p, src) in packets.iter().zip([1234, 5678]) {
        assert!(p.crc_ok && p.bad_blocks == 0, "{p:?}");
        assert_eq!(p.header.src, src);
        let u = lrrp::udp(p).expect("UDP");
        assert_eq!((u.sport, u.dport), (4001, 4001));
        let l = Lrrp::parse(&u.payload).expect("LRRP");
        assert!(
            (l.lat.unwrap() - 37.7749).abs() < 1e-6 && (l.lon.unwrap() + 122.4194).abs() < 1e-6,
            "{l:?}"
        );
    }
}

#[test]
fn does_not_lock_on_noise() {
    let mut noise = vec![declab_pmr::fsk4::Iq::new(0.0, 0.0); 24_000 * 20];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 99);
    let mut rx = Receiver::new(Detector::Discriminator);
    let mut ev = Vec::new();
    let mut locked = 0;
    for chunk in noise.chunks(2400) {
        rx.process(chunk, &mut ev);
        locked += usize::from(rx.locked().is_some());
    }
    assert!(locked < 5, "locked {locked} of 200 blocks on noise");
    assert!(rx.stats.offset_hz.abs() < 1.0, "AFC wandered to {} Hz", rx.stats.offset_hz);
}

#[test]
fn decodes_a_tier3_site() {
    let (bits, lc, csbks, slc, frames) = site();
    let mut iq = sim::modulate(&bits, RATE, 0.05);
    sim::add_noise(&mut iq, RATE, 30.0, 300.0, 7);
    for det in [Detector::Product, Detector::Discriminator] {
        let ev = run(&iq, det);
        let headers = ev
            .iter()
            .filter(|e| matches!(e, Event::VoiceHeader { slot: 0, lc: l, .. } if *l == lc))
            .count();
        let terms = ev
            .iter()
            .filter(|e| matches!(e, Event::Terminator { slot: 0, lc: l, .. } if *l == lc))
            .count();
        let emb = ev
            .iter()
            .filter(|e| matches!(e, Event::EmbeddedLc { slot: 0, lc: l } if *l == lc))
            .count();
        assert_eq!((headers, terms, emb), (1, 1, 2), "{det:?}: {ev:#?}");
        let voice: Vec<_> = ev
            .iter()
            .filter_map(|e| match e {
                Event::Voice {
                    slot: 0, frames, ..
                } => Some(frames),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(voice.len() as u32, frames, "{det:?}");
        for (n, f) in voice.iter().enumerate() {
            assert_eq!(f.bits, sim::test_frame(n as u32), "frame {n}");
            assert_eq!(f.errors, 0);
        }
        for c in &csbks {
            assert!(
                ev.iter()
                    .any(|e| matches!(e, Event::Csbk { slot: 1, csbk, .. } if csbk == c)),
                "{det:?} {c:?}"
            );
        }
        assert!(ev.contains(&Event::ShortLc(slc)), "{det:?}");
    }
}
