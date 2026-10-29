// made by CasualArclamp
//! Write a simulated DMR repeater to an IQ file, for the GUI: a group call
//! on TS1 with a Tier III control channel on TS2, then two LRRP location
//! reports, looped.
//!
//! `cargo run --release -p declab-pmr --example dmr_make -- OUT.cf32 [C/N dB]`
//! then `declab-gui OUT.cf32 --rate 48000 --decoder dmr`.

use std::io::Write;

use declab_pmr::dmr::Source;
use declab_pmr::dmr::data::{DataHeader, Format, Rate, sap};
use declab_pmr::dmr::lrrp::Lrrp;
use declab_pmr::dmr::pdu::{Csbk, FullLc, GrantKind, ServiceOptions, ShortLc, SysCode};
use declab_pmr::dmr::sim::{self, Repeater};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args.first().cloned().unwrap_or_else(|| "out/sim_dmr.cf32".into());
    let cn: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let rate = 48_000.0;

    let sys = SysCode((0b01 << 14) | (12 << 7) | (4 << 2));
    let mut r = Repeater::new(1, ShortLc::ControlParms { syscode14: sys.0 >> 2, reg: true, counter: 1 });
    let lc = FullLc::GroupVoice { opts: ServiceOptions(0), group: 2350, source: 2_341_234 };
    let csbks = [
        Csbk::Aloha { syscode: sys, ms: 0, reg: true, backoff: 2, mask: 0, version: 1 },
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
        Csbk::Bcast { ann: 6, parms1: 0x42, reg: true, backoff: 2, syscode: sys, parms2: 0x1234 },
    ];
    let idle = sim::idle_burst(Source::Bs, 1);
    r.push(&idle);
    r.push(&idle);
    let mut k = 0;
    r.voice_call(&lc, 10, |rep| {
        k += 1;
        sim::csbk_burst(Source::Bs, rep.cc, &csbks[k % csbks.len()])
    });
    for (n, (lat, lon)) in [(37.7749, -122.4194), (37.7790, -122.4180)].into_iter().enumerate() {
        let msg = Lrrp::location_response(n as u32 + 1, lat, lon, (2026, 10, 29, 14, 5, 9 + n as u8 * 30));
        let ip = sim::ipv4_udp([12, 0, 4, 210], [13, 0, 0, 100], 4001, 4001, &msg);
        let h = DataHeader { group: false, format: Format::Confirmed, sap: sap::IP, pad: 0, dst: 100, src: 1234, blocks: 0 };
        for b in sim::data_packet(Source::Bs, 1, h, &ip, Rate::ThreeQuarter) {
            r.push(&b);
            r.push(&sim::csbk_burst(Source::Bs, 1, &csbks[0]));
        }
    }
    for _ in 0..6 {
        r.push(&idle);
    }
    if std::env::var_os("DMR_CHECK").is_some() {
        // The same signal at the receiver's own rate, decoded here, with the
        // offset and lock reported as it goes.
        use declab_pmr::dmr::receiver::RATE;
        use declab_pmr::dmr::{Event, Receiver};
        let mut iq = sim::modulate(&r.bits, RATE, 0.2);
        sim::add_noise(&mut iq, RATE, cn, 600.0, 42);
        let mut rx = Receiver::new(declab_pmr::fsk4::Detector::Discriminator);
        let mut ev = Vec::new();
        let chunk_len: usize = std::env::var("DMR_CHECK").ok().and_then(|v| v.parse().ok()).unwrap_or(2400);
        for (n, chunk) in iq.chunks(chunk_len).enumerate() {
            rx.process(chunk, &mut ev);
            let voice = ev.iter().filter(|e| matches!(e, Event::Voice { .. })).count();
            let pk = ev.iter().filter(|e| matches!(e, Event::Packet { .. })).count();
            println!(
                "{:4.1} s locked {:?} offset {:+7.0} Hz voice bursts {voice} packets {pk} bad {}",
                (n + 1) as f64 * 0.1,
                rx.locked(),
                rx.stats.offset_hz,
                rx.stats.bursts_bad
            );
        }
    }
    let mut iq = sim::modulate(&r.bits, rate, 0.2);
    sim::add_noise(&mut iq, rate, cn, 600.0, 42);
    // Well below full scale, as a radio's samples would be.
    iq.iter_mut().for_each(|s| *s *= 0.25);
    let mut f = std::io::BufWriter::new(std::fs::File::create(&path)?);
    for s in &iq {
        f.write_all(&s.re.to_le_bytes())?;
        f.write_all(&s.im.to_le_bytes())?;
    }
    println!("{path}: {:.1} s at {rate} S/s, C/N {cn} dB", iq.len() as f64 / rate);
    Ok(())
}
