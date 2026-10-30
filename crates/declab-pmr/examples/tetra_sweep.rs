// made by CasualArclamp
//! Sensitivity of the TETRA downlink receiver: a simulated cell with a
//! group call at falling C/N (in 25 kHz), the SNR gate on noise, and CPU.
//!
//! `cargo run --release -p declab-pmr --example tetra_sweep [-- SEEDS [OFFSET_HZ [diff]]]`
//!
//! Columns, against a clean run: SYNC PDUs (BSCH), SYSINFO (BNCH),
//! signalling PDUs, traffic slots delivered, and the traffic slots' raw
//! bit error rate (what the speech decoder's own FEC then faces).

use declab_pmr::fsk4::Iq;
use declab_pmr::tetra::receiver::RATE;
use declab_pmr::tetra::sim::{self, Cell};
use declab_pmr::tetra::{Event, Receiver};

struct Count {
    sync: usize,
    sysinfo: usize,
    signals: usize,
    traffic: usize,
    errors: usize,
    bits: usize,
    stray: usize,
}

fn count(ev: &[Event], first_traffic: u32) -> Count {
    let mut c = Count {
        sync: 0,
        sysinfo: 0,
        signals: 0,
        traffic: 0,
        errors: 0,
        bits: 0,
        stray: 0,
    };
    let mut n = first_traffic;
    for e in ev {
        match e {
            Event::Sync(_) => c.sync += 1,
            Event::SysInfo(_) => c.sysinfo += 1,
            Event::Signal { .. } => c.signals += 1,
            // The call's full traffic slots (timeslot 2); anything else
            // reported as traffic is an AACH mistake, counted apart.
            Event::Traffic {
                tn: 1,
                half: false,
                llr,
                ..
            } => {
                c.traffic += 1;
                // Traffic slots are numbered in order; a missed one shifts
                // the count, so match against the nearest few.
                let best = (n..n + 8)
                    .map(|k| {
                        let want = sim::test_traffic(k);
                        (
                            k,
                            llr.iter()
                                .zip(&want)
                                .filter(|&(&l, &b)| u8::from(l < 0.0) != b)
                                .count(),
                        )
                    })
                    .min_by_key(|x| x.1)
                    .expect("four candidates");
                n = best.0 + 1;
                c.errors += best.1;
                c.bits += llr.len();
            }
            Event::Traffic { .. } => c.stray += 1,
        }
    }
    c
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seeds: u64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(4);
    let offset: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(500.0);
    // A third argument "diff" compares differential detection.
    let coherent = args.get(2).map(String::as_str) != Some("diff");

    let mut cell = Cell::new(234, 1234, 17);
    let (bits, _) = sim::call(&mut cell, 9001, 4_000_123, 60);
    let clean = sim::modulate(&bits, RATE, 0.1);
    let secs = clean.len() as f64 / RATE;

    let run = |iq: &[Iq], gate: Option<f32>| {
        let mut rx = Receiver::new();
        rx.coherent = coherent;
        if let Some(g) = gate {
            rx.snr_gate_db = g;
        }
        let mut ev = Vec::new();
        for c in iq.chunks(7200) {
            rx.process(c, &mut ev);
        }
        (rx, ev)
    };
    let t = std::time::Instant::now();
    let (_, ev) = run(&clean, None);
    println!(
        "CPU: {:.2} % of one core, locked",
        100.0 * t.elapsed().as_secs_f64() / secs
    );
    let reference = count(&ev, 0);

    let mut noise = vec![Iq::new(0.0, 0.0); RATE as usize * 120];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 77);
    let t = std::time::Instant::now();
    let (rx, ev) = run(&noise, Some(f32::MIN));
    println!(
        "noise 120 s, gate off: {} candidates, {} BSCH passed, eye SNR max {:.1} dB, {} events (hunting CPU {:.2} %)",
        rx.stats.candidates,
        rx.stats.syncs,
        rx.stats.sync_snr_max,
        ev.len(),
        100.0 * t.elapsed().as_secs_f64() / 120.0
    );
    let t = std::time::Instant::now();
    let (rx, ev) = run(&noise, None);
    println!(
        "noise 120 s, gate on: {} events, {} candidates gated (hunting CPU {:.2} %)",
        ev.len(),
        rx.stats.gated,
        100.0 * t.elapsed().as_secs_f64() / 120.0
    );

    println!(
        "{seeds} runs a point, {offset} Hz offset; clean: {} SYNC, {} SYSINFO, {} signalling, {} traffic slots",
        reference.sync, reference.sysinfo, reference.signals, reference.traffic
    );
    println!("C/N dB | SYNC   SYSINFO signal  traffic raw BER  stray");
    for cn in (2..=16).rev().step_by(1).map(|x| x as f64) {
        let mut tot = Count {
            sync: 0,
            sysinfo: 0,
            signals: 0,
            traffic: 0,
            errors: 0,
            bits: 0,
            stray: 0,
        };
        for seed in 0..seeds {
            let mut iq = clean.clone();
            sim::add_noise(&mut iq, RATE, cn, offset, 1 + seed * 7919);
            let (_, ev) = run(&iq, None);
            let c = count(&ev, 0);
            tot.sync += c.sync;
            tot.sysinfo += c.sysinfo;
            tot.signals += c.signals;
            tot.traffic += c.traffic;
            tot.errors += c.errors;
            tot.bits += c.bits;
            tot.stray += c.stray;
        }
        let pct = |a: usize, b: usize| 100.0 * a as f64 / (b as f64 * seeds as f64).max(1.0);
        println!(
            "{cn:6.1} | {:5.1}% {:5.1}%  {:5.1}%  {:5.1}%  {:5.2}%  {}",
            pct(tot.sync, reference.sync),
            pct(tot.sysinfo, reference.sysinfo),
            pct(tot.signals, reference.signals),
            pct(tot.traffic, reference.traffic),
            100.0 * tot.errors as f64 / tot.bits.max(1) as f64,
            tot.stray
        );
    }
}
