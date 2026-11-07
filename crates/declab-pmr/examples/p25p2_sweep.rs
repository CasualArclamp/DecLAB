// made by CasualArclamp
//! Sensitivity of the P25 phase 2 receiver: a simulated traffic channel
//! (calls on both slots) at falling C/N (in 12.5 kHz), the SNR gate on
//! noise, and CPU.
//!
//! `cargo run --release -p declab-pmr --example p25p2_sweep [-- SEEDS [OFFSET_HZ]]`
//!
//! Columns: voice frames that came out as sent (of those sent); ACCHs that
//! passed their CRC and that failed; when the scrambling seed was found
//! (seconds in); the channel BER the voice FEC saw.

use std::collections::HashSet;

use declab_pmr::fsk4::Iq;
use declab_pmr::p25p2::receiver::RATE;
use declab_pmr::p25p2::sim::{self, Call};
use declab_pmr::p25p2::{Event, Receiver, scramble};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seeds: u64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(4);
    let offset: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(300.0);
    let seed = scramble::seed(0xBEE00, 0x3A1, 0x293);
    let calls = [
        Some(Call { group: 4001, source: 1_234_567, algid: 0x80 }),
        Some(Call { group: 77, source: 7_654_321, algid: 0x80 }),
    ];
    let (dibits, frames) = sim::channel(seed, calls, 20);
    let sent: HashSet<[u8; 49]> = (1..=frames).map(declab_pmr::dmr::sim::test_frame).collect();
    let clean = sim::modulate(&dibits, RATE, 0.3);
    let secs = clean.len() as f64 / RATE;

    let t = std::time::Instant::now();
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in clean.chunks(2400) {
        rx.process(c, &mut ev);
    }
    println!("CPU: {:.2} % of one core decoding ({secs:.1} s of signal)", 100.0 * t.elapsed().as_secs_f64() / secs);

    // The gate: every sync candidate's fit on noise alone, gate off.
    let mut noise = vec![Iq::new(0.0, 0.0); (RATE * 120.0) as usize];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 77);
    let t = std::time::Instant::now();
    let mut rx = Receiver::new();
    rx.snr_gate_db = f32::MIN;
    let (mut ev, mut best, mut n) = (Vec::new(), f32::MIN, 0u64);
    for c in noise.chunks(240) {
        let before = rx.stats.sync_quality;
        rx.process(c, &mut ev);
        if rx.stats.sync_quality != before {
            n += 1;
            best = best.max(rx.stats.sync_snr_db);
        }
    }
    println!(
        "gate off, noise 120 s: ~{n} sync candidates, best fit {best:.1} dB, {} events, {} timeslots locked (hunting CPU {:.2} %)",
        ev.len(),
        rx.stats.timeslots,
        100.0 * t.elapsed().as_secs_f64() / 120.0
    );
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in noise.chunks(2400) {
        rx.process(c, &mut ev);
    }
    println!("gate on, noise 120 s: {} events, {} candidates gated, {} timeslots locked", ev.len(), rx.stats.gated, rx.stats.timeslots);

    println!("C/N dB  voice ok      ACCH ok/fail   seed at   BER %   eye SNR");
    for cn in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 15.0] {
        let (mut ok, mut acch, mut fail, mut bits, mut errs, mut eye) = (0u64, 0u64, 0u64, 0u64, 0u64, 0f32);
        let mut found: Vec<f64> = Vec::new();
        for s in 0..seeds {
            let mut iq = clean.clone();
            sim::add_noise(&mut iq, RATE, cn, offset, 100 + s);
            let mut rx = Receiver::new();
            let mut ev = Vec::new();
            let mut at = None;
            for (k, c) in iq.chunks(2400).enumerate() {
                rx.process(c, &mut ev);
                if at.is_none() && rx.stats.scrambling.is_some() {
                    at = Some((k + 1) as f64 * 2400.0 / RATE);
                }
            }
            if let Some(a) = at {
                found.push(a);
            }
            for e in &ev {
                if let Event::Voice { frames, .. } = e {
                    ok += frames.iter().filter(|f| sent.contains(&f.bits)).count() as u64;
                }
            }
            acch += rx.stats.acch_ok;
            fail += rx.stats.crc_fail;
            bits += rx.stats.voice_bits;
            errs += rx.stats.voice_bit_errors;
            eye += rx.stats.eye_snr_db;
        }
        let seed_at = if found.len() as u64 == seeds {
            format!("{:.2} s", found.iter().sum::<f64>() / found.len() as f64)
        } else {
            format!("{}/{seeds}", found.len())
        };
        println!(
            "{cn:6.1}  {:5.1} %  {:6}/{:<6}  {seed_at:>8}  {:6.2}  {:5.1}",
            100.0 * ok as f64 / (u64::from(frames) * seeds) as f64,
            acch,
            fail,
            100.0 * errs as f64 / bits.max(1) as f64,
            eye / seeds as f32
        );
    }
}
