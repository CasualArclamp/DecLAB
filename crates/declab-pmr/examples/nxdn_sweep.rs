// made by CasualArclamp
//! Sensitivity of the NXDN48 receiver: a simulated call at falling C/N
//! (measured in 6.25 kHz), and where the SNR gate sits.
//!
//! `cargo run --release -p declab-pmr --example nxdn_sweep [-- SEEDS [OFFSET_HZ [CUTOFFS]]]`
//!
//! Columns: voice frames whose protected bits came out right; SACCH
//! superframe messages and FACCH1 messages decoded, of those sent; the
//! channel BER the AMBE Golay words saw.

use declab_pmr::fsk4::Iq;
use declab_pmr::nxdn::receiver::{CHANNEL_HZ, RATE};
use declab_pmr::nxdn::{Event, Message, Receiver, sim};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seeds: u64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(4);
    let offset: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(300.0);
    let chans: Vec<f64> = args
        .get(2)
        .map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_else(|| vec![CHANNEL_HZ]);

    let msg = Message::vcall(0x01, 1234, 77, true);
    let superframes = 6;
    let (mut bits, frames) = sim::call(5, &msg, superframes);
    // A second call, so the measure includes acquiring twice.
    let (b2, f2) = sim::call(5, &msg, superframes);
    bits.extend(b2);
    let frames = frames + f2;
    let clean = sim::modulate(&bits, RATE, 0.3);

    let t = std::time::Instant::now();
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in clean.chunks(2400) {
        rx.process(c, &mut ev);
    }
    let secs = clean.len() as f64 / RATE;
    println!(
        "CPU: {:.2} % of one core decoding",
        100.0 * t.elapsed().as_secs_f64() / secs
    );

    // The gate: SYNC eye SNR on noise, and on the signal near threshold.
    let snrs = |iq: &[Iq]| {
        let mut rx = Receiver::new();
        rx.snr_gate_db = f32::MIN;
        let (mut ev, mut v, mut seen) = (Vec::new(), Vec::new(), 0);
        for c in iq.chunks(240) {
            rx.process(c, &mut ev);
            if rx.stats.syncs > seen {
                seen = rx.stats.syncs;
                v.push(rx.stats.sync_snr_db);
            }
        }
        v.sort_by(f32::total_cmp);
        v
    };
    let pct = |v: &[f32], p: f32| {
        if v.is_empty() {
            f32::NAN
        } else {
            v[((v.len() - 1) as f32 * p) as usize]
        }
    };
    let mut noise = vec![Iq::new(0.0, 0.0); 24_000 * 120];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 77);
    let t = std::time::Instant::now();
    let v = snrs(&noise);
    println!(
        "gate: noise 120 s, {} FSW candidates, eye SNR max {:.1} median {:.1} dB (hunting CPU {:.2} %)",
        v.len(),
        v.last().copied().unwrap_or(f32::NAN),
        pct(&v, 0.5),
        100.0 * t.elapsed().as_secs_f64() / 120.0
    );
    // With the gate on: what does two minutes of noise produce?
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    let mut locked = 0;
    for c in noise.chunks(2400) {
        rx.process(c, &mut ev);
        locked += usize::from(rx.locked());
    }
    println!(
        "gate on, noise 120 s: {} events, locked in {locked} of {} blocks, {} candidates gated",
        ev.len(),
        noise.len() / 2400,
        rx.stats.gated
    );
    for cn in [4.0, 6.0] {
        let mut iq = clean.clone();
        sim::add_noise(&mut iq, RATE, cn, offset, 9);
        let v = snrs(&iq);
        println!(
            "gate: C/N {cn} dB, eye SNR 5% {:.1} median {:.1} dB",
            pct(&v, 0.05),
            pct(&v, 0.5)
        );
    }

    println!("{seeds} runs a point, {frames} voice frames, {offset} Hz offset, cut-offs {chans:?}");
    println!("C/N dB | voice   SACCH  FACCH  BER (per cut-off)");
    for cn in (1..=14).rev().map(|x| x as f64) {
        print!("{cn:6.1} |");
        for &chan in &chans {
            let (mut good, mut sacch, mut facch, mut errs, mut nbits) =
                (0u32, 0u32, 0u32, 0u64, 0u64);
            for seed in 0..seeds {
                let mut iq = clean.clone();
                sim::add_noise(&mut iq, RATE, cn, offset, 1 + seed * 7919);
                let mut rx = Receiver::with_channel(chan);
                let mut ev = Vec::new();
                for c in iq.chunks(2400) {
                    rx.process(c, &mut ev);
                }
                for e in &ev {
                    match e {
                        Event::Voice { frames: f } => {
                            for fr in f {
                                if (0..frames).any(|k| {
                                    declab_pmr::dmr::sim::test_frame(k)[..24] == fr.bits[..24]
                                }) {
                                    good += 1;
                                }
                            }
                        }
                        Event::Sacch { msg: m } if m.call() == msg.call() => sacch += 1,
                        Event::Facch { .. } => facch += 1,
                        _ => {}
                    }
                }
                errs += rx.stats.voice_bit_errors;
                nbits += rx.stats.voice_bits;
            }
            let n = seeds as f64;
            print!(
                " {:5.1}% {:5.1}% {:5.1}% {:5.2}% |",
                100.0 * f64::from(good) / (f64::from(frames) * n),
                100.0 * f64::from(sacch) / (2.0 * superframes as f64 * n),
                // Two calls, three FACCH1 frames each (two set-up, one release).
                100.0 * f64::from(facch) / (6.0 * n),
                100.0 * errs as f64 / nbits.max(1) as f64
            );
        }
        println!();
    }
}
