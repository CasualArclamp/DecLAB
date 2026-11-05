// made by CasualArclamp
//! The receiver on a cs8 or cf32 file, timed: messages decoded, and the
//! receiver's CPU as a share of one core in real time (fastest of three
//! runs, since other builds make timings noisy), with the burst finder's
//! FFT alone beside it.
//!
//! ```text
//! cargo run --release -p declab-ism --example filetest -- out/sim_ism_433.920MHz_2500ksps.cs8 [RATE]
//! ```

use std::time::Instant;

use declab_ism::burst::BurstFinder;
use declab_ism::receiver::Event;
use declab_ism::{Receiver, ReceiverConfig};
use num_complex::Complex32;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("IN.cs8|IN.cf32");
    let rate: f64 = args.get(2).map_or(2.5e6, |s| s.parse().expect("RATE"));
    let bytes = std::fs::read(path).expect("read");
    let iq: Vec<Complex32> = if path.ends_with(".cf32") {
        bytes
            .as_chunks::<8>()
            .0
            .iter()
            .map(|c| {
                Complex32::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect()
    } else {
        bytes
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| Complex32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
            .collect()
    };
    let secs = iq.len() as f64 / rate;

    let mut best_rx = f64::MAX;
    let mut best_fft = f64::MAX;
    let mut decoded = 0;
    let mut unknown = 0;
    for _ in 0..3 {
        let t = Instant::now();
        let mut rx = Receiver::new(rate, &ReceiverConfig::default());
        for c in iq.chunks(65_536) {
            rx.process(c);
        }
        rx.flush();
        best_rx = best_rx.min(t.elapsed().as_secs_f64());
        decoded = rx
            .events
            .iter()
            .filter(|e| matches!(e, Event::Decoded(_)))
            .count();
        unknown = rx
            .events
            .iter()
            .filter(|e| matches!(e, Event::Unknown(_)))
            .count();

        let t = Instant::now();
        let mut bf = BurstFinder::new(rate);
        for c in iq.chunks(65_536) {
            bf.process(c);
            bf.done.clear();
        }
        best_fft = best_fft.min(t.elapsed().as_secs_f64());
        let mut bf = BurstFinder::new(rate);
        let mut nz = declab_ism::sim::Noise::new(2);
        let q: Vec<Complex32> = (0..(5.0 * rate) as usize).map(|_| nz.gauss(1e-4)).collect();
        let t = Instant::now();
        for c in q.chunks(65_536) {
            bf.process(c);
            bf.done.clear();
        }
        println!(
            "finder on 5 s of noise: {:.2} %, {} opened",
            100.0 * t.elapsed().as_secs_f64() / 5.0,
            bf.opened
        );
    }
    // The same length of noise alone: the FFT with no burst open.
    let mut noise = declab_ism::sim::Noise::new(1);
    let quiet: Vec<Complex32> = (0..iq.len().min((10.0 * rate) as usize))
        .map(|_| noise.gauss(1e-4))
        .collect();
    let mut best_quiet = f64::MAX;
    let mut quiet_bursts = 0;
    for _ in 0..3 {
        let t = Instant::now();
        let mut rx = Receiver::new(rate, &ReceiverConfig::default());
        for c in quiet.chunks(65_536) {
            rx.process(c);
        }
        best_quiet = best_quiet.min(t.elapsed().as_secs_f64());
        quiet_bursts = rx.stats.bursts;
    }
    println!("{path}: {secs:.1} s, {decoded} messages, {unknown} unknown");
    println!(
        "noise alone: {:.2} % of a core, {quiet_bursts} false bursts in {:.0} s",
        100.0 * best_quiet / (quiet.len() as f64 / rate),
        quiet.len() as f64 / rate
    );
    println!(
        "receiver {:.2} % of a core; burst finder alone (FFT + cut-outs) {:.2} %",
        100.0 * best_rx / secs,
        100.0 * best_fft / secs
    );
}
