// made by CasualArclamp
//! A simulated ISM band: every device sample this crate knows (weather
//! sensors, remotes, tyre-pressure sensors, meters), each sent once or more
//! at its own frequency and level somewhere in ±800 kHz, some at the same
//! time, in noise. Written as cs8 at 2.5 MS/s with the tuning in the name,
//! so the GUI and CLI pick it up as it is:
//!
//! ```text
//! cargo run --release -p declab-ism --example ism_make -- out/sim_ism_433.920MHz_2500ksps.cs8 [SECONDS] [MODEL…]
//! cargo run --release -p declab-gui -- out/sim_ism_433.920MHz_2500ksps.cs8 --decoder ism --chan 433.92M
//! ```
//!
//! With models named (`Nexus-TH Schrader`, any part of a model), only
//! those are sent, over and over.

use declab_ism::sim::{self, Noise, Rf};
use num_complex::Complex32;

const RATE: f64 = 2_500_000.0;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("OUT.cs8");
    let seconds: f64 = args.get(2).map_or(30.0, |s| s.parse().expect("SECONDS"));
    let wanted: Vec<&str> = args.iter().skip(3).map(String::as_str).collect();
    let samples: Vec<_> = declab_ism::devices::samples()
        .filter(|(_, s)| wanted.is_empty() || wanted.iter().any(|w| s.model.contains(w)))
        .collect();
    assert!(!samples.is_empty(), "no sample matches {wanted:?}");

    let n = (seconds * RATE) as usize;
    let mut iq = vec![Complex32::default(); n];
    let mut noise = Noise::new(2026);
    for x in iq.iter_mut() {
        *x = noise.gauss(1e-4);
    }
    // A cheap repeatable "random" from the noise generator.
    let mut rnd = |lo: f64, hi: f64| {
        let z = noise.gauss(1.0);
        lo + (hi - lo) * ((z.re.atan2(z.im) as f64 / std::f64::consts::TAU) + 0.5)
    };
    // The samples in turn, round and round until the file is full (as
    // sensors repeat), now and then two at once.
    let mut sent = 0;
    let mut t = 0.3;
    while t < seconds - 0.5 {
        for (dev, s) in &samples {
            let lv = sim::levels(dev, s);
            let dur = sim::duration_us(&lv) as f64 * 1e-6;
            if t + dur > seconds - 0.1 {
                break;
            }
            // Each device keeps its frequency from round to round.
            let k = samples
                .iter()
                .position(|(_, x)| std::ptr::eq(*x, *s))
                .unwrap() as f64;
            let offset = -780e3 + (k * 0.618_034 % 1.0) * 1.56e6;
            let rf = Rf {
                offset_hz: offset + rnd(-5e3, 5e3),
                amplitude: rnd(0.008, 0.05) as f32,
                deviation_hz: rnd(20e3, 45e3),
            };
            sim::add_burst(
                &mut iq,
                RATE,
                (t * RATE) as usize,
                &lv,
                dev.modulation.is_fsk(),
                rf,
            );
            sent += 1;
            // The next one starts before this one ends now and then.
            t += dur * rnd(0.4, 1.2) + rnd(0.05, 0.4);
            if t >= seconds - 0.5 {
                break;
            }
        }
    }
    // int8: noise at ~9 LSB rms, the strongest bursts at ~60.
    let scale = 1270.0f32;
    let mut bytes = Vec::with_capacity(n * 2);
    for z in &iq {
        bytes.push((z.re * scale).round().clamp(-127.0, 127.0) as i8 as u8);
        bytes.push((z.im * scale).round().clamp(-127.0, 127.0) as i8 as u8);
    }
    std::fs::write(path, bytes).expect("write");
    println!(
        "{path}: {seconds:.1} s, {sent} transmissions of {} samples",
        samples.len()
    );
}
