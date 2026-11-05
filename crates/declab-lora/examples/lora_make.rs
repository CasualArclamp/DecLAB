// made by CasualArclamp
//! A simulated 869.525 MHz channel: every sample packet of both meshes
//! (`declab_lora::sim::samples`), each at its own level, carrier offset and
//! crystal error, in noise. Written as cs8 (or cf32, by the extension) at
//! 1 MS/s with the tuning in the name, so the GUI and CLI know it.
//!
//! ```text
//! cargo run --release -p declab-lora --example lora_make -- out/sim_lora_869.525MHz_1000ksps.cs8 [SECONDS]
//! cargo run --release -p declab-gui -- out/sim_lora_869.525MHz_1000ksps.cs8 --decoder lora --chan 869.525M
//! ```

use declab_lora::sim::{self, Noise};
use num_complex::Complex32;

const RATE: f64 = 1e6;
const CENTRE: f64 = 869.525e6;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("OUT.cs8|OUT.cf32");
    let seconds: f64 = args.get(2).map_or(30.0, |s| s.parse().expect("SECONDS"));
    let n = (seconds * RATE) as usize;
    let mut iq = vec![Complex32::default(); n];
    // Noise power 1e-4 a sample (−40 dBFS); signal SNRs in their own
    // bandwidth from −6 to +20 dB.
    let noise = 1e-4;
    let samples = sim::samples();
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut rnd = |a: f64, b: f64| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        a + (b - a) * ((seed >> 11) as f64 / (1u64 << 53) as f64)
    };
    let mut t = 0.3;
    let mut sent = 0;
    'outer: loop {
        for s in &samples {
            let dur = s.tx.airtime(s.bytes.len());
            if t + dur > seconds - 0.2 {
                break 'outer;
            }
            let snr = rnd(-6.0, 20.0);
            // In-band noise is noise × bw / RATE.
            let amp = (noise * 10f64.powf(snr / 10.0) * s.tx.bw / RATE).sqrt() as f32;
            let ppm = rnd(-3.0, 3.0);
            let freq = ppm * 1e-6 * CENTRE + rnd(-500.0, 500.0);
            sim::modulate(
                &s.tx,
                &s.bytes,
                RATE,
                freq,
                ppm,
                amp,
                (t * RATE) as usize,
                &mut iq,
            );
            sent += 1;
            if std::env::var_os("LORA_DEBUG").is_some() {
                println!("{t:7.3} s  {}  {snr:5.1} dB  {freq:+6.0} Hz", s.what);
            }
            t += dur + rnd(0.1, 0.6);
        }
    }
    iq.truncate(n);
    Noise::new(7).add(&mut iq, noise);
    let bytes: Vec<u8> = if path.ends_with(".cf32") {
        iq.iter()
            .flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()].concat())
            .collect()
    } else {
        // int8: noise at 3 LSB rms, the strongest packets near full scale.
        let scale = 300.0f32;
        iq.iter()
            .flat_map(|z| {
                [
                    (z.re * scale).round().clamp(-127.0, 127.0) as i8 as u8,
                    (z.im * scale).round().clamp(-127.0, 127.0) as i8 as u8,
                ]
            })
            .collect()
    };
    std::fs::write(path, bytes).expect("write");
    println!(
        "{path}: {seconds:.1} s, {sent} packets ({} kinds)",
        samples.len()
    );
}
