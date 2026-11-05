// made by CasualArclamp
//! Sensitivity: frames decoded against SNR (in the signal's bandwidth) for
//! a few presets, with a carrier offset and crystal error, and the SNR the
//! receiver reports. Semtech's figures for comparison: SF7 −7.5 dB, SF8
//! −10, SF9 −12.5, SF10 −15, SF11 −17.5, SF12 −20.
//!
//! ```text
//! cargo run --release -p declab-lora --example lora_sweep -- [FRAMES] [PRESET…]
//! ```

use declab_lora::Receiver;
use declab_lora::receiver::{Config, INPUT_RATE};
use declab_lora::sim::{self, Noise, Tx};
use num_complex::Complex32;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let frames: usize = args.get(1).map_or(20, |s| s.parse().expect("FRAMES"));
    let presets: Vec<(&str, Tx)> = vec![
        ("ShortFast SF7/250", Tx::meshtastic("ShortFast")),
        ("MediumFast SF9/250", Tx::meshtastic("MediumFast")),
        ("LongFast SF11/250", Tx::meshtastic("LongFast")),
        ("LongSlow SF12/125", Tx::meshtastic("LongSlow")),
        ("MeshCore SF8/62.5", Tx::meshcore(62.5e3, 8, 8)),
    ];
    let want: Vec<String> = args.iter().skip(2).cloned().collect();
    let payload: Vec<u8> = (0..40u8).map(|i| i.wrapping_mul(29) ^ 0x5c).collect();
    for (name, tx) in presets {
        if !want.is_empty() && !want.iter().any(|w| name.contains(w.as_str())) {
            continue;
        }
        let floor = -2.5 * (tx.sf as f64 - 4.0);
        print!("{name:20}");
        for step in 0..8 {
            let snr = floor - 4.0 + step as f64 * 1.5;
            if std::env::var_os("LORA_DEBUG").is_some() {
                eprintln!("== {snr}");
            }
            let gap = (0.03 * INPUT_RATE) as usize;
            let mut x = Vec::new();
            let mut at = gap;
            for k in 0..frames {
                let ppm = (k as f64 * 0.37).sin() * 3.0;
                sim::modulate(
                    &tx,
                    &payload,
                    INPUT_RATE,
                    ppm * 869.525 + 300.0,
                    ppm,
                    1.0,
                    at,
                    &mut x,
                );
                at = x.len() + gap;
            }
            x.resize(at, Complex32::default());
            let p = 10f64.powf(-snr / 10.0) * INPUT_RATE / tx.bw;
            Noise::new(step as u64 + 11).add(&mut x, p);
            let scale = 1.0 / (4.0 * p.sqrt() as f32 + 1.0);
            for c in &mut x {
                *c *= scale;
            }
            let mut rx = Receiver::new(INPUT_RATE, Config::default());
            rx.centre_hz = 869.525e6;
            for b in x.chunks(65_536) {
                rx.process(b);
            }
            let good: Vec<_> = rx
                .frames
                .iter()
                .filter(|f| {
                    f.frame
                        .as_ref()
                        .is_some_and(|f| f.crc_ok && f.payload == payload)
                })
                .collect();
            if std::env::var_os("LORA_STATS").is_some() {
                let want = declab_lora::codec::encode(
                    &payload,
                    tx.sf,
                    tx.cr,
                    true,
                    declab_lora::codec::ldro(tx.sf, tx.bw),
                );
                for f in rx
                    .frames
                    .iter()
                    .filter(|f| f.frame.as_ref().is_some_and(|f| !f.crc_ok))
                {
                    let bad: Vec<(usize, u16, u16)> = f
                        .symbols
                        .iter()
                        .zip(&want)
                        .enumerate()
                        .filter(|(_, (a, b))| a != b)
                        .map(|(k, (a, b))| (k, *a, *b))
                        .collect();
                    eprintln!(
                        "  {:.3}: cfo {:.1} {} symbols differ: {:?}",
                        f.at_s,
                        f.cfo_hz,
                        bad.len(),
                        &bad[..bad.len().min(12)]
                    );
                }
                eprintln!(
                    "{snr}: {:?} frames {:?}",
                    rx.stats(),
                    rx.frames
                        .iter()
                        .map(|f| (f.at_s, f.frame.as_ref().map(|f| f.crc_ok)))
                        .collect::<Vec<_>>()
                );
            }
            let est = good.iter().map(|f| f.snr_db as f64).sum::<f64>() / good.len().max(1) as f64;
            print!("  {snr:6.1}:{:3}/{frames} ({est:5.1})", good.len());
        }
        println!();
    }
}
