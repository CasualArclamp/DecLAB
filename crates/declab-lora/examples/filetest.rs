// made by CasualArclamp
//! The LoRa receiver on a cs8 or cf32 file at 1 MS/s, timed: every frame
//! (preset, sync word, SNR, offset) and what the meshes made of it, then
//! the receiver's CPU on the file and on noise alone (fastest of 3; one
//! core = 100 %).
//!
//! ```text
//! cargo run --release -p declab-lora --example filetest -- out/sim_lora_869.525MHz_1000ksps.cs8 [CENTRE_HZ]
//! ```

use std::time::Instant;

use declab_lora::receiver::{Config, INPUT_RATE};
use declab_lora::{Keys, Payload, Receiver, decode_payload, sim::Noise};
use num_complex::Complex32;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("IN.cs8|IN.cf32");
    let centre: f64 = args
        .get(2)
        .map_or(869.525e6, |s| s.parse().expect("CENTRE_HZ"));
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
    let secs = iq.len() as f64 / INPUT_RATE;
    let keys = Keys::default();
    let mut best = f64::MAX;
    for run in 0..3 {
        let mut rx = Receiver::new(INPUT_RATE, Config::default());
        rx.centre_hz = centre;
        let t = Instant::now();
        for b in iq.chunks(65_536) {
            rx.process(b);
        }
        best = best.min(t.elapsed().as_secs_f64());
        if run == 0 {
            for f in &rx.frames {
                let fr = f.frame.as_ref().unwrap();
                let what = if !fr.crc_ok {
                    "CRC failed".to_string()
                } else {
                    match decode_payload(f.sync, &fr.payload, &keys) {
                        Payload::Meshtastic(p) => format!(
                            "Meshtastic {} → {} {} {}",
                            declab_lora::meshtastic::node_name(p.header.from),
                            declab_lora::meshtastic::node_name(p.header.to),
                            p.kind,
                            p.summary
                        ),
                        Payload::MeshCore(p) => format!(
                            "MeshCore {} {} {}",
                            declab_lora::meshcore::ROUTES[p.route as usize],
                            declab_lora::meshcore::payload_type_name(p.payload_type),
                            p.summary
                        ),
                        Payload::Other => format!("{} bytes", fr.payload.len()),
                    }
                };
                println!(
                    "{:7.3} s  SF{} {:5.1} kHz 4/{} sync {:>4}  {:5.1} dB  {:+7.0} Hz  {}",
                    f.at_s,
                    f.sf,
                    f.bw_hz / 1e3,
                    fr.header.cr + 4,
                    f.sync.map_or("?".into(), |s| format!("{s:02x}")),
                    f.snr_db,
                    f.cfo_hz,
                    what
                );
            }
            println!("{:?}", rx.stats());
        }
    }
    println!(
        "file: {secs:.1} s, receiver {:.2} % of a core",
        100.0 * best / secs
    );
    let mut noise = vec![Complex32::default(); 10_000_000];
    Noise::new(3).add(&mut noise, 1e-4);
    let mut best = f64::MAX;
    for _ in 0..3 {
        let mut rx = Receiver::new(INPUT_RATE, Config::default());
        let t = Instant::now();
        for b in noise.chunks(65_536) {
            rx.process(b);
        }
        best = best.min(t.elapsed().as_secs_f64());
    }
    println!(
        "noise alone: receiver {:.2} % of a core",
        100.0 * best / 10.0
    );
}
