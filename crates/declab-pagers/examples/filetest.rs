// made by CasualArclamp
//! Decode a 50 kS/s cf32 channel file and time the receiver, with and
//! without the Manchester chain (debug and CPU aid).
//!
//! ```text
//! cargo run --release -p declab-pagers --example filetest -- IN.cf32
//! ```
use declab_core::Iq;
use declab_pagers::{Receiver, ReceiverConfig};
use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).expect("IN.cf32");
    let b = std::fs::read(path).unwrap();
    let iq: Vec<Iq> = b
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| {
            let f = |k: usize| f32::from_le_bytes(c[k..k + 4].try_into().unwrap());
            Iq::new(f(0), f(4))
        })
        .collect();
    let secs = iq.len() as f64 / 50_000.0;
    for manchester in [vec![], vec![512]] {
        let cfg = ReceiverConfig {
            manchester: manchester.clone(),
            ..Default::default()
        };
        // Fastest of five: other builds make single timings noisy.
        let mut best = f64::MAX;
        let mut out = Vec::new();
        for _ in 0..5 {
            out.clear();
            let mut rx = Receiver::new(50_000.0, &cfg);
            let t = Instant::now();
            for blk in iq.chunks(4096) {
                rx.process_iq(blk, &mut out);
            }
            rx.finish(&mut out);
            best = best.min(t.elapsed().as_secs_f64());
        }
        println!(
            "manchester {manchester:?}: {:.2} % of a core, {} pages",
            100.0 * best / secs,
            out.len()
        );
        for p in &out {
            println!("  {p}");
        }
    }
}
