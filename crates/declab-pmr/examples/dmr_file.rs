// made by CasualArclamp
//! Run the DMR receiver over a 24 kS/s cf32 file and print every event but
//! voice (counted), with the receiver's counters at the end: what a real
//! channel carries burst by burst.
//!
//! `cargo run --release -p declab-pmr --example dmr_file -- IN.cf32`

use declab_pmr::dmr::{Event, Receiver};
use declab_pmr::fsk4::{Detector, Iq};

fn main() -> std::io::Result<()> {
    let path = std::env::args().nth(1).expect("a 24 kS/s cf32 file");
    let raw = std::fs::read(path)?;
    // Rust note: `as_chunks::<4>()` views the bytes as `[u8; 4]` floats.
    let f: Vec<f32> = raw
        .as_chunks::<4>()
        .0
        .iter()
        .map(|&b| f32::from_le_bytes(b))
        .collect();
    let iq: Vec<Iq> = f
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&[re, im]| Iq::new(re, im))
        .collect();
    let mut rx = Receiver::new(Detector::Discriminator);
    let mut ev = Vec::new();
    let mut voice = 0;
    for (n, chunk) in iq.chunks(2400).enumerate() {
        rx.process(chunk, &mut ev);
        let t = (n + 1) as f64 * 0.1;
        for e in ev.drain(..) {
            match e {
                Event::Voice { .. } => voice += 1,
                e => println!("{t:6.1} {e:?}"),
            }
        }
    }
    println!("{voice} voice bursts");
    println!("{:#?}", rx.stats);
    Ok(())
}
