// made by CasualArclamp
//! Write a simulated P25 phase 2 traffic channel to an IQ file, for the
//! GUI: a call on each slot (the second encrypted), scrambled with
//! WACN BEE00, system 3A1, NAC 293 (found by the receiver from the air).
//!
//! `cargo run --release -p declab-pmr --example p25p2_make -- OUT.cf32 [C/N dB]`
//! then `declab-gui OUT.cf32 --rate 48000 --decoder voice`.

use std::io::Write;

use declab_pmr::p25p2::scramble;
use declab_pmr::p25p2::sim::{self, Call};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args.first().cloned().unwrap_or_else(|| "out/sim_p25p2.cf32".into());
    let cn: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let rate = 48_000.0;
    let seed = scramble::seed(0xBEE00, 0x3A1, 0x293);
    let calls = [
        Some(Call { group: 4001, source: 1_234_567, algid: 0x80 }),
        Some(Call { group: 77, source: 7_654_321, algid: 0x84 }),
    ];
    let (dibits, frames) = sim::channel(seed, calls, 30);
    let mut iq = sim::modulate(&dibits, rate, 0.3);
    sim::add_noise(&mut iq, rate, cn, 520.0, 42);
    // Well below full scale, as a radio's samples would be.
    iq.iter_mut().for_each(|s| *s *= 0.25);
    let mut f = std::io::BufWriter::new(std::fs::File::create(&path)?);
    for s in &iq {
        f.write_all(&s.re.to_le_bytes())?;
        f.write_all(&s.im.to_le_bytes())?;
    }
    println!("{path}: {:.1} s at {rate} S/s, {frames} voice frames", iq.len() as f64 / rate);
    Ok(())
}
