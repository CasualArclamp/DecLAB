// made by CasualArclamp
//! Write a simulated TETRA cell with a group call, short data and a status
//! to an IQ file, for the GUI.
//!
//! `cargo run --release -p declab-pmr --example tetra_make -- OUT.cf32 [C/N dB]`
//! then `declab-gui OUT.cf32 --rate 72000 --decoder voice`.

use std::io::Write;

use declab_pmr::tetra::receiver::RATE;
use declab_pmr::tetra::sim::{self, Cell};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .cloned()
        .unwrap_or_else(|| "out/sim_tetra.cf32".into());
    let cn: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let mut cell = Cell::new(234, 1234, 17);
    let (bits, _) = sim::call(&mut cell, 9001, 4_000_123, 60);
    let mut iq = sim::modulate(&bits, RATE, 0.3);
    sim::add_noise(&mut iq, RATE, cn, 400.0, 42);
    // Well below full scale, as a radio's samples would be.
    iq.iter_mut().for_each(|s| *s *= 0.25);
    let mut f = std::io::BufWriter::new(std::fs::File::create(&path)?);
    for s in &iq {
        f.write_all(&s.re.to_le_bytes())?;
        f.write_all(&s.im.to_le_bytes())?;
    }
    println!(
        "{path}: {:.1} s at {RATE} S/s, C/N {cn} dB",
        iq.len() as f64 / RATE
    );
    Ok(())
}
