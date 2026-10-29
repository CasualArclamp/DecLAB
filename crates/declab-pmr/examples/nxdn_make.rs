// made by CasualArclamp
//! Write a simulated NXDN48 call to an IQ file, for the GUI.
//!
//! `cargo run --release -p declab-pmr --example nxdn_make -- OUT.cf32 [C/N dB]`
//! then `declab-gui OUT.cf32 --rate 48000 --decoder nxdn`.

use std::io::Write;

use declab_pmr::nxdn::{Message, sim};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .cloned()
        .unwrap_or_else(|| "out/sim_nxdn.cf32".into());
    let cn: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let rate = 48_000.0;
    let msg = Message::vcall(0x01, 1234, 77, true);
    let (mut bits, _) = sim::call(5, &msg, 8);
    let (b2, _) = sim::call(5, &Message::vcall(0x01, 2001, 9, true), 5);
    bits.extend(b2);
    let mut iq = sim::modulate(&bits, rate, 0.3);
    sim::add_noise(&mut iq, rate, cn, -450.0, 42);
    // Well below full scale, as a radio's samples would be.
    iq.iter_mut().for_each(|s| *s *= 0.25);
    let mut f = std::io::BufWriter::new(std::fs::File::create(&path)?);
    for s in &iq {
        f.write_all(&s.re.to_le_bytes())?;
        f.write_all(&s.im.to_le_bytes())?;
    }
    println!(
        "{path}: {:.1} s at {rate} S/s, C/N {cn} dB",
        iq.len() as f64 / rate
    );
    Ok(())
}
