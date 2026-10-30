// made by CasualArclamp
//! Write a simulated P25 phase 1 call and control channel to an IQ file,
//! for the GUI.
//!
//! `cargo run --release -p declab-pmr --example p25_make -- OUT.cf32 [C/N dB]`
//! then `declab-gui OUT.cf32 --rate 48000 --decoder voice`.

use std::io::Write;

use declab_pmr::p25::{Band, Lc, ServiceOptions, Tsbk, sim};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .cloned()
        .unwrap_or_else(|| "out/sim_p25.cf32".into());
    let cn: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let rate = 48_000.0;
    let lc = Lc::GroupVoice {
        opts: ServiceOptions(0),
        group: 101,
        source: 1_234_567,
    };
    let (mut bits, _) = sim::call(0x293, &lc, 6);
    let iden = Tsbk::Iden {
        id: 1,
        band: Band {
            base_hz: 851_006_250.0,
            spacing_hz: 6250.0,
            slots: 1,
        },
    };
    let grant = Tsbk::GroupGrant {
        opts: ServiceOptions(0),
        band: 1,
        channel: 0x10,
        group: 202,
        source: 77,
    };
    let rfss = Tsbk::RfssStatus {
        system: 0x3a1,
        rfss: 2,
        site: 9,
        band: 1,
        channel: 0x21,
    };
    for k in 0..30 {
        let t = match k % 3 {
            0 => vec![iden],
            1 => vec![rfss, grant],
            _ => vec![rfss],
        };
        bits.extend(sim::tsdu(0x293, &t));
    }
    let mut iq = sim::modulate(&bits, rate, 0.3);
    sim::add_noise(&mut iq, rate, cn, 520.0, 42);
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
