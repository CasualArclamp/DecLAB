// made by CasualArclamp
//! `declab list` shows the decoders; `declab selftest <id>` runs one over a
//! second of generated noise, proving the host side of the plug-in API.
//! File and radio input arrive with the shared front end (DESIGN.md, M1).

use anyhow::{Context, bail};
use declab_api::{Block, Iq, Sink, Tuning};

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("list") | None => {
            for f in declab_decoders::ALL {
                let i = f.info();
                println!("{:<12} {:<16} {:<15} {}", i.id, i.name, i.family.label(), i.summary);
            }
            Ok(())
        }
        Some("selftest") => {
            let id = args.get(1).context("usage: declab selftest <decoder id>")?;
            selftest(id)
        }
        Some(other) => bail!("unknown command {other:?} (try `list` or `selftest <id>`)"),
    }
}

fn selftest(id: &str) -> anyhow::Result<()> {
    let factory = declab_decoders::find(id).with_context(|| format!("no decoder {id:?}"))?;
    let spec = factory.input(20e6);
    let rate = spec.sample_rate.unwrap_or(20e6);
    let mut dec = factory.create();
    let mut sink = Sink::default();

    // White noise at -20 dBFS from a small xorshift, so no rand dependency.
    let mut state = 0x2545_f491_4f6c_dd1du64;
    let mut noise = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 40) as f32 / (1u64 << 24) as f32 - 0.5
    };
    let scale = 0.1 * 12f32.sqrt() / 2f32.sqrt();
    let mut iq = vec![Iq::default(); spec.block_len];
    let tuning = Tuning { center_hz: 0.0, sample_rate: rate, generation: 0 };

    let mut first = 0u64;
    while (first as f64) < rate {
        for s in iq.iter_mut() {
            *s = Iq::new(noise() * scale, noise() * scale);
        }
        let block = Block { iq: &iq, tuning, first_sample: first, dropped_before: 0 };
        sink.clear();
        dec.process(&block, &mut sink);
        for m in &sink.measurements {
            println!("{:>10.0}  {:?}  {} = {:.2} {}", first, sink.lock, m.name, m.value, m.unit);
        }
        first += iq.len() as u64;
    }
    Ok(())
}
