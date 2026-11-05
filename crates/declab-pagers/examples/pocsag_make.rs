// made by CasualArclamp
//! A simulated on-site paging transmitter: POCSAG 512 Manchester-coded,
//! a burst every few seconds after most of a second of bare carrier, as
//! seen on air. Written as cf32 at 250 kS/s with the carrier 40 kHz above
//! the centre, for the GUI:
//!
//! ```text
//! cargo run --release -p declab-pagers --example pocsag_make -- out/sim_pocsag_m.cf32 [SECONDS]
//! cargo run --release -p declab-gui -- out/sim_pocsag_m.cf32 --rate 250000 --decoder pagers --chan 40k
//! ```

use declab_pagers::modulate::SignalBuilder;
use declab_pagers::pocsag::{Body, PocsagMessage};

const RATE: f64 = 250_000.0;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("OUT.cf32");
    let seconds: f64 = args.get(2).map_or(20.0, |s| s.parse().expect("SECONDS"));
    let mut sig = SignalBuilder::new(RATE);
    let mut n = 0u32;
    sig.silence(0.5);
    while sig.seconds() < seconds {
        let capcode = 100_000 + 8 * (n % 12);
        let body = match n % 4 {
            0 => Body::Tone,
            1 => Body::Numeric(format!("{:03}", 10 + n)),
            _ => Body::Alpha(format!("Table {} ready", 10 + n)),
        };
        let msgs = [PocsagMessage {
            capcode,
            function: if matches!(body, Body::Alpha(_)) { 3 } else { 0 },
            body,
        }];
        // The carrier comes up most of a second before the preamble.
        sig.carrier(0.8);
        sig.pocsag_manchester(512, &msgs);
        sig.silence(2.5);
        n += 1;
    }
    let iq = sig.to_iq(40_000.0, Some(18.0), 1.0 / 4800.0, 11);
    let mut bytes = Vec::with_capacity(iq.len() * 8);
    // Well below full scale, as a radio's ADC would see it.
    for z in &iq {
        bytes.extend_from_slice(&(0.2 * z.re).to_le_bytes());
        bytes.extend_from_slice(&(0.2 * z.im).to_le_bytes());
    }
    std::fs::write(path, bytes).expect("write");
    println!("{path}: {:.1} s, {n} pages", sig.seconds());
}
