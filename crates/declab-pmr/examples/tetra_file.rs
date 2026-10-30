// made by CasualArclamp
//! Run the TETRA receiver over a 72 kS/s cf32 file and print each slot's
//! training fit: what a real carrier looks like slot by slot.
//!
//! `cargo run --release -p declab-pmr --example tetra_file -- IN.cf32`

use declab_pmr::fsk4::Iq;
use declab_pmr::tetra::mac::Signal;
use declab_pmr::tetra::{Event, Receiver};

fn main() -> std::io::Result<()> {
    let path = std::env::args().nth(1).expect("a 72 kS/s cf32 file");
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
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    let mut seen = 0;
    for chunk in iq.chunks(255) {
        rx.process(chunk, &mut ev);
        for e in ev.drain(..) {
            match e {
                Event::Traffic { tn, .. } => println!("    traffic tn {tn}"),
                Event::Signal {
                    signal: Some(Signal::NwrkBroadcast { neighbours }),
                    ..
                } => {
                    for n in neighbours {
                        println!("    neighbour {n:?}");
                    }
                }
                Event::Signal { tn, .. } => println!("    signal tn {tn}"),
                e => println!("    {e:?}"),
            }
        }
        let st = &rx.stats;
        if st.slots != seen {
            seen = st.slots;
            println!(
                "slot {:5} next tn {} fr {:2}  q {:.2}  snr {:5.1} dB  bad {}  resyncs {}  locked {}  off {:.0}",
                st.slots,
                st.tn,
                st.frame,
                st.quality,
                st.eye_snr_db,
                st.slots_bad,
                st.resyncs,
                st.locked,
                st.offset_hz
            );
        }
    }
    Ok(())
}
