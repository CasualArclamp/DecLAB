// made by CasualArclamp
//! Run the NXDN48 receiver over a 24 kS/s cf32 file and print every frame:
//! its LICH (type, function, option, direction, parity), whether it passed,
//! the eye SNR, and every event but voice (counted). What a real channel
//! carries frame by frame.
//!
//! `cargo run --release -p declab-pmr --example nxdn_file -- IN.cf32 [96] [bits]`

use declab_pmr::fsk4::Iq;
use declab_pmr::nxdn::receiver::{Event, Receiver};
use declab_pmr::nxdn::{Mode, receiver};

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("a 24 kS/s cf32 file");
    let mode = if args.iter().any(|a| a == "96") {
        Mode::Nxdn96
    } else {
        Mode::Nxdn48
    };
    let show_bits = args.iter().any(|a| a == "bits");
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
    let mut rx = Receiver::with_channel(mode, receiver::channel_for(mode).0);
    rx.frame_log = Some(Vec::new());
    let mut ev = Vec::new();
    let mut voice = 0;
    for (n, chunk) in iq.chunks(2400).enumerate() {
        rx.process(chunk, &mut ev);
        let t = (n + 1) as f64 * 0.1;
        for fr in rx.frame_log.as_mut().expect("set").drain(..) {
            let l = fr.lich;
            print!(
                "{t:6.1} LICH {:02x} {:?} fct {} {:?} {} parity {} {} {:5.1} dB",
                l.0,
                l.rfct(),
                l.fct(),
                l.steal(),
                if l.outbound() { "out" } else { "in " },
                if l.parity_ok() { "ok " } else { "BAD" },
                if fr.good { "good" } else { "bad " },
                fr.eye_snr_db
            );
            if show_bits {
                let hex: String = fr
                    .bits
                    .chunks(8)
                    .map(|c| {
                        format!(
                            "{:02x}",
                            c.iter().fold(0u8, |v, &b| (v << 1) | b) << (8 - c.len())
                        )
                    })
                    .collect();
                print!("  {hex}");
            }
            println!();
        }
        for e in ev.drain(..) {
            match e {
                Event::Voice { .. } => voice += 1,
                e => println!("{t:6.1}   {e:?}"),
            }
        }
    }
    println!("{voice} voice events");
    println!("{:#?}", rx.stats);
    Ok(())
}
