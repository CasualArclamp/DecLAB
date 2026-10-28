// made by CasualArclamp
//! Sensitivity sweep: for each mode, pages decoded exactly (and false pages)
//! against C/N in 16 kHz, with a tuning offset, at the 50 kS/s channel rate.
//!
//! ```text
//! cargo run --release -p declab-pagers --example sweep [-- OFFSET_HZ [CN_LIST]]
//! ```

use declab_pagers::flex::{self, FlexBody, FlexPage};
use declab_pagers::modulate::SignalBuilder;
use declab_pagers::pocsag::{Body, PocsagMessage};
use declab_pagers::{Page, Receiver, ReceiverConfig};

const RATE: f64 = 50_000.0;

/// A mode's test signal and the pages it should give (capcode, text).
fn signal(mode: usize) -> (SignalBuilder, Vec<(u64, String)>, &'static str) {
    let mut sig = SignalBuilder::new(RATE);
    let mut want = Vec::new();
    sig.silence(0.3);
    let names = [
        "POCSAG512",
        "POCSAG1200",
        "POCSAG2400",
        "FLEX1600/2",
        "FLEX3200/2",
        "FLEX3200/4",
        "FLEX6400/4",
    ];
    if mode < 3 {
        let baud = [512, 1200, 2400][mode];
        for t in 0..12u32 {
            let msgs: Vec<PocsagMessage> = (0..2u32)
                .map(|k| {
                    let capcode = 100_000 + t * 10 + k;
                    let text = format!("tx {t} page {k} low SNR test 0123456789");
                    want.push((u64::from(capcode), text.clone()));
                    PocsagMessage {
                        capcode,
                        function: 3,
                        body: Body::Alpha(text),
                    }
                })
                .collect();
            sig.pocsag(baud, &msgs);
            sig.silence(0.25);
        }
    } else {
        let (baud, levels) = flex::SPEEDS[mode - 3];
        for f in 0..8u8 {
            let pages: Vec<FlexPage> = (0..4u64)
                .map(|k| {
                    let capcode = 200_000 + u64::from(f) * 10 + k;
                    let text = format!("frame {f} page {k} low SNR test 0123456789");
                    want.push((capcode, text.clone()));
                    FlexPage {
                        capcode,
                        body: FlexBody::Alpha(text),
                    }
                })
                .collect();
            sig.flex(&flex::encode_frame(baud, levels, 0, f, &pages).unwrap());
        }
        sig.silence(0.3);
    }
    (sig, want, names[mode])
}

fn decode(iq: &[declab_core::Iq]) -> Vec<Page> {
    let mut rx = Receiver::new(RATE, &ReceiverConfig::default());
    let mut out = Vec::new();
    for b in iq.chunks(5000) {
        rx.process_iq(b, &mut out);
    }
    rx.finish(&mut out);
    out
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let offset: f64 = args.first().and_then(|a| a.parse().ok()).unwrap_or(300.0);
    let cns: Vec<f64> = args
        .get(1)
        .map(|s| s.split(',').filter_map(|v| v.parse().ok()).collect())
        .unwrap_or_else(|| vec![-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0]);
    print!("{:<12}", "C/N dB");
    for cn in &cns {
        print!("{cn:>9}");
    }
    println!("   (pages ok/sent, +false)");
    let t0 = std::time::Instant::now();
    let mut audio_s = 0.0;
    for mode in 0..7 {
        let (sig, want, name) = signal(mode);
        print!("{name:<12}");
        for (i, &cn) in cns.iter().enumerate() {
            let iq = sig.to_iq(offset, Some(cn), 1.0 / 9600.0, 11 + i as u64);
            audio_s += iq.len() as f64 / RATE;
            let pages = decode(&iq);
            if std::env::var("SWEEP_SHOW").is_ok() {
                for p in &pages {
                    let ok = want.iter().any(|(c, t)| p.capcode == *c && &p.text == t);
                    if !ok {
                        eprintln!(
                            "  {} {p}",
                            if want.iter().any(|(c, _)| *c == p.capcode) {
                                "WRONG"
                            } else {
                                "FALSE"
                            }
                        );
                    }
                }
            }
            let ok = want
                .iter()
                .filter(|(c, t)| pages.iter().any(|p| p.capcode == *c && &p.text == t))
                .count();
            let false_pages = pages
                .iter()
                .filter(|p| !want.iter().any(|(c, _)| *c == p.capcode))
                .count();
            let cell = if false_pages > 0 {
                format!("{ok}/{}+{false_pages}", want.len())
            } else {
                format!("{ok}/{}", want.len())
            };
            print!("{cell:>9}");
        }
        println!();
    }
    eprintln!(
        "{audio_s:.0} s of signal decoded in {:.1} s",
        t0.elapsed().as_secs_f64()
    );
}
