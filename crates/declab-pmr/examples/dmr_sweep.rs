// made by CasualArclamp
//! Sensitivity of the DMR receiver: a simulated Tier III site (a group call
//! on TS1, CSBKs on TS2) at falling C/N, both frequency detectors.
//!
//! `cargo run --release -p declab-pmr --example dmr_sweep [-- SEEDS [OFFSET_HZ [CUTOFFS]]]`
//! (CUTOFFS, e.g. `4500,5500,6500`, compares channel filters instead.)
//!
//! Columns per detector: voice frames whose protected bits (C0, C1) came
//! out right, of those sent; CSBKs decoded; the channel BER the AMBE Golay
//! words saw; and whether the call's LC was read (header or embedded).

use declab_pmr::dmr::pdu::{Csbk, FullLc, GrantKind, ServiceOptions, ShortLc, SysCode};
use declab_pmr::dmr::receiver::{CHANNEL_HZ, RATE};
use declab_pmr::dmr::sim::{self, Repeater};
use declab_pmr::dmr::{Event, Receiver, Source};
use declab_pmr::fsk4::Detector;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seeds: u64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(6);
    let offset: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(300.0);
    // Optional: channel low-pass cut-offs to compare (Hz), discriminator only.
    let chans: Vec<f64> = args
        .get(2)
        .map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_default();

    let sys = SysCode((0b01 << 14) | (12 << 7) | (4 << 2));
    let mut r = Repeater::new(
        1,
        ShortLc::ControlParms {
            syscode14: sys.0 >> 2,
            reg: true,
            counter: 1,
        },
    );
    let lc = FullLc::GroupVoice {
        opts: ServiceOptions(0),
        group: 2350,
        source: 2_341_234,
    };
    let grant = Csbk::Grant {
        kind: GrantKind::TalkgroupVoice,
        lpcn: 3,
        slot: 0,
        late_entry: false,
        emergency: false,
        offset: false,
        target: 2350,
        source: 2_341_234,
    };
    let idle = sim::idle_burst(Source::Bs, 1);
    r.push(&idle);
    r.push(&idle);
    let mut csbks_sent = 0;
    r.voice_call(&lc, 8, |rep| {
        csbks_sent += 1;
        sim::csbk_burst(Source::Bs, rep.cc, &grant)
    });
    r.push(&idle);
    let frames = r.frames;
    let clean = sim::modulate(&r.bits, RATE, 0.05);

    // CPU: the receiver alone on the clean signal, fastest of three.
    let secs = clean.len() as f64 / RATE;
    let best = (0..3)
        .map(|_| {
            let t = std::time::Instant::now();
            let mut rx = Receiver::new(Detector::Discriminator);
            let mut ev = Vec::new();
            for chunk in clean.chunks(2400) {
                rx.process(chunk, &mut ev);
            }
            t.elapsed().as_secs_f64()
        })
        .fold(f64::MAX, f64::min);
    println!(
        "CPU: {:.1} ms for {secs:.1} s of signal, {:.2} % of one core",
        best * 1e3,
        100.0 * best / secs
    );
    // Hunting costs more: the SYNC correlators run at every sample.
    let mut noise = vec![declab_pmr::fsk4::Iq::new(0.0, 0.0); clean.len()];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 5);
    let t = std::time::Instant::now();
    let mut rx = Receiver::new(Detector::Discriminator);
    let mut ev = Vec::new();
    for chunk in noise.chunks(2400) {
        rx.process(chunk, &mut ev);
    }
    println!(
        "CPU hunting on noise: {:.2} % of one core",
        100.0 * t.elapsed().as_secs_f64() / secs
    );
    println!(
        "{seeds} runs a point, {frames} voice frames and {csbks_sent} CSBKs a run, {offset} Hz offset"
    );
    println!("C/N dB | product: voice  CSBK   BER   LC | discriminator: voice  CSBK   BER   LC");
    for cn in (2..=16).rev().map(|x| x as f64) {
        print!("{cn:6.1} |");
        let runs: Vec<(Detector, f64)> = if chans.is_empty() {
            vec![
                (Detector::Product, CHANNEL_HZ),
                (Detector::Discriminator, CHANNEL_HZ),
            ]
        } else {
            chans
                .iter()
                .map(|&c| (Detector::Discriminator, c))
                .collect()
        };
        for (det, chan) in runs {
            let (mut good, mut cs, mut lcs, mut errs, mut bits) = (0u32, 0u32, 0u32, 0u64, 0u64);
            for seed in 0..seeds {
                let mut iq = clean.clone();
                sim::add_noise(&mut iq, RATE, cn, offset, 1 + seed * 7919);
                let mut rx = Receiver::with_channel(det, chan);
                let mut ev = Vec::new();
                for chunk in iq.chunks(2400) {
                    rx.process(chunk, &mut ev);
                }
                let mut n = 0u32;
                let mut lc_seen = false;
                for e in &ev {
                    match e {
                        Event::Voice {
                            slot: 0, frames: f, ..
                        } => {
                            for fr in f {
                                // Frames are numbered in order; a lost burst
                                // shifts the count, so match by content.
                                let _ = n;
                                n += 1;
                                if (0..frames).any(|k| sim::test_frame(k)[..24] == fr.bits[..24]) {
                                    good += 1;
                                }
                            }
                        }
                        Event::Csbk { csbk, .. } if *csbk == grant => cs += 1,
                        Event::VoiceHeader { lc: l, .. } | Event::EmbeddedLc { lc: l, .. }
                            if *l == lc =>
                        {
                            lc_seen = true
                        }
                        _ => {}
                    }
                }
                lcs += u32::from(lc_seen);
                errs += rx.stats.voice_bit_errors;
                bits += rx.stats.voice_bits;
            }
            let total = f64::from(frames) * seeds as f64;
            print!(
                " {:12.1}% {:5.1}% {:5.2}% {:3}/{} |",
                100.0 * f64::from(good) / total,
                100.0 * f64::from(cs) / (csbks_sent as f64 * seeds as f64),
                100.0 * errs as f64 / bits.max(1) as f64,
                lcs,
                seeds
            );
        }
        println!();
    }
}
