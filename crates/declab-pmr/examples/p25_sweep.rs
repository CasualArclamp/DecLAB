// made by CasualArclamp
//! Sensitivity of the P25 phase 1 receiver: a simulated call and a control
//! channel at falling C/N (in 12.5 kHz), where the SNR gate sits, and CPU.
//!
//! `cargo run --release -p declab-pmr --example p25_sweep [-- SEEDS [OFFSET_HZ [CUTOFF_HZ]]]`
//!
//! Columns: voice frames whose protected bits came out right; link control
//! words decoded; TSBKs decoded; the voice channel BER.

use declab_pmr::fsk4::Iq;
use declab_pmr::p25::receiver::RATE;
use declab_pmr::p25::{Band, Event, Lc, Receiver, ServiceOptions, Tsbk, sim};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seeds: u64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(4);
    let offset: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(300.0);
    let chan: f64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(declab_pmr::p25::receiver::CHANNEL_HZ);

    let lc = Lc::GroupVoice {
        opts: ServiceOptions(0),
        group: 101,
        source: 1_234_567,
    };
    let (mut bits, frames) = sim::call(0x293, &lc, 4);
    let (b2, f2) = sim::call(0x293, &lc, 4);
    bits.extend(b2);
    let frames = frames + f2;
    let lcs_sent = 2 * (4 + 1);
    let grant = Tsbk::GroupGrant {
        opts: ServiceOptions(0),
        band: 1,
        channel: 0x10,
        group: 101,
        source: 77,
    };
    let iden = Tsbk::Iden {
        id: 1,
        band: Band {
            base_hz: 851_006_250.0,
            spacing_hz: 6250.0,
            slots: 1,
        },
    };
    let mut cc = Vec::new();
    let mut tsbks_sent = 0;
    for k in 0..40 {
        let t = if k % 2 == 0 {
            vec![grant]
        } else {
            vec![iden, grant]
        };
        tsbks_sent += t.len();
        cc.extend(sim::tsdu(0x3a1, &t));
    }
    let clean = sim::modulate(&bits, RATE, 0.3);
    let clean_cc = sim::modulate(&cc, RATE, 0.3);

    let secs = clean.len() as f64 / RATE;
    let t = std::time::Instant::now();
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in clean.chunks(2400) {
        rx.process(c, &mut ev);
    }
    println!(
        "CPU: {:.2} % of one core decoding",
        100.0 * t.elapsed().as_secs_f64() / secs
    );

    let snrs = |iq: &[Iq]| {
        let mut rx = Receiver::new();
        rx.snr_gate_db = f32::MIN;
        let (mut ev, mut v, mut seen) = (Vec::new(), Vec::new(), 0);
        for c in iq.chunks(240) {
            rx.process(c, &mut ev);
            if rx.stats.syncs > seen {
                seen = rx.stats.syncs;
                v.push(rx.stats.sync_snr_db);
            }
        }
        v.sort_by(f32::total_cmp);
        v
    };
    let pct = |v: &[f32], p: f32| {
        if v.is_empty() {
            f32::NAN
        } else {
            v[((v.len() - 1) as f32 * p) as usize]
        }
    };
    let mut noise = vec![Iq::new(0.0, 0.0); 24_000 * 120];
    sim::add_noise(&mut noise, RATE, 0.0, 0.0, 77);
    let t = std::time::Instant::now();
    let v = snrs(&noise);
    println!(
        "gate: noise 120 s, {} sync candidates, eye SNR max {:.1} median {:.1} dB (hunting CPU {:.2} %)",
        v.len(),
        v.last().copied().unwrap_or(f32::NAN),
        pct(&v, 0.5),
        100.0 * t.elapsed().as_secs_f64() / 120.0
    );
    let mut rx = Receiver::new();
    let mut ev = Vec::new();
    for c in noise.chunks(2400) {
        rx.process(c, &mut ev);
    }
    println!(
        "gate on, noise 120 s: {} events, {} candidates gated",
        ev.len(),
        rx.stats.gated
    );
    for cn in [4.0, 6.0] {
        let mut iq = clean.clone();
        sim::add_noise(&mut iq, RATE, cn, offset, 9);
        let v = snrs(&iq);
        println!(
            "gate: C/N {cn} dB, eye SNR 5% {:.1} median {:.1} dB",
            pct(&v, 0.05),
            pct(&v, 0.5)
        );
    }

    println!("{seeds} runs a point, {frames} voice frames, {offset} Hz offset, cut-off {chan} Hz");
    println!("C/N dB | voice   LC     TSBK   BER");
    for cn in (1..=14).rev().map(|x| x as f64) {
        let (mut good, mut lcs, mut tsbks, mut errs, mut nbits) = (0u32, 0u32, 0usize, 0u64, 0u64);
        for seed in 0..seeds {
            let mut iq = clean.clone();
            sim::add_noise(&mut iq, RATE, cn, offset, 1 + seed * 7919);
            let mut rx = Receiver::with_channel(chan);
            let mut ev = Vec::new();
            for c in iq.chunks(2400) {
                rx.process(c, &mut ev);
            }
            for e in &ev {
                match e {
                    Event::Voice { frames: f } => {
                        for fr in f {
                            if (0..frames).any(|k| sim::test_frame(k)[..81] == fr.bits[..81]) {
                                good += 1;
                            }
                        }
                    }
                    Event::Lc { lc: l } if *l == lc => lcs += 1,
                    _ => {}
                }
            }
            errs += rx.stats.voice_bit_errors;
            nbits += rx.stats.voice_bits;
            let mut iq = clean_cc.clone();
            sim::add_noise(&mut iq, RATE, cn, offset, 2 + seed * 7919);
            let mut rx = Receiver::with_channel(chan);
            let mut ev = Vec::new();
            for c in iq.chunks(2400) {
                rx.process(c, &mut ev);
            }
            tsbks += ev
                .iter()
                .filter(|e| matches!(e, Event::Tsbk { .. }))
                .count();
        }
        let n = seeds as f64;
        println!(
            "{cn:6.1} | {:5.1}% {:5.1}% {:5.1}% {:5.2}%",
            100.0 * f64::from(good) / (f64::from(frames) * n),
            100.0 * f64::from(lcs) / (lcs_sent as f64 * n),
            100.0 * tsbks as f64 / (tsbks_sent as f64 * n),
            100.0 * errs as f64 / nbits.max(1) as f64
        );
    }
}
