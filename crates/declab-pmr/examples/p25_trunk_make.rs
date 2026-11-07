// made by CasualArclamp
//! Write a simulated trunked P25 site to an IQ file: a phase 1 control
//! channel 50 kHz below the centre, granting TG 4001 onto a phase 2
//! traffic channel 40 kHz above it, which keys up for the call. The
//! control channel's network status carries the WACN and system the
//! traffic channel is scrambled with. For the GUI's call following.
//!
//! `cargo run --release -p declab-pmr --example p25_trunk_make -- OUT.cf32 [C/N dB]`
//! then `declab-gui OUT.cf32 --rate 240000 --freq 851M --decoder voice --chan 850.95M`.

use std::io::Write;

use declab_pmr::fsk4::Iq;
use declab_pmr::p25::{Band, ServiceOptions, Tsbk, sim as p1};
use declab_pmr::p25p2::scramble;
use declab_pmr::p25p2::sim::{self as p2, Call};

const RATE: f64 = 240_000.0;
const CENTRE: f64 = 851_000_000.0;
const CC_HZ: f64 = CENTRE - 50_000.0;
const TRAFFIC_HZ: f64 = CENTRE + 40_000.0;
const NAC: u16 = 0x293;
const WACN: u32 = 0xBEE00;
const SYSTEM: u16 = 0x3A1;

/// Shift a baseband signal to `hz` from the centre.
fn shift(iq: &mut [Iq], hz: f64) {
    let w = std::f64::consts::TAU * hz / RATE;
    for (n, z) in iq.iter_mut().enumerate() {
        *z *= Iq::from_polar(1.0, (w * n as f64) as f32);
    }
}

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args.first().cloned().unwrap_or_else(|| "out/sim_p25_trunk.cf32".into());
    let cn: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(18.0);

    // The traffic channel: TDMA band, 12.5 kHz spacing, two slots a
    // carrier; channel number 2k is carrier k's first slot.
    let band = Band { base_hz: TRAFFIC_HZ - 12_500.0 * 8.0, spacing_hz: 12_500.0, slots: 2 };
    let channel = 16u16;
    assert!((band.downlink_hz(channel) - TRAFFIC_HZ).abs() < 1.0);
    let iden = Tsbk::Iden { id: 2, band };
    let rfss = Tsbk::RfssStatus { system: SYSTEM, rfss: 1, site: 4, band: 2, channel: 0 };
    let net = Tsbk::NetStatus { wacn: WACN, system: SYSTEM, band: 2, channel: 0 };
    let grant = Tsbk::GroupGrant { opts: ServiceOptions(0), band: 2, channel, group: 4001, source: 1_234_567 };

    // Phase 2 call: keyed from 4 s, 20 superframes of voice (7.2 s).
    let call_at = 4.0;
    let seed = scramble::seed(WACN, SYSTEM, NAC);
    let (d2, frames) = p2::channel(seed, [Some(Call { group: 4001, source: 1_234_567, algid: 0x80 }), None], 20);
    let traffic = p2::modulate(&d2, RATE, 0.0);
    let call_len = traffic.len() as f64 / RATE;
    let total = call_at + call_len + 8.0;

    // The control channel: status at first, grants (twice a second) while
    // the call stands.
    let mut bits = Vec::new();
    let mut k = 0usize;
    while (bits.len() as f64 / 2.0) / 4800.0 < total {
        let t = (bits.len() as f64 / 2.0) / 4800.0;
        let in_call = t >= call_at - 0.3 && t < call_at + call_len;
        let tsbk = match k % 4 {
            0 if in_call => grant,
            0 => iden,
            1 => rfss,
            2 => net,
            _ if in_call => grant,
            _ => iden,
        };
        bits.extend(p1::tsdu(NAC, &[tsbk]));
        k += 1;
    }
    let mut iq = p1::modulate(&bits, RATE, 0.0);
    iq.truncate((total * RATE) as usize);
    iq.resize((total * RATE) as usize, Iq::new(0.0, 0.0));
    shift(&mut iq, CC_HZ - CENTRE);
    let mut t = traffic;
    shift(&mut t, TRAFFIC_HZ - CENTRE);
    let start = (call_at * RATE) as usize;
    for (n, z) in t.into_iter().enumerate() {
        if let Some(x) = iq.get_mut(start + n) {
            *x += z;
        }
    }
    // Noise for the C/N in 12.5 kHz of each carrier.
    declab_pmr::fsk4::add_noise(&mut iq, RATE, cn, 12_500.0, 0.0, 42);
    iq.iter_mut().for_each(|s| *s *= 0.25);
    let mut f = std::io::BufWriter::new(std::fs::File::create(&path)?);
    for s in &iq {
        f.write_all(&s.re.to_le_bytes())?;
        f.write_all(&s.im.to_le_bytes())?;
    }
    println!(
        "{path}: {total:.1} s at {RATE} S/s centred on {:.3} MHz; control {:.3} MHz, TG 4001 on {:.3} MHz from {call_at} s ({frames} voice frames)",
        CENTRE / 1e6,
        CC_HZ / 1e6,
        TRAFFIC_HZ / 1e6
    );
    Ok(())
}
