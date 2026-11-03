// made by CasualArclamp
//! A simulated satellite carrier for testing the `sat` decoder, 4 samples a
//! symbol, written as cf32 (the sample rate is 4 × the symbol rate):
//!
//! ```text
//! cargo run --release -p declab-decoders --example sat_make -- OUT KIND [ES_N0_DB] [SECONDS] [OFFSET_HZ] [RS] [TS]
//! KIND: s2ts   DVB-S2 QPSK 1/2, pilots, a transport stream
//!       s2tv   DVB-S2 8PSK 3/4, pilots, a transport stream (room for TV)
//!       s2x    DVB-S2X (QPSK 13/45), a transport stream
//!       gse    DVB-S2 QPSK 1/2, GSE carrying IP (DNS and multicast radio)
//!       dvbs   DVB-S QPSK 3/4, a transport stream
//! ```
//!
//! OUT ending `.cs8` is written as 8-bit IQ (a quarter of cf32's size, for
//! long files). TS (s2ts, s2tv and dvbs) is a transport stream file sent, looped,
//! in place of the test packets, so the player has a programme to play. It
//! goes out at the channel's TS rate (printed), so make it constant-rate at
//! that rate, e.g. `ffmpeg -i IN.ts -map 0:p:SID -c copy -f mpegts -muxrate
//! 4355051 OUT.ts` for s2tv at 2 MS/s.
//!
//! The test signals come from DecSAT's modulator (`decsat-mod`).

use std::io::Write;

use decsat_core::{Iq, RollOff};
use decsat_gse::Variant;
use decsat_mod::{FrameSpec, GseBbFramer, PlFramer, Shaper, TsBbFramer};
use decsat_modem::conv::Rate;
use decsat_modem::dvbs::DvbsTx;

/// xorshift64* (test signals only).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn uniform(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Unit-power complex Gaussian noise.
    fn gauss(&mut self) -> Iq {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let t = std::f64::consts::TAU * u2;
        Iq::new((r * t.cos()) as f32, (r * t.sin()) as f32) * std::f32::consts::FRAC_1_SQRT_2
    }
}

fn main() -> std::io::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let out = a.get(1).expect("OUT");
    let kind = a.get(2).map_or("s2ts", String::as_str);
    let esn0: f64 = a.get(3).map_or(30.0, |s| s.parse().unwrap());
    let seconds: f64 = a.get(4).map_or(4.0, |s| s.parse().unwrap());
    let offset: f64 = a.get(5).map_or(0.0, |s| s.parse().unwrap());
    let rs: f64 = a.get(6).map_or(2e6, |s| s.parse().unwrap());
    let file: Vec<[u8; 188]> = a.get(7).map_or_else(Vec::new, |p| {
        let b = std::fs::read(p).expect("TS");
        // Rust note: `as_chunks` gives fixed-size `&[u8; 188]` packets (and
        // a short tail, dropped).
        b.as_chunks::<188>().0.iter().filter(|c| c[0] == 0x47).copied().collect()
    });
    let source = |seed| TsBbFramer::new(seed).with_packets(file.clone());
    let sps = 4;
    let n_sym = (rs * seconds) as usize;

    let (syms, alpha) = match kind {
        "dvbs" => {
            // EN 300 421 §4.4: RS(204,188) then the 3/4 inner code, 2 bits a symbol.
            eprintln!("TS rate {:.0} bit/s", rs * 2.0 * 0.75 * 188.0 / 204.0);
            let mut ts = source(3);
            let mut tx = DvbsTx::new(Rate::R3_4);
            let mut syms = Vec::new();
            while syms.len() < n_sym + 20_000 {
                tx.packet(&ts.packet(), &mut syms);
            }
            (syms[20_000..].to_vec(), 0.35)
        }
        "gse" => {
            let mut f = PlFramer::new(0, 1).with_source(Box::new(GseBbFramer::new(1, Variant::STANDARD).with_dns()));
            f.set_roll_off(RollOff::R20);
            (f.build_schedule(&[FrameSpec::new(1, false, true)], n_sym), 0.20)
        }
        "s2x" => {
            let mut f = PlFramer::new(0, 2);
            f.set_roll_off(RollOff::R20);
            (f.build_schedule(&[FrameSpec::s2x(144, true)], n_sym), 0.20)
        }
        "s2tv" => {
            // EN 302 307-1 table 5a: Kbch 48408 for 3/4, less the 80-bit
            // BBHEADER; 8PSK: 21600 + 90 PLHEADER + 14 pilot blocks of 36.
            eprintln!("TS rate {:.0} bit/s", rs * 48_328.0 / 22_194.0);
            let mut f = PlFramer::new(0, 3).with_source(Box::new(source(3)));
            f.set_roll_off(RollOff::R20);
            // MODCOD 14: 8PSK 3/4 (table 12).
            (f.build_schedule(&[FrameSpec::new(14, false, true)], n_sym), 0.20)
        }
        _ => {
            // Table 5a: Kbch 32208 for 1/2, less the BBHEADER; QPSK: 32400 +
            // 90 + 22 pilot blocks of 36.
            eprintln!("TS rate {:.0} bit/s", rs * 32_128.0 / 33_282.0);
            // PlFramer::new's own source is TsBbFramer::new(seed ^ 0x7E57).
            let mut f = PlFramer::new(0, 3).with_source(Box::new(source(3 ^ 0x7E57)));
            f.set_roll_off(RollOff::R20);
            // MODCOD 4: QPSK 1/2 (EN 302 307-1 table 12).
            (f.build_schedule(&[FrameSpec::new(4, false, true)], n_sym), 0.20)
        }
    };

    let mut shaper = Shaper::new(sps, alpha, 16);
    let mut x = Vec::with_capacity(syms.len() * sps);
    shaper.process(&syms, &mut x);

    // Noise for Es/N0: signal power per sample is 1/sps of a symbol's, so the
    // noise per sample is sps/(Es/N0) of the signal power per symbol.
    let p_sig = x.iter().map(|s| s.norm_sqr() as f64).sum::<f64>() / x.len() as f64;
    let sigma = (p_sig * sps as f64 / 10f64.powf(esn0 / 10.0)).sqrt() as f32;
    let mut rng = Rng(0x5A7E_11D3);
    let w = std::f64::consts::TAU * offset / (rs * sps as f64);
    let cs8 = out.ends_with(".cs8");
    // cs8: full scale at 4 × the RMS, which leaves room for the peaks.
    let k = 127.0 / (4.0 * (p_sig as f32 + sigma * sigma).sqrt());
    let mut file = std::io::BufWriter::new(std::fs::File::create(out)?);
    for (n, s) in x.iter().enumerate() {
        let ph = w * n as f64;
        let y = *s * Iq::new(ph.cos() as f32, ph.sin() as f32) + rng.gauss() * sigma;
        if cs8 {
            let q = |v: f32| (v * k).round().clamp(-127.0, 127.0) as i8 as u8;
            file.write_all(&[q(y.re), q(y.im)])?;
        } else {
            file.write_all(&y.re.to_le_bytes())?;
            file.write_all(&y.im.to_le_bytes())?;
        }
    }
    file.flush()?;
    eprintln!("wrote {out}: {kind}, {:.1} s, Es/N0 {esn0} dB, {} S/s", x.len() as f64 / (rs * sps as f64), rs * sps as f64);
    Ok(())
}
