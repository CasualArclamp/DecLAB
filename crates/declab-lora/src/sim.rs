// made by CasualArclamp
//! A LoRa transmitter on paper, for tests and the simulator: preamble,
//! sync word, 2.25 down-chirps and the coded payload, phase-continuous at
//! any sample rate, with a carrier offset and a crystal error (which moves
//! the chip rate as well as the carrier, as a real one does). And sample
//! packets of both meshes, made up (no real nodes, keys or places).

use num_complex::Complex32;

use crate::codec;
use crate::meshcore::{self, Channel as McChannel};
use crate::meshtastic::{self, BROADCAST, Channel as MtChannel, Header};
use crate::proto::Writer;

/// How a transmitter is set.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tx {
    pub bw: f64,
    pub sf: u8,
    /// Coding rate 4/(4 + cr).
    pub cr: u8,
    pub sync: u8,
    pub preamble: usize,
}

impl Tx {
    /// A Meshtastic preset by name ("LongFast").
    pub fn meshtastic(preset: &str) -> Tx {
        let p = meshtastic::PRESETS
            .iter()
            .find(|p| p.0 == preset)
            .expect("a preset");
        Tx {
            bw: p.1,
            sf: p.2,
            cr: p.3 - 4,
            sync: meshtastic::SYNC_WORD,
            preamble: 16,
        }
    }

    pub fn meshcore(bw: f64, sf: u8, cr45678: u8) -> Tx {
        Tx {
            bw,
            sf,
            cr: cr45678 - 4,
            sync: meshcore::SYNC_WORD,
            preamble: 16,
        }
    }

    /// Air time of a payload, seconds.
    pub fn airtime(&self, len: usize) -> f64 {
        let ldro = codec::ldro(self.sf, self.bw);
        let h = codec::Header {
            len: len as u8,
            cr: self.cr,
            crc: true,
        };
        let syms = self.preamble as f64 + 4.25 + codec::symbol_count(self.sf, &h, ldro) as f64;
        syms * (1u32 << self.sf) as f64 / self.bw
    }
}

/// Adds one frame into `out` from sample `at` (growing it as needed), at
/// `fs` samples a second, `freq_hz` off the centre, the transmitter's
/// crystal `ppm` fast (chip rate and chirp width; put its carrier error
/// in `freq_hz`), amplitude `amp`.
#[allow(clippy::too_many_arguments)] // a transmitter's knobs, all different
pub fn modulate(
    tx: &Tx,
    payload: &[u8],
    fs: f64,
    freq_hz: f64,
    ppm: f64,
    amp: f32,
    at: usize,
    out: &mut Vec<Complex32>,
) {
    let n = 1usize << tx.sf;
    let ldro = codec::ldro(tx.sf, tx.bw);
    let syms = codec::encode(payload, tx.sf, tx.cr, true, ldro);
    // (start in symbols, length in symbols, up-chirp value or None for a
    // down-chirp).
    let mut segs: Vec<(f64, f64, Option<u16>)> = Vec::new();
    let mut t = 0.0;
    let mut push = |len: f64, v: Option<u16>, segs: &mut Vec<_>| {
        segs.push((t, len, v));
        t += len;
    };
    for _ in 0..tx.preamble {
        push(1.0, Some(0), &mut segs);
    }
    push(1.0, Some(((tx.sync >> 4) as u16) * 8), &mut segs);
    push(1.0, Some(((tx.sync & 15) as u16) * 8), &mut segs);
    push(1.0, None, &mut segs);
    push(1.0, None, &mut segs);
    push(0.25, None, &mut segs);
    for &s in &syms {
        push(1.0, Some(s), &mut segs);
    }
    let e = 1.0 + ppm * 1e-6;
    let bw = tx.bw * e;
    let ts = n as f64 / bw;
    let total = (t * ts * fs).ceil() as usize;
    if out.len() < at + total {
        out.resize(at + total, Complex32::default());
    }
    // The chirps' phase is integrated exactly (in cycles of bw·Ts): a
    // sample-by-sample sum would put a phase step at each frequency wrap,
    // between samples, that a real chirp does not have.
    let integral = |u: f64, v: Option<u16>| -> f64 {
        match v {
            // ∫₀ᵘ (frac(x + v/N) − ½) dx: whole periods add nothing.
            Some(v) => {
                let w0 = v as f64 / n as f64;
                let g = |w: f64| {
                    let f = w.fract();
                    (f * f - f) / 2.0
                };
                g(u + w0) - g(w0)
            }
            None => -(u * u / 2.0 - u / 2.0),
        }
    };
    let mut base = 0.0f64;
    let mut seg = 0;
    for i in 0..total {
        let tt = i as f64 / fs / ts;
        while seg + 1 < segs.len() && tt >= segs[seg + 1].0 {
            base += integral(segs[seg].1, segs[seg].2);
            base = base.fract();
            seg += 1;
        }
        let (s0, _, v) = segs[seg];
        let cycles = (base + integral(tt - s0, v)) * bw * ts + freq_hz * i as f64 / fs;
        out[at + i] +=
            Complex32::from_polar(amp, (2.0 * std::f64::consts::PI * cycles.fract()) as f32);
    }
}

/// A small, fast noise source (xorshift and Box–Muller).
pub struct Noise(u64);

impl Noise {
    pub fn new(seed: u64) -> Self {
        Noise(seed | 1)
    }

    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Complex Gaussian noise of power `p` (both rails together).
    pub fn add(&mut self, x: &mut [Complex32], p: f64) {
        let s = (p / 2.0).sqrt();
        for v in x {
            let (u1, u2) = (self.uniform(), self.uniform());
            let r = (-2.0 * u1.ln()).sqrt() * s;
            let a = 2.0 * std::f64::consts::PI * u2;
            *v += Complex32::new((r * a.cos()) as f32, (r * a.sin()) as f32);
        }
    }
}

/// A made-up node number.
pub const NODE_A: u32 = 0xa1b2_c3d4;
pub const NODE_B: u32 = 0x0e5f_7788;
pub const NODE_C: u32 = 0x77ab_0012;

/// One sample packet: what it is, how it is sent, its bytes.
pub struct Sample {
    pub what: &'static str,
    pub tx: Tx,
    pub bytes: Vec<u8>,
}

/// Packets of both meshes: Meshtastic on LongFast (node info, text,
/// position, telemetry, a direct message that cannot be read), MeshCore
/// (a repeater's advert, a Public channel message, a direct message, an
/// ack) on its 250 kHz SF11 preset and on a narrow 62.5 kHz SF8 one.
pub fn samples() -> Vec<Sample> {
    let lf = MtChannel::preset_default("LongFast");
    let mt = Tx::meshtastic("LongFast");
    let hdr = |from: u32, to: u32, id: u32| Header {
        to,
        from,
        id,
        // hop limit 3 of 3.
        flags: 3 | (3 << 5),
        ..Default::default()
    };
    let user = Writer::default()
        .bytes(1, format!("!{NODE_A:08x}").as_bytes())
        .bytes(2, b"ALFA Base")
        .bytes(3, b"ALFA")
        .varint(5, 4) // TBEAM
        .varint(7, 2) // ROUTER
        .done();
    let pos = Writer::default()
        .fixed32(1, 455_000_000u32) // 45.5° N (a made-up place)
        .fixed32(2, 72_000_000u32) // 7.2° E
        .varint(3, 412)
        .varint(19, 9)
        .done();
    let dm = Writer::default()
        .varint(1, 87)
        .float(2, 4.02)
        .float(3, 12.5)
        .float(4, 1.25)
        .varint(5, 86_400)
        .done();
    let tele = Writer::default()
        .fixed32(1, 1_794_000_000)
        .bytes(2, &dm)
        .done();
    let env = Writer::default()
        .float(1, 18.4)
        .float(2, 57.0)
        .float(3, 1013.2)
        .done();
    let tele_env = Writer::default().bytes(3, &env).done();
    let mut pki = hdr(NODE_B, NODE_A, 0x1000_0004).bytes().to_vec();
    pki[13] = 0; // channel 0: PKI
    pki.extend((0..40u8).map(|i| i.wrapping_mul(73) ^ 0x3c));

    let pub_ch = McChannel::public();
    let key_r = [0x3fu8; 32];
    let key_c = [0x91u8; 32];
    let mc = Tx::meshcore(250e3, 11, 5);
    let mcn = Tx::meshcore(62.5e3, 8, 8);
    let advert =
        meshcore::advert_payload(&key_r, 1_794_000_100, 2, Some((45.48, 7.25)), "Echo Rptr");
    let chat_advert = meshcore::advert_payload(&key_c, 1_794_000_200, 1, None, "Delta");
    let grp =
        meshcore::group_text_payload(&pub_ch, 1_794_000_300, "Delta: anyone on the north side?");
    let mut direct = vec![0x3f, 0x91, 0x12, 0x34];
    direct.extend((0..32u8).map(|i| i.wrapping_mul(29) ^ 0xa5));

    vec![
        Sample {
            what: "Meshtastic node info",
            tx: mt,
            bytes: meshtastic::build(&lf, hdr(NODE_A, BROADCAST, 0x1000_0001), 4, &user),
        },
        Sample {
            what: "Meshtastic text",
            tx: mt,
            bytes: meshtastic::build(
                &lf,
                hdr(NODE_A, BROADCAST, 0x1000_0002),
                1,
                b"Good morning from the hill",
            ),
        },
        Sample {
            what: "Meshtastic position",
            tx: mt,
            bytes: meshtastic::build(&lf, hdr(NODE_C, BROADCAST, 0x1000_0003), 3, &pos),
        },
        Sample {
            what: "Meshtastic telemetry",
            tx: mt,
            bytes: meshtastic::build(&lf, hdr(NODE_B, BROADCAST, 0x1000_0005), 67, &tele),
        },
        Sample {
            what: "Meshtastic environment",
            tx: mt,
            bytes: meshtastic::build(&lf, hdr(NODE_C, BROADCAST, 0x1000_0006), 67, &tele_env),
        },
        Sample {
            what: "Meshtastic direct (PKI)",
            tx: mt,
            bytes: pki,
        },
        Sample {
            what: "MeshCore advert",
            tx: mc,
            bytes: meshcore::build(4, 1, &[], &advert),
        },
        Sample {
            what: "MeshCore Public text",
            tx: mc,
            bytes: meshcore::build(5, 1, &[0x3f], &grp),
        },
        Sample {
            what: "MeshCore direct",
            tx: mc,
            bytes: meshcore::build(2, 2, &[0x3f], &direct),
        },
        Sample {
            what: "MeshCore advert (narrow)",
            tx: mcn,
            bytes: meshcore::build(4, 1, &[0x3f, 0x22], &chat_advert),
        },
        Sample {
            what: "MeshCore ack (narrow)",
            tx: mcn,
            bytes: meshcore::build(3, 2, &[], &[0xde, 0xad, 0xbe, 0xef]),
        },
    ]
}
