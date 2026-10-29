// made by CasualArclamp
//! A DMR modulator for the tests and the sweep: bursts built from PDUs,
//! 4FSK through the transmit RRC (clause 10.2), and noise at a chosen C/N.
//!
//! [`Repeater`] makes a base station's outbound stream: a CACH before each
//! burst, slots alternating TS1/TS2, the Short LC spread over the CACHs.

use super::data::{DataHeader, Format, Rate, crc9, crc32, trellis_encode};
use super::pdu::{Csbk, FullLc, ShortLc};
use super::{
    ALPHA, DATA_SYNC, DataType, OUTER_DEV_HZ, SYMBOL_RATE, Source, VOICE_SYNC, ambe, bptc, layout,
    mask,
};
use crate::fec;
use crate::fsk4::{Iq, rrc};

fn sync_of(table: &[(Source, u64); 4], s: Source) -> u64 {
    table
        .iter()
        .find(|(x, _)| *x == s)
        .expect("every source has a sync")
        .1
}

fn bits48(p: u64) -> [u8; 48] {
    std::array::from_fn(|i| ((p >> (47 - i)) & 1) as u8)
}

/// A data or control burst: the BPTC'd 12 bytes, the slot type, the data
/// SYNC of `source`.
pub fn data_burst(source: Source, cc: u8, dt: DataType, data: &[u8; 12]) -> [u8; 264] {
    let info = bptc::encode_196(data);
    let slot = fec::golay_20_8().encode((u32::from(cc & 15) << 4) | u32::from(dt.to_u8()));
    let mut b = [0u8; 264];
    b[layout::INFO_A].copy_from_slice(&info[..98]);
    b[layout::INFO_B].copy_from_slice(&info[98..]);
    for i in 0..10 {
        b[layout::SLOT_A.start + i] = ((slot >> (19 - i)) & 1) as u8;
        b[layout::SLOT_B.start + i] = ((slot >> (9 - i)) & 1) as u8;
    }
    b[layout::CENTRE].copy_from_slice(&bits48(sync_of(&DATA_SYNC, source)));
    b
}

/// A voice LC header or terminator: the LC, its RS parity under the mask.
pub fn lc_burst(source: Source, cc: u8, lc: &FullLc, terminator: bool) -> [u8; 264] {
    let bytes = lc.to_bytes();
    let p = fec::rs129_parity(&bytes);
    let (m, dt) = if terminator {
        (mask::TERMINATOR_LC, DataType::TerminatorLc)
    } else {
        (mask::VOICE_LC_HEADER, DataType::VoiceLcHeader)
    };
    let mut d = [0u8; 12];
    d[..9].copy_from_slice(&bytes);
    for j in 0..3 {
        d[9 + j] = p[j] ^ m[j];
    }
    data_burst(source, cc, dt, &d)
}

/// A CSBK burst: the 10 bytes, CRC-CCITT under the CSBK mask.
pub fn csbk_burst(source: Source, cc: u8, csbk: &Csbk) -> [u8; 264] {
    let bytes = csbk.to_bytes();
    let mut bits = Vec::with_capacity(80);
    bptc::to_bits(&bytes, &mut bits);
    let crc = fec::crc_ccitt(&bits) ^ mask::CSBK;
    let mut d = [0u8; 12];
    d[..10].copy_from_slice(&bytes);
    d[10..].copy_from_slice(&crc.to_be_bytes());
    data_burst(source, cc, DataType::Csbk, &d)
}

/// A burst carrying 196 bits as they are (rate ¾ blocks).
pub fn raw_burst(source: Source, cc: u8, dt: DataType, info: &[u8; 196]) -> [u8; 264] {
    let mut b = data_burst(source, cc, dt, &[0; 12]);
    b[layout::INFO_A].copy_from_slice(&info[..98]);
    b[layout::INFO_B].copy_from_slice(&info[98..]);
    b
}

/// A data packet's bursts: its header, then its blocks (rate ½ or ¾,
/// confirmed with DBSN and CRC-9 or unconfirmed), the CRC-32 at the end.
pub fn data_packet(
    source: Source,
    cc: u8,
    mut h: DataHeader,
    payload: &[u8],
    rate: Rate,
) -> Vec<[u8; 264]> {
    let confirmed = h.format == Format::Confirmed;
    let size = match rate {
        Rate::Half => 12,
        Rate::ThreeQuarter => 18,
    } - if confirmed { 2 } else { 0 };
    // Pad so the data and the 4 CRC octets fill whole blocks.
    let blocks = (payload.len() + 4).div_ceil(size);
    let pad = blocks * size - 4 - payload.len();
    let mut user = payload.to_vec();
    user.resize(payload.len() + pad, 0);
    let crc = crc32(&user);
    user.extend_from_slice(&crc);
    h.pad = pad as u8;
    h.blocks = blocks as u8;
    let hb = h.to_bytes();
    let mut bits = Vec::with_capacity(80);
    bptc::to_bits(&hb, &mut bits);
    let hcrc = fec::crc_ccitt(&bits) ^ mask::DATA_HEADER;
    let mut d = [0u8; 12];
    d[..10].copy_from_slice(&hb);
    d[10..].copy_from_slice(&hcrc.to_be_bytes());
    let mut out = vec![data_burst(source, cc, DataType::DataHeader, &d)];
    for (n, chunk) in user.chunks(size).enumerate() {
        let mut block = Vec::with_capacity(size + 2);
        if confirmed {
            let m = if rate == Rate::Half { 0x0F0 } else { 0x1FF };
            let c = crc9(chunk, n as u8) ^ m;
            block.push(((n as u8) << 1) | (c >> 8) as u8);
            block.push(c as u8);
        }
        block.extend_from_slice(chunk);
        out.push(match rate {
            Rate::Half => data_burst(
                source,
                cc,
                DataType::Rate12,
                block[..].try_into().expect("12 bytes"),
            ),
            Rate::ThreeQuarter => raw_burst(
                source,
                cc,
                DataType::Rate34,
                &trellis_encode(block[..].try_into().expect("18 bytes")),
            ),
        });
    }
    out
}

/// An IPv4/UDP datagram (no options, checksum left 0).
pub fn ipv4_udp(src: [u8; 4], dst: [u8; 4], sport: u16, dport: u16, payload: &[u8]) -> Vec<u8> {
    let total = 28 + payload.len();
    let mut p = vec![
        0x45,
        0,
        (total >> 8) as u8,
        total as u8,
        0,
        1,
        0,
        0,
        64,
        17,
        0,
        0,
    ];
    p.extend_from_slice(&src);
    p.extend_from_slice(&dst);
    let ulen = 8 + payload.len();
    p.extend_from_slice(&sport.to_be_bytes());
    p.extend_from_slice(&dport.to_be_bytes());
    p.extend_from_slice(&[(ulen >> 8) as u8, ulen as u8, 0, 0]);
    p.extend_from_slice(payload);
    p
}

/// An idle burst (its content is fill).
pub fn idle_burst(source: Source, cc: u8) -> [u8; 264] {
    data_burst(source, cc, DataType::Idle, &[0; 12])
}

/// What sits in a voice burst's centre.
pub enum Centre<'a> {
    /// Burst A: the voice SYNC.
    Sync(Source),
    /// Bursts B–F: EMB (colour code, LCSS) round 32 embedded bits.
    Emb {
        cc: u8,
        lcss: u8,
        embedded: &'a [u8],
    },
}

/// A voice burst: three AMBE frames round the centre.
pub fn voice_burst(frames: &[[u8; 49]; 3], centre: Centre<'_>) -> [u8; 264] {
    let f: Vec<[u8; 72]> = frames.iter().map(ambe::encode).collect();
    let mut b = [0u8; 264];
    b[..72].copy_from_slice(&f[0]);
    b[72..108].copy_from_slice(&f[1][..36]);
    b[156..192].copy_from_slice(&f[1][36..]);
    b[192..].copy_from_slice(&f[2]);
    match centre {
        Centre::Sync(s) => b[layout::CENTRE].copy_from_slice(&bits48(sync_of(&VOICE_SYNC, s))),
        Centre::Emb { cc, lcss, embedded } => {
            let emb = fec::qr_16_7().encode((u32::from(cc & 15) << 3) | u32::from(lcss & 3));
            for i in 0..8 {
                b[layout::EMB_A.start + i] = ((emb >> (15 - i)) & 1) as u8;
                b[layout::EMB_B.start + i] = ((emb >> (7 - i)) & 1) as u8;
            }
            b[layout::EMBEDDED].copy_from_slice(embedded);
        }
    }
    b
}

/// A CACH: TACT (AT, TC, LCSS) and 17 payload bits, interleaved (B.9).
pub fn cach(at: bool, tc: u8, lcss: u8, payload: &[u8]) -> [u8; 24] {
    let tact = fec::hamming_7_4()
        .encode((u32::from(at) << 3) | (u32::from(tc & 1) << 2) | u32::from(lcss & 3));
    let mut c = [0u8; 24];
    for (k, &p) in layout::TACT.iter().enumerate() {
        c[p] = ((tact >> (6 - k)) & 1) as u8;
    }
    for (k, &p) in layout::CACH_PAYLOAD.iter().enumerate() {
        c[p] = payload[k];
    }
    c
}

/// A deterministic stand-in for speech: 49 "parameter bits" per frame.
pub fn test_frame(n: u32) -> [u8; 49] {
    let mut x = n.wrapping_mul(2_654_435_761).wrapping_add(12345);
    std::array::from_fn(|_| {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        (x & 1) as u8
    })
}

/// A base station's outbound, slot by slot.
pub struct Repeater {
    pub cc: u8,
    pub bits: Vec<u8>,
    slot: u8,
    /// The Short LC being sent in the CACHs, and which of its four parts is
    /// next.
    slc: [u8; 68],
    slc_part: usize,
    /// Voice frames sent so far (the frame content is [`test_frame`]).
    pub frames: u32,
}

impl Repeater {
    pub fn new(cc: u8, short_lc: ShortLc) -> Repeater {
        Repeater {
            cc,
            bits: Vec::new(),
            slot: 0,
            slc: bptc::encode_short_lc(short_lc.to_u32()),
            slc_part: 0,
            frames: 0,
        }
    }

    /// The slot the next burst goes in (0 = TS1).
    pub fn next_slot(&self) -> u8 {
        self.slot
    }

    /// Append one 30 ms slot: CACH then `burst`.
    pub fn push(&mut self, burst: &[u8; 264]) {
        let lcss = [0b01, 0b11, 0b11, 0b10][self.slc_part];
        let p = &self.slc[17 * self.slc_part..17 * self.slc_part + 17];
        let c = cach(true, self.slot, lcss, p);
        self.bits.extend_from_slice(&c);
        self.bits.extend_from_slice(burst);
        self.slc_part = (self.slc_part + 1) % 4;
        self.slot ^= 1;
    }

    /// A voice call on the next slot and every other one after it: header,
    /// `superframes` of A–F with the LC embedded in B–E, terminator. `other`
    /// fills the interleaved slots.
    pub fn voice_call(
        &mut self,
        lc: &FullLc,
        superframes: usize,
        mut other: impl FnMut(&mut Repeater) -> [u8; 264],
    ) {
        let emb = bptc::encode_embedded(&lc.to_bytes());
        let header = lc_burst(Source::Bs, self.cc, lc, false);
        self.push(&header);
        let o = other(self);
        self.push(&o);
        for _ in 0..superframes {
            for letter in 0..6 {
                let frames = [
                    test_frame(self.frames),
                    test_frame(self.frames + 1),
                    test_frame(self.frames + 2),
                ];
                self.frames += 3;
                let burst = match letter {
                    0 => voice_burst(&frames, Centre::Sync(Source::Bs)),
                    1..=4 => voice_burst(
                        &frames,
                        Centre::Emb {
                            cc: self.cc,
                            lcss: [0b01, 0b11, 0b11, 0b10][letter - 1],
                            embedded: &emb[32 * (letter - 1)..32 * letter],
                        },
                    ),
                    _ => voice_burst(
                        &frames,
                        Centre::Emb {
                            cc: self.cc,
                            lcss: 0,
                            embedded: &[0; 32],
                        },
                    ),
                };
                self.push(&burst);
                let o = other(self);
                self.push(&o);
            }
        }
        let t = lc_burst(Source::Bs, self.cc, lc, true);
        self.push(&t);
        let o = other(self);
        self.push(&o);
    }
}

/// 4FSK from bits (two a symbol, clause 10.2) at `rate`, unit amplitude,
/// with `pad_s` of unmodulated carrier either side.
pub fn modulate(bits: &[u8], rate: f64, pad_s: f64) -> Vec<Iq> {
    let sps = (rate / SYMBOL_RATE).round() as usize;
    let h = rrc(ALPHA, sps, 8);
    let n_sym = bits.len() / 2;
    let pad = (pad_s * rate) as usize;
    let mut impulses = vec![0f32; pad + n_sym * sps + h.len() + pad];
    for s in 0..n_sym {
        let d = (bits[2 * s] << 1) | bits[2 * s + 1];
        impulses[pad + s * sps] = super::symbol(d);
    }
    // The pulse shaping, then deviation: an RRC summed to 1 passes a symbol
    // held over many symbols at its level, so scale the impulses by sps.
    let k = (OUTER_DEV_HZ / 3.0) as f32 * sps as f32;
    let mut phase = 0f64;
    let mut out = Vec::with_capacity(impulses.len());
    for i in 0..impulses.len() {
        let mut f = 0f32;
        for (j, &t) in h.iter().enumerate() {
            if i >= j {
                f += impulses[i - j] * t;
            }
        }
        phase += 2.0 * std::f64::consts::PI * f64::from(f * k) / rate;
        out.push(Iq::from_polar(1.0, phase as f32));
    }
    out
}

/// Add white Gaussian noise for a carrier-to-noise ratio `cn_db` measured in
/// a 12.5 kHz channel (the carrier has unit power), and a frequency offset.
pub fn add_noise(iq: &mut [Iq], rate: f64, cn_db: f64, offset_hz: f64, seed: u64) {
    let n_power = rate / 12_500.0 * 10f64.powf(-cn_db / 10.0);
    let sigma = (n_power / 2.0).sqrt() as f32;
    let mut s = seed | 1;
    let mut uniform = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        ((s >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    };
    let w = 2.0 * std::f64::consts::PI * offset_hz / rate;
    for (i, x) in iq.iter_mut().enumerate() {
        let (u1, u2) = (uniform(), uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let a = 2.0 * std::f64::consts::PI * u2;
        let rot = Iq::from_polar(1.0, (w * i as f64) as f32);
        *x = *x * rot + Iq::new((r * a.cos()) as f32 * sigma, (r * a.sin()) as f32 * sigma);
    }
}
