// made by CasualArclamp
//! A P25 phase 1 modulator for the tests and the sweep: frames built as
//! MMDVMHost builds them, status symbols inserted, then C4FM (as 4FSK).

use super::fec::{
    self, RS_24_12, RS_24_16, RS_36_20, from_hex, golay24_encode, hamming_10_6, nid_encode, to_hex,
};
use super::{
    ALPHA, FS, Header, IMBE_AT, LDU_LC_AT, Lc, OUTER_DEV_HZ, SYMBOL_RATE, Tsbk, duid, frame_bits,
    imbe_frame, stripped,
};
use crate::dmr::data::trellis12_encode;
use crate::fsk4::Iq;

fn bits_of(bytes: &[u8], n: usize) -> Vec<u8> {
    (0..n).map(|i| (bytes[i / 8] >> (7 - i % 8)) & 1).collect()
}

/// A frame's bits, status left out: sync and NID filled in.
fn frame_body(nac: u16, d: u8, tsbks: usize) -> Vec<u8> {
    let mut f = vec![0u8; stripped(frame_bits(d, tsbks))];
    for (i, b) in f[..48].iter_mut().enumerate() {
        *b = ((FS >> (47 - i)) & 1) as u8;
    }
    f[48..112].copy_from_slice(&nid_encode(nac, d));
    f
}

/// The status symbols put back (each pair "01" here; real ones carry the
/// inbound channel's state).
fn with_status(body: &[u8], raw_len: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(raw_len);
    let mut it = body.iter();
    for r in 0..raw_len {
        match r % 72 {
            70 => out.push(0),
            71 => out.push(1),
            _ => out.push(*it.next().unwrap_or(&0)),
        }
    }
    out
}

/// RS-encoded symbols to the 6-bit words' bits.
fn rs_bits(code: &fec::Rs, data_bits: &[u8]) -> Vec<u8> {
    let data = to_hex(data_bits);
    let mut all = data.clone();
    all.extend(code.encode(&data));
    let mut out = Vec::new();
    from_hex(&all, &mut out);
    out
}

pub fn hdu(nac: u16, h: &Header) -> Vec<u8> {
    let mut f = frame_body(nac, duid::HDU, 0);
    let rs = rs_bits(&RS_36_20, &bits_of(&h.to_bytes(), 120));
    let mut at = 112;
    for word in rs.chunks(6) {
        let d = word.iter().fold(0u32, |v, &b| (v << 1) | u32::from(b));
        let g = golay24_encode(d) & 0x3ffff;
        for k in 0..18 {
            f[at + k] = ((g >> (17 - k)) & 1) as u8;
        }
        at += 18;
    }
    with_status(&f, frame_bits(duid::HDU, 0))
}

/// LDU1 (link control) or LDU2 (encryption sync: 12 bytes), nine voice
/// frames.
pub fn ldu(nac: u16, d: u8, lc_or_es: &[u8], voice: &[[u8; 88]; 9]) -> Vec<u8> {
    let mut f = frame_body(nac, d, 0);
    for (k, v) in voice.iter().enumerate() {
        let at = stripped(IMBE_AT[k]);
        f[at..at + 144].copy_from_slice(&imbe_frame::encode(v));
    }
    let rs = if d == duid::LDU1 {
        rs_bits(&RS_24_12, &bits_of(lc_or_es, 72))
    } else {
        rs_bits(&RS_24_16, &bits_of(lc_or_es, 96))
    };
    // 24 six-bit words, four to each 40-bit piece as Hamming (10,6).
    let ham = hamming_10_6();
    for (p, &raw) in LDU_LC_AT.iter().enumerate() {
        let at = stripped(raw);
        for w in 0..4 {
            let six = &rs[(4 * p + w) * 6..(4 * p + w) * 6 + 6];
            let c = ham.encode(six.iter().fold(0u32, |v, &b| (v << 1) | u32::from(b)));
            for k in 0..10 {
                f[at + 10 * w + k] = ((c >> (9 - k)) & 1) as u8;
            }
        }
    }
    with_status(&f, frame_bits(d, 0))
}

pub fn tdu(nac: u16) -> Vec<u8> {
    with_status(&frame_body(nac, duid::TDU, 0), frame_bits(duid::TDU, 0))
}

pub fn tdulc(nac: u16, lc: &Lc) -> Vec<u8> {
    let mut f = frame_body(nac, duid::TDULC, 0);
    let rs = rs_bits(&RS_24_12, &bits_of(&lc.to_bytes(), 72));
    for (w, twelve) in rs.chunks(12).enumerate() {
        let g = golay24_encode(twelve.iter().fold(0u32, |v, &b| (v << 1) | u32::from(b)));
        for k in 0..24 {
            f[112 + 24 * w + k] = ((g >> (23 - k)) & 1) as u8;
        }
    }
    with_status(&f, frame_bits(duid::TDULC, 0))
}

/// A TSDU of one to three TSBKs (all but the last with the last-block flag
/// cleared).
pub fn tsdu(nac: u16, tsbks: &[Tsbk]) -> Vec<u8> {
    let n = tsbks.len();
    let mut f = frame_body(nac, duid::TSDU, n);
    for (k, t) in tsbks.iter().enumerate() {
        let mut b = [0u8; 12];
        b[..10].copy_from_slice(&t.to_bytes());
        if k + 1 < n {
            b[0] &= 0x7f;
        }
        let mut bits = Vec::with_capacity(80);
        crate::dmr::bptc::to_bits(&b[..10], &mut bits);
        let crc = crate::fec::crc_ccitt(&bits);
        b[10..].copy_from_slice(&crc.to_be_bytes());
        let coded = trellis12_encode(&b);
        f[112 + 196 * k..112 + 196 * (k + 1)].copy_from_slice(&coded);
    }
    with_status(&f, frame_bits(duid::TSDU, n))
}

/// Deterministic stand-in speech: 88 parameter bits per frame.
pub fn test_frame(n: u32) -> [u8; 88] {
    let mut x = n.wrapping_mul(2_654_435_761).wrapping_add(777);
    std::array::from_fn(|_| {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        (x & 1) as u8
    })
}

/// A conventional group call: HDU, `ldus` pairs of LDU1/LDU2, TDULC.
/// Returns the bits and the voice frames sent (`test_frame` in order).
pub fn call(nac: u16, lc: &Lc, ldus: usize) -> (Vec<u8>, u32) {
    let tg = lc.call().map_or(0, |c| c.1 as u16);
    let mut bits = hdu(
        nac,
        &Header {
            mi: [0; 9],
            mfid: 0,
            algid: 0x80,
            kid: 0,
            tgid: tg,
        },
    );
    let mut n = 0;
    let mut es = [0u8; 12];
    es[9] = 0x80;
    for _ in 0..ldus {
        for d in [duid::LDU1, duid::LDU2] {
            let v: [[u8; 88]; 9] = std::array::from_fn(|k| test_frame(n + k as u32));
            n += 9;
            let payload: &[u8] = if d == duid::LDU1 { &lc.to_bytes() } else { &es };
            bits.extend(ldu(nac, d, payload, &v));
        }
    }
    bits.extend(tdulc(nac, lc));
    (bits, n)
}

pub fn modulate(bits: &[u8], rate: f64, pad_s: f64) -> Vec<Iq> {
    let symbols: Vec<f32> = bits
        .as_chunks::<2>()
        .0
        .iter()
        .map(|d| crate::dmr::symbol((d[0] << 1) | d[1]))
        .collect();
    crate::fsk4::modulate(&symbols, rate, SYMBOL_RATE, OUTER_DEV_HZ, ALPHA, pad_s)
}

/// Noise for a C/N measured in the 12.5 kHz channel.
pub fn add_noise(iq: &mut [Iq], rate: f64, cn_db: f64, offset_hz: f64, seed: u64) {
    crate::fsk4::add_noise(iq, rate, cn_db, 12_500.0, offset_hz, seed);
}
