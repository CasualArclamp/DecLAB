// made by CasualArclamp
//! An NXDN48 modulator for the tests and the sweep: frames built as
//! MMDVMHost builds them (LICH, SACCH, FACCH1 or four voice frames,
//! scrambled), then 4FSK.

use super::{
    ALPHA, FSW, Lich, Message, OUTER_DEV_HZ, Rfct, SYMBOL_RATE, Steal, facch1_encode, layout,
    sacch_encode, scrambler_bit,
};
use crate::dmr::ambe;
use crate::fsk4::Iq;

/// What a frame's 288 payload bits carry.
pub enum Payload<'a> {
    Voice(&'a [[u8; 49]; 4]),
    /// The same message in both FACCH1 halves.
    Facch(&'a Message),
}

/// The 26 SACCH bits: structure (2), RAN (6), 18 message bits.
pub fn sacch_bits(structure: u8, ran: u8, data18: &[u8]) -> Vec<u8> {
    let mut b = Vec::with_capacity(26);
    b.extend((0..2).rev().map(|i| (structure >> i) & 1));
    b.extend((0..6).rev().map(|i| (ran >> i) & 1));
    b.extend_from_slice(&data18[..18]);
    b
}

fn bits_of(bytes: &[u8], n: usize) -> Vec<u8> {
    (0..n).map(|i| (bytes[i / 8] >> (7 - i % 8)) & 1).collect()
}

/// One 384-bit frame, scrambled.
pub fn frame(lich: Lich, sacch26: &[u8], payload: Payload<'_>) -> Vec<u8> {
    let mut f = vec![0u8; super::FRAME_BITS];
    for (i, b) in f[..20].iter_mut().enumerate() {
        *b = ((FSW >> (19 - i)) & 1) as u8;
    }
    for i in 0..8 {
        f[layout::LICH + 2 * i] = (lich.0 >> (7 - i)) & 1;
        f[layout::LICH + 2 * i + 1] = 1;
    }
    f[layout::SACCH..layout::PAYLOAD].copy_from_slice(&sacch_encode(sacch26));
    match payload {
        Payload::Voice(frames) => {
            for (k, d) in frames.iter().enumerate() {
                let at = layout::PAYLOAD + k * layout::VCH_BITS;
                f[at..at + 72].copy_from_slice(&ambe::encode(d));
            }
        }
        Payload::Facch(m) => {
            let coded = facch1_encode(&bits_of(&m.0, 80));
            f[layout::PAYLOAD..layout::PAYLOAD + 144].copy_from_slice(&coded);
            f[layout::PAYLOAD + 144..].copy_from_slice(&coded);
        }
    }
    for (i, b) in f.iter_mut().enumerate() {
        *b ^= scrambler_bit(i);
    }
    f
}

/// A conventional voice call: two FACCH1 frames with the call, voice
/// frames with the call in the SACCH superframe (four frames to a
/// message), then a FACCH1 release. Returns the frames and the voice frames
/// sent (their content is `dmr::sim::test_frame` in order).
pub fn call(ran: u8, msg: &Message, superframes: usize) -> (Vec<u8>, u32) {
    let mut bits = Vec::new();
    let idle18 = [0u8; 18];
    let head = Lich::new(Rfct::Rtch, 0, Steal::Both, true);
    for _ in 0..2 {
        bits.extend(frame(
            head,
            &sacch_bits(0, ran, &idle18),
            Payload::Facch(msg),
        ));
    }
    let sacch_msg = bits_of(&msg.0, 72);
    let voice = Lich::new(Rfct::Rtch, 2, Steal::None, true);
    let mut n = 0u32;
    for _ in 0..superframes {
        for part in 0..4usize {
            // Structure: 1/4 → 3, 2/4 → 2, 3/4 → 1, 4/4 → 0.
            let s = sacch_bits(3 - part as u8, ran, &sacch_msg[18 * part..18 * part + 18]);
            let frames = [0, 1, 2, 3].map(|k| crate::dmr::sim::test_frame(n + k));
            n += 4;
            bits.extend(frame(voice, &s, Payload::Voice(&frames)));
        }
    }
    let rel = Message::vcall(
        0x08,
        msg.call().map_or(0, |c| c.0),
        msg.call().map_or(0, |c| c.1),
        true,
    );
    bits.extend(frame(
        head,
        &sacch_bits(0, ran, &idle18),
        Payload::Facch(&rel),
    ));
    (bits, n)
}

/// 4FSK from frame bits at `rate`.
pub fn modulate(bits: &[u8], rate: f64, pad_s: f64) -> Vec<Iq> {
    let symbols: Vec<f32> = bits
        .as_chunks::<2>()
        .0
        .iter()
        .map(|d| super::symbol((d[0] << 1) | d[1]))
        .collect();
    crate::fsk4::modulate(&symbols, rate, SYMBOL_RATE, OUTER_DEV_HZ, ALPHA, pad_s)
}

/// Noise for a C/N measured in the 6.25 kHz channel.
pub fn add_noise(iq: &mut [Iq], rate: f64, cn_db: f64, offset_hz: f64, seed: u64) {
    crate::fsk4::add_noise(iq, rate, cn_db, 6_250.0, offset_hz, seed);
}
