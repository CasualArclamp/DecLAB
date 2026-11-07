// made by CasualArclamp
//! A phase 2 traffic channel for tests and the simulator: timeslots built
//! as [`super::receiver`] reads them (the layouts are OP25's), scrambled,
//! then H-DQPSK.

use super::scramble::Scrambler;
use super::{
    CHANNEL_OF, DUID_AT, ESS_A_AT, ESS_B_AT, I_ISCH, MacMsg, MacOp, MacPdu, S_ISCH, SLOT_SYMBOLS,
    SUPERFRAME, SYMBOL_RATE, VOICE_AT, acch_dibits, acch_layout, crc12, duid, duid_codeword,
    i_isch_value, rs63, scrambled,
};
use crate::dmr::ambe;
use crate::fsk4::{Iq, rrc};

/// What one timeslot carries.
#[derive(Clone)]
pub enum Burst {
    /// Four voice frames and this burst's quarter of the ESS (4 hexbits).
    V4 { frames: [[u8; 49]; 4], ess_b: [u8; 4] },
    /// Two voice frames and the ESS's parity (28 hexbits).
    V2 { frames: [[u8; 49]; 2], ess_a: [u8; 28] },
    Facch(MacPdu),
    Sacch(MacPdu),
}

/// The encryption sync (ALGID, KID, MI) as 16 hexbits, and its RS parity.
pub fn ess(algid: u8, kid: u16, mi: [u8; 9]) -> ([u8; 16], [u8; 28]) {
    let mut bits = Vec::with_capacity(96);
    let mut push = |v: u64, n: usize| bits.extend((0..n).rev().map(|i| ((v >> i) & 1) as u8));
    push(u64::from(algid), 8);
    push(u64::from(kid), 16);
    for b in mi {
        push(u64::from(b), 8);
    }
    let hex: Vec<u8> = bits.chunks(6).map(|c| c.iter().fold(0, |a, &b| (a << 1) | b)).collect();
    let mut d = [0u8; rs63::K];
    d[19..35].copy_from_slice(&hex);
    let p = rs63::encode(&d);
    (hex.try_into().expect("16"), p)
}

/// A site's transmitter on one traffic channel.
pub struct Site {
    scr: Scrambler,
    /// The timeslot the next push fills (counting from 0, never wrapping).
    pub ts: usize,
    pub dibits: Vec<u8>,
}

impl Site {
    pub fn new(seed: u64) -> Site {
        Site { scr: Scrambler::new(seed), ts: 0, dibits: Vec::new() }
    }

    /// The superframe timeslot (0–11) the next push fills.
    pub fn timeslot(&self) -> usize {
        self.ts % SUPERFRAME
    }

    pub fn push(&mut self, burst: &Burst) {
        let t = self.timeslot();
        let mut d = [0u8; SLOT_SYMBOLS];
        let isch = match i_isch_value(t) {
            Some(v) => I_ISCH[v],
            None => S_ISCH,
        };
        for (k, x) in d[..20].iter_mut().enumerate() {
            *x = ((isch >> (38 - 2 * k)) & 3) as u8;
        }
        // Burst dibit i is timeslot dibit i + 10.
        let mut b = [0u8; 170];
        let put_bits = |b: &mut [u8; 170], at: &[usize], bits: &[u8]| {
            for (k, &i) in at.iter().enumerate() {
                b[i] = (bits[2 * k] << 1) | bits[2 * k + 1];
            }
        };
        let put_hex = |b: &mut [u8; 170], from: usize, hex: &[u8]| {
            for (k, &h) in hex.iter().enumerate() {
                for j in 0..3 {
                    b[from + 3 * k + j] = (h >> (4 - 2 * j)) & 3;
                }
            }
        };
        let voice = |b: &mut [u8; 170], at: usize, f: &[u8; 49]| {
            let bits = ambe::encode(f);
            let pos: Vec<usize> = (at..at + 36).collect();
            put_bits(b, &pos, &bits);
        };
        let d_type = match burst {
            Burst::V4 { frames, ess_b } => {
                for (f, &at) in frames.iter().zip(&VOICE_AT) {
                    voice(&mut b, at, f);
                }
                put_hex(&mut b, ESS_B_AT, ess_b);
                duid::V4
            }
            Burst::V2 { frames, ess_a } => {
                for (f, &at) in frames.iter().zip(&VOICE_AT) {
                    voice(&mut b, at, f);
                }
                // 28 hexbits, the dibit at 132 (DUID) skipped.
                let mut at = ESS_A_AT;
                for (k, &h) in ess_a.iter().enumerate() {
                    for j in 0..3 {
                        b[at + j] = (h >> (4 - 2 * j)) & 3;
                    }
                    at += if k == 15 { 4 } else { 3 };
                }
                duid::V2
            }
            Burst::Facch(p) | Burst::Sacch(p) => {
                let fast = matches!(burst, Burst::Facch(_));
                let (j0, sent, len) = acch_layout(fast);
                let bytes = p.to_bytes(len / 8);
                let mut bits: Vec<u8> = bytes
                    .iter()
                    .flat_map(|&x| (0..8).rev().map(move |i| (x >> i) & 1))
                    .collect();
                let c = crc12(&bits);
                bits.extend((0..12).rev().map(|i| ((c >> i) & 1) as u8));
                let mut word = [0u8; rs63::K];
                for (k, h) in bits.chunks(6).enumerate() {
                    word[j0 + k] = h.iter().fold(0, |a, &x| (a << 1) | x);
                }
                let par = rs63::encode(&word);
                let full: Vec<u8> = word.iter().chain(&par).copied().collect();
                let sent_bits: Vec<u8> = full[j0..j0 + sent]
                    .iter()
                    .flat_map(|&h| (0..6).rev().map(move |i| (h >> i) & 1))
                    .collect();
                put_bits(&mut b, &acch_dibits(fast), &sent_bits);
                if fast { duid::FACCH_S } else { duid::SACCH_S }
            }
        };
        if scrambled(d_type) {
            for (i, x) in b.iter_mut().enumerate().skip(11) {
                *x ^= self.scr.dibit(t, i);
            }
        }
        let cw = duid_codeword(d_type);
        for (k, &i) in DUID_AT.iter().enumerate() {
            b[i] = (cw >> (6 - 2 * k)) & 3;
        }
        d[20..].copy_from_slice(&b[10..]);
        self.dibits.extend_from_slice(&d);
        self.ts += 1;
    }
}

/// A call on one logical channel: who, and its encryption (0x80 clear).
#[derive(Clone, Copy)]
pub struct Call {
    pub group: u16,
    pub source: u32,
    pub algid: u8,
}

/// What each channel does in one superframe.
#[derive(Clone, Copy)]
enum Phase {
    Idle,
    Ptt(Call),
    Voice(Call),
    End(Call),
}

/// A traffic channel with `calls[ch]` on each channel (or idle) for
/// `superframes` of voice each (a superframe of PTT before, one of the end
/// after, idle around): its dibits and the voice frames sent in all.
pub fn channel(seed: u64, calls: [Option<Call>; 2], superframes: usize) -> (Vec<u8>, u32) {
    let mut site = Site::new(seed);
    let mut frames = 0u32;
    let mut n = 0u32;
    let mi = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x11];
    let mut plan: Vec<[Phase; 2]> = vec![[Phase::Idle; 2]];
    let ph = |c: Option<Call>, f: fn(Call) -> Phase| c.map_or(Phase::Idle, f);
    plan.push([ph(calls[0], Phase::Ptt), ph(calls[1], Phase::Ptt)]);
    for _ in 0..superframes {
        plan.push([ph(calls[0], Phase::Voice), ph(calls[1], Phase::Voice)]);
    }
    plan.push([ph(calls[0], Phase::End), ph(calls[1], Phase::End)]);
    plan.push([Phase::Idle; 2]);
    for sf in plan {
        // Each channel's voice timeslots in order: 4V ×4 then 2V.
        let mut vb = [0usize; 2];
        for (t, &ch) in CHANNEL_OF.iter().enumerate() {
            let ch = ch as usize;
            let sacch = t >= 10;
            let user = |c: &Call| MacMsg::GroupVoice { opts: if c.algid != 0x80 { 0x40 } else { 0 }, group: c.group, source: c.source };
            let pdu = |op, msgs| MacPdu { op, ptt: None, messages: msgs };
            let burst = match sf[ch] {
                Phase::Idle => {
                    let p = pdu(MacOp::Idle, vec![]);
                    if sacch { Burst::Sacch(p) } else { Burst::Facch(p) }
                }
                Phase::Ptt(c) => {
                    let p = MacPdu { op: MacOp::Ptt, ptt: Some((c.source, c.group, c.algid, 0x0101)), messages: vec![] };
                    if sacch { Burst::Sacch(pdu(MacOp::Active, vec![user(&c)])) } else { Burst::Facch(p) }
                }
                Phase::End(c) => {
                    let p = MacPdu { op: MacOp::EndPtt, ptt: Some((c.source, c.group, 0x80, 0)), messages: vec![] };
                    if sacch { Burst::Sacch(pdu(MacOp::Idle, vec![])) } else { Burst::Facch(p) }
                }
                Phase::Voice(c) => {
                    if sacch {
                        Burst::Sacch(pdu(MacOp::Active, vec![user(&c)]))
                    } else {
                        let (b16, a28) = ess(c.algid, 0x0101, mi);
                        let k = vb[ch];
                        vb[ch] += 1;
                        let mut f = || {
                            n += 1;
                            frames += 1;
                            crate::dmr::sim::test_frame(n)
                        };
                        if k < 4 {
                            let fr = [f(), f(), f(), f()];
                            Burst::V4 { frames: fr, ess_b: b16[4 * k..4 * k + 4].try_into().expect("4") }
                        } else {
                            Burst::V2 { frames: [f(), f()], ess_a: a28 }
                        }
                    }
                }
            };
            site.push(&burst);
        }
    }
    (site.dibits, frames)
}

/// H-DQPSK: each dibit turns the carrier by its symbol × π/4 (+3 → +3π/4;
/// TIA-102.BBAB), root-raised-cosine shaped (α 0.2), unit power, at
/// `rate`, with `pad_s` of silence either side.
pub fn modulate(dibits: &[u8], rate: f64, pad_s: f64) -> Vec<Iq> {
    let sps = (rate / SYMBOL_RATE).round() as usize;
    let h = rrc(0.2, sps, 8);
    let pad = (pad_s * rate) as usize;
    let mut imp = vec![Iq::new(0.0, 0.0); pad + dibits.len() * sps + h.len() + pad];
    let mut ph = 0f64;
    for (k, &d) in dibits.iter().enumerate() {
        ph += f64::from(crate::dmr::symbol(d)) * std::f64::consts::FRAC_PI_4;
        imp[pad + k * sps] = Iq::from_polar(1.0, ph as f32);
    }
    let mut out = vec![Iq::new(0.0, 0.0); imp.len()];
    for (i, o) in out.iter_mut().enumerate() {
        let mut acc = Iq::new(0.0, 0.0);
        for (j, &t) in h.iter().enumerate() {
            if i >= j {
                acc += imp[i - j] * t;
            }
        }
        *o = acc;
    }
    let body = &out[pad..pad + dibits.len() * sps];
    let p = body.iter().map(|z| z.norm_sqr()).sum::<f32>() / body.len().max(1) as f32;
    let k = 1.0 / p.max(1e-12).sqrt();
    out.iter_mut().for_each(|z| *z *= k);
    out
}

/// White noise for a C/N in 12.5 kHz, and a frequency offset.
pub fn add_noise(iq: &mut [Iq], rate: f64, cn_db: f64, offset_hz: f64, seed: u64) {
    crate::fsk4::add_noise(iq, rate, cn_db, 12_500.0, offset_hz, seed);
}
