// made by CasualArclamp
//! TETRA (ETSI EN 300 392-2) downlink, π/4-DQPSK: the continuous downlink
//! bursts of a base station, the logical channels in them and enough of
//! the MAC, LLC, MLE and CMCE to follow a cell: its identity, broadcasts,
//! call set-ups, transmit grants, status and short data. Speech (TCH/S) is
//! handed on as soft bits for the ACELP codec (`crate::vocoder::acelp`).
//!
//! - [`coding`]: RCPC, Reed-Muller, CRC, interleaving, scrambling (clause 8).
//! - [`mac`]: the PDUs (clauses 14, 18, 21).
//! - [`receiver`]: IQ to events.
//! - [`sim`]: a simulated base station for the tests and sweeps.

pub mod coding;
pub mod mac;
pub mod receiver;
pub mod sim;

pub use mac::{Pdu, Signal};
pub use receiver::{Event, Receiver};

/// Clause 5.3: 36 kbit/s, two bits a symbol.
pub const SYMBOL_RATE: f64 = 18_000.0;
/// Clause 5.5: the square root raised cosine's roll-off.
pub const ALPHA: f64 = 0.35;
/// Clause 9.3.3: 255 symbols, 510 bits, a timeslot.
pub const SLOT_SYMBOLS: usize = 255;
pub const SLOT_BITS: usize = 510;

/// A phase transition in units of π/4 for the dibit B(2k−1), B(2k),
/// clause 5.4 table 5.1.
pub fn transition(b1: u8, b2: u8) -> i32 {
    match (b1 & 1, b2 & 1) {
        (0, 0) => 1,
        (0, 1) => 3,
        (1, 1) => -3,
        _ => -1,
    }
}

/// The dibit for a transition of `d`·π/4 (d odd, taken mod 8).
pub fn dibit(d: i32) -> (u8, u8) {
    match d.rem_euclid(8) {
        1 => (0, 0),
        3 => (0, 1),
        5 => (1, 1),
        _ => (1, 0),
    }
}

fn parse(s: &str) -> Vec<u8> {
    s.bytes()
        .filter(|b| b.is_ascii_digit())
        .map(|b| b - b'0')
        .collect()
}

/// Clause 9.4.4.3.2 (9.4)–(9.6): normal training sequences 1, 2 and 3.
pub fn train_n() -> Vec<u8> {
    parse("11 01 00 00 11 10 10 01 11 01 00")
}
pub fn train_p() -> Vec<u8> {
    parse("01 11 10 10 01 00 00 11 01 11 10")
}
pub fn train_q() -> Vec<u8> {
    parse("10 11 01 11 00 00 01 10 10 11 01")
}
/// Clause 9.4.4.3.4 (9.11): the synchronization training sequence.
pub fn train_y() -> Vec<u8> {
    parse("11 00 00 01 10 01 11 00 11 10 10 01 11 00 00 01 10 01 11")
}
/// Clause 9.4.4.3.1: the frequency correction field.
pub fn freq_correction() -> Vec<u8> {
    let mut f = vec![1u8; 8];
    f.extend([0u8; 64]);
    f.extend([1u8; 8]);
    f
}

/// Burst fields as 0-based bit ranges, clause 9.4.4.2.5 table 9.7 and
/// 9.4.4.2.6 table 9.9 (BN − 1).
pub mod field {
    use std::ops::Range;
    /// Both kinds: training sequence 3, q11–q22 then q1–q10.
    pub const Q_HEAD: Range<usize> = 0..12;
    pub const Q_TAIL: Range<usize> = 500..510;
    /// Normal continuous downlink burst.
    pub const NDB_BKN1: Range<usize> = 14..230;
    pub const NDB_BB1: Range<usize> = 230..244;
    pub const NDB_TRAIN: Range<usize> = 244..266;
    pub const NDB_BB2: Range<usize> = 266..282;
    pub const BKN2: Range<usize> = 282..498;
    /// Synchronization continuous downlink burst.
    pub const SDB_FREQ: Range<usize> = 14..94;
    pub const SDB_SB: Range<usize> = 94..214;
    pub const SDB_TRAIN: Range<usize> = 214..252;
    pub const SDB_BB: Range<usize> = 252..282;
}

/// Phase adjustment symbols: (symbol number, n1, n2), clause 9.4.4.3.6
/// table 9.15: ha, hb (normal), hc, hd (synchronization).
pub const PHASE_ADJ_NDB: [(usize, usize, usize); 2] = [(7, 8, 122), (250, 123, 249)];
pub const PHASE_ADJ_SDB: [(usize, usize, usize); 2] = [(7, 8, 108), (250, 109, 249)];

/// Which training sequence a normal burst carries, table 9.13.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Burst {
    /// Training 1: one full-slot channel (SCH/F or TCH).
    Full,
    /// Training 2: two half-slot channels (SCH/HD, BNCH, STCH, TCH half).
    Half,
    /// The synchronization burst: BSCH and one half-slot channel.
    Sync,
}

/// Set the phase adjustment bits of a burst, (9.14): the transition at the
/// adjustment symbol cancels the sum of the transitions over n1..n2.
pub fn set_phase_adjustment(b: &mut [u8; SLOT_BITS], adj: &[(usize, usize, usize)]) {
    for &(h, n1, n2) in adj {
        let sum: i32 = (n1..=n2)
            .map(|n| transition(b[2 * n - 2], b[2 * n - 1]))
            .sum();
        let (x, y) = dibit(-sum);
        b[2 * h - 2] = x;
        b[2 * h - 1] = y;
    }
}

/// A normal continuous downlink burst, table 9.7.
pub fn normal_burst(full: bool, bkn1: &[u8], bb: &[u8], bkn2: &[u8]) -> [u8; SLOT_BITS] {
    let mut b = [0u8; SLOT_BITS];
    let q = train_q();
    b[field::Q_HEAD].copy_from_slice(&q[10..22]);
    b[field::Q_TAIL].copy_from_slice(&q[..10]);
    b[field::NDB_BKN1].copy_from_slice(bkn1);
    b[field::NDB_BB1].copy_from_slice(&bb[..14]);
    b[field::NDB_TRAIN].copy_from_slice(&if full { train_n() } else { train_p() });
    b[field::NDB_BB2].copy_from_slice(&bb[14..]);
    b[field::BKN2].copy_from_slice(bkn2);
    set_phase_adjustment(&mut b, &PHASE_ADJ_NDB);
    b
}

/// A synchronization continuous downlink burst, table 9.9.
pub fn sync_burst(sb: &[u8], bb: &[u8], bkn2: &[u8]) -> [u8; SLOT_BITS] {
    let mut b = [0u8; SLOT_BITS];
    let q = train_q();
    b[field::Q_HEAD].copy_from_slice(&q[10..22]);
    b[field::Q_TAIL].copy_from_slice(&q[..10]);
    b[field::SDB_FREQ].copy_from_slice(&freq_correction());
    b[field::SDB_SB].copy_from_slice(sb);
    b[field::SDB_TRAIN].copy_from_slice(&train_y());
    b[field::SDB_BB].copy_from_slice(bb);
    b[field::BKN2].copy_from_slice(bkn2);
    set_phase_adjustment(&mut b, &PHASE_ADJ_SDB);
    b
}

/// Downlink carrier frequency from a frequency band, carrier number and
/// offset code, clause 21.4.4.1: base + carrier · 25 kHz + offset. The base
/// for band n is n · 100 MHz (ETSI TS 100 392-15).
pub fn carrier_hz(band: u8, carrier: u16, offset: u8) -> f64 {
    let off = match offset & 3 {
        0 => 0.0,
        1 => 6_250.0,
        2 => -6_250.0,
        _ => 12_500.0,
    };
    f64::from(band) * 100e6 + f64::from(carrier) * 25_000.0 + off
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The phase adjustment makes the burst's training sequences land on
    /// the same absolute phases whatever the data.
    #[test]
    fn phase_adjustment_balances() {
        let mut x = 12345u32;
        let mut rnd = |n: usize| -> Vec<u8> {
            (0..n)
                .map(|_| {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    (x & 1) as u8
                })
                .collect()
        };
        for _ in 0..4 {
            let b = normal_burst(true, &rnd(216), &rnd(30), &rnd(216));
            let sum = |a: usize, z: usize| -> i32 {
                (a..=z)
                    .map(|n| transition(b[2 * n - 2], b[2 * n - 1]))
                    .sum::<i32>()
                    .rem_euclid(8)
            };
            assert_eq!(sum(7, 122), 0);
            assert_eq!(sum(123, 250), 0);
            let s = sync_burst(&rnd(120), &rnd(30), &rnd(216));
            let sum = |a: usize, z: usize| -> i32 {
                (a..=z)
                    .map(|n| transition(s[2 * n - 2], s[2 * n - 1]))
                    .sum::<i32>()
                    .rem_euclid(8)
            };
            assert_eq!(sum(7, 108), 0);
            assert_eq!(sum(109, 250), 0);
        }
    }
}
