// made by CasualArclamp
//! DAB's convolutional code and its puncturing (EN 300 401 §11).
//!
//! The mother code is rate 1/4, K = 7, generators 133, 171, 145, 133
//! (octal, §11.1.1); the newest bit is the generators' most significant
//! tap. Every coded block (a FIC block, a sub-channel's share of a CIF)
//! starts from the zero state and ends with six zero tail bits.
//!
//! Puncturing (§11.1.2) keeps the mother bits marked in a 32-bit vector
//! PI_i (8 input bits, 8 + i kept: rate 8/(8 + i)); a block of 128 mother
//! bits uses one vector four times, and the 24 tail bits use PI_X. A
//! [`Plan`] lists the runs of blocks and the vector of each.
//!
//! Conventions: bits are one per byte (0/1); soft bits are i8, positive
//! for a 0, negative for a 1, 0 for none (a punctured position).

/// The generators, newest bit first (bit 6).
pub const GENERATORS: [u8; 4] = [0o133, 0o171, 0o145, 0o133];

/// The order in which the eight 4-bit groups of a vector gain a kept bit as
/// i rises (bit-reversed: §11.1.2 Table 29's pattern).
const GROUP_ORDER: [usize; 8] = [0, 4, 2, 6, 1, 5, 3, 7];

/// Puncturing vector PI_i, i = 1…24, as 32 bits, the first mother bit the
/// most significant (EN 300 401 Table 29).
///
/// Table 29 is regular: each of the eight groups of four mother bits (one
/// input bit's four outputs) keeps 1–4 of them, from the first; PI_8 keeps
/// two in every group, and from there each step adds one bit to the next
/// group in [`GROUP_ORDER`].
pub fn pi(i: u8) -> u32 {
    assert!((1..=24).contains(&i), "PI_{i}");
    let base = (i as usize - 1) / 8; // 0, 1, 2
    let extra = (i as usize - 1) % 8; // groups 0..=extra get one more
    let mut v = 0u32;
    for (rank, &g) in GROUP_ORDER.iter().enumerate() {
        let keep = base + 1 + usize::from(rank <= extra);
        let group = (0xFu32 << (4 - keep)) & 0xF;
        v |= group << (28 - 4 * g);
    }
    v
}

/// PI_X, the tail's 24 bits (Table 30): 1100 repeated.
pub const PI_X: u32 = 0xCCCCCC;

/// A puncturing plan: runs of 128-mother-bit blocks with their vector,
/// then the tail, then `pad` zero bits to fill the sub-channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Plan {
    pub runs: Vec<(usize, u8)>,
    pub pad: usize,
}

/// Equal error protection profile sets (§11.3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EepSet {
    /// Bit rates n × 8 kbit/s.
    A,
    /// Bit rates n × 32 kbit/s.
    B,
}

/// One UEP profile: sub-channel size (CUs), protection level, bit rate
/// (kbit/s), L1–L4, PI1–PI4, padding bits.
pub type UepRow = (u16, u8, u16, [u8; 4], [u8; 4], u8);

/// UEP profiles (§11.3.1 Table 31).
#[rustfmt::skip]
pub const UEP_TABLE: [UepRow; 64] = [
    (16, 5, 32, [3, 4, 17, 0], [5, 3, 2, 0], 0),
    (21, 4, 32, [3, 3, 18, 0], [11, 6, 5, 0], 0),
    (24, 3, 32, [3, 4, 14, 3], [15, 9, 6, 8], 0),
    (29, 2, 32, [3, 4, 14, 3], [22, 13, 8, 13], 0),
    (35, 1, 32, [3, 5, 13, 3], [24, 17, 12, 17], 4),
    (24, 5, 48, [4, 3, 26, 3], [5, 4, 2, 3], 0),
    (29, 4, 48, [3, 4, 26, 3], [9, 6, 4, 6], 0),
    (35, 3, 48, [3, 4, 26, 3], [15, 10, 6, 9], 4),
    (42, 2, 48, [3, 4, 26, 3], [24, 14, 8, 15], 0),
    (52, 1, 48, [3, 5, 25, 3], [24, 18, 13, 18], 0),
    (29, 5, 56, [6, 10, 23, 3], [5, 4, 2, 3], 0),
    (35, 4, 56, [6, 10, 23, 3], [9, 6, 4, 5], 0),
    (42, 3, 56, [6, 12, 21, 3], [16, 7, 6, 9], 0),
    (52, 2, 56, [6, 10, 23, 3], [23, 13, 8, 13], 8),
    (32, 5, 64, [6, 9, 31, 2], [5, 3, 2, 3], 0),
    (42, 4, 64, [6, 9, 33, 0], [11, 6, 5, 0], 0),
    (48, 3, 64, [6, 12, 27, 3], [16, 8, 6, 9], 0),
    (58, 2, 64, [6, 10, 29, 3], [23, 13, 8, 13], 8),
    (70, 1, 64, [6, 11, 28, 3], [24, 18, 12, 18], 4),
    (40, 5, 80, [6, 10, 41, 3], [6, 3, 2, 3], 0),
    (52, 4, 80, [6, 10, 41, 3], [11, 6, 5, 6], 0),
    (58, 3, 80, [6, 11, 40, 3], [16, 8, 6, 7], 0),
    (70, 2, 80, [6, 10, 41, 3], [23, 13, 8, 13], 8),
    (84, 1, 80, [6, 10, 41, 3], [24, 17, 12, 18], 4),
    (48, 5, 96, [7, 9, 53, 3], [5, 4, 2, 4], 0),
    (58, 4, 96, [7, 10, 52, 3], [9, 6, 4, 6], 0),
    (70, 3, 96, [6, 12, 51, 3], [16, 9, 6, 10], 4),
    (84, 2, 96, [6, 10, 53, 3], [22, 12, 9, 12], 0),
    (104, 1, 96, [6, 13, 50, 3], [24, 18, 13, 19], 0),
    (58, 5, 112, [14, 17, 50, 3], [5, 4, 2, 5], 0),
    (70, 4, 112, [11, 21, 49, 3], [9, 6, 4, 8], 0),
    (84, 3, 112, [11, 23, 47, 3], [16, 8, 6, 9], 0),
    (104, 2, 112, [11, 21, 49, 3], [23, 12, 9, 14], 4),
    (64, 5, 128, [12, 19, 62, 3], [5, 3, 2, 4], 0),
    (84, 4, 128, [11, 21, 61, 3], [11, 6, 5, 7], 0),
    (96, 3, 128, [11, 22, 60, 3], [16, 9, 6, 10], 4),
    (116, 2, 128, [11, 21, 61, 3], [22, 12, 9, 14], 0),
    (140, 1, 128, [11, 20, 62, 3], [24, 17, 13, 19], 8),
    (80, 5, 160, [11, 19, 87, 3], [5, 4, 2, 4], 0),
    (104, 4, 160, [11, 23, 83, 3], [11, 6, 5, 9], 0),
    (116, 3, 160, [11, 24, 82, 3], [16, 8, 6, 11], 0),
    (140, 2, 160, [11, 21, 85, 3], [22, 11, 9, 13], 0),
    (168, 1, 160, [11, 22, 84, 3], [24, 18, 12, 19], 0),
    (96, 5, 192, [11, 20, 110, 3], [6, 4, 2, 5], 0),
    (116, 4, 192, [11, 22, 108, 3], [10, 6, 4, 9], 0),
    (140, 3, 192, [11, 24, 106, 3], [16, 10, 6, 11], 0),
    (168, 2, 192, [11, 20, 110, 3], [22, 13, 9, 13], 8),
    (208, 1, 192, [11, 21, 109, 3], [24, 20, 13, 24], 0),
    (116, 5, 224, [12, 22, 131, 3], [8, 6, 2, 6], 4),
    (140, 4, 224, [12, 26, 127, 3], [12, 8, 4, 11], 0),
    (168, 3, 224, [11, 20, 134, 3], [16, 10, 7, 9], 0),
    (208, 2, 224, [11, 22, 132, 3], [24, 16, 10, 15], 0),
    (232, 1, 224, [11, 24, 130, 3], [24, 20, 12, 20], 4),
    (128, 5, 256, [11, 24, 154, 3], [6, 5, 2, 5], 0),
    (168, 4, 256, [11, 24, 154, 3], [12, 9, 5, 10], 4),
    (192, 3, 256, [11, 27, 151, 3], [16, 10, 7, 10], 0),
    (232, 2, 256, [11, 22, 156, 3], [24, 14, 10, 13], 8),
    (280, 1, 256, [11, 26, 152, 3], [24, 19, 14, 18], 4),
    (160, 5, 320, [11, 26, 200, 3], [8, 5, 2, 6], 4),
    (208, 4, 320, [11, 25, 201, 3], [13, 9, 5, 10], 8),
    (280, 2, 320, [11, 26, 200, 3], [24, 17, 9, 17], 0),
    (192, 5, 384, [11, 27, 247, 3], [8, 6, 2, 7], 0),
    (280, 3, 384, [11, 24, 250, 3], [16, 9, 7, 10], 4),
    (416, 1, 384, [12, 28, 245, 3], [24, 20, 14, 23], 8),
];

impl Plan {
    /// The FIC's plan in mode I (§11.2): 21 blocks at PI_16, 3 at PI_15.
    pub fn fic() -> Plan {
        Plan {
            runs: vec![(21, 16), (3, 15)],
            pad: 0,
        }
    }

    /// An EEP sub-channel of `cu` capacity units at `level` (1–4) of `set`
    /// (§11.3.2 Tables 32 and 33). `None` if the size does not fit.
    pub fn eep(set: EepSet, level: u8, cu: usize) -> Option<Plan> {
        let (l1, l2, p1, p2) = match set {
            EepSet::A => {
                let per = [12, 8, 6, 4][level.checked_sub(1)? as usize % 4];
                if level > 4 || !cu.is_multiple_of(per) || cu == 0 {
                    return None;
                }
                let n = cu / per;
                match level {
                    1 => (6 * n - 3, 3, 24, 23),
                    2 if n == 1 => (5, 1, 13, 12),
                    2 => (2 * n - 3, 4 * n + 3, 14, 13),
                    3 => (6 * n - 3, 3, 8, 7),
                    _ => (4 * n - 3, 2 * n + 3, 3, 2),
                }
            }
            EepSet::B => {
                let per = [27, 21, 18, 15][level.checked_sub(1)? as usize % 4];
                if level > 4 || !cu.is_multiple_of(per) || cu == 0 {
                    return None;
                }
                let n = cu / per;
                let (p1, p2) = [(10, 9), (6, 5), (4, 3), (2, 1)][level as usize - 1];
                (24 * n - 3, 3, p1, p2)
            }
        };
        Some(Plan {
            runs: vec![(l1, p1), (l2, p2)],
            pad: 0,
        })
    }

    /// A UEP sub-channel, by its Table 31 index (FIG 0/1's table index).
    pub fn uep(index: usize) -> Option<Plan> {
        let &(_, _, _, l, p, pad) = UEP_TABLE.get(index)?;
        Some(Plan {
            runs: l
                .iter()
                .zip(p)
                .filter(|(l, _)| **l > 0)
                .map(|(&l, p)| (l as usize, p))
                .collect(),
            pad: pad as usize,
        })
    }

    /// Input (information) bits a block carries.
    pub fn info_bits(&self) -> usize {
        self.runs.iter().map(|&(l, _)| 32 * l).sum()
    }

    /// Mother-code bits, tail included.
    pub fn mother_bits(&self) -> usize {
        4 * (self.info_bits() + 6)
    }

    /// Bits sent, tail and padding included.
    pub fn coded_bits(&self) -> usize {
        self.runs
            .iter()
            .map(|&(l, p)| 4 * l * (8 + p as usize))
            .sum::<usize>()
            + 12
            + self.pad
    }

    /// The code rate, information bits over bits sent, in lowest terms
    /// (EEP 3-A is 1/2, the FIC 1/3).
    pub fn code_rate(&self) -> (usize, usize) {
        let (k, n) = (self.info_bits(), self.coded_bits());
        let g = gcd(k, n).max(1);
        (k / g, n / g)
    }

    /// The kept/erased pattern over the mother bits, in order.
    fn mask(&self) -> impl Iterator<Item = bool> + '_ {
        let runs = self.runs.iter().flat_map(|&(l, p)| {
            let v = pi(p);
            (0..128 * l).map(move |i| (v >> (31 - i % 32)) & 1 == 1)
        });
        runs.chain((0..24).map(|i| (PI_X >> (23 - i)) & 1 == 1))
    }

    /// Spread `coded` (at least `coded_bits()` soft bits; padding ignored)
    /// over the mother code's positions, zeros where bits were punctured.
    pub fn depuncture(&self, coded: &[i8], mother: &mut Vec<i8>) {
        mother.clear();
        mother.reserve(self.mother_bits());
        let mut it = coded.iter();
        for keep in self.mask() {
            mother.push(if keep { *it.next().unwrap_or(&0) } else { 0 });
        }
    }

    /// Keep the mother bits the plan sends, then the padding (zeros).
    pub fn puncture(&self, mother: &[u8], out: &mut Vec<u8>) {
        debug_assert_eq!(mother.len(), self.mother_bits());
        out.extend(
            self.mask()
                .zip(mother)
                .filter(|(k, _)| *k)
                .map(|(_, &b)| b),
        );
        out.extend(std::iter::repeat_n(0, self.pad));
    }
}

fn gcd(a: usize, b: usize) -> usize {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Encode `bits` with the mother code, six zero tail bits appended: four
/// mother bits per input bit, in generator order.
pub fn encode(bits: &[u8], out: &mut Vec<u8>) {
    let mut reg = 0u8;
    for &b in bits.iter().chain(&[0u8; 6]) {
        reg = (reg >> 1) | ((b & 1) << 6);
        for g in GENERATORS {
            out.push(((reg & g).count_ones() & 1) as u8);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_29_rows() {
        assert_eq!(pi(1), 0b1100_1000_1000_1000_1000_1000_1000_1000);
        assert_eq!(pi(2), 0b1100_1000_1000_1000_1100_1000_1000_1000);
        assert_eq!(pi(8), 0b1100_1100_1100_1100_1100_1100_1100_1100);
        assert_eq!(pi(9), 0b1110_1100_1100_1100_1100_1100_1100_1100);
        assert_eq!(pi(16), 0b1110_1110_1110_1110_1110_1110_1110_1110);
        assert_eq!(pi(24), 0xFFFF_FFFF);
        for i in 1..=24u8 {
            assert_eq!(pi(i).count_ones(), 8 + i as u32);
        }
    }

    #[test]
    fn fic_block_fills_its_share_of_the_fic() {
        let p = Plan::fic();
        assert_eq!(p.info_bits(), 768); // three FIBs
        assert_eq!(p.coded_bits(), 2304);
        assert_eq!(p.code_rate(), (1, 3));
        assert_eq!(Plan::eep(EepSet::A, 3, 48).unwrap().code_rate(), (1, 2));
        assert_eq!(Plan::eep(EepSet::B, 4, 15).unwrap().code_rate(), (4, 5));
    }

    #[test]
    fn every_uep_profile_adds_up() {
        // Each row's input bits must be its bit rate × 24 ms and its coded
        // bits its size × 64: a check on every number in the table.
        for (i, &(cu, _, kbps, ..)) in UEP_TABLE.iter().enumerate() {
            let p = Plan::uep(i).unwrap();
            assert_eq!(p.info_bits(), kbps as usize * 24, "row {i} input");
            assert_eq!(p.coded_bits(), cu as usize * 64, "row {i} coded");
        }
    }

    #[test]
    fn every_eep_profile_adds_up() {
        for level in 1..=4u8 {
            for n in 1..=48usize {
                let cu = [12, 8, 6, 4][level as usize - 1] * n;
                let p = Plan::eep(EepSet::A, level, cu).unwrap();
                assert_eq!(p.info_bits(), 8 * n * 24, "{level}-A n {n}");
                assert_eq!(p.coded_bits(), cu * 64, "{level}-A n {n}");
            }
            for n in 1..=8usize {
                let cu = [27, 21, 18, 15][level as usize - 1] * n;
                if cu > 864 {
                    continue;
                }
                let p = Plan::eep(EepSet::B, level, cu).unwrap();
                assert_eq!(p.info_bits(), 32 * n * 24, "{level}-B n {n}");
                assert_eq!(p.coded_bits(), cu * 64, "{level}-B n {n}");
            }
        }
        assert!(Plan::eep(EepSet::A, 3, 7).is_none());
    }

    #[test]
    fn puncture_and_depuncture_agree() {
        let p = Plan::eep(EepSet::A, 3, 48).unwrap();
        let bits: Vec<u8> = (0..p.info_bits()).map(|i| ((i * 7) % 3 == 0) as u8).collect();
        let mut mother = Vec::new();
        encode(&bits, &mut mother);
        let mut tx = Vec::new();
        p.puncture(&mother, &mut tx);
        assert_eq!(tx.len(), p.coded_bits());
        let soft: Vec<i8> = tx.iter().map(|&b| if b == 0 { 50 } else { -50 }).collect();
        let mut back = Vec::new();
        p.depuncture(&soft, &mut back);
        for (m, s) in mother.iter().zip(&back) {
            if *s != 0 {
                assert_eq!(*m == 0, *s > 0);
            }
        }
    }
}
