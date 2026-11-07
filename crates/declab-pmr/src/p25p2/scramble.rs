// made by CasualArclamp
//! The phase 2 scrambling sequence, and finding its seed from the air.
//!
//! The sequence (OP25 `apps/tdma/lfsr.py`, GPL-3.0-or-later, © 2013, 2014
//! Max H. Parke KA1RBI): the 44-bit seed WACN (20) · system ID (12) · NAC
//! (12) is multiplied by a fixed 44×44 matrix into the register of six
//! coupled shift registers (4, 5, 6, 5, 14 and 10 bits), whose top bit is
//! sent and which then steps; 4320 bits, two per symbol, cover the 12
//! timeslots of a superframe.
//!
//! Every step is linear over GF(2), so each sequence bit is the parity of
//! some of the register's starting bits: [`Model`] keeps those 44-bit masks.
//! An ACCH as sent is a codeword of a binary linear code (the RS code read
//! bit by bit, punctured), so each of its parity checks, applied to the
//! scrambled bits, gives one linear equation in the register: the
//! codeword's share is zero and only the sequence's is left. A SACCH gives
//! 132 equations, a FACCH 114, for 44 unknowns; when they all agree the
//! register is known, and the seed is the matrix's inverse applied to it.
//! A bit error leaves the equations inconsistent, so the next clean burst
//! is waited for.

use std::sync::OnceLock;

use super::{SLOT_SYMBOLS, SUPERFRAME, acch_dibits, acch_layout, rs63};

/// Sequence bits a superframe.
pub const BITS: usize = 2 * SLOT_SYMBOLS * SUPERFRAME;

/// The seed's three parts packed as the LFSR takes them.
pub fn seed(wacn: u32, system: u16, nac: u16) -> u64 {
    (u64::from(wacn & 0xf_ffff) << 24) | (u64::from(system & 0xfff) << 12) | u64::from(nac & 0xfff)
}

/// (WACN, system, NAC) from a packed seed.
pub fn split(seed: u64) -> (u32, u16, u16) {
    ((seed >> 24) as u32 & 0xf_ffff, (seed >> 12) as u16 & 0xfff, seed as u16 & 0xfff)
}

/// The seed's matrix (lfsr.py's `M`): row i has ones at columns i, i+4,
/// i+9, i+15, i+20 and i+34 (those under 44). Bits are numbered from the
/// most significant, as `mk_array` lays them out.
fn seed_to_register(seed: u64) -> u64 {
    const OFFSETS: [usize; 6] = [0, 4, 9, 15, 20, 34];
    let mut reg = 0u64;
    for i in 0..44 {
        if (seed >> (43 - i)) & 1 == 1 {
            for &o in &OFFSETS {
                if i + o < 44 {
                    reg ^= 1 << (43 - (i + o));
                }
            }
        }
    }
    reg
}

/// One step of the register (lfsr.py `cyc_reg`).
fn step(reg: u64) -> u64 {
    let s1 = (reg >> 40) & 0xf;
    let s2 = (reg >> 35) & 0x1f;
    let s3 = (reg >> 29) & 0x3f;
    let s4 = (reg >> 24) & 0x1f;
    let s5 = (reg >> 10) & 0x3fff;
    let s6 = reg & 0x3ff;
    let cy1 = (s1 >> 3) & 1;
    let x = |s: u64, top: u32| cy1 ^ ((s >> top) & 1);
    let (x1, x2, x3, x4, x5) = (x(s2, 4), x(s3, 5), x(s4, 4), x(s5, 13), x(s6, 9));
    let s1 = ((s1 << 1) & 0xf) | x1;
    let s2 = ((s2 << 1) & 0x1f) | x2;
    let s3 = ((s3 << 1) & 0x3f) | x3;
    let s4 = ((s4 << 1) & 0x1f) | x4;
    let s5 = ((s5 << 1) & 0x3fff) | x5;
    let s6 = ((s6 << 1) & 0x3ff) | cy1;
    (s1 << 40) | (s2 << 35) | (s3 << 29) | (s4 << 24) | (s5 << 10) | s6
}

fn run(mut reg: u64) -> Vec<u8> {
    let mut out = Vec::with_capacity(BITS);
    for _ in 0..BITS {
        out.push(((reg >> 43) & 1) as u8);
        reg = step(reg);
    }
    out
}

/// A superframe's scrambling: the dibit to XOR onto each symbol.
#[derive(Clone)]
pub struct Scrambler {
    pub seed: u64,
    dibits: Vec<u8>,
}

impl Scrambler {
    pub fn new(seed: u64) -> Scrambler {
        Scrambler::from_register(seed, seed_to_register(seed))
    }

    fn from_register(seed: u64, reg: u64) -> Scrambler {
        let bits = run(reg);
        let dibits = bits.chunks(2).map(|p| (p[0] << 1) | p[1]).collect();
        Scrambler { seed, dibits }
    }

    /// The mask for burst dibit `i` (OP25's counting: from the timeslot's
    /// 10th symbol) of timeslot `slot`.
    #[inline]
    pub fn dibit(&self, slot: usize, i: usize) -> u8 {
        self.dibits[(slot * SLOT_SYMBOLS + i) % self.dibits.len()]
    }
}

/// Solve `rows` (each: 44-bit coefficients, right-hand side) over GF(2):
/// the unique solution, or `None` if they disagree or leave it open.
fn solve(mut rows: Vec<(u64, u8)>) -> Option<u64> {
    let mut pivots: Vec<(u32, usize)> = Vec::with_capacity(44);
    let mut r = 0;
    for bit in (0..44).rev() {
        let Some(p) = (r..rows.len()).find(|&k| (rows[k].0 >> bit) & 1 == 1) else {
            continue;
        };
        rows.swap(r, p);
        let (pv, pb) = rows[r];
        for (k, row) in rows.iter_mut().enumerate() {
            if k != r && (row.0 >> bit) & 1 == 1 {
                row.0 ^= pv;
                row.1 ^= pb;
            }
        }
        pivots.push((bit, r));
        r += 1;
    }
    // Rows left over must have reduced to 0 = 0.
    if pivots.len() < 44 || rows[r..].iter().any(|&(_, b)| b != 0) {
        return None;
    }
    Some(pivots.iter().fold(0u64, |x, &(bit, k)| x | (u64::from(rows[k].1) << bit)))
}

/// What recovery needs, worked out once.
pub struct Model {
    /// Sequence bit t = parity of (register & `g[t]`).
    g: Vec<u64>,
    /// Seed bit i = parity of (register & `reg_to_seed[i]`).
    reg_to_seed: Vec<u64>,
    /// Per ACCH kind (SACCH, FACCH): each parity check as the set of sent
    /// bits it covers (a bitset over up to 320 bits).
    checks: [Vec<[u64; 5]>; 2],
}

impl Model {
    pub fn get() -> &'static Model {
        static M: OnceLock<Model> = OnceLock::new();
        M.get_or_init(|| {
            // Each register bit's own sequence, then transposed into masks.
            let mut g = vec![0u64; BITS];
            for k in 0..44 {
                for (t, b) in run(1 << k).into_iter().enumerate() {
                    g[t] |= u64::from(b) << k;
                }
            }
            // Register bit j as a mask over the seed's bits (linear), then
            // the seed's bits from the register: solve, column by column.
            let mut fwd = [0u64; 44];
            for i in 0..44 {
                let r = seed_to_register(1 << i);
                for (j, f) in fwd.iter_mut().enumerate() {
                    *f |= ((r >> j) & 1) << i;
                }
            }
            // The seed s_j whose register is bit j alone; by linearity seed
            // bit i is then the parity of the register over those j whose
            // s_j has bit i.
            let s: Vec<u64> = (0..44)
                .map(|j| {
                    let rows: Vec<(u64, u8)> = (0..44).map(|k| (fwd[k], u8::from(k == j))).collect();
                    solve(rows).expect("the matrix is invertible")
                })
                .collect();
            let reg_to_seed = (0..44)
                .map(|i| (0..44).fold(0u64, |m, j| m | (((s[j] >> i) & 1) << j)))
                .collect();
            Model { g, reg_to_seed, checks: [Self::checks(false), Self::checks(true)] }
        })
    }

    /// The parity checks of the sent ACCH code: G = [I | P] in sending
    /// order (data hexbits come first), so check j covers the data bits u
    /// with P[u][j] and parity bit j itself.
    fn checks(fast: bool) -> Vec<[u64; 5]> {
        let (j0, sent, _) = acch_layout(fast);
        let kd = rs63::K - j0;
        let (kb, rb) = (6 * kd, 6 * (sent - kd));
        let mut checks = vec![[0u64; 5]; rb];
        for u in 0..kb {
            let mut d = [0u8; rs63::K];
            d[j0 + u / 6] = 1 << (5 - u % 6);
            let p = rs63::encode(&d);
            for j in 0..rb {
                if (p[j / 6] >> (5 - j % 6)) & 1 == 1 {
                    checks[j][u / 64] |= 1 << (u % 64);
                }
            }
        }
        for (j, c) in checks.iter_mut().enumerate() {
            let b = kb + j;
            c[b / 64] |= 1 << (b % 64);
        }
        checks
    }

    /// The register from one scrambled ACCH's hard bits as sent (`bits`, in
    /// [`acch_dibits`] order, high bit first) in timeslot `slot`: the
    /// scrambler, if every check agrees.
    pub fn recover(&self, fast: bool, slot: usize, bits: &[u8]) -> Option<Scrambler> {
        let dibits = acch_dibits(fast);
        let t = |p: usize| 2 * (slot * SLOT_SYMBOLS + dibits[p / 2]) + p % 2;
        let rows: Vec<(u64, u8)> = self.checks[usize::from(fast)]
            .iter()
            .map(|c| {
                let (mut coef, mut rhs) = (0u64, 0u8);
                for (w, &word) in c.iter().enumerate() {
                    let mut m = word;
                    while m != 0 {
                        let p = w * 64 + m.trailing_zeros() as usize;
                        m &= m - 1;
                        coef ^= self.g[t(p) % BITS];
                        rhs ^= bits[p] & 1;
                    }
                }
                (coef, rhs)
            })
            .collect();
        let reg = solve(rows)?;
        let seed = self
            .reg_to_seed
            .iter()
            .enumerate()
            .fold(0u64, |s, (i, &m)| s | (u64::from((reg & m).count_ones() as u8 & 1) << i));
        Some(Scrambler::from_register(seed, reg))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_masks_reproduce_the_sequence() {
        let s = seed(0xBEE00, 0x3A1, 0x293);
        let direct = run(seed_to_register(s));
        let m = Model::get();
        let reg = seed_to_register(s);
        for (t, &b) in direct.iter().enumerate().step_by(7) {
            assert_eq!((reg & m.g[t]).count_ones() as u8 & 1, b, "bit {t}");
        }
    }

    #[test]
    fn seed_comes_back_from_the_register() {
        let m = Model::get();
        for s in [seed(0xBEE00, 0x3A1, 0x293), seed(1, 0x18, 0x293), seed(0xfffff, 0xfff, 0xfff)] {
            let reg = seed_to_register(s);
            let back = m.reg_to_seed.iter().enumerate().fold(0u64, |a, (i, &k)| {
                a | (u64::from((reg & k).count_ones() as u8 & 1) << i)
            });
            assert_eq!(back, s);
        }
    }
}
