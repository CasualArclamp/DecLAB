// made by CasualArclamp
//! The phase reference symbol and the frequency interleaver, mode I.

use std::sync::OnceLock;

use num_complex::Complex32;

use crate::params::{CARRIERS, FFT};

/// EN 300 401 §14.3.2 Table 39 (mode I): for each run of 32 carriers from
/// k′ = kmin, the row i of `H` and the offset n; φ_k = π/2 · (h[i][k − k′] + n).
const PRS_RUNS: [(i32, u8, u8); 48] = [
    (-768, 0, 1),
    (-736, 1, 2),
    (-704, 2, 0),
    (-672, 3, 1),
    (-640, 0, 3),
    (-608, 1, 2),
    (-576, 2, 2),
    (-544, 3, 3),
    (-512, 0, 2),
    (-480, 1, 1),
    (-448, 2, 2),
    (-416, 3, 3),
    (-384, 0, 1),
    (-352, 1, 2),
    (-320, 2, 3),
    (-288, 3, 3),
    (-256, 0, 2),
    (-224, 1, 2),
    (-192, 2, 2),
    (-160, 3, 1),
    (-128, 0, 1),
    (-96, 1, 3),
    (-64, 2, 1),
    (-32, 3, 2),
    (1, 0, 3),
    (33, 3, 1),
    (65, 2, 1),
    (97, 1, 1),
    (129, 0, 2),
    (161, 3, 2),
    (193, 2, 1),
    (225, 1, 0),
    (257, 0, 2),
    (289, 3, 2),
    (321, 2, 3),
    (353, 1, 3),
    (385, 0, 0),
    (417, 3, 2),
    (449, 2, 1),
    (481, 1, 3),
    (513, 0, 3),
    (545, 3, 3),
    (577, 2, 3),
    (609, 1, 0),
    (641, 0, 3),
    (673, 3, 0),
    (705, 2, 1),
    (737, 1, 1),
];

/// EN 300 401 §14.3.2: the time-invariant parameters h_{i,j}.
const H: [[u8; 32]; 4] = [
    [
        0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1, 0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1,
        1,
    ],
    [
        0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0, 0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3,
        0,
    ],
    [
        0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3, 0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1,
        3,
    ],
    [
        0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2, 0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3,
        2,
    ],
];

/// The phase of carrier `k` in the phase reference symbol, in quarter turns
/// (0–3), or `None` for k = 0 and carriers outside ±768.
pub fn prs_quarter(k: i32) -> Option<u8> {
    if k == 0 || k.abs() > 768 {
        return None;
    }
    let &(k0, i, n) = PRS_RUNS.iter().rev().find(|r| r.0 <= k)?;
    Some((H[i as usize][(k - k0) as usize] + n) % 4)
}

/// The phase reference symbol as FFT bins (unit carriers, 0 elsewhere).
pub fn prs_bins() -> &'static [Complex32; FFT] {
    static T: OnceLock<Box<[Complex32; FFT]>> = OnceLock::new();
    T.get_or_init(|| {
        let mut t = Box::new([Complex32::new(0.0, 0.0); FFT]);
        for k in -768..=768 {
            if let Some(q) = prs_quarter(k) {
                // e^{jπq/2} exactly.
                t[crate::params::bin(k)] = [
                    Complex32::new(1.0, 0.0),
                    Complex32::new(0.0, 1.0),
                    Complex32::new(-1.0, 0.0),
                    Complex32::new(0.0, -1.0),
                ][q as usize];
            }
        }
        t
    })
}

/// The frequency interleaver (EN 300 401 §14.6.1): `carrier()[n]` is the
/// carrier k that QPSK symbol n (0…1535) of a data symbol goes out on.
///
/// Π(0) = 0, Π(i) = (13·Π(i−1) + 511) mod 2048; the Π that fall in
/// 256…1792, less 1024, taken in order, give k = Π − 1024.
pub fn carrier() -> &'static [i16; CARRIERS] {
    static T: OnceLock<[i16; CARRIERS]> = OnceLock::new();
    T.get_or_init(|| {
        let mut t = [0i16; CARRIERS];
        let mut p = 0u32;
        let mut n = 0;
        for i in 0..FFT {
            if i > 0 {
                p = (13 * p + 511) % FFT as u32;
            }
            if (256..=1792).contains(&p) && p != 1024 {
                t[n] = p as i16 - 1024;
                n += 1;
            }
        }
        debug_assert_eq!(n, CARRIERS);
        t
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interleaver_is_a_permutation_of_the_active_carriers() {
        let mut seen = std::collections::HashSet::new();
        for &k in carrier() {
            assert!(k != 0 && k.abs() <= 768);
            assert!(seen.insert(k));
        }
        assert_eq!(seen.len(), CARRIERS);
        // Π(1) = 511 is the first in range: k = −513.
        assert_eq!(carrier()[0], -513);
    }

    #[test]
    fn prs_covers_every_active_carrier() {
        let n = prs_bins().iter().filter(|c| c.norm_sqr() > 0.5).count();
        assert_eq!(n, CARRIERS);
        assert_eq!(prs_quarter(0), None);
        assert_eq!(prs_quarter(-768), Some(1));
    }
}
