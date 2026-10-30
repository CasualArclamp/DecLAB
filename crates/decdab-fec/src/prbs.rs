// made by CasualArclamp
//! Energy dispersal (EN 300 401 §10): the PRBS of x⁹ + x⁵ + 1, the
//! register starting at all ones, added to every FIB group and every
//! sub-channel's logical frame from its first bit (§10.1, §10.2).

/// The first `n` bits of the PRBS, one per byte.
pub fn sequence(n: usize) -> Vec<u8> {
    let mut reg = 0x1FFu16;
    (0..n)
        .map(|_| {
            let b = ((reg >> 8) ^ (reg >> 4)) & 1;
            reg = ((reg << 1) | b) & 0x1FF;
            b as u8
        })
        .collect()
}

/// A cached PRBS, long enough for the largest block (a 384 kbit/s
/// sub-channel's 9216 bits a CIF).
pub struct Prbs {
    seq: Vec<u8>,
}

impl Default for Prbs {
    fn default() -> Self {
        Prbs {
            seq: sequence(16 * 1024),
        }
    }
}

impl Prbs {
    /// XOR the sequence into `bits` (one per byte).
    pub fn apply(&self, bits: &mut [u8]) {
        for (b, p) in bits.iter_mut().zip(&self.seq) {
            *b ^= p;
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn first_bits() {
        // §10.1: "0000 0111 1011 1110 …".
        let s = super::sequence(16);
        assert_eq!(s, [0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0]);
    }
}
