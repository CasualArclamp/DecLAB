// made by CasualArclamp
//! The small block codes, CRCs and the Reed-Solomon code DMR protects its
//! signalling with (ETSI TS 102 361-1 annex B), decoded for weak signals.
//!
//! Soft input everywhere is a log-likelihood ratio per bit, **positive for a
//! 0**. The codes short enough to enumerate (Golay (20,8), QR (16,7),
//! Hamming (7,4)) are decoded by maximum likelihood over every codeword; the
//! longer Hamming and Golay codes by Chase-II: the hard decision and its
//! least reliable bits flipped every way, each tried through the algebraic
//! decoder, the codeword nearest the soft input kept. Both are worth about
//! 2 dB over hard decisions.
//!
//! A codeword is kept in a `u32` the way the spec tables print it: the first
//! transmitted bit is the most significant (bit `n - 1`).

use std::sync::OnceLock;

/// The hard decision of an LLR.
#[inline]
pub fn hard(llr: f32) -> u32 {
    u32::from(llr < 0.0)
}

/// Pack hard decisions of `llr` (first bit most significant).
pub fn pack_hard(llr: &[f32]) -> u32 {
    llr.iter().fold(0, |w, &l| (w << 1) | hard(l))
}

/// Pack 0/1 bits (first bit most significant).
pub fn pack(bits: &[u8]) -> u64 {
    bits.iter().fold(0, |w, &b| (w << 1) | u64::from(b & 1))
}

/// Unpack the low `n` bits of `w` into 0/1 bits, most significant first.
pub fn unpack(w: u64, n: usize, out: &mut Vec<u8>) {
    out.extend((0..n).rev().map(|i| ((w >> i) & 1) as u8));
}

/// How well codeword `w` (`n` bits) agrees with `llr`: the correlation
/// Σ ±llr, larger is better.
#[inline]
pub fn soft_metric(w: u32, llr: &[f32]) -> f32 {
    let n = llr.len();
    let mut m = 0.0;
    for (i, &l) in llr.iter().enumerate() {
        if (w >> (n - 1 - i)) & 1 == 0 {
            m += l;
        } else {
            m -= l;
        }
    }
    m
}

/// A systematic binary (n, k) block code, built from its generator matrix.
pub struct BlockCode {
    pub n: u32,
    pub k: u32,
    /// The parity bits each data bit generates; data bit 0 (the first
    /// sent) first.
    parity_rows: Vec<u32>,
    /// Syndrome → the codeword bit (u32 bit index) a single error there
    /// gives, or -1.
    single: Vec<i8>,
    /// Every codeword by data value, for maximum-likelihood decoding (only
    /// built for k ≤ 8).
    codewords: Vec<u32>,
}

impl BlockCode {
    /// From the generator matrix's rows as the spec prints them ("1 0 0 …"),
    /// one row per data bit.
    pub fn from_rows(rows: &[&str]) -> BlockCode {
        let k = rows.len() as u32;
        let bits: Vec<Vec<u32>> = rows
            .iter()
            .map(|r| r.chars().filter_map(|c| c.to_digit(2)).collect())
            .collect();
        let n = bits[0].len() as u32;
        let mut parity_rows = Vec::with_capacity(rows.len());
        for (i, row) in bits.iter().enumerate() {
            assert_eq!(row.len() as u32, n, "generator rows differ in length");
            for (j, &b) in row[..k as usize].iter().enumerate() {
                assert_eq!(b, u32::from(i == j), "the generator is not systematic");
            }
            parity_rows.push(row[k as usize..].iter().fold(0, |w, &b| (w << 1) | b));
        }
        let r = n - k;
        let mut code = BlockCode {
            n,
            k,
            parity_rows,
            single: vec![-1; 1 << r],
            codewords: Vec::new(),
        };
        for pos in 0..n {
            let s = code.syndrome(1 << pos) as usize;
            if code.single[s] == -1 {
                code.single[s] = pos as i8;
            }
        }
        if k <= 8 {
            code.codewords = (0..1u32 << k).map(|d| code.encode(d)).collect();
        }
        code
    }

    pub fn parity(&self, data: u32) -> u32 {
        let mut p = 0;
        for (i, &row) in self.parity_rows.iter().enumerate() {
            if (data >> (self.k as usize - 1 - i)) & 1 == 1 {
                p ^= row;
            }
        }
        p
    }

    pub fn encode(&self, data: u32) -> u32 {
        (data << (self.n - self.k)) | self.parity(data)
    }

    pub fn syndrome(&self, word: u32) -> u32 {
        let r = self.n - self.k;
        self.parity(word >> r) ^ (word & ((1 << r) - 1))
    }

    pub fn data(&self, word: u32) -> u32 {
        word >> (self.n - self.k)
    }

    /// Correct up to one bit error: the codeword, or `None` if the syndrome
    /// is not that of a single error (a (16,11,4) double error).
    pub fn correct1(&self, word: u32) -> Option<u32> {
        match self.syndrome(word) {
            0 => Some(word),
            s => match self.single[s as usize] {
                -1 => None,
                p => Some(word ^ (1 << p)),
            },
        }
    }

    /// Maximum-likelihood decode over every codeword (k ≤ 8): the data and
    /// how many hard decisions it overrules.
    pub fn ml(&self, llr: &[f32]) -> (u32, u32) {
        debug_assert_eq!(llr.len() as u32, self.n);
        let mut best = (f32::MIN, 0u32);
        for (d, &w) in self.codewords.iter().enumerate() {
            let m = soft_metric(w, llr);
            if m > best.0 {
                best = (m, d as u32);
            }
        }
        let w = self.codewords[best.1 as usize];
        (best.1, (w ^ pack_hard(llr)).count_ones())
    }

    /// Chase-II with the `flips` least reliable bits: the best codeword the
    /// single-error corrector reaches, or `None` if none is.
    pub fn chase(&self, llr: &[f32], flips: usize) -> Option<u32> {
        chase(llr, flips, |w| self.correct1(w))
    }
}

/// Chase-II around any hard decoder of `llr.len()`-bit words.
pub fn chase(llr: &[f32], flips: usize, decode: impl Fn(u32) -> Option<u32>) -> Option<u32> {
    let n = llr.len();
    let h = pack_hard(llr);
    // The least reliable positions, as u32 bit masks. Rust note: a small
    // fixed array and an insertion sort, so nothing is allocated per word.
    let mut weak = [(f32::MAX, 0u32); 6];
    let flips = flips.min(weak.len());
    for (i, &l) in llr.iter().enumerate() {
        let a = l.abs();
        if a < weak[flips - 1].0 {
            let mut j = flips - 1;
            while j > 0 && weak[j - 1].0 > a {
                weak[j] = weak[j - 1];
                j -= 1;
            }
            weak[j] = (a, 1 << (n - 1 - i));
        }
    }
    let mut best: Option<(f32, u32)> = None;
    for pattern in 0..1u32 << flips {
        let mut t = h;
        for (b, w) in weak[..flips].iter().enumerate() {
            if (pattern >> b) & 1 == 1 {
                t ^= w.1;
            }
        }
        if let Some(w) = decode(t) {
            let m = soft_metric(w, llr);
            if best.is_none_or(|b| m > b.0) {
                best = Some((m, w));
            }
        }
    }
    best.map(|b| b.1)
}

// Generator matrices, ETSI TS 102 361-1 clause B.3 (tables B.11 to B.17).

const GOLAY_20_8: [&str; 8] = [
    "1 0 0 0 0 0 0 0 0 0 1 1 1 1 0 1 1 0 1 0",
    "0 1 0 0 0 0 0 0 1 1 0 1 1 0 0 1 1 0 0 1",
    "0 0 1 0 0 0 0 0 0 1 1 0 1 1 0 0 1 1 0 1",
    "0 0 0 1 0 0 0 0 0 0 1 1 0 1 1 0 0 1 1 1",
    "0 0 0 0 1 0 0 0 1 1 0 1 1 1 0 0 0 1 1 0",
    "0 0 0 0 0 1 0 0 1 0 1 0 1 0 0 1 0 1 1 1",
    "0 0 0 0 0 0 1 0 1 0 0 1 0 0 1 1 1 1 1 0",
    "0 0 0 0 0 0 0 1 1 0 0 0 1 1 1 0 1 0 1 1",
];
const QR_16_7: [&str; 7] = [
    "1 0 0 0 0 0 0 0 0 1 0 0 1 1 1 1",
    "0 1 0 0 0 0 0 1 0 0 0 1 1 1 1 0",
    "0 0 1 0 0 0 0 1 1 0 1 1 0 1 1 1",
    "0 0 0 1 0 0 0 1 1 1 1 0 0 0 1 0",
    "0 0 0 0 1 0 0 1 1 1 0 0 1 0 0 1",
    "0 0 0 0 0 1 0 0 1 1 1 0 0 1 0 1",
    "0 0 0 0 0 0 1 0 0 1 1 1 0 0 1 1",
];
const HAMMING_17_12: [&str; 12] = [
    "1 0 0 0 0 0 0 0 0 0 0 0 1 1 0 1 1",
    "0 1 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1",
    "0 0 1 0 0 0 0 0 0 0 0 0 1 1 1 0 1",
    "0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 0 0",
    "0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 0",
    "0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1",
    "0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 1",
    "0 0 0 0 0 0 0 1 0 0 0 0 1 1 0 1 0",
    "0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 0 1",
    "0 0 0 0 0 0 0 0 0 1 0 0 1 0 1 0 0",
    "0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 1 0",
    "0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 1",
];
const HAMMING_13_9: [&str; 9] = [
    "1 0 0 0 0 0 0 0 0 1 1 1 1",
    "0 1 0 0 0 0 0 0 0 1 1 1 0",
    "0 0 1 0 0 0 0 0 0 0 1 1 1",
    "0 0 0 1 0 0 0 0 0 1 0 1 0",
    "0 0 0 0 1 0 0 0 0 0 1 0 1",
    "0 0 0 0 0 1 0 0 0 1 0 1 1",
    "0 0 0 0 0 0 1 0 0 1 1 0 0",
    "0 0 0 0 0 0 0 1 0 0 1 1 0",
    "0 0 0 0 0 0 0 0 1 0 0 1 1",
];
const HAMMING_15_11: [&str; 11] = [
    "1 0 0 0 0 0 0 0 0 0 0 1 0 0 1",
    "0 1 0 0 0 0 0 0 0 0 0 1 1 0 1",
    "0 0 1 0 0 0 0 0 0 0 0 1 1 1 1",
    "0 0 0 1 0 0 0 0 0 0 0 1 1 1 0",
    "0 0 0 0 1 0 0 0 0 0 0 0 1 1 1",
    "0 0 0 0 0 1 0 0 0 0 0 1 0 1 0",
    "0 0 0 0 0 0 1 0 0 0 0 0 1 0 1",
    "0 0 0 0 0 0 0 1 0 0 0 1 0 1 1",
    "0 0 0 0 0 0 0 0 1 0 0 1 1 0 0",
    "0 0 0 0 0 0 0 0 0 1 0 0 1 1 0",
    "0 0 0 0 0 0 0 0 0 0 1 0 0 1 1",
];
const HAMMING_16_11: [&str; 11] = [
    "1 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1",
    "0 1 0 0 0 0 0 0 0 0 0 1 1 0 1 0",
    "0 0 1 0 0 0 0 0 0 0 0 1 1 1 1 1",
    "0 0 0 1 0 0 0 0 0 0 0 1 1 1 0 0",
    "0 0 0 0 1 0 0 0 0 0 0 0 1 1 1 0",
    "0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 1",
    "0 0 0 0 0 0 1 0 0 0 0 0 1 0 1 1",
    "0 0 0 0 0 0 0 1 0 0 0 1 0 1 1 0",
    "0 0 0 0 0 0 0 0 1 0 0 1 1 0 0 1",
    "0 0 0 0 0 0 0 0 0 1 0 0 1 1 0 1",
    "0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 1",
];
const HAMMING_7_4: [&str; 4] = [
    "1 0 0 0 1 0 1",
    "0 1 0 0 1 1 1",
    "0 0 1 0 1 1 0",
    "0 0 0 1 0 1 1",
];

/// Rust note: `OnceLock` builds each code the first time it is asked for and
/// then hands out the same `&'static` reference to every thread.
macro_rules! code {
    ($name:ident, $rows:expr) => {
        pub fn $name() -> &'static BlockCode {
            static C: OnceLock<BlockCode> = OnceLock::new();
            C.get_or_init(|| BlockCode::from_rows(&$rows))
        }
    };
}
code!(golay_20_8, GOLAY_20_8);
code!(qr_16_7, QR_16_7);
code!(hamming_17_12, HAMMING_17_12);
code!(hamming_13_9, HAMMING_13_9);
code!(hamming_15_11, HAMMING_15_11);
code!(hamming_16_11, HAMMING_16_11);
code!(hamming_7_4, HAMMING_7_4);

/// The Golay (23,12,7) code of the AMBE+2 voice frame, with the generator
/// mbelib uses (`ecc_const.h`, ISC licence, © 2010 mbelib Author): one
/// 11-bit parity word per data bit, the most significant data bit first.
/// It corrects any three errors through a 2048-entry syndrome table (the
/// code is perfect: every syndrome is a pattern of three errors or fewer).
pub struct Golay23 {
    table: Vec<u32>,
}

const GOLAY23_GEN: [u32; 12] = [
    0x63a, 0x31d, 0x7b4, 0x3da, 0x1ed, 0x6cc, 0x366, 0x1b3, 0x6e3, 0x54b, 0x49f, 0x475,
];

impl Golay23 {
    pub fn get() -> &'static Golay23 {
        static G: OnceLock<Golay23> = OnceLock::new();
        G.get_or_init(|| {
            let mut table = vec![u32::MAX; 2048];
            let mut put = |e: u32| {
                let s = Golay23::syndrome(e) as usize;
                if table[s] == u32::MAX {
                    table[s] = e;
                }
            };
            put(0);
            for a in 0..23 {
                put(1 << a);
                for b in a + 1..23 {
                    put((1 << a) | (1 << b));
                    for c in b + 1..23 {
                        put((1 << a) | (1 << b) | (1 << c));
                    }
                }
            }
            Golay23 { table }
        })
    }

    pub fn parity(data: u32) -> u32 {
        let mut p = 0;
        for (i, &g) in GOLAY23_GEN.iter().enumerate() {
            if (data >> (11 - i)) & 1 == 1 {
                p ^= g;
            }
        }
        p
    }

    pub fn encode(data: u32) -> u32 {
        (data << 11) | Golay23::parity(data)
    }

    fn syndrome(w: u32) -> u32 {
        Golay23::parity(w >> 11) ^ (w & 0x7ff)
    }

    /// The nearest codeword (≤ 3 errors away, always: the code is perfect).
    pub fn correct(&self, w: u32) -> u32 {
        w ^ self.table[Golay23::syndrome(w) as usize]
    }

    /// Chase-II over the 4 least reliable bits: soft decoding to about 7
    /// errors' worth of reach where the soft input says where they are.
    pub fn soft(&self, llr: &[f32; 23]) -> u32 {
        chase(llr, 4, |w| Some(self.correct(w))).expect("the hard decoder always answers")
    }
}

/// CRC-CCITT of 0/1 bits, ETSI TS 102 361-1 clause B.3.8: x^16+x^12+x^5+1,
/// initial remainder 0, the result inverted.
pub fn crc_ccitt(bits: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in bits {
        let top = (crc >> 15) as u8 ^ (b & 1);
        crc <<= 1;
        if top == 1 {
            crc ^= 0x1021;
        }
    }
    !crc
}

/// The 8-bit CRC of the Short LC, clause B.3.7: x^8+x^2+x+1, no inversion.
pub fn crc8(bits: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in bits {
        let top = (crc >> 7) ^ (b & 1);
        crc <<= 1;
        if top == 1 {
            crc ^= 0x07;
        }
    }
    crc
}

/// The 5-bit checksum of the embedded LC, clause B.3.11.
pub fn cs5(lc: &[u8; 9]) -> u8 {
    (lc.iter().map(|&b| u32::from(b)).sum::<u32>() % 31) as u8
}

/// GF(2^8) over x^8+x^4+x^3+x^2+1 (clause B.3.6, tables B.19/B.20).
struct Gf {
    exp: [u8; 512],
    log: [u8; 256],
}

fn gf() -> &'static Gf {
    static G: OnceLock<Gf> = OnceLock::new();
    G.get_or_init(|| {
        let mut g = Gf {
            exp: [0; 512],
            log: [0; 256],
        };
        let mut x: u16 = 1;
        for i in 0..255 {
            g.exp[i] = x as u8;
            g.exp[i + 255] = x as u8;
            g.log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x100 != 0 {
                x ^= 0x11d;
            }
        }
        g
    })
}

fn gf_mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        return 0;
    }
    let g = gf();
    g.exp[g.log[a as usize] as usize + g.log[b as usize] as usize]
}

fn gf_div(a: u8, b: u8) -> u8 {
    if a == 0 {
        return 0;
    }
    let g = gf();
    g.exp[g.log[a as usize] as usize + 255 - g.log[b as usize] as usize]
}

/// The parity columns of the Reed-Solomon (12,9) generator matrix, table B.18.
const RS_PARITY: [[u8; 3]; 9] = [
    [0x1C, 0xBC, 0xFD],
    [0x89, 0x31, 0x08],
    [0xAD, 0x41, 0x36],
    [0x7D, 0x71, 0x16],
    [0xF3, 0xA6, 0x3A],
    [0x08, 0x83, 0x7B],
    [0x3F, 0x6F, 0x02],
    [0x6C, 0x0D, 0xA7],
    [0x0E, 0x38, 0x40],
];

/// The three RS (12,9) parity bytes of a 9-byte LC (before the CRC mask).
pub fn rs129_parity(lc: &[u8; 9]) -> [u8; 3] {
    let mut p = [0u8; 3];
    for (m, row) in lc.iter().zip(RS_PARITY) {
        for j in 0..3 {
            p[j] ^= gf_mul(*m, row[j]);
        }
    }
    p
}

/// Check an LC against its received (mask removed) parity, correcting one
/// wrong byte: the number of bytes corrected, or `None` if it does not check.
pub fn rs129_check(lc: &mut [u8; 9], parity: &[u8; 3]) -> Option<u32> {
    let calc = rs129_parity(lc);
    let d = [
        calc[0] ^ parity[0],
        calc[1] ^ parity[1],
        calc[2] ^ parity[2],
    ];
    if d == [0; 3] {
        return Some(0);
    }
    // One wrong parity byte: only that column differs.
    if d.iter().filter(|&&x| x != 0).count() == 1 {
        return Some(1);
    }
    // One wrong data byte k with error e: d = e · (parity row k).
    for (k, row) in RS_PARITY.iter().enumerate() {
        let e = gf_div(d[0], row[0]);
        if e != 0 && (0..3).all(|j| gf_mul(e, row[j]) == d[j]) {
            lc[k] ^= e;
            return Some(1);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_have_their_distance() {
        for (code, d) in [(golay_20_8(), 8), (qr_16_7(), 6), (hamming_7_4(), 3)] {
            let min = code.codewords[1..]
                .iter()
                .map(|w| w.count_ones())
                .min()
                .unwrap();
            assert_eq!(min, d);
        }
        // Every single error of the Hamming codes has its own syndrome.
        for c in [
            hamming_13_9(),
            hamming_15_11(),
            hamming_16_11(),
            hamming_17_12(),
        ] {
            for d in [0u32, 1, 0x155, (1 << c.k) - 1] {
                let w = c.encode(d);
                for p in 0..c.n {
                    assert_eq!(c.correct1(w ^ (1 << p)), Some(w));
                }
            }
        }
    }

    #[test]
    fn golay23_is_perfect_and_corrects_three() {
        let g = Golay23::get();
        assert!(g.table.iter().all(|&e| e != u32::MAX));
        let w = Golay23::encode(0xA5C);
        assert_eq!(g.correct(w ^ 0b100_0000_0010_0000_0000_0001), w);
    }

    #[test]
    fn rs_matches_the_generator_polynomial_and_corrects_a_byte() {
        // g(x) = x^3 + 0E x^2 + 38 x + 40 (B.10): the last row of B.18 is
        // x^3 mod g, i.e. g's lower coefficients.
        assert_eq!(
            rs129_parity(&[0, 0, 0, 0, 0, 0, 0, 0, 1]),
            [0x0E, 0x38, 0x40]
        );
        let lc = [0x00, 0x00, 0x20, 0x00, 0x09, 0x2F, 0x23, 0x6D, 0x7B];
        let p = rs129_parity(&lc);
        let mut bad = lc;
        bad[4] ^= 0x5A;
        assert_eq!(rs129_check(&mut bad, &p), Some(1));
        assert_eq!(bad, lc);
    }

    #[test]
    fn soft_decoders_beat_hard_errors() {
        let c = golay_20_8();
        let w = c.encode(0x3C);
        // Three hard errors on weak bits, the rest strong.
        let llr: Vec<f32> = (0..20)
            .map(|i| {
                let b = (w >> (19 - i)) & 1;
                let s: f32 = if b == 0 { 4.0 } else { -4.0 };
                if [2, 9, 15].contains(&i) {
                    -0.3 * s.signum()
                } else {
                    s
                }
            })
            .collect();
        assert_eq!(c.ml(&llr).0, 0x3C);
        let h = hamming_15_11();
        let w = h.encode(0x5A5);
        let llr: Vec<f32> = (0..15)
            .map(|i| {
                let s = if (w >> (14 - i)) & 1 == 0 { 3.0 } else { -3.0 };
                if i == 3 || i == 12 { -0.2 * s } else { s }
            })
            .collect();
        assert_eq!(h.chase(&llr, 3), Some(w));
    }
}
