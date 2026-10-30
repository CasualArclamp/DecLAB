// made by CasualArclamp
//! P25 phase 1's codes (TIA-102.BAAA), as MMDVMHost implements them
//! (GPL-2.0-or-later, © Jonathan Naylor G4KLX; its Reed-Solomon encode
//! matrices are the spec's tables): BCH (63,16) for the NID, Golay (24,12)
//! and its shortened (18,6), Hamming (10,6), IMBE's Hamming (15,11), and
//! Reed-Solomon (24,12), (24,16), (36,20) over GF(2⁶).
//!
//! Decoding is ours and soft where it pays: the NID and the short codes by
//! maximum likelihood over their codewords, Golay (23,12) by Chase. The
//! Reed-Solomon codes by Berlekamp–Massey (errors only).

use std::sync::OnceLock;

use crate::fec::{BlockCode, Golay23, chase, hard, pack_hard, soft_metric};

// --- BCH (63,16) for the NID ------------------------------------------------

/// The generator polynomial's coefficients, g0 first (MMDVMHost `BCH.cpp`;
/// the spec's 6331 1413 6723 5453 octal).
const BCH_G: [u8; 48] = [
    1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
    0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1,
];

/// The 64 NID bits for a NAC and DUID: NAC (12), DUID (4), 47 BCH parity
/// bits (the remainder of x⁴⁷·i(x) by g(x), the data bits taken last first,
/// as MMDVMHost's encoder takes them), then a parity bit set for the LDUs.
pub fn nid_encode(nac: u16, duid: u8) -> [u8; 64] {
    let mut d = [0u8; 16];
    for (i, b) in d.iter_mut().enumerate() {
        let v = (u32::from(nac & 0xfff) << 4) | u32::from(duid & 15);
        *b = ((v >> (15 - i)) & 1) as u8;
    }
    // The LFSR of Lin & Costello's systematic encoder.
    let mut bb = [0u8; 47];
    for i in (0..16).rev() {
        let fb = d[i] ^ bb[46];
        for j in (1..47).rev() {
            bb[j] = bb[j - 1] ^ (fb & BCH_G[j]);
        }
        bb[0] = fb & BCH_G[0];
    }
    let mut out = [0u8; 64];
    out[..16].copy_from_slice(&d);
    out[16..63].copy_from_slice(&bb);
    out[63] = u8::from(matches!(duid, 5 | 10));
    out
}

/// The DUIDs a NID may carry (HDU, TDU, LDU1, TSDU, LDU2, PDU, TDULC).
pub const DUIDS: [u8; 7] = [0, 3, 5, 7, 10, 12, 15];

/// All NIDs of one NAC as 63-bit words (the parity bit aside), by DUID.
fn nids_of(nac: u16) -> [u64; 7] {
    DUIDS.map(|duid| {
        nid_encode(nac, duid)[..63]
            .iter()
            .fold(0u64, |w, &b| (w << 1) | u64::from(b))
    })
}

/// Decode a NID from its 64 soft bits: (NAC, DUID, hard decisions it
/// overrules). With the NAC known only its seven NIDs are tried; else every
/// NAC's (the first time, on acquisition).
pub fn nid_decode(llr: &[f32], nac: Option<u16>) -> (u16, u8, u32) {
    let h = llr[..63]
        .iter()
        .fold(0u64, |w, &l| (w << 1) | u64::from(hard(l)));
    // Correlation Σ ±llr of a 63-bit word, by its set bits.
    let total: f32 = llr[..63].iter().sum();
    let metric = |w: u64| {
        let mut m = total;
        let mut x = w;
        while x != 0 {
            let b = x.trailing_zeros() as usize;
            m -= 2.0 * llr[62 - b];
            x &= x - 1;
        }
        m
    };
    let mut best = (f32::MIN, 0u16, 0u8, 0u64);
    let try_nac = |n: u16, best: &mut (f32, u16, u8, u64)| {
        for (w, &duid) in nids_of(n).iter().zip(&DUIDS) {
            let m = metric(*w);
            if m > best.0 {
                *best = (m, n, duid, *w);
            }
        }
    };
    match nac {
        Some(n) => try_nac(n, &mut best),
        None => {
            // Rust note: a table of every NAC's NIDs, built once and shared.
            static ALL: OnceLock<Vec<[u64; 7]>> = OnceLock::new();
            let all = ALL.get_or_init(|| (0..4096u16).map(nids_of).collect());
            for (n, ws) in all.iter().enumerate() {
                for (w, &duid) in ws.iter().zip(&DUIDS) {
                    let m = metric(*w);
                    if m > best.0 {
                        best = (m, n as u16, duid, *w);
                    }
                }
            }
        }
    }
    (best.1, best.2, (best.3 ^ h).count_ones())
}

// --- Golay (24,12) and (18,6) -------------------------------------------------

/// Golay (24,12): 12 data bits, 11 parity (mbelib's generator), an overall
/// even parity bit (MMDVMHost `ENCODING_TABLE_24128`).
pub fn golay24_encode(data: u32) -> u32 {
    let w23 = Golay23::encode(data & 0xfff);
    (w23 << 1) | (w23.count_ones() & 1)
}

/// Decode 24 soft bits: the data and the bits corrected.
pub fn golay24_decode(llr: &[f32]) -> (u32, u32) {
    let w: [f32; 23] = llr[..23].try_into().expect("23");
    let c = Golay23::get().soft(&w);
    (c >> 11, (c ^ pack_hard(&w)).count_ones())
}

/// The shortened Golay (18,6) of the HDU: the low 18 bits of the 24-bit
/// codeword of a 6-bit value. Decoded by maximum likelihood over its 64.
pub fn golay18_decode(llr: &[f32]) -> (u8, u32) {
    static WORDS: OnceLock<Vec<u32>> = OnceLock::new();
    let words = WORDS.get_or_init(|| (0..64).map(|d| golay24_encode(d) & 0x3ffff).collect());
    let mut best = (f32::MIN, 0usize);
    for (d, &w) in words.iter().enumerate() {
        let m = soft_metric(w, &llr[..18]);
        if m > best.0 {
            best = (m, d);
        }
    }
    (
        best.1 as u8,
        (words[best.1] ^ pack_hard(&llr[..18])).count_ones(),
    )
}

// --- Hamming (10,6) and IMBE's (15,11) --------------------------------------

/// Hamming (10,6,3) of the LDU's link control (MMDVMHost `encode1063`).
pub fn hamming_10_6() -> &'static BlockCode {
    static C: OnceLock<BlockCode> = OnceLock::new();
    C.get_or_init(|| {
        // Rows: data bit d puts 1s in the parity bits whose equation has it.
        // p0 = d0^d1^d2^d5, p1 = d0^d1^d3^d5, p2 = d0^d2^d3^d4,
        // p3 = d1^d2^d3^d4.
        let eqs: [&[usize]; 4] = [&[0, 1, 2, 5], &[0, 1, 3, 5], &[0, 2, 3, 4], &[1, 2, 3, 4]];
        let rows: Vec<String> = (0..6)
            .map(|d| {
                let mut r: Vec<&str> = (0..6).map(|j| if j == d { "1" } else { "0" }).collect();
                r.extend(eqs.iter().map(|e| if e.contains(&d) { "1" } else { "0" }));
                r.join(" ")
            })
            .collect();
        let refs: Vec<&str> = rows.iter().map(String::as_str).collect();
        BlockCode::from_rows(&refs)
    })
}

/// IMBE's Hamming (15,11): a word is valid when each of mbelib's four
/// `hammingGenerator` masks covers an even number of its 1s (bit 14 the
/// first data bit, bits 3..0 the parity).
const IMBE_HAMMING_MASKS: [u32; 4] = [0x7f08, 0x78e4, 0x66d2, 0x55b1];

fn imbe_hamming_syndrome(w: u32) -> u32 {
    IMBE_HAMMING_MASKS
        .iter()
        .fold(0, |s, &m| (s << 1) | ((w & m).count_ones() & 1))
}

pub fn imbe_hamming_encode(data11: u32) -> u32 {
    let mut w = (data11 & 0x7ff) << 4;
    for (i, &m) in IMBE_HAMMING_MASKS.iter().enumerate() {
        if (w & m).count_ones() & 1 == 1 {
            w |= 1 << (3 - i);
        }
    }
    w
}

/// Soft decode (Chase over the three weakest bits): the 15-bit word.
pub fn imbe_hamming_decode(llr: &[f32]) -> u32 {
    static SINGLE: OnceLock<[i8; 16]> = OnceLock::new();
    let single = SINGLE.get_or_init(|| {
        let mut t = [-1i8; 16];
        for b in 0..15 {
            t[imbe_hamming_syndrome(1 << b) as usize] = b as i8;
        }
        t
    });
    let fix = |w: u32| match imbe_hamming_syndrome(w) {
        0 => Some(w),
        s => match single[s as usize] {
            -1 => None,
            b => Some(w ^ (1 << b)),
        },
    };
    chase(&llr[..15], 3, fix).unwrap_or_else(|| pack_hard(&llr[..15]))
}

// --- Reed-Solomon over GF(64) ---------------------------------------------------

/// GF(2⁶) over x⁶ + x + 1 (MMDVMHost `rsGFexp`/`rsGFlog`).
struct Gf64 {
    exp: [u8; 126],
    log: [u8; 64],
}

fn gf() -> &'static Gf64 {
    static G: OnceLock<Gf64> = OnceLock::new();
    G.get_or_init(|| {
        let mut g = Gf64 {
            exp: [0; 126],
            log: [0; 64],
        };
        let mut x: u8 = 1;
        for i in 0..63 {
            g.exp[i] = x;
            g.exp[i + 63] = x;
            g.log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x40 != 0 {
                x ^= 0x43;
            }
        }
        g
    })
}

fn mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        return 0;
    }
    let g = gf();
    g.exp[g.log[a as usize] as usize + g.log[b as usize] as usize]
}

fn inv(a: u8) -> u8 {
    let g = gf();
    g.exp[(63 - g.log[a as usize] as usize) % 63]
}

fn pow_alpha(e: usize) -> u8 {
    gf().exp[e % 63]
}

/// A shortened Reed-Solomon code given by the parity columns of its
/// generator matrix (the spec's tables, octal there), `k` data symbols.
pub struct Rs {
    k: usize,
    parity: &'static [[u8; 16]],
    n: usize,
}

/// RS (24,12,13): the LDU1 and TDULC link control.
pub const RS_24_12: Rs = Rs {
    k: 12,
    parity: &RS241213,
    n: 24,
};
/// RS (24,16,9): the LDU2's encryption sync.
pub const RS_24_16: Rs = Rs {
    k: 16,
    parity: &RS24169,
    n: 24,
};
/// RS (36,20,17): the HDU.
pub const RS_36_20: Rs = Rs {
    k: 20,
    parity: &RS362017,
    n: 36,
};

// The parity columns of MMDVMHost's ENCODE_MATRIX_241213, _24169 and
// _362017 (octal, as there), padded to 16 columns.
#[rustfmt::skip]
const RS241213: [[u8; 16]; 12] = [
    [0o62, 0o44, 0o03, 0o25, 0o14, 0o16, 0o27, 0o03, 0o53, 0o04, 0o36, 0o47, 0, 0, 0, 0],
    [0o11, 0o12, 0o11, 0o11, 0o16, 0o64, 0o67, 0o55, 0o01, 0o76, 0o26, 0o73, 0, 0, 0, 0],
    [0o03, 0o01, 0o05, 0o75, 0o14, 0o06, 0o20, 0o44, 0o66, 0o06, 0o70, 0o66, 0, 0, 0, 0],
    [0o21, 0o70, 0o27, 0o45, 0o16, 0o67, 0o23, 0o64, 0o73, 0o33, 0o44, 0o21, 0, 0, 0, 0],
    [0o30, 0o22, 0o03, 0o75, 0o15, 0o15, 0o33, 0o15, 0o51, 0o03, 0o53, 0o50, 0, 0, 0, 0],
    [0o01, 0o41, 0o27, 0o56, 0o76, 0o64, 0o21, 0o53, 0o04, 0o25, 0o01, 0o12, 0, 0, 0, 0],
    [0o61, 0o76, 0o21, 0o55, 0o76, 0o01, 0o63, 0o35, 0o30, 0o13, 0o64, 0o70, 0, 0, 0, 0],
    [0o24, 0o22, 0o71, 0o56, 0o21, 0o35, 0o73, 0o42, 0o57, 0o74, 0o43, 0o76, 0, 0, 0, 0],
    [0o72, 0o42, 0o05, 0o20, 0o43, 0o47, 0o33, 0o56, 0o01, 0o16, 0o13, 0o76, 0, 0, 0, 0],
    [0o72, 0o14, 0o65, 0o54, 0o35, 0o25, 0o41, 0o16, 0o15, 0o40, 0o71, 0o26, 0, 0, 0, 0],
    [0o73, 0o65, 0o36, 0o61, 0o42, 0o22, 0o17, 0o04, 0o44, 0o20, 0o25, 0o05, 0, 0, 0, 0],
    [0o71, 0o05, 0o55, 0o03, 0o71, 0o34, 0o60, 0o11, 0o74, 0o02, 0o41, 0o50, 0, 0, 0, 0],
];
#[rustfmt::skip]
const RS24169: [[u8; 16]; 16] = [
    [0o51, 0o45, 0o67, 0o15, 0o64, 0o67, 0o52, 0o12, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o57, 0o25, 0o63, 0o73, 0o71, 0o22, 0o40, 0o15, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o05, 0o01, 0o31, 0o04, 0o16, 0o54, 0o25, 0o76, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o73, 0o07, 0o47, 0o14, 0o41, 0o77, 0o47, 0o11, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o75, 0o15, 0o51, 0o51, 0o17, 0o67, 0o17, 0o57, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o20, 0o32, 0o14, 0o42, 0o75, 0o42, 0o70, 0o54, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o02, 0o75, 0o43, 0o05, 0o01, 0o40, 0o12, 0o64, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o24, 0o74, 0o15, 0o72, 0o24, 0o26, 0o74, 0o61, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o42, 0o64, 0o07, 0o22, 0o61, 0o20, 0o40, 0o65, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o32, 0o32, 0o55, 0o41, 0o57, 0o66, 0o21, 0o77, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o65, 0o36, 0o25, 0o07, 0o50, 0o16, 0o40, 0o51, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o64, 0o06, 0o54, 0o32, 0o76, 0o46, 0o14, 0o36, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o62, 0o63, 0o74, 0o70, 0o05, 0o27, 0o37, 0o46, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o55, 0o43, 0o34, 0o71, 0o57, 0o76, 0o50, 0o64, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o24, 0o23, 0o23, 0o05, 0o50, 0o70, 0o42, 0o23, 0, 0, 0, 0, 0, 0, 0, 0],
    [0o67, 0o75, 0o45, 0o60, 0o57, 0o24, 0o06, 0o26, 0, 0, 0, 0, 0, 0, 0, 0],
];
#[rustfmt::skip]
const RS362017: [[u8; 16]; 20] = [
    [0o74, 0o37, 0o34, 0o06, 0o02, 0o07, 0o44, 0o64, 0o26, 0o14, 0o26, 0o44, 0o54, 0o13, 0o77, 0o05],
    [0o04, 0o17, 0o50, 0o24, 0o11, 0o05, 0o30, 0o57, 0o33, 0o03, 0o02, 0o02, 0o15, 0o16, 0o25, 0o26],
    [0o07, 0o23, 0o37, 0o46, 0o56, 0o75, 0o43, 0o45, 0o55, 0o21, 0o50, 0o31, 0o45, 0o27, 0o71, 0o62],
    [0o26, 0o05, 0o07, 0o63, 0o63, 0o27, 0o63, 0o40, 0o06, 0o04, 0o40, 0o45, 0o47, 0o30, 0o75, 0o07],
    [0o23, 0o73, 0o73, 0o41, 0o72, 0o34, 0o21, 0o51, 0o67, 0o16, 0o31, 0o74, 0o11, 0o21, 0o12, 0o21],
    [0o24, 0o51, 0o25, 0o23, 0o22, 0o41, 0o74, 0o66, 0o74, 0o65, 0o70, 0o36, 0o67, 0o45, 0o64, 0o01],
    [0o52, 0o33, 0o14, 0o02, 0o20, 0o06, 0o14, 0o25, 0o52, 0o23, 0o35, 0o74, 0o75, 0o75, 0o43, 0o27],
    [0o55, 0o62, 0o56, 0o25, 0o73, 0o60, 0o15, 0o30, 0o13, 0o17, 0o20, 0o02, 0o70, 0o55, 0o14, 0o47],
    [0o54, 0o51, 0o32, 0o65, 0o77, 0o12, 0o54, 0o13, 0o35, 0o32, 0o56, 0o12, 0o75, 0o01, 0o72, 0o63],
    [0o74, 0o41, 0o30, 0o41, 0o43, 0o22, 0o51, 0o06, 0o64, 0o33, 0o03, 0o47, 0o27, 0o12, 0o55, 0o47],
    [0o54, 0o70, 0o11, 0o03, 0o13, 0o22, 0o16, 0o57, 0o03, 0o45, 0o72, 0o31, 0o30, 0o56, 0o35, 0o22],
    [0o51, 0o07, 0o72, 0o30, 0o65, 0o54, 0o06, 0o21, 0o36, 0o63, 0o50, 0o61, 0o64, 0o52, 0o01, 0o60],
    [0o01, 0o65, 0o32, 0o70, 0o13, 0o44, 0o73, 0o24, 0o12, 0o52, 0o21, 0o55, 0o12, 0o35, 0o14, 0o72],
    [0o11, 0o70, 0o05, 0o10, 0o65, 0o24, 0o15, 0o77, 0o22, 0o24, 0o24, 0o74, 0o07, 0o44, 0o07, 0o46],
    [0o06, 0o02, 0o65, 0o11, 0o41, 0o20, 0o45, 0o42, 0o46, 0o54, 0o35, 0o12, 0o40, 0o64, 0o65, 0o33],
    [0o34, 0o31, 0o01, 0o15, 0o44, 0o64, 0o16, 0o24, 0o52, 0o16, 0o06, 0o62, 0o20, 0o13, 0o55, 0o57],
    [0o63, 0o43, 0o25, 0o44, 0o77, 0o63, 0o17, 0o17, 0o64, 0o14, 0o40, 0o74, 0o31, 0o72, 0o54, 0o06],
    [0o71, 0o21, 0o70, 0o44, 0o56, 0o04, 0o30, 0o74, 0o04, 0o23, 0o71, 0o70, 0o63, 0o45, 0o56, 0o43],
    [0o02, 0o01, 0o53, 0o74, 0o02, 0o14, 0o52, 0o74, 0o12, 0o57, 0o24, 0o63, 0o15, 0o42, 0o52, 0o33],
    [0o34, 0o35, 0o02, 0o23, 0o21, 0o27, 0o22, 0o33, 0o64, 0o42, 0o05, 0o73, 0o51, 0o46, 0o73, 0o60],
];

impl Rs {
    /// The parity symbols of `data` (k symbols of 6 bits).
    pub fn encode(&self, data: &[u8]) -> Vec<u8> {
        let r = self.n - self.k;
        let mut p = vec![0u8; r];
        for (m, row) in data.iter().zip(self.parity) {
            for j in 0..r {
                p[j] ^= mul(*m, row[j]);
            }
        }
        p
    }

    /// Correct `word` (n symbols: data then parity) in place, errors only:
    /// the symbols corrected, or `None` if it cannot be. The first symbol
    /// is the polynomial's highest term, the roots α¹ … α^(n−k) (as the
    /// spec's matrices give: the test checks they agree).
    pub fn decode(&self, word: &mut [u8]) -> Option<usize> {
        let n = self.n;
        let r = n - self.k;
        let deg = |i: usize| n - 1 - i;
        let syn: Vec<u8> = (1..=r)
            .map(|j| {
                word.iter()
                    .enumerate()
                    .fold(0u8, |s, (i, &c)| s ^ mul(c, pow_alpha(j * deg(i))))
            })
            .collect();
        if syn.iter().all(|&s| s == 0) {
            return Some(0);
        }
        // Berlekamp–Massey for the error locator Λ(x).
        let mut lambda = vec![0u8; r + 1];
        lambda[0] = 1;
        let mut b = lambda.clone();
        let (mut l, mut m, mut bb) = (0usize, 1usize, 1u8);
        for k in 0..r {
            let mut d = syn[k];
            for i in 1..=l {
                d ^= mul(lambda[i], syn[k - i]);
            }
            if d == 0 {
                m += 1;
                continue;
            }
            let coef = mul(d, inv(bb));
            let t = lambda.clone();
            for i in m..=r {
                lambda[i] ^= mul(coef, b[i - m]);
            }
            if 2 * l <= k {
                l = k + 1 - l;
                b = t;
                bb = d;
                m = 1;
            } else {
                m += 1;
            }
        }
        // Chien search over the n positions.
        let mut locs = Vec::new();
        for i in 0..n {
            let xinv = pow_alpha(63 - deg(i) % 63);
            let v = lambda.iter().rev().fold(0u8, |acc, &c| mul(acc, xinv) ^ c);
            if v == 0 {
                locs.push(i);
            }
        }
        if locs.len() != l || l == 0 {
            return None;
        }
        // Forney: Ω(x) = S(x)Λ(x) mod x^r; with the first root α¹ the
        // error value is Ω(X⁻¹)/Λ'(X⁻¹).
        let mut omega = vec![0u8; r];
        for i in 0..r {
            for j in 0..=i.min(r) {
                if j < lambda.len() {
                    omega[i] ^= mul(lambda[j], syn[i - j]);
                }
            }
        }
        for &i in &locs {
            let xinv = inv(pow_alpha(deg(i)));
            let om = omega.iter().rev().fold(0u8, |acc, &c| mul(acc, xinv) ^ c);
            // Λ'(x): the odd terms (characteristic 2).
            let mut dl = 0u8;
            let mut p = 1u8;
            for (k, &c) in lambda.iter().enumerate().skip(1) {
                if k % 2 == 1 {
                    dl ^= mul(c, p);
                }
                p = mul(p, xinv);
            }
            if dl == 0 {
                return None;
            }
            word[i] ^= mul(om, inv(dl));
        }
        let ok = (1..=r).all(|j| {
            word.iter()
                .enumerate()
                .fold(0u8, |s, (i, &c)| s ^ mul(c, pow_alpha(j * deg(i))))
                == 0
        });
        ok.then_some(locs.len())
    }
}

/// Bits (MSB first) to 6-bit symbols and back.
pub fn to_hex(bits: &[u8]) -> Vec<u8> {
    bits.chunks(6)
        .map(|c| c.iter().fold(0u8, |v, &b| (v << 1) | b))
        .collect()
}

pub fn from_hex(hex: &[u8], out: &mut Vec<u8>) {
    for &h in hex {
        out.extend((0..6).rev().map(|i| (h >> i) & 1));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rs_matrices_agree_with_the_decoder_and_it_corrects() {
        for code in [&RS_24_12, &RS_24_16, &RS_36_20] {
            let data: Vec<u8> = (0..code.k).map(|i| ((i * 37 + 5) % 64) as u8).collect();
            let mut w = data.clone();
            w.extend(code.encode(&data));
            assert_eq!(
                code.decode(&mut w.clone()),
                Some(0),
                "the spec's codewords are this code's"
            );
            let t = (code.n - code.k) / 2;
            let mut bad = w.clone();
            for e in 0..t {
                bad[(e * 7 + 1) % code.n] ^= (e as u8 % 63) + 1;
            }
            assert_eq!(code.decode(&mut bad), Some(t));
            assert_eq!(bad, w);
        }
    }

    #[test]
    fn nid_round_trip_and_soft_decode() {
        let nid = nid_encode(0x293, 5);
        assert_eq!(nid[63], 1);
        let mut llr: Vec<f32> = nid
            .iter()
            .map(|&b| if b == 0 { 2.0 } else { -2.0 })
            .collect();
        for i in [3, 10, 20, 30, 40, 50, 60] {
            llr[i] *= -0.3;
        }
        assert_eq!(nid_decode(&llr, None), (0x293, 5, 7));
        assert_eq!(nid_decode(&llr, Some(0x293)).1, 5);
    }

    #[test]
    fn short_codes() {
        let w = golay24_encode(0xA5C);
        assert_eq!(w.count_ones() % 2, 0);
        let mut llr: Vec<f32> = (0..24)
            .map(|i| if (w >> (23 - i)) & 1 == 0 { 2.0 } else { -2.0 })
            .collect();
        llr[5] = -llr[5];
        llr[17] = -llr[17];
        assert_eq!(golay24_decode(&llr).0, 0xA5C);
        let w18 = golay24_encode(0x2D) & 0x3ffff;
        let mut l18: Vec<f32> = (0..18)
            .map(|i| {
                if (w18 >> (17 - i)) & 1 == 0 {
                    2.0
                } else {
                    -2.0
                }
            })
            .collect();
        l18[2] = -l18[2];
        l18[9] = -l18[9];
        l18[15] = -l18[15];
        assert_eq!(golay18_decode(&l18).0, 0x2D);
        let h = imbe_hamming_encode(0x5A3);
        assert_eq!(imbe_hamming_syndrome(h), 0);
        for b in 0..15 {
            let llr: Vec<f32> = (0..15)
                .map(|i| {
                    let v = if ((h ^ (1 << b)) >> (14 - i)) & 1 == 0 {
                        2.0
                    } else {
                        -2.0
                    };
                    if i == 14 - b { v * 0.2 } else { v }
                })
                .collect();
            assert_eq!(imbe_hamming_decode(&llr), h, "error at bit {b}");
        }
        let c = hamming_10_6();
        assert_eq!(c.encode(0b101101) >> 4, 0b101101);
    }
}
