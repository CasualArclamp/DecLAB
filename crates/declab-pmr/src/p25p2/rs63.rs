// made by CasualArclamp
//! The Reed-Solomon (63,35) code over GF(2⁶) phase 2 shortens and punctures
//! for the FACCH, SACCH and ESS: primitive polynomial x⁶ + x + 1, roots
//! α¹ … α²⁸ (OP25's `ezpwd::RS<63,35>`, whose defaults these are). A word is
//! 63 symbols, index 0 the highest term: 35 data, then 28 parity.
//!
//! Decoded for errors and erasures (the punctured parity are erasures, the
//! shortened data known zeros), Berlekamp–Massey seeded with the erasure
//! locator, then Chien and Forney: as Phil Karn's `decode_rs` (LGPL; the
//! algorithm, not the code).

use std::sync::OnceLock;

pub const N: usize = 63;
pub const K: usize = 35;
pub const ROOTS: usize = N - K;

struct Gf {
    exp: [u8; 126],
    log: [u8; 64],
    /// The generator polynomial, highest term first (degree 28, monic).
    gen_poly: [u8; ROOTS + 1],
}

fn gf() -> &'static Gf {
    static G: OnceLock<Gf> = OnceLock::new();
    G.get_or_init(|| {
        let mut g = Gf { exp: [0; 126], log: [0; 64], gen_poly: [0; ROOTS + 1] };
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
        // ∏ (x − αⁱ), i = 1..28, kept lowest term first while building.
        let mut p = vec![1u8];
        for i in 1..=ROOTS {
            let a = g.exp[i];
            let mut q = vec![0u8; p.len() + 1];
            for (j, &c) in p.iter().enumerate() {
                q[j + 1] ^= c;
                q[j] ^= mul_with(&g, c, a);
            }
            p = q;
        }
        for (j, &c) in p.iter().rev().enumerate() {
            g.gen_poly[j] = c;
        }
        g
    })
}

fn mul_with(g: &Gf, a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        0
    } else {
        g.exp[g.log[a as usize] as usize + g.log[b as usize] as usize]
    }
}

#[inline]
fn mul(a: u8, b: u8) -> u8 {
    mul_with(gf(), a, b)
}

fn inv(a: u8) -> u8 {
    let g = gf();
    g.exp[(63 - g.log[a as usize] as usize) % 63]
}

fn alpha(e: usize) -> u8 {
    gf().exp[e % 63]
}

/// The 28 parity symbols of 35 data symbols.
pub fn encode(data: &[u8; K]) -> [u8; ROOTS] {
    let g = gf();
    let mut rem = [0u8; ROOTS];
    for &d in data {
        let fb = d ^ rem[0];
        rem.copy_within(1.., 0);
        rem[ROOTS - 1] = 0;
        if fb != 0 {
            for (r, &c) in rem.iter_mut().zip(&g.gen_poly[1..]) {
                *r ^= mul(fb, c);
            }
        }
    }
    rem
}

/// Correct `word` (63 symbols) in place, `erasures` its symbols known to be
/// unreliable (their values ignored), `zeros` the first symbols that must
/// be zero (shortened). The errors corrected (erasures not counted), or
/// `None` when it cannot be.
pub fn decode(word: &mut [u8; N], erasures: &[usize], zeros: usize) -> Option<usize> {
    let deg = |i: usize| N - 1 - i;
    for &e in erasures {
        word[e] = 0;
    }
    let mut syn = [0u8; ROOTS];
    for (j, s) in syn.iter_mut().enumerate() {
        // Horner over the word, highest term first, at α^(j+1).
        let a = alpha(j + 1);
        *s = word.iter().fold(0u8, |acc, &c| mul(acc, a) ^ c);
    }
    let rho = erasures.len();
    if rho > ROOTS {
        return None;
    }
    if syn.iter().all(|&s| s == 0) {
        return Some(0);
    }
    // Λ(x), lowest term first, starts as the erasure locator ∏(1 − X x).
    let mut lambda = [0u8; ROOTS + 1];
    lambda[0] = 1;
    for (n, &e) in erasures.iter().enumerate() {
        let x = alpha(deg(e));
        for i in (1..=n + 1).rev() {
            lambda[i] ^= mul(x, lambda[i - 1]);
        }
    }
    let mut b = lambda;
    let mut el = rho;
    for r in rho + 1..=ROOTS {
        let mut discr = 0u8;
        for i in 0..r {
            discr ^= mul(lambda[i], syn[r - i - 1]);
        }
        if discr == 0 {
            b.copy_within(0..ROOTS, 1);
            b[0] = 0;
            continue;
        }
        let mut t = lambda;
        for i in 0..ROOTS {
            t[i + 1] ^= mul(discr, b[i]);
        }
        if 2 * el < r + rho {
            el = r + rho - el;
            let di = inv(discr);
            for i in 0..=ROOTS {
                b[i] = mul(lambda[i], di);
            }
        } else {
            b.copy_within(0..ROOTS, 1);
            b[0] = 0;
        }
        lambda = t;
    }
    let deg_lambda = (0..=ROOTS).rev().find(|&i| lambda[i] != 0).unwrap_or(0);
    if deg_lambda == 0 || 2 * (deg_lambda - rho.min(deg_lambda)) + rho > ROOTS {
        return None;
    }
    // Chien search: position i is in error when Λ(X⁻¹) = 0, X = α^deg(i).
    let mut locs = Vec::with_capacity(deg_lambda);
    for i in 0..N {
        let xinv = alpha(63 - deg(i) % 63);
        let v = lambda[..=deg_lambda].iter().rev().fold(0u8, |acc, &c| mul(acc, xinv) ^ c);
        if v == 0 {
            locs.push(i);
        }
    }
    if locs.len() != deg_lambda {
        return None;
    }
    // Ω(x) = S(x)Λ(x) mod x²⁸; with the first root α¹ the value is
    // Ω(X⁻¹)/Λ'(X⁻¹).
    let mut omega = [0u8; ROOTS];
    for i in 0..ROOTS {
        for j in 0..=i.min(deg_lambda) {
            omega[i] ^= mul(lambda[j], syn[i - j]);
        }
    }
    let mut errors = 0;
    for &i in &locs {
        let xinv = alpha(63 - deg(i) % 63);
        let num = omega.iter().rev().fold(0u8, |acc, &c| mul(acc, xinv) ^ c);
        // Λ'(x): the odd terms, each down one power.
        let mut den = 0u8;
        let mut p = 1u8;
        let x2 = mul(xinv, xinv);
        for k in (1..=deg_lambda).step_by(2) {
            den ^= mul(lambda[k], p);
            p = mul(p, x2);
        }
        if den == 0 {
            return None;
        }
        let v = mul(num, inv(den));
        if i < zeros && v != 0 {
            return None;
        }
        word[i] ^= v;
        if !erasures.contains(&i) {
            errors += 1;
        }
    }
    if word[..zeros].iter().any(|&s| s != 0) {
        return None;
    }
    Some(errors)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn word(seed: u32) -> [u8; N] {
        let mut d = [0u8; K];
        let mut s = seed;
        for x in d.iter_mut() {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *x = ((s >> 16) & 63) as u8;
        }
        let p = encode(&d);
        let mut w = [0u8; N];
        w[..K].copy_from_slice(&d);
        w[K..].copy_from_slice(&p);
        w
    }

    #[test]
    fn codewords_have_zero_syndromes() {
        let w = word(1);
        for j in 1..=ROOTS {
            let a = alpha(j);
            assert_eq!(w.iter().fold(0u8, |acc, &c| mul(acc, a) ^ c), 0);
        }
    }

    #[test]
    fn corrects_errors_and_erasures() {
        for seed in 0..50 {
            let good = word(seed);
            // Nine punctured, as a FACCH's, and five errors.
            let erasures: Vec<usize> = (54..63).collect();
            let mut w = good;
            for e in &erasures {
                w[*e] = 0x2a;
            }
            for k in 0..5 {
                w[(seed as usize * 7 + k * 11) % 54] ^= 1 + k as u8;
            }
            assert_eq!(decode(&mut w, &erasures, 0), Some(5), "seed {seed}");
            assert_eq!(w, good);
        }
    }

    #[test]
    fn gives_up_beyond_its_reach() {
        let good = word(3);
        let mut w = good;
        for k in 0..20 {
            w[k * 3] ^= 0x15;
        }
        let r = decode(&mut w, &[], 0);
        assert!(r.is_none() || w != good);
    }
}
