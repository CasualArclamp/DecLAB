// made by CasualArclamp
//! The little cryptography the meshes need, written out so the crate pulls
//! in nothing: AES-128 and AES-256 (FIPS-197; Meshtastic's channels are AES-CTR,
//! MeshCore's AES-ECB), SHA-256 and HMAC-SHA-256 (FIPS 180-4, RFC 2104;
//! MeshCore's channel hash and MAC) and base64 (RFC 4648; the keys are
//! written in it). Speed does not matter here: a packet is a few blocks.

/// The AES S-box, built at compile time from its definition (the
/// multiplicative inverse in GF(2⁸) then the affine map, FIPS-197 §5.1.1)
/// rather than typed out.
const SBOX: [u8; 256] = {
    let mut s = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        // Inverse by brute force: fine in a `const` block.
        let mut inv = 0u8;
        if i != 0 {
            let mut j = 1;
            while j < 256 {
                if gmul(i as u8, j as u8) == 1 {
                    inv = j as u8;
                    break;
                }
                j += 1;
            }
        }
        let b = inv;
        s[i] = b ^ b.rotate_left(1) ^ b.rotate_left(2) ^ b.rotate_left(3) ^ b.rotate_left(4) ^ 0x63;
        i += 1;
    }
    s
};

const INV_SBOX: [u8; 256] = {
    let mut s = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        s[SBOX[i] as usize] = i as u8;
        i += 1;
    }
    s
};

/// Multiplication in GF(2⁸) modulo x⁸+x⁴+x³+x+1.
const fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    while b != 0 {
        if b & 1 != 0 {
            p ^= a;
        }
        let hi = a & 0x80;
        a <<= 1;
        if hi != 0 {
            a ^= 0x1b;
        }
        b >>= 1;
    }
    p
}

/// AES-128 or AES-256, its round keys expanded once.
#[derive(Clone)]
pub struct Aes {
    rk: Vec<[u8; 16]>,
}

impl std::fmt::Debug for Aes {
    // Keys are not printed.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Aes({} rounds)", self.rk.len() - 1)
    }
}

impl Aes {
    /// A 16- or 32-byte key; None for any other length.
    pub fn new(key: &[u8]) -> Option<Self> {
        let nk = match key.len() {
            16 => 4,
            32 => 8,
            _ => return None,
        };
        let rounds = nk + 6;
        let words = 4 * (rounds + 1);
        // Key expansion (FIPS-197 §5.2).
        let mut w = vec![[0u8; 4]; words];
        for (i, wi) in w.iter_mut().take(nk).enumerate() {
            wi.copy_from_slice(&key[4 * i..4 * i + 4]);
        }
        let mut rcon = 1u8;
        let sub = |t: [u8; 4]| t.map(|x| SBOX[x as usize]);
        for i in nk..words {
            let mut t = w[i - 1];
            if i % nk == 0 {
                t = sub([t[1], t[2], t[3], t[0]]);
                t[0] ^= rcon;
                rcon = gmul(rcon, 2);
            } else if nk > 6 && i % nk == 4 {
                t = sub(t);
            }
            for k in 0..4 {
                w[i][k] = w[i - nk][k] ^ t[k];
            }
        }
        let rk = (0..=rounds)
            .map(|r| {
                let mut k = [0u8; 16];
                for c in 0..4 {
                    k[4 * c..4 * c + 4].copy_from_slice(&w[4 * r + c]);
                }
                k
            })
            .collect();
        Some(Aes { rk })
    }

    pub fn encrypt(&self, b: &mut [u8; 16]) {
        let last = self.rk.len() - 1;
        xor16(b, &self.rk[0]);
        for r in 1..=last {
            for x in b.iter_mut() {
                *x = SBOX[*x as usize];
            }
            shift_rows(b);
            if r != last {
                mix_columns(b);
            }
            xor16(b, &self.rk[r]);
        }
    }

    pub fn decrypt(&self, b: &mut [u8; 16]) {
        let last = self.rk.len() - 1;
        xor16(b, &self.rk[last]);
        for r in (0..last).rev() {
            inv_shift_rows(b);
            for x in b.iter_mut() {
                *x = INV_SBOX[*x as usize];
            }
            xor16(b, &self.rk[r]);
            if r != 0 {
                inv_mix_columns(b);
            }
        }
    }

    /// CTR mode: the counter block is `iv`, incremented as one 128-bit
    /// big-endian number (Meshtastic's, from its `CryptoEngine`).
    pub fn ctr(&self, iv: &[u8; 16], data: &mut [u8]) {
        let mut ctr = *iv;
        for chunk in data.chunks_mut(16) {
            let mut ks = ctr;
            self.encrypt(&mut ks);
            for (d, k) in chunk.iter_mut().zip(ks) {
                *d ^= k;
            }
            for i in (0..16).rev() {
                ctr[i] = ctr[i].wrapping_add(1);
                if ctr[i] != 0 {
                    break;
                }
            }
        }
    }
}

fn xor16(b: &mut [u8; 16], k: &[u8; 16]) {
    for (x, y) in b.iter_mut().zip(k) {
        *x ^= y;
    }
}

// The state is column-major: byte 4c+r is row r of column c.
fn shift_rows(b: &mut [u8; 16]) {
    let s = *b;
    for c in 0..4 {
        for r in 0..4 {
            b[4 * c + r] = s[4 * ((c + r) % 4) + r];
        }
    }
}

fn inv_shift_rows(b: &mut [u8; 16]) {
    let s = *b;
    for c in 0..4 {
        for r in 0..4 {
            b[4 * ((c + r) % 4) + r] = s[4 * c + r];
        }
    }
}

fn mix_columns(b: &mut [u8; 16]) {
    for c in 0..4 {
        let a = [b[4 * c], b[4 * c + 1], b[4 * c + 2], b[4 * c + 3]];
        for r in 0..4 {
            b[4 * c + r] =
                gmul(a[r], 2) ^ gmul(a[(r + 1) % 4], 3) ^ a[(r + 2) % 4] ^ a[(r + 3) % 4];
        }
    }
}

fn inv_mix_columns(b: &mut [u8; 16]) {
    for c in 0..4 {
        let a = [b[4 * c], b[4 * c + 1], b[4 * c + 2], b[4 * c + 3]];
        for r in 0..4 {
            b[4 * c + r] = gmul(a[r], 14)
                ^ gmul(a[(r + 1) % 4], 11)
                ^ gmul(a[(r + 2) % 4], 13)
                ^ gmul(a[(r + 3) % 4], 9);
        }
    }
}

/// SHA-256 round constants (FIPS 180-4 §4.2.2).
const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

pub fn sha256(parts: &[&[u8]]) -> [u8; 32] {
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let len: usize = parts.iter().map(|p| p.len()).sum();
    let mut msg: Vec<u8> = Vec::with_capacity(len + 72);
    for p in parts {
        msg.extend_from_slice(p);
    }
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&((len as u64) * 8).to_be_bytes());
    for block in msg.chunks(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes(block[4 * i..4 * i + 4].try_into().unwrap());
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let mut v = h;
        for i in 0..64 {
            let s1 = v[4].rotate_right(6) ^ v[4].rotate_right(11) ^ v[4].rotate_right(25);
            let ch = (v[4] & v[5]) ^ (!v[4] & v[6]);
            let t1 = v[7]
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = v[0].rotate_right(2) ^ v[0].rotate_right(13) ^ v[0].rotate_right(22);
            let maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
            let t2 = s0.wrapping_add(maj);
            v = [
                t1.wrapping_add(t2),
                v[0],
                v[1],
                v[2],
                v[3].wrapping_add(t1),
                v[4],
                v[5],
                v[6],
            ];
        }
        for (a, b) in h.iter_mut().zip(v) {
            *a = a.wrapping_add(b);
        }
    }
    let mut out = [0u8; 32];
    for (i, x) in h.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&x.to_be_bytes());
    }
    out
}

/// HMAC-SHA-256 (RFC 2104); `key` at most 64 bytes.
pub fn hmac_sha256(key: &[u8], data: &[u8]) -> [u8; 32] {
    let mut k = [0u8; 64];
    k[..key.len()].copy_from_slice(key);
    let mut ipad = [0u8; 64];
    let mut opad = [0u8; 64];
    for i in 0..64 {
        ipad[i] = k[i] ^ 0x36;
        opad[i] = k[i] ^ 0x5c;
    }
    let inner = sha256(&[&ipad, data]);
    sha256(&[&opad, &inner])
}

/// Base64 (RFC 4648, padding optional); None if it is not base64.
pub fn base64_decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let (mut acc, mut bits) = (0u32, 0);
    for c in s.trim().bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            b'=' => break,
            _ => return None,
        };
        acc = acc << 6 | v as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
    }
    Some(out)
}

pub fn base64_encode(b: &[u8]) -> String {
    const A: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::new();
    for c in b.chunks(3) {
        let n = (c[0] as u32) << 16
            | (*c.get(1).unwrap_or(&0) as u32) << 8
            | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            if i <= c.len() {
                s.push(A[(n >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{x:02x}")).collect()
    }

    #[test]
    fn aes_fips197() {
        // FIPS-197 Appendix C.1.
        let key: [u8; 16] = core::array::from_fn(|i| i as u8);
        let mut b: [u8; 16] = core::array::from_fn(|i| (i as u8) * 0x11);
        let aes = Aes::new(&key).unwrap();
        aes.encrypt(&mut b);
        assert_eq!(hex(&b), "69c4e0d86a7b0430d8cdb78070b4c55a");
        aes.decrypt(&mut b);
        assert_eq!(hex(&b), "00112233445566778899aabbccddeeff");
        // C.3: AES-256.
        let key: [u8; 32] = core::array::from_fn(|i| i as u8);
        let aes = Aes::new(&key).unwrap();
        aes.encrypt(&mut b);
        assert_eq!(hex(&b), "8ea2b7ca516745bfeafc49904b496089");
        aes.decrypt(&mut b);
        assert_eq!(hex(&b), "00112233445566778899aabbccddeeff");
    }

    #[test]
    fn sha256_and_hmac() {
        assert_eq!(
            hex(&sha256(&[b"abc"])),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        // RFC 4231 test case 2.
        assert_eq!(
            hex(&hmac_sha256(b"Jefe", b"what do ya want for nothing?")),
            "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"
        );
    }

    #[test]
    fn base64_round_trip() {
        let k = base64_decode("1PG7OiApB1nwvP+rz05pAQ==").unwrap();
        assert_eq!(hex(&k), "d4f1bb3a20290759f0bcffabcf4e6901");
        assert_eq!(base64_encode(&k), "1PG7OiApB1nwvP+rz05pAQ==");
    }
}
