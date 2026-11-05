// made by CasualArclamp
//! LoRa's coding chain, both ways: gray mapping, the diagonal interleaver,
//! the Hamming codes, the explicit header, whitening and the payload CRC.
//!
//! Ported from gr-lora_sdr (Tapparel et al., EPFL; GPL-3.0-or-later):
//! `gray_mapping`, `gray_demap`, `interleaver`, `deinterleaver`,
//! `hamming_enc`, `hamming_dec`, `header`, `header_decoder`, `whitening`,
//! `dewhitening`, `add_crc`, `crc_verif` and `tables.h`. LoRa itself is
//! Semtech's and unpublished; gr-lora_sdr's layout is what real SX127x and
//! SX126x chips send and receive.

/// The whitening sequence XORed onto payload bytes (not the header or
/// CRC): an 8-bit LFSR, x⁸+x⁶+x⁵+x⁴+1, from 0xFF (gr-lora_sdr `tables.h`,
/// built here rather than typed out).
pub const WHITENING: [u8; 255] = {
    let mut t = [0u8; 255];
    let mut s: u8 = 0xFF;
    let mut i = 0;
    while i < 255 {
        t[i] = s;
        let fb = ((s >> 7) ^ (s >> 5) ^ (s >> 4) ^ (s >> 3)) & 1;
        s = (s << 1) | fb;
        i += 1;
    }
    t
};

/// Hamming codewords for CR 4/6 … 4/8 (crop the low bits), data bits first,
/// as `[d0 d1 d2 d3 | p0 p1 p2 p3]` with d0 the nibble's LSB written first
/// (gr-lora_sdr `hamming_dec`'s `cw_LUT`, indexed by the bit-reversed
/// nibble).
const CW: [u8; 16] = [
    0, 23, 45, 58, 78, 89, 99, 116, 139, 156, 166, 177, 197, 210, 232, 255,
];
/// CR 4/5: four data bits and one overall parity bit.
const CW5: [u8; 16] = [
    0, 24, 40, 48, 72, 80, 96, 120, 136, 144, 160, 184, 192, 216, 232, 240,
];

fn rev4(x: u8) -> u8 {
    ((x & 1) << 3) | ((x & 2) << 1) | ((x & 4) >> 1) | ((x & 8) >> 3)
}

/// One nibble's codeword, `4 + cr` bits.
pub fn hamming_encode(nibble: u8, cr: u8) -> u8 {
    let r = rev4(nibble & 15) as usize;
    if cr == 1 {
        CW5[r] >> 3
    } else {
        CW[r] >> (4 - cr)
    }
}

/// The nibble nearest a codeword (fewest bits differing): corrects one
/// error at 4/7 and 4/8 and picks a likely nibble otherwise; the CRC says
/// whether it was right. Returns the nibble and how many bits differed.
pub fn hamming_decode(cw: u8, cr: u8) -> (u8, u32) {
    let mut best = (0u8, u32::MAX);
    for r in 0..16u8 {
        let c = if cr == 1 {
            CW5[r as usize] >> 3
        } else {
            CW[r as usize] >> (4 - cr)
        };
        let d = (c ^ cw).count_ones();
        if d < best.1 {
            best = (rev4(r), d);
        }
    }
    best
}

/// The explicit header (gr-lora_sdr `header_decoder`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Payload bytes (CRC not counted).
    pub len: u8,
    /// Coding rate 4/(4+cr), cr 1..=4.
    pub cr: u8,
    pub crc: bool,
}

/// The header's 5-bit checksum over its first three nibbles.
fn header_checksum(n: &[u8]) -> u8 {
    let b = |i: usize, bit: u8| (n[i] >> bit) & 1;
    let c4 = b(0, 3) ^ b(0, 2) ^ b(0, 1) ^ b(0, 0);
    let c3 = b(0, 3) ^ b(1, 3) ^ b(1, 2) ^ b(1, 1) ^ b(2, 0);
    let c2 = b(0, 2) ^ b(1, 3) ^ b(1, 0) ^ b(2, 3) ^ b(2, 1);
    let c1 = b(0, 1) ^ b(1, 2) ^ b(1, 0) ^ b(2, 2) ^ b(2, 1) ^ b(2, 0);
    let c0 = b(0, 0) ^ b(1, 1) ^ b(2, 3) ^ b(2, 2) ^ b(2, 1) ^ b(2, 0);
    (c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0
}

impl Header {
    pub fn nibbles(&self) -> [u8; 5] {
        let mut n = [
            self.len >> 4,
            self.len & 15,
            (self.cr << 1) | self.crc as u8,
            0,
            0,
        ];
        let c = header_checksum(&n);
        n[3] = c >> 4;
        n[4] = c & 15;
        n
    }

    /// From the first five nibbles of the first block; None if the
    /// checksum fails or it makes no sense.
    pub fn parse(n: &[u8]) -> Option<Header> {
        let chk = ((n[3] & 1) << 4) | n[4];
        if chk != header_checksum(n) {
            return None;
        }
        let h = Header {
            len: (n[0] << 4) | n[1],
            cr: n[2] >> 1,
            crc: n[2] & 1 == 1,
        };
        (h.len > 0 && (1..=4).contains(&h.cr)).then_some(h)
    }
}

/// Low data-rate optimisation: on when a symbol lasts more than 16 ms
/// (what Semtech's drivers and RadioLib choose by themselves).
pub fn ldro(sf: u8, bw_hz: f64) -> bool {
    (1u32 << sf) as f64 / bw_hz > 0.016
}

/// Symbols after the preamble, header block included (gr-lora_sdr
/// `frame_sync`: 8 + ⌈(2L − SF + 7 + 4·CRC) / (SF − 2·LDRO)⌉ · (4 + CR)).
pub fn symbol_count(sf: u8, h: &Header, ldro: bool) -> usize {
    let num = 2 * h.len as i32 - sf as i32 + 7 + 4 * h.crc as i32;
    let den = sf as i32 - 2 * ldro as i32;
    8 + ((num.max(0) + den - 1) / den) as usize * (4 + h.cr as usize)
}

/// CRC-16/CCITT (poly 0x1021, init 0), MSB first (gr-lora_sdr `crc16`).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// LoRa's payload CRC: CRC-16 over all but the last two bytes, XORed with
/// those two (gr-lora_sdr `crc_verif`).
pub fn payload_crc(p: &[u8]) -> u16 {
    match p.len() {
        0 => 0,
        1 => p[0] as u16,
        n => crc16(&p[..n - 2]) ^ p[n - 1] as u16 ^ ((p[n - 2] as u16) << 8),
    }
}

/// Gray code and its inverse. LoRa sends the inverse-gray of the data so
/// that a symbol a bin off costs one bit.
pub fn gray(x: u16) -> u16 {
    x ^ (x >> 1)
}

pub fn gray_inverse(mut x: u16) -> u16 {
    let mut s = x;
    while x > 1 {
        x >>= 1;
        s ^= x;
    }
    s
}

/// Interleave one block of `sf_app` codewords (each `cw_len` bits) into
/// `cw_len` symbols of `sf_app` bits (gr-lora_sdr `interleaver`).
fn interleave(cws: &[u8], sf_app: usize, cw_len: usize, out: &mut Vec<u16>) {
    for i in 0..cw_len {
        let mut sym = 0u16;
        for j in 0..sf_app {
            // Symbol bit j (MSB first) is bit i (MSB first) of codeword
            // (i − j − 1) mod sf_app.
            let k = (i as isize - j as isize - 1).rem_euclid(sf_app as isize) as usize;
            let bit = (cws[k] >> (cw_len - 1 - i)) & 1;
            sym |= (bit as u16) << (sf_app - 1 - j);
        }
        out.push(sym);
    }
}

/// The reverse: `cw_len` symbols of `sf_app` bits back to `sf_app`
/// codewords (gr-lora_sdr `deinterleaver`).
fn deinterleave(syms: &[u16], sf_app: usize, cw_len: usize, cws: &mut [u8]) {
    cws[..sf_app].fill(0);
    for (i, &s) in syms.iter().enumerate().take(cw_len) {
        for j in 0..sf_app {
            let bit = (s >> (sf_app - 1 - j)) & 1;
            let k = (i as isize - j as isize - 1).rem_euclid(sf_app as isize) as usize;
            cws[k] |= (bit as u8) << (cw_len - 1 - i);
        }
    }
}

/// A payload as chirp symbols (bin values 0..2^SF), header block first:
/// what a transmitter sends after its preamble, sync word and downchirps.
pub fn encode(payload: &[u8], sf: u8, cr: u8, crc: bool, ldro: bool) -> Vec<u16> {
    assert!(
        (7..=12).contains(&sf)
            && (1..=4).contains(&cr)
            && !payload.is_empty()
            && payload.len() < 256
    );
    let h = Header {
        len: payload.len() as u8,
        cr,
        crc,
    };
    // Nibbles: header, then the whitened payload low nibble first, then
    // the CRC (not whitened) low nibble first.
    let mut nib: Vec<u8> = h.nibbles().to_vec();
    for (i, &b) in payload.iter().enumerate() {
        let w = b ^ WHITENING[i % 255];
        nib.push(w & 15);
        nib.push(w >> 4);
    }
    if crc {
        let c = payload_crc(payload);
        for k in 0..4 {
            nib.push(((c >> (4 * k)) & 15) as u8);
        }
    }
    let n = 1u16 << sf;
    let mut out = Vec::new();
    let mut syms = Vec::with_capacity(8);
    let mut pos = 0;
    let mut first = true;
    while pos < nib.len() || first {
        let reduced = first || ldro;
        let sf_app = sf as usize - if reduced { 2 } else { 0 };
        let cr_app = if first { 4 } else { cr };
        let cw_len = 4 + cr_app as usize;
        let mut cws = [0u8; 12];
        for c in cws.iter_mut().take(sf_app) {
            let x = nib.get(pos).copied().unwrap_or(0);
            pos += 1;
            *c = hamming_encode(x, cr_app);
        }
        syms.clear();
        interleave(&cws, sf_app, cw_len, &mut syms);
        for &s in &syms {
            // Reduced rate: a parity bit and a zero below the data bits
            // (gr-lora_sdr `interleaver`); then inverse gray, plus one.
            let v = if reduced {
                (s << 2) | (((s.count_ones() & 1) as u16) << 1)
            } else {
                s
            };
            out.push((gray_inverse(v) + 1) % n);
        }
        first = false;
    }
    out
}

/// The header block's checksum failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeaderFailed;

/// What [`Decoder`] made of a whole frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub header: Header,
    pub payload: Vec<u8>,
    pub crc_ok: bool,
    /// Bits the Hamming decoder found wrong (a quality hint).
    pub corrected: u32,
}

/// Turns demodulated symbols (already reduced to `sf_app` bits and gray
/// mapped: see [`symbol_value`]) into a header and payload, block by block.
#[derive(Debug, Clone)]
pub struct Decoder {
    sf: u8,
    ldro: bool,
    pub header: Option<Header>,
    nib: Vec<u8>,
    block: Vec<u16>,
    corrected: u32,
}

/// The data bits a symbol carries: `bin` is the FFT peak after the CFO is
/// taken out (gr-lora_sdr `fft_demod`: (bin − 1) mod 2^SF, ÷ 4 at reduced
/// rate, then gray).
pub fn symbol_value(bin: u16, sf: u8, reduced: bool) -> u16 {
    let n = 1u16 << sf;
    let v = (bin + n - 1) % n;
    gray(if reduced { v >> 2 } else { v })
}

impl Decoder {
    pub fn new(sf: u8, ldro: bool) -> Self {
        Decoder {
            sf,
            ldro,
            header: None,
            nib: Vec::with_capacity(600),
            block: Vec::with_capacity(8),
            corrected: 0,
        }
    }

    /// Whether the next symbol is reduced rate (the header block, or LDRO).
    pub fn reduced(&self) -> bool {
        self.header.is_none() || self.ldro
    }

    /// Symbols this frame has after the preamble, once the header is known.
    pub fn total_symbols(&self) -> Option<usize> {
        self.header.map(|h| symbol_count(self.sf, &h, self.ldro))
    }

    /// One symbol's value (from [`symbol_value`]). Fails when the header
    /// block fails its checksum (the frame is lost).
    pub fn push(&mut self, v: u16) -> Result<(), HeaderFailed> {
        self.block.push(v);
        let (sf_app, cw_len, cr) = match self.header {
            None => (self.sf as usize - 2, 8, 4),
            Some(h) => (
                self.sf as usize - if self.ldro { 2 } else { 0 },
                4 + h.cr as usize,
                h.cr,
            ),
        };
        if self.block.len() < cw_len {
            return Ok(());
        }
        let mut cws = [0u8; 12];
        deinterleave(&self.block, sf_app, cw_len, &mut cws);
        self.block.clear();
        for &c in &cws[..sf_app] {
            let (n, e) = hamming_decode(c, cr);
            self.corrected += e;
            self.nib.push(n);
        }
        if self.header.is_none() {
            self.header = Some(Header::parse(&self.nib).ok_or(HeaderFailed)?);
            self.nib.drain(..5);
        }
        Ok(())
    }

    /// The frame, once every symbol is in.
    pub fn finish(&self) -> Option<Frame> {
        let h = self.header?;
        let len = h.len as usize;
        if self.nib.len() < 2 * len + if h.crc { 4 } else { 0 } {
            return None;
        }
        let payload: Vec<u8> = (0..len)
            .map(|i| (self.nib[2 * i] | (self.nib[2 * i + 1] << 4)) ^ WHITENING[i % 255])
            .collect();
        let crc_ok = if h.crc {
            let c = &self.nib[2 * len..2 * len + 4];
            let got = c[0] as u16 | (c[1] as u16) << 4 | (c[2] as u16) << 8 | (c[3] as u16) << 12;
            got == payload_crc(&payload)
        } else {
            true
        };
        Some(Frame {
            header: h,
            payload,
            crc_ok,
            corrected: self.corrected,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn whitening_matches_gr_lora_sdr() {
        // tables.h's first row.
        assert_eq!(
            WHITENING[..16],
            [
                0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E, 0xBC, 0x78,
                0xF1, 0xE3
            ]
        );
    }

    #[test]
    fn hamming_corrects() {
        for cr in 1..=4u8 {
            for x in 0..16u8 {
                let c = hamming_encode(x, cr);
                assert_eq!(hamming_decode(c, cr).0, x);
                if cr >= 3 {
                    for b in 0..4 + cr {
                        assert_eq!(
                            hamming_decode(c ^ (1 << b), cr).0,
                            x,
                            "cr {cr} x {x} bit {b}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn header_round_trip() {
        let h = Header {
            len: 37,
            cr: 1,
            crc: true,
        };
        assert_eq!(Header::parse(&h.nibbles()), Some(h));
        let mut bad = h.nibbles();
        bad[1] ^= 4;
        assert_eq!(Header::parse(&bad), None);
    }

    #[test]
    fn frames_round_trip() {
        let payload: Vec<u8> = (0..60u8).map(|i| i.wrapping_mul(37) ^ 0x5a).collect();
        for sf in 7..=12u8 {
            for cr in 1..=4u8 {
                for ldro in [false, true] {
                    let syms = encode(&payload, sf, cr, true, ldro);
                    let mut d = Decoder::new(sf, ldro);
                    for &s in &syms {
                        let r = d.reduced();
                        d.push(symbol_value(s, sf, r)).unwrap();
                    }
                    assert_eq!(
                        d.total_symbols(),
                        Some(syms.len()),
                        "sf {sf} cr {cr} ldro {ldro}"
                    );
                    let f = d.finish().unwrap();
                    assert!(
                        f.crc_ok && f.payload == payload,
                        "sf {sf} cr {cr} ldro {ldro}"
                    );
                }
            }
        }
    }
}
