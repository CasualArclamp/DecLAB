// made by CasualArclamp
//! The bit buffer the slicers fill and the device decoders read, and the
//! bit helpers (CRCs, LFSR digests, reflections, UART framing) they use.
//!
//! Ported from rtl_433's `bitbuffer.c` and `bit_util.c` (Copyright (C) 2015
//! Tommy Vestermark and the rtl_433 authors, GPL-2.0-or-later). Names and
//! semantics are kept so the device decoders port line for line; the one
//! change is that a row never spills into the next (rows are capped at
//! [`COLS`] bytes), and [`BitBuffer::find_repeated_row`] returns an
//! `Option` instead of -1.

/// Bytes a row holds, as rtl_433's `BITBUF_COLS`.
pub const COLS: usize = 128;
/// Rows, as rtl_433's `BITBUF_ROWS`.
pub const ROWS: usize = 50;
/// Each row is stored with two spare bytes so a decoder reading a byte or
/// a 16-bit word past its last bit reads zeros instead of panicking.
const STRIDE: usize = COLS + 2;
const MAX_ROW_BITS: u16 = (COLS * 8) as u16;

/// rtl_433's `bitbuffer_t`: up to [`ROWS`] rows of bits, MSB first.
///
/// Rust note: `bb` is a fixed array of fixed arrays (no allocation per
/// row); a `BitBuffer` is ~6.6 kB, so keep one and `clear()` it rather than
/// making new ones in a loop.
#[derive(Clone)]
pub struct BitBuffer {
    pub num_rows: usize,
    pub bits_per_row: [u16; ROWS],
    pub syncs_before_row: [u16; ROWS],
    pub bb: [[u8; STRIDE]; ROWS],
}

impl Default for BitBuffer {
    fn default() -> Self {
        BitBuffer {
            num_rows: 0,
            bits_per_row: [0; ROWS],
            syncs_before_row: [0; ROWS],
            bb: [[0; STRIDE]; ROWS],
        }
    }
}

impl std::fmt::Debug for BitBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for r in 0..self.num_rows {
            writeln!(f, "[{r:02}] {}", self.row_string(r))?;
        }
        Ok(())
    }
}

#[inline]
fn bit_at(bytes: &[u8], bit: usize) -> u8 {
    (bytes[bit >> 3] >> (7 - (bit & 7))) & 1
}

/// rtl_433's `bitrow_get_bit`.
#[inline]
pub fn get_bit(row: &[u8], bit: usize) -> u8 {
    bit_at(row, bit)
}

/// rtl_433's `bitrow_get_byte`: the 8 bits from `bit` on.
#[inline]
pub fn get_byte(row: &[u8], bit: usize) -> u8 {
    let i = bit >> 3;
    let s = bit & 7;
    let hi = row[i] << s;
    let lo = if s == 0 {
        0
    } else {
        row.get(i + 1).copied().unwrap_or(0) >> (8 - s)
    };
    hi | lo
}

impl BitBuffer {
    pub fn new() -> Self {
        Self::default()
    }

    /// From rtl_433's text form: `"{36}5a2f81f40"`, rows split by `/` or
    /// a new `{n}`. Used by tests and the simulator.
    pub fn parse(code: &str) -> Self {
        let mut b = BitBuffer::new();
        let mut width: Option<usize> = None;
        let mut chars = code.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                ' ' | '\t' | '\r' | '\n' => {}
                '0' if matches!(chars.peek(), Some('x') | Some('X')) => {
                    chars.next();
                }
                '{' => {
                    if let Some(w) = width {
                        b.set_width(w);
                    }
                    if b.num_rows > 0 {
                        b.add_row();
                    }
                    let mut n = String::new();
                    for d in chars.by_ref() {
                        if d == '}' {
                            break;
                        }
                        n.push(d);
                    }
                    width = n.trim().parse().ok();
                }
                '/' => {
                    if let Some(w) = width.take() {
                        b.set_width(w);
                    }
                    b.add_row();
                }
                _ => {
                    if let Some(d) = c.to_digit(16) {
                        for k in (0..4).rev() {
                            b.add_bit(((d >> k) & 1) as u8);
                        }
                    }
                }
            }
        }
        if let Some(w) = width {
            b.set_width(w);
        }
        b
    }

    pub fn clear(&mut self) {
        // All of it: a decoder may have rewritten rows or shortened them
        // (or the row count) and left bits past their ends, which the next
        // slicer would OR into. 6.6 kB of memset is nothing beside a decode.
        for row in self.bb.iter_mut() {
            row.fill(0);
        }
        self.bits_per_row = [0; ROWS];
        self.syncs_before_row = [0; ROWS];
        self.num_rows = 0;
    }

    pub fn add_bit(&mut self, bit: u8) {
        if self.num_rows == 0 {
            self.num_rows = 1;
        }
        let r = self.num_rows - 1;
        let n = self.bits_per_row[r];
        if n >= MAX_ROW_BITS {
            return;
        }
        let n = n as usize;
        self.bb[r][n / 8] |= (bit & 1) << (7 - n % 8);
        self.bits_per_row[r] += 1;
    }

    pub fn add_row(&mut self) {
        if self.num_rows == 0 {
            self.num_rows = 1;
        }
        if self.num_rows < ROWS {
            self.num_rows += 1;
        } else {
            // Out of rows: reuse the last, as rtl_433 does.
            let r = ROWS - 1;
            let n = (self.bits_per_row[r] as usize).div_ceil(8);
            self.bb[r][..n].fill(0);
            self.bits_per_row[r] = 0;
        }
    }

    pub fn add_sync(&mut self) {
        if self.num_rows == 0 {
            self.num_rows = 1;
        }
        if self.bits_per_row[self.num_rows - 1] > 0 {
            self.add_row();
        }
        self.syncs_before_row[self.num_rows - 1] += 1;
    }

    /// Set the last row's length, padding with zeros or truncating.
    fn set_width(&mut self, width: usize) {
        if self.num_rows == 0 {
            self.num_rows = 1;
        }
        let r = self.num_rows - 1;
        let width = width.min(COLS * 8);
        let cur = self.bits_per_row[r] as usize;
        if cur > width {
            let from = width.div_ceil(8);
            self.bb[r][from..cur.div_ceil(8)].fill(0);
            if !width.is_multiple_of(8) {
                self.bb[r][width / 8] &= (0xff00u16 >> (width % 8)) as u8;
            }
        }
        self.bits_per_row[r] = width as u16;
    }

    /// The row's bytes (with the spare zero bytes after them).
    #[inline]
    pub fn row(&self, row: usize) -> &[u8] {
        &self.bb[row]
    }

    pub fn bits(&self, row: usize) -> usize {
        self.bits_per_row[row] as usize
    }

    /// `len` bits from bit `pos` of `row` into `out`, MSB first.
    pub fn extract_bytes(&self, row: usize, pos: usize, out: &mut [u8], len: usize) {
        if len == 0 {
            return;
        }
        let bits = &self.bb[row];
        let bytes = len.div_ceil(8);
        if pos & 7 == 0 {
            let p = pos / 8;
            for (i, o) in out.iter_mut().take(bytes).enumerate() {
                *o = bits.get(p + i).copied().unwrap_or(0);
            }
        } else {
            for (i, o) in out.iter_mut().take(bytes).enumerate() {
                let b = pos + i * 8;
                *o = if b / 8 < STRIDE { get_byte(bits, b) } else { 0 };
            }
        }
        if len & 7 != 0 {
            out[(len - 1) / 8] &= (0xff00u16 >> (len & 7)) as u8;
        }
    }

    pub fn invert(&mut self) {
        for r in 0..self.num_rows {
            let n = self.bits_per_row[r] as usize;
            if n > 0 {
                let last_col = (n - 1) / 8;
                let last_bits = (n - 1) % 8 + 1;
                for c in 0..=last_col {
                    self.bb[r][c] = !self.bb[r][c];
                }
                if last_bits < 8 {
                    self.bb[r][last_col] ^= 0xffu8 >> last_bits;
                }
            }
        }
    }

    pub fn nrzs_decode(&mut self) {
        self.nrz_decode(true);
    }

    pub fn nrzm_decode(&mut self) {
        self.nrz_decode(false);
    }

    fn nrz_decode(&mut self, space: bool) {
        for r in 0..self.num_rows {
            let n = self.bits_per_row[r] as usize;
            if n > 0 {
                let last_col = (n - 1) / 8;
                let last_bits = (n - 1) % 8 + 1;
                let mut prev = 0u8;
                for c in 0..=last_col {
                    let b = self.bb[r][c];
                    let mask = (prev << 7) | (b >> 1);
                    prev = b;
                    self.bb[r][c] = if space { b ^ !mask } else { b ^ mask };
                }
                self.bb[r][last_col] &= 0xffu8 << (8 - last_bits);
            }
        }
    }

    /// The first position at or after `start` where `pattern` (its first
    /// `pattern_bits` bits) occurs in `row`; the row's length if none.
    pub fn search(&self, row: usize, start: usize, pattern: &[u8], pattern_bits: usize) -> usize {
        let bits = &self.bb[row];
        let len = self.bits_per_row[row] as usize;
        let mut ipos = start;
        let mut ppos = 0;
        while ipos < len && ppos < pattern_bits {
            if bit_at(bits, ipos) == bit_at(pattern, ppos) {
                ppos += 1;
                ipos += 1;
                if ppos == pattern_bits {
                    return ipos - pattern_bits;
                }
            } else {
                ipos -= ppos;
                ipos += 1;
                ppos = 0;
            }
        }
        len
    }

    /// Manchester (IEEE 802.3: 01 = 1, 10 = 0) from bit `start` of `row`
    /// into `out` until a pair is not a transition or `max` bits (0: no
    /// limit); returns the input position reached.
    pub fn manchester_decode(
        &self,
        row: usize,
        start: usize,
        out: &mut BitBuffer,
        max: usize,
    ) -> usize {
        let bits = &self.bb[row];
        let mut len = self.bits_per_row[row] as usize;
        let mut ipos = start;
        if max != 0 && len > start + max * 2 {
            len = start + max * 2;
        }
        while ipos < len {
            let b1 = bit_at(bits, ipos);
            let b2 = bit_at(bits, ipos + 1);
            ipos += 2;
            if b1 == b2 {
                break;
            }
            out.add_bit(b2);
        }
        ipos
    }

    pub fn differential_manchester_decode(
        &self,
        row: usize,
        start: usize,
        out: &mut BitBuffer,
        max: usize,
    ) -> usize {
        let bits = &self.bb[row];
        let mut len = self.bits_per_row[row] as usize;
        let mut ipos = start;
        let mut bit2 = 0u8;
        if max != 0 && len > start + max * 2 {
            len = start + max * 2;
        }
        while ipos < len {
            let bit1 = bit_at(bits, ipos);
            bit2 = bit_at(bits, ipos + 1);
            ipos += 2;
            let bit3 = bit_at(bits, ipos);
            if bit1 != bit2 {
                if bit2 != bit3 {
                    out.add_bit(0);
                } else {
                    bit2 = bit1;
                    ipos -= 1;
                    break;
                }
            } else {
                bit2 = 1 - bit1;
                ipos -= 2;
                break;
            }
        }
        while ipos < len {
            let bit1 = bit_at(bits, ipos);
            ipos += 1;
            if bit1 == bit2 {
                break;
            }
            bit2 = bit_at(bits, ipos);
            ipos += 1;
            out.add_bit(if bit1 == bit2 { 1 } else { 0 });
        }
        ipos
    }

    pub fn compare_rows(&self, a: usize, b: usize, max_bits: usize) -> bool {
        let la = self.bits_per_row[a] as usize;
        let lb = self.bits_per_row[b] as usize;
        if max_bits == 0 || la < max_bits || lb < max_bits {
            la == lb && self.bb[a][..la.div_ceil(8)] == self.bb[b][..lb.div_ceil(8)]
        } else {
            let last = (max_bits - 1) / 8;
            let mask = (0xff00u16 >> (max_bits & 7)) as u8;
            self.bb[a][..max_bits / 8] == self.bb[b][..max_bits / 8]
                && (self.bb[a][last] & mask) == (self.bb[b][last] & mask)
        }
    }

    pub fn count_repeats(&self, row: usize, max_bits: usize) -> usize {
        (0..self.num_rows)
            .filter(|&i| self.compare_rows(row, i, max_bits))
            .count()
    }

    /// The first row of at least `min_bits` bits that occurs at least
    /// `min_repeats` times.
    pub fn find_repeated_row(&self, min_repeats: usize, min_bits: usize) -> Option<usize> {
        (0..self.num_rows)
            .find(|&i| self.bits(i) >= min_bits && self.count_repeats(i, 0) >= min_repeats)
    }

    /// As [`find_repeated_row`](Self::find_repeated_row), comparing only the
    /// first `min_bits` bits.
    pub fn find_repeated_prefix(&self, min_repeats: usize, min_bits: usize) -> Option<usize> {
        (0..self.num_rows)
            .find(|&i| self.bits(i) >= min_bits && self.count_repeats(i, min_bits) >= min_repeats)
    }

    /// `{36}5a2f81f40`: the row as rtl_433 prints it.
    pub fn row_string(&self, row: usize) -> String {
        let n = self.bits(row);
        let mut s = format!("{{{n}}}");
        for b in &self.bb[row][..n.div_ceil(8)] {
            s.push_str(&format!("{b:02x}"));
        }
        s
    }
}

pub fn reverse8(x: u8) -> u8 {
    x.reverse_bits()
}

pub fn reverse32(x: u32) -> u32 {
    // rtl_433 reverses each byte in place and the byte order: the same as
    // reversing all 32 bits.
    x.reverse_bits()
}

pub fn reflect_bytes(msg: &mut [u8]) {
    for b in msg {
        *b = b.reverse_bits();
    }
}

pub fn reflect4(x: u8) -> u8 {
    let x = ((x & 0xcc) >> 2) | ((x & 0x33) << 2);
    ((x & 0xaa) >> 1) | ((x & 0x55) << 1)
}

pub fn reflect_nibbles(msg: &mut [u8]) {
    for b in msg {
        *b = reflect4(*b);
    }
}

#[inline]
fn byte_at(msg: &[u8], i: usize) -> u8 {
    msg.get(i).copied().unwrap_or(0)
}

/// 4 data bits then a stuff bit of 1, repeated; returns the nibbles found.
pub fn extract_nibbles_4b1s(
    msg: &[u8],
    mut offset: usize,
    mut num_bits: usize,
    dst: &mut [u8],
) -> usize {
    let mut ret = 0;
    while num_bits >= 5 && ret < dst.len() {
        let mut bits =
            ((byte_at(msg, offset / 8) as u16) << 8) | byte_at(msg, offset / 8 + 1) as u16;
        bits >>= 11 - (offset % 8);
        if bits & 1 != 1 {
            break;
        }
        dst[ret] = ((bits >> 1) & 0xf) as u8;
        ret += 1;
        offset += 5;
        num_bits -= 5;
    }
    ret
}

fn bit_of(msg: &[u8], pos: usize) -> u8 {
    (byte_at(msg, pos / 8) >> (7 - (pos % 8))) & 1
}

fn byte_from(msg: &[u8], pos: usize) -> u8 {
    if pos.is_multiple_of(8) {
        byte_at(msg, pos / 8)
    } else {
        let w = ((byte_at(msg, pos / 8) as u16) << 8) | byte_at(msg, pos / 8 + 1) as u16;
        (w >> (8 - pos % 8)) as u8
    }
}

/// UART 8N1 framed bytes, LSB first on the air: returns the bytes found.
pub fn extract_bytes_uart_8n1(
    msg: &[u8],
    mut offset: usize,
    mut num_bits: usize,
    dst: &mut [u8],
) -> usize {
    let mut ret = 0;
    while num_bits >= 10 && ret < dst.len() {
        let start = bit_of(msg, offset);
        let data = byte_from(msg, offset + 1);
        let stop = bit_of(msg, offset + 9);
        offset += 10;
        if start != 0 || stop != 1 {
            break;
        }
        dst[ret] = data.reverse_bits();
        ret += 1;
        num_bits -= 10;
    }
    ret
}

/// UART 8N2, skipping to the first start bit.
pub fn extract_bytes_uart_8n2(
    msg: &[u8],
    mut offset: usize,
    mut num_bits: usize,
    dst: &mut [u8],
) -> usize {
    let mut ret = 0;
    while num_bits > 11 && bit_of(msg, offset) != 0 {
        offset += 1;
        num_bits -= 1;
    }
    while num_bits >= 11 && ret < dst.len() {
        let start = bit_of(msg, offset);
        let data = byte_from(msg, offset + 1);
        let s1 = bit_of(msg, offset + 9);
        let s2 = bit_of(msg, offset + 10);
        offset += 11;
        if start != 0 || s1 != 1 || s2 != 1 {
            break;
        }
        dst[ret] = data.reverse_bits();
        ret += 1;
        num_bits -= 11;
    }
    ret
}

/// UART 8O1 with inverted start/stop, MSB first (as rtl_433's).
pub fn extract_bytes_uart_8o1(
    msg: &[u8],
    mut offset: usize,
    mut num_bits: usize,
    dst: &mut [u8],
) -> usize {
    let mut ret = 0;
    while num_bits >= 11 && ret < dst.len() {
        let start = bit_of(msg, offset);
        let data = byte_from(msg, offset + 1);
        let parity = bit_of(msg, offset + 9);
        let stop = bit_of(msg, offset + 10);
        offset += 11;
        if start != 1 || parity as i32 != parity8(data) || stop != 0 {
            break;
        }
        dst[ret] = data;
        ret += 1;
        num_bits -= 11;
    }
    ret
}

fn symbol_match(msg: &[u8], offset: usize, num_bits: usize, symbol: u32) -> usize {
    let len = (symbol & 0x1f) as usize;
    if num_bits < len {
        return 0;
    }
    for pos in 0..len {
        if bit_of(msg, offset + pos) as u32 != (symbol >> (31 - pos)) & 1 {
            return 0;
        }
    }
    len
}

/// Bits coded as symbols (`zero`, `one`, `sync`: the pattern in the top
/// bits, its length in the low 5 bits), as rtl_433's
/// `extract_bits_symbols`; returns the bits written to `dst`.
pub fn extract_bits_symbols(
    msg: &[u8],
    mut offset: usize,
    mut num_bits: usize,
    zero: u32,
    one: u32,
    sync: u32,
    dst: &mut [u8],
) -> usize {
    let mut n = 0;
    while num_bits >= 1 {
        let l = symbol_match(msg, offset, num_bits, sync);
        if l > 0 {
            offset += l;
            num_bits -= l;
            continue;
        }
        let l = symbol_match(msg, offset, num_bits, zero);
        if l > 0 {
            offset += l;
            num_bits -= l;
            n += 1;
            continue;
        }
        let l = symbol_match(msg, offset, num_bits, one);
        if l > 0 && n / 8 < dst.len() {
            offset += l;
            num_bits -= l;
            dst[n / 8] |= 0x80 >> (n % 8);
            n += 1;
            continue;
        }
        break;
    }
    n
}

pub fn crc4(msg: &[u8], polynomial: u8, init: u8) -> u8 {
    let mut rem: u32 = (init as u32) << 4;
    let poly = (polynomial as u32) << 4;
    for &b in msg {
        rem ^= b as u32;
        for _ in 0..8 {
            rem = if rem & 0x80 != 0 {
                (rem << 1) ^ poly
            } else {
                rem << 1
            };
        }
    }
    ((rem >> 4) & 0x0f) as u8
}

pub fn crc7(msg: &[u8], polynomial: u8, init: u8) -> u8 {
    let mut rem: u32 = (init as u32) << 1;
    let poly = (polynomial as u32) << 1;
    for &b in msg {
        rem ^= b as u32;
        for _ in 0..8 {
            rem = if rem & 0x80 != 0 {
                (rem << 1) ^ poly
            } else {
                rem << 1
            };
        }
    }
    ((rem >> 1) & 0x7f) as u8
}

pub fn crc8(msg: &[u8], polynomial: u8, init: u8) -> u8 {
    let mut rem = init;
    for &b in msg {
        rem ^= b;
        for _ in 0..8 {
            rem = if rem & 0x80 != 0 {
                (rem << 1) ^ polynomial
            } else {
                rem << 1
            };
        }
    }
    rem
}

pub fn crc8le(msg: &[u8], polynomial: u8, init: u8) -> u8 {
    let mut rem = init.reverse_bits();
    let poly = polynomial.reverse_bits();
    for &b in msg {
        rem ^= b;
        for _ in 0..8 {
            rem = if rem & 1 != 0 {
                (rem >> 1) ^ poly
            } else {
                rem >> 1
            };
        }
    }
    rem
}

pub fn crc16lsb(msg: &[u8], polynomial: u16, init: u16) -> u16 {
    let mut rem = init;
    for &b in msg {
        rem ^= b as u16;
        for _ in 0..8 {
            rem = if rem & 1 != 0 {
                (rem >> 1) ^ polynomial
            } else {
                rem >> 1
            };
        }
    }
    rem
}

pub fn crc16(msg: &[u8], polynomial: u16, init: u16) -> u16 {
    let mut rem = init;
    for &b in msg {
        rem ^= (b as u16) << 8;
        for _ in 0..8 {
            rem = if rem & 0x8000 != 0 {
                (rem << 1) ^ polynomial
            } else {
                rem << 1
            };
        }
    }
    rem
}

pub fn lfsr_digest8(msg: &[u8], gen_: u8, mut key: u8) -> u8 {
    let mut sum = 0;
    for &data in msg {
        for i in (0..8).rev() {
            if (data >> i) & 1 != 0 {
                sum ^= key;
            }
            key = if key & 1 != 0 {
                (key >> 1) ^ gen_
            } else {
                key >> 1
            };
        }
    }
    sum
}

pub fn lfsr_digest8_reverse(msg: &[u8], gen_: u8, mut key: u8) -> u8 {
    let mut sum = 0;
    for &data in msg.iter().rev() {
        for i in (0..8).rev() {
            if (data >> i) & 1 != 0 {
                sum ^= key;
            }
            key = if key & 1 != 0 {
                (key >> 1) ^ gen_
            } else {
                key >> 1
            };
        }
    }
    sum
}

pub fn lfsr_digest8_reflect(msg: &[u8], gen_: u8, mut key: u8) -> u8 {
    let mut sum = 0;
    for &data in msg.iter().rev() {
        for i in 0..8 {
            if (data >> i) & 1 != 0 {
                sum ^= key;
            }
            key = if key & 0x80 != 0 {
                (key << 1) ^ gen_
            } else {
                key << 1
            };
        }
    }
    sum
}

pub fn lfsr_digest16(msg: &[u8], gen_: u16, mut key: u16) -> u16 {
    let mut sum = 0;
    for &data in msg {
        for i in (0..8).rev() {
            if (data >> i) & 1 != 0 {
                sum ^= key;
            }
            key = if key & 1 != 0 {
                (key >> 1) ^ gen_
            } else {
                key >> 1
            };
        }
    }
    sum
}

/// CCITT data whitening (x^9 + x^5 + 1, key 0x1FF), byte by byte.
pub fn ccitt_whitening(buf: &mut [u8]) {
    let mut key_msb: u8 = 0x01;
    let mut key_lsb: u8 = 0xff;
    for b in buf {
        *b ^= key_lsb.reverse_bits();
        for _ in 0..8 {
            let prev = key_msb;
            key_msb = (key_lsb & 1) ^ ((key_lsb >> 5) & 1);
            key_lsb = ((prev << 7) & 0x80) | (key_lsb >> 1);
        }
    }
}

/// IBM data whitening (same LFSR as CCITT, bit by bit).
pub fn ibm_whitening(buf: &mut [u8]) {
    let mut key_msb: u8 = 0x01;
    let mut key_lsb: u8 = 0xff;
    for b in buf {
        *b ^= key_lsb;
        for _ in 0..8 {
            let prev = key_msb;
            key_msb = (key_lsb & 1) ^ ((key_lsb >> 5) & 1);
            key_lsb = (key_lsb >> 1) | ((prev << 7) & 0x80);
        }
    }
}

pub fn parity8(b: u8) -> i32 {
    (b.count_ones() & 1) as i32
}

pub fn parity_bytes(msg: &[u8]) -> i32 {
    msg.iter().fold(0, |p, &b| p ^ parity8(b))
}

pub fn xor_bytes(msg: &[u8]) -> u8 {
    msg.iter().fold(0, |x, &b| x ^ b)
}

pub fn add_bytes(msg: &[u8]) -> i32 {
    msg.iter().map(|&b| b as i32).sum()
}

pub fn add_nibbles(msg: &[u8]) -> i32 {
    msg.iter()
        .map(|&b| (b >> 4) as i32 + (b & 0xf) as i32)
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_and_parse() {
        let b = BitBuffer::parse("{36}5a2f81f40 {8}ff");
        assert_eq!(b.num_rows, 2);
        assert_eq!(b.bits(0), 36);
        assert_eq!(&b.row(0)[..5], &[0x5a, 0x2f, 0x81, 0xf4, 0x00]);
        assert_eq!(b.row_string(1), "{8}ff");
        let mut b = BitBuffer::parse("{12}746");
        b.nrzs_decode();
        // rtl_433's own unit test (bitbuffer.c).
        assert_eq!(&b.row(0)[..2], &[0xb1, 0xa0]);
    }

    #[test]
    fn search_and_repeats() {
        let b = BitBuffer::parse("{24}aa2dd4 {24}aa2dd4 {16}1234");
        assert_eq!(b.search(0, 0, &[0x2d, 0xd4], 16), 8);
        assert_eq!(b.find_repeated_row(2, 24), Some(0));
        assert_eq!(b.find_repeated_row(3, 24), None);
    }

    #[test]
    fn manchester() {
        // 01 10 01 01 → 1 0 1 1
        let b = BitBuffer::parse("{8}65");
        let mut o = BitBuffer::new();
        b.manchester_decode(0, 0, &mut o, 0);
        assert_eq!(o.bits(0), 4);
        assert_eq!(o.row(0)[0], 0xb0);
    }

    #[test]
    fn crcs() {
        // CRC-8 poly 0x31 init 0 of "123456789" is 0xA2 (CRC-8/MAXIM is
        // the reflected one: 0xA1).
        let m = b"123456789";
        assert_eq!(crc8(m, 0x07, 0x00), 0xf4); // CRC-8/SMBUS
        assert_eq!(crc8le(m, 0x31, 0x00), 0xa1); // CRC-8/MAXIM
        assert_eq!(crc16(m, 0x1021, 0xffff), 0x29b1); // CRC-16/CCITT-FALSE
        assert_eq!(crc16lsb(m, 0x8408, 0x0000), 0x2189); // CRC-16/KERMIT
        assert_eq!(parity8(0x07), 1);
    }
}
