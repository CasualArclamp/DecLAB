// made by CasualArclamp
//! The flex decoder: a user-written decoder from a spec string, rtl_433's
//! `-X "n=name,m=OOK_PWM,s=400,l=800,r=7000,…"`.
//!
//! Ported from rtl_433's `src/devices/flex.c` (Copyright (C) 2017
//! Christian W. Zuckschwerdt, GPL-2.0-or-later): the same keys, defaults,
//! sanity checks and decode steps (row/bit/repeat filters, invert,
//! reflect, match, preamble alignment, symbol, UART, differential
//! Manchester and Manchester decoding, unique/countonly output, getters
//! with value maps).
//!
//! Differences from rtl_433:
//! - Errors come back as `Err(message)` instead of printing and exiting;
//!   `help` (or an empty spec) returns the help text as the error.
//! - `m=OOK_MC_OSV1` is refused (the OSv1 slicer is not ported);
//!   `m=OOK_NRZS` is accepted (DecLAB has that slicer).
//! - The multi-row output is flat: `count`, `num_rows`, then for each row
//!   `len`, `data` and its getters (rtl_433 nests them in a `rows` array and
//!   adds a `codes` array, which is left out).
//! - A getter's `%` format is applied with a small printf (`%d %i %u %x %X
//!   %o`, flags `0 - + #`, a width) and the field becomes text; without a
//!   format it is an integer as in rtl_433.
//! - Names, map values and formats are not truncated at 31 characters.

use std::sync::Mutex;

use crate::bits::{
    BitBuffer, COLS, extract_bits_symbols, extract_bytes_uart_8n1, extract_bytes_uart_8n2,
    extract_bytes_uart_8o1, reflect_bytes,
};
use crate::device::{
    Ctx, DECODE_ABORT_EARLY, DECODE_ABORT_LENGTH, DECODE_FAIL_SANITY, Device, Group, Message,
    Modulation,
};

/// rtl_433's `-X help` text, returned as the error for `help`.
pub const HELP: &str = "Flex decoder spec: \"key=value[,key=value...]\"
Common keys:
  name=<name> (or n=), modulation=<modulation> (or m=),
  short=<us> (or s=), long=<us> (or l=), sync=<us> (or y=),
  reset=<us> (or r=), gap=<us> (or g=), tolerance=<us> (or t=),
  priority=<n> (or prio=): run only as a fallback
<modulation>: OOK_MC_ZEROBIT, OOK_PCM, OOK_RZ, OOK_PPM, OOK_PWM, OOK_DMC,
  OOK_PIWM_RAW, OOK_PIWM_DC, OOK_NRZS, FSK_PCM, FSK_PWM, FSK_MC_ZEROBIT
PCM/RZ  short: pulse width, long: bit period
PPM     short: '0' gap, long: '1' gap
PWM     short: '1' pulse, long: '0' pulse, sync: sync pulse (optional)
common  gap: largest gap within a row, reset: largest gap within a message,
        tolerance: largest pulse deviation (optional)
Options:
  bits=<n>, rows=<n>, repeats=<n> (also bits>=n, bits<=n, …)
  invert, reflect, decode_uart=<8n1|8n2|8o1>, decode_dm, decode_mc
  match=<bits>, preamble=<bits> ({<bit count>}<hex>)
  symbol_zero=<bits>, symbol_one=<bits>, symbol_sync=<bits>
  get=[@<offset>:][{<bits>}<mask>:]<name>[:%<format>][:[<n>:<text> ...]]
  unique: one message per transmission, countonly: only the count
E.g. n=doorbell,m=OOK_PWM,s=400,l=800,r=7000,g=1000,match={24}0xa9878c,repeats>=3";

const GETTER_SLOTS: usize = 12;
const GETTER_MAP_SLOTS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Uart {
    N1,
    N2,
    O1,
}

/// rtl_433's `flex_get`: a field read from each row.
#[derive(Debug, Clone)]
struct Getter {
    bit_offset: usize,
    bit_count: usize,
    /// Take only these bits (from `bit_offset`, MSB first), squeezed
    /// together; 0: take `bit_count` bits as they are.
    mask: u64,
    name: &'static str,
    /// Values shown as text instead of the number.
    map: Vec<(u64, String)>,
    format: Option<String>,
}

/// A user-written decoder from a spec string (rtl_433's `-X`).
#[derive(Debug, Clone)]
pub struct Flex {
    pub spec: String,
    /// The name (`n=`), shown as the model.
    pub name: String,
    /// The coding and timings; its `decode` is unused (see [`Flex::decode`]).
    pub device: Device,
    min_rows: usize,
    max_rows: usize,
    min_bits: usize,
    max_bits: usize,
    min_repeats: usize,
    /// Parsed but, as in rtl_433, not checked.
    #[allow(dead_code)]
    max_repeats: usize,
    invert: bool,
    reflect: bool,
    unique: bool,
    count_only: bool,
    /// `match=`: bits and their count (0: none).
    match_bits: Vec<u8>,
    match_len: usize,
    preamble_bits: Vec<u8>,
    preamble_len: usize,
    /// Symbols as `extract_bits_symbols` takes them: the pattern in the
    /// top bits, the length in the low 5 (0: none).
    symbol_zero: u32,
    symbol_one: u32,
    symbol_sync: u32,
    getters: Vec<Getter>,
    decode_uart: Option<Uart>,
    decode_dm: bool,
    decode_mc: bool,
}

/// A getter's name as `&'static str`, which `Message` keys need: each
/// distinct name is leaked once and reused (a few bytes per name ever used).
///
/// Rust note: `Box::leak` turns an owned string into one that lives for
/// the rest of the program; the `Mutex` keeps the list safe to share
/// between threads.
fn intern(s: &str) -> &'static str {
    static NAMES: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());
    let mut names = NAMES.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(n) = names.iter().find(|n| **n == s) {
        return n;
    }
    let n: &'static str = Box::leak(s.to_owned().into_boxed_str());
    names.push(n);
    n
}

#[inline]
fn bit(bytes: &[u8], b: usize) -> u64 {
    bytes
        .get(b >> 3)
        .map_or(0, |x| ((x >> (7 - (b & 7))) & 1) as u64)
}

/// rtl_433's `compact_number`: the bits under the mask's set bits, from
/// `bit_offset` on, squeezed together.
fn compact_number(data: &[u8], mut bit_offset: usize, mask: u64) -> u64 {
    let top_bit = 64 - mask.leading_zeros() as usize;
    let mut val = 0u64;
    for b in (0..top_bit).rev() {
        if mask & (1u64 << b) != 0 {
            val = (val << 1) | bit(data, bit_offset);
        }
        bit_offset += 1;
    }
    val
}

/// rtl_433's `extract_number`: `bit_count` (≤ 64) bits from `bit_offset`.
fn extract_number(data: &[u8], bit_offset: usize, bit_count: usize) -> u64 {
    (0..bit_count).fold(0u64, |v, i| (v << 1) | bit(data, bit_offset + i))
}

/// rtl_433's `print_row_bytes`: hex, cut to the nibbles the bits need.
fn row_hex(bytes: &[u8], num_bits: usize) -> String {
    let mut s = String::with_capacity(num_bits / 4 + 2);
    for b in &bytes[..num_bits.div_ceil(8)] {
        s.push_str(&format!("{b:02x}"));
    }
    s.truncate(num_bits.div_ceil(4));
    s
}

/// A small printf for one integer: `%[flags][width][length](d|i|u|x|X|o)`
/// with text around it (`"%d C"`, `"0x%04X"`).
fn format_int(fmt: &str, v: u64) -> String {
    let mut out = String::new();
    let mut chars = fmt.chars().peekable();
    let mut done = false;
    while let Some(c) = chars.next() {
        if c != '%' {
            out.push(c);
            continue;
        }
        if chars.peek() == Some(&'%') {
            chars.next();
            out.push('%');
            continue;
        }
        let (mut zero, mut left, mut plus, mut alt) = (false, false, false, false);
        while let Some(&f) = chars.peek() {
            match f {
                '0' => zero = true,
                '-' => left = true,
                '+' => plus = true,
                '#' => alt = true,
                ' ' => {}
                _ => break,
            }
            chars.next();
        }
        let mut width = 0usize;
        while let Some(d) = chars.peek().and_then(|c| c.to_digit(10)) {
            width = width * 10 + d as usize;
            chars.next();
        }
        while matches!(chars.peek(), Some('l' | 'h' | 'z' | 'j' | 't' | 'q')) {
            chars.next();
        }
        let conv = chars.next().unwrap_or('d');
        let body = if done {
            String::new() // only one number to show
        } else {
            done = true;
            match conv {
                'x' => format!("{}{v:x}", if alt && v != 0 { "0x" } else { "" }),
                'X' => format!("{}{v:X}", if alt && v != 0 { "0X" } else { "" }),
                'o' => format!("{}{v:o}", if alt && v != 0 { "0" } else { "" }),
                'd' | 'i' => format!("{}{}", if plus { "+" } else { "" }, v as i64),
                _ => v.to_string(),
            }
        };
        if body.len() >= width {
            out.push_str(&body);
        } else if left {
            out.push_str(&body);
            out.push_str(&" ".repeat(width - body.len()));
        } else if zero {
            // Zeros go after a sign or 0x prefix.
            let split = if body.starts_with("0x") || body.starts_with("0X") {
                2
            } else if body.starts_with('+') || body.starts_with('-') {
                1
            } else {
                0
            };
            out.push_str(&body[..split]);
            out.push_str(&"0".repeat(width - body.len()));
            out.push_str(&body[split..]);
        } else {
            out.push_str(&" ".repeat(width - body.len()));
            out.push_str(&body);
        }
    }
    out
}

/// rtl_433's `parse_modulation`, case-insensitive.
fn parse_modulation(s: &str) -> Result<Modulation, String> {
    let m = match s.to_ascii_uppercase().as_str() {
        "OOK_MC_ZEROBIT" => Modulation::OokManchesterZeroBit,
        // RZ and NRZ share the PCM slicer: short < long makes it RZ.
        "OOK_PCM" | "OOK_RZ" => Modulation::OokPcm,
        "OOK_PPM" => Modulation::OokPpm,
        "OOK_PWM" => Modulation::OokPwm,
        "OOK_DMC" => Modulation::OokDmc,
        "OOK_PIWM_RAW" => Modulation::OokPiwmRaw,
        "OOK_PIWM_DC" => Modulation::OokPiwmDc,
        "OOK_NRZS" => Modulation::OokNrzs,
        "FSK_PCM" => Modulation::FskPcm,
        "FSK_PWM" => Modulation::FskPwm,
        "FSK_MC_ZEROBIT" => Modulation::FskManchesterZeroBit,
        "OOK_MC_OSV1" => {
            return Err(
                "flex: modulation OOK_MC_OSV1 is not supported (no Oregon v1 slicer)".into(),
            );
        }
        _ => {
            return Err(format!(
                "flex: unknown modulation \"{s}\" (OOK_MC_ZEROBIT, OOK_PCM, OOK_RZ, OOK_PPM, OOK_PWM, OOK_DMC, \
                 OOK_PIWM_RAW, OOK_PIWM_DC, OOK_NRZS, FSK_PCM, FSK_PWM, FSK_MC_ZEROBIT)"
            ));
        }
    };
    Ok(m)
}

/// rtl_433's `parse_float`: a width in µs.
fn parse_float(val: Option<&str>, key: &str) -> Result<f32, String> {
    let v = val.ok_or_else(|| format!("flex: {key}: missing number argument"))?;
    if v.is_empty() {
        return Err(format!("flex: {key}: empty number argument"));
    }
    v.parse::<f32>()
        .ok()
        .filter(|x| x.is_finite() && *x >= 0.0)
        .ok_or_else(|| format!("flex: {key}: invalid number argument ({v})"))
}

/// rtl_433's `parse_atoiv`: a whole number, `def` when absent or empty.
fn parse_int(val: Option<&str>, def: usize, key: &str) -> Result<usize, String> {
    match val {
        None | Some("") => Ok(def),
        Some(v) => v
            .parse::<usize>()
            .map_err(|_| format!("flex: {key}: invalid number argument ({v})")),
    }
}

fn parse_flag(val: Option<&str>, key: &str) -> Result<bool, String> {
    Ok(parse_int(val, 1, key)? != 0)
}

/// rtl_433's `parse_bits`: one row `{n}hex` (or bare hex), up to 1024 bits.
fn parse_bits(val: Option<&str>, key: &str) -> Result<(Vec<u8>, usize), String> {
    let v = val
        .filter(|v| !v.is_empty())
        .ok_or_else(|| format!("flex: {key}: missing bits (e.g. {{24}}0xa9878c)"))?;
    let bits = BitBuffer::parse(v);
    if bits.num_rows != 1 {
        return Err(format!(
            "flex: {key}: needs exactly one bit row ({} found)",
            bits.num_rows
        ));
    }
    let len = bits.bits(0);
    if len == 0 {
        return Err(format!("flex: {key}: no bits in \"{v}\""));
    }
    if len > COLS * 8 {
        return Err(format!("flex: {key}: up to 1024 bits ({len} found)"));
    }
    Ok((bits.row(0)[..len.div_ceil(8)].to_vec(), len))
}

/// rtl_433's `parse_symbol`: up to 27 bits, packed with its length.
fn parse_symbol(val: Option<&str>, key: &str) -> Result<u32, String> {
    let v = val
        .filter(|v| !v.is_empty())
        .ok_or_else(|| format!("flex: {key}: missing bits"))?;
    let bits = BitBuffer::parse(v);
    if bits.num_rows != 1 {
        return Err(format!(
            "flex: {key}: needs exactly one bit row ({} found)",
            bits.num_rows
        ));
    }
    let len = bits.bits(0);
    if len == 0 || len > 27 {
        return Err(format!("flex: {key}: 1 to 27 bits ({len} found)"));
    }
    let b = bits.row(0);
    Ok(u32::from_be_bytes([b[0], b[1], b[2], b[3]]) | len as u32)
}

/// A C `strtol(s, &end, 0)`: decimal, `0x` hex or leading-0 octal; returns
/// the number and the rest.
fn strtol0(s: &str) -> (Option<u64>, &str) {
    let (radix, digits) = if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        (16, h)
    } else if let Some(o) = s.strip_prefix('0').filter(|o| !o.is_empty()) {
        (8, o)
    } else {
        (10, s)
    };
    let end = digits
        .find(|c: char| !c.is_digit(radix))
        .unwrap_or(digits.len());
    if end == 0 {
        // "0" alone, or no digits at all.
        return match s.strip_prefix('0') {
            Some(rest) => (Some(0), rest),
            None => (None, s),
        };
    }
    (
        u64::from_str_radix(&digits[..end], radix).ok(),
        &digits[end..],
    )
}

/// rtl_433's `parse_map`: `[key:text key:text …]`; returns the rest.
fn parse_map<'a>(arg: &'a str, g: &mut Getter) -> Result<&'a str, String> {
    let mut c = arg.trim_start_matches(' ');
    c = c.strip_prefix('[').unwrap_or(c);
    loop {
        c = c.trim_start_matches(' ');
        if c.is_empty() {
            return Ok(c);
        }
        if let Some(rest) = c.strip_prefix(']') {
            return Ok(rest);
        }
        let (key, rest) = strtol0(c);
        let key =
            key.ok_or_else(|| format!("flex: get {}: map key is not a number ({c})", g.name))?;
        c = rest.trim_start_matches(' ');
        c = c.strip_prefix(':').unwrap_or(c).trim_start_matches(' ');
        let e = c.find([' ', ']']).unwrap_or(c.len());
        if g.map.len() >= GETTER_MAP_SLOTS {
            return Err(format!("flex: get: at most {GETTER_MAP_SLOTS} map values"));
        }
        g.map.push((key, c[..e].to_string()));
        c = &c[e..];
    }
}

/// rtl_433's `parse_getter`: `@offset`, `{bits}mask`, `%format`, `name`
/// and `[map]`, split by `:`.
fn parse_getter(arg: &str) -> Result<Getter, String> {
    let mut g = Getter {
        bit_offset: 0,
        bit_count: 0,
        mask: 0,
        name: "",
        map: Vec::new(),
        format: None,
    };
    let mut name = String::new();
    let mut rest = arg;
    while !rest.is_empty() {
        if rest.starts_with('[') {
            rest = parse_map(rest, &mut g)?;
            continue;
        }
        let (tok, next) = match rest.find(':') {
            Some(i) => (&rest[..i], &rest[i + 1..]),
            None => (rest, ""),
        };
        if let Some(off) = tok.strip_prefix('@') {
            g.bit_offset = match strtol0(off) {
                (Some(v), "") => v as usize,
                _ => return Err(format!("flex: get: bad bit offset \"{tok}\"")),
            };
        } else if tok.starts_with('{') || tok.starts_with(|c: char| c.is_ascii_digit()) {
            let (bits, len) = parse_bits(Some(tok), "get")?;
            if len > 64 {
                return Err(format!("flex: get: up to 64 bits ({len} found)"));
            }
            g.bit_count = len;
            g.mask = extract_number(&bits, 0, len);
        } else if tok.starts_with('%') {
            g.format = Some(tok.to_string());
        } else if !tok.is_empty() {
            name = tok.to_string();
        }
        rest = next;
    }
    if name.is_empty() {
        return Err(format!("flex: get \"{arg}\": missing name"));
    }
    if g.bit_count == 0 {
        return Err(format!(
            "flex: get \"{arg}\": missing bit count (e.g. {{8}})"
        ));
    }
    g.name = intern(&name);
    Ok(g)
}

fn parse_uart(val: Option<&str>) -> Result<Uart, String> {
    match val.map(|v| v.to_ascii_lowercase()).as_deref() {
        Some("8n1") => Ok(Uart::N1),
        Some("8n2") => Ok(Uart::N2),
        Some("8o1") => Ok(Uart::O1),
        _ => Err(format!(
            "flex: decode_uart: unknown uart mode \"{}\" (8n1, 8n2, 8o1)",
            val.unwrap_or("")
        )),
    }
}

/// Replace row `r` with `len` bits from `src`, zeroing the rest of the row
/// (the buffer's `clear` only zeroes the bytes a row's length covers).
fn set_row(bb: &mut BitBuffer, r: usize, src: &[u8], len: usize) {
    let row = &mut bb.bb[r];
    let n = len.div_ceil(8).min(row.len());
    row[..n].copy_from_slice(&src[..n]);
    row[n..].fill(0);
    if !len.is_multiple_of(8) {
        row[n - 1] &= (0xff00u16 >> (len % 8)) as u8;
    }
    bb.bits_per_row[r] = len as u16;
}

impl Flex {
    /// Parse rtl_433's `-X` spec.
    pub fn parse(spec: &str) -> Result<Flex, String> {
        let trimmed = spec.trim();
        if trimmed.is_empty()
            || trimmed.starts_with('?')
            || "help".starts_with(&trimmed.to_ascii_lowercase())
        {
            return Err(HELP.to_string());
        }
        let mut f = Flex {
            spec: spec.to_string(),
            name: String::new(),
            device: Device {
                name: "flex",
                group: Group::Flex,
                ..Device::DEFAULT
            },
            min_rows: 0,
            max_rows: 0,
            min_bits: 0,
            max_bits: 0,
            min_repeats: 0,
            max_repeats: 0,
            invert: false,
            reflect: false,
            unique: false,
            count_only: false,
            match_bits: Vec::new(),
            match_len: 0,
            preamble_bits: Vec::new(),
            preamble_len: 0,
            symbol_zero: 0,
            symbol_one: 0,
            symbol_sync: 0,
            getters: Vec::new(),
            decode_uart: None,
            decode_dm: false,
            decode_mc: false,
        };
        let mut modulation = None;

        // rtl_433's `getkwargs`: split at commas, then at the first `=`.
        // Whitespace is removed from keys and trimmed from values.
        for part in spec.split(',') {
            let (key, val) = match part.find('=') {
                Some(i) => (&part[..i], Some(part[i + 1..].trim())),
                None => (part, None),
            };
            let key: String = key
                .chars()
                .filter(|c| !c.is_whitespace())
                .collect::<String>()
                .to_ascii_lowercase();
            match key.as_str() {
                "" => continue,
                "n" | "name" => {
                    let v = val.filter(|v| !v.is_empty()).ok_or("flex: name: missing")?;
                    f.name = v.to_string();
                }
                "m" | "modulation" => {
                    modulation = Some(parse_modulation(val.ok_or("flex: modulation: missing")?)?);
                }
                "s" | "short" => f.device.short_width = parse_float(val, "short")?,
                "l" | "long" => f.device.long_width = parse_float(val, "long")?,
                "y" | "sync" => f.device.sync_width = parse_float(val, "sync")?,
                "g" | "gap" => f.device.gap_limit = parse_float(val, "gap")?,
                "r" | "reset" => f.device.reset_limit = parse_float(val, "reset")?,
                "t" | "tolerance" => f.device.tolerance = parse_float(val, "tolerance")?,
                "prio" | "priority" => f.device.priority = parse_int(val, 0, "priority")? as u32,

                // `bits>=24` splits at its `=` into the key `bits>`.
                "bits>" => f.min_bits = parse_int(val, 0, "bits")?,
                "bits<" => f.max_bits = parse_int(val, 0, "bits")?,
                "bits" => {
                    f.min_bits = parse_int(val, 0, "bits")?;
                    f.max_bits = f.min_bits;
                }
                "rows>" => f.min_rows = parse_int(val, 0, "rows")?,
                "rows<" => f.max_rows = parse_int(val, 0, "rows")?,
                "rows" => {
                    f.min_rows = parse_int(val, 0, "rows")?;
                    f.max_rows = f.min_rows;
                }
                "repeats>" => f.min_repeats = parse_int(val, 0, "repeats")?,
                "repeats<" => f.max_repeats = parse_int(val, 0, "repeats")?,
                "repeats" => {
                    f.min_repeats = parse_int(val, 0, "repeats")?;
                    f.max_repeats = f.min_repeats;
                }

                "invert" => f.invert = parse_flag(val, "invert")?,
                "reflect" => f.reflect = parse_flag(val, "reflect")?,
                "match" => (f.match_bits, f.match_len) = parse_bits(val, "match")?,
                "preamble" => (f.preamble_bits, f.preamble_len) = parse_bits(val, "preamble")?,
                "countonly" => f.count_only = parse_flag(val, "countonly")?,
                "unique" => f.unique = parse_flag(val, "unique")?,
                "decode_uart" => f.decode_uart = Some(parse_uart(val)?),
                "decode_dm" => f.decode_dm = parse_flag(val, "decode_dm")?,
                "decode_mc" => f.decode_mc = parse_flag(val, "decode_mc")?,
                "symbol_zero" => f.symbol_zero = parse_symbol(val, "symbol_zero")?,
                "symbol_one" => f.symbol_one = parse_symbol(val, "symbol_one")?,
                "symbol_sync" => f.symbol_sync = parse_symbol(val, "symbol_sync")?,
                "get" => {
                    if f.getters.len() >= GETTER_SLOTS {
                        return Err(format!("flex: at most {GETTER_SLOTS} get= fields"));
                    }
                    f.getters.push(parse_getter(val.unwrap_or(""))?);
                }
                _ => return Err(format!("flex: unknown keyword \"{key}\" (try \"help\")")),
            }
        }

        if f.min_bits < f.match_len {
            f.min_bits = f.match_len;
        }
        if f.min_bits > 0 && f.min_repeats < 1 {
            f.min_repeats = 1;
        }

        // Sanity checks, as rtl_433's.
        if f.name.is_empty() {
            return Err("flex: missing name (n=…)".into());
        }
        let Some(m) = modulation else {
            return Err("flex: missing modulation (m=…)".into());
        };
        f.device.modulation = m;
        let d = &f.device;
        if d.short_width == 0.0 {
            return Err("flex: missing short width (s=…)".into());
        }
        if !matches!(
            m,
            Modulation::OokManchesterZeroBit | Modulation::FskManchesterZeroBit
        ) && d.long_width == 0.0
        {
            return Err("flex: missing long width (l=…)".into());
        }
        if d.reset_limit == 0.0 {
            return Err("flex: missing reset limit (r=…)".into());
        }
        if matches!(
            m,
            Modulation::OokDmc | Modulation::OokPiwmRaw | Modulation::OokPiwmDc
        ) && d.tolerance == 0.0
        {
            return Err("flex: missing tolerance (t=…), needed by this modulation".into());
        }
        if f.symbol_zero != 0 && f.symbol_one == 0 {
            return Err("flex: symbol_one missing".into());
        }
        if f.symbol_one != 0 && f.symbol_zero == 0 {
            return Err("flex: symbol_zero missing".into());
        }
        Ok(f)
    }

    /// Each getter's field, appended to `m` (rtl_433's `render_getters`).
    fn render_getters(&self, mut m: Message, bytes: &[u8]) -> Message {
        for g in &self.getters {
            let val = if g.mask != 0 {
                compact_number(bytes, g.bit_offset, g.mask)
            } else {
                extract_number(bytes, g.bit_offset, g.bit_count)
            };
            m = if let Some((_, text)) = g.map.iter().find(|(k, _)| *k == val) {
                m.str(g.name, text.clone())
            } else if let Some(fmt) = &g.format {
                m.str(g.name, format_int(fmt, val))
            } else {
                m.int(g.name, val as i64)
            };
        }
        m
    }

    /// rtl_433's `flex_callback`: filter, transform and output the rows.
    pub fn decode(&self, ctx: &mut Ctx, bits: &mut BitBuffer) -> i32 {
        let num_rows = bits.num_rows;
        // Discard short or unwanted bit buffers.
        if num_rows < self.min_rows || (self.max_rows != 0 && num_rows > self.max_rows) {
            return DECODE_ABORT_LENGTH;
        }
        let mut match_count = (0..num_rows)
            .filter(|&i| {
                let n = bits.bits(i);
                n >= self.min_bits && (self.max_bits == 0 || n <= self.max_bits)
            })
            .count();
        if match_count == 0 {
            return DECODE_ABORT_LENGTH;
        }
        // Discard unless min_repeats, min_bits (max_repeats is not
        // checked, as in rtl_433).
        let Some(mut r) = bits.find_repeated_row(self.min_repeats, self.min_bits) else {
            return DECODE_ABORT_EARLY;
        };

        if self.invert {
            bits.invert();
        }
        if self.reflect {
            for i in 0..num_rows {
                let n = bits.bits(i).div_ceil(8);
                reflect_bytes(&mut bits.bb[i][..n]);
            }
        }

        // Discard unless some row holds the match bits.
        if self.match_len > 0 {
            let found: Vec<usize> = (0..num_rows)
                .filter(|&i| bits.search(i, 0, &self.match_bits, self.match_len) < bits.bits(i))
                .collect();
            if found.is_empty() {
                return DECODE_FAIL_SANITY;
            }
            r = found[0];
            match_count = found.len();
        }

        // Discard unless some row holds the preamble; align those that do
        // to just after it.
        if self.preamble_len > 0 {
            let mut first = None;
            match_count = 0;
            let mut tmp = [0u8; COLS];
            for i in 0..num_rows {
                let pos = bits.search(i, 0, &self.preamble_bits, self.preamble_len);
                if pos < bits.bits(i) {
                    first.get_or_insert(i);
                    match_count += 1;
                    let pos = pos + self.preamble_len;
                    let len = bits.bits(i) - pos;
                    tmp.fill(0);
                    bits.extract_bytes(i, pos, &mut tmp, len);
                    set_row(bits, i, &tmp, len);
                }
            }
            match first {
                Some(i) => r = i,
                None => return DECODE_FAIL_SANITY,
            }
        }

        let mut tmp = [0u8; COLS];
        if self.symbol_zero != 0 {
            for i in 0..num_rows {
                tmp.fill(0);
                let len = extract_bits_symbols(
                    bits.row(i),
                    0,
                    bits.bits(i),
                    self.symbol_zero,
                    self.symbol_one,
                    self.symbol_sync,
                    &mut tmp,
                );
                set_row(bits, i, &tmp, len);
            }
        }

        if let Some(mode) = self.decode_uart {
            for i in 0..num_rows {
                tmp.fill(0);
                let (row, n) = (bits.row(i), bits.bits(i));
                let len = match mode {
                    Uart::N1 => extract_bytes_uart_8n1(row, 0, n, &mut tmp),
                    Uart::N2 => extract_bytes_uart_8n2(row, 0, n, &mut tmp),
                    Uart::O1 => extract_bytes_uart_8o1(row, 0, n, &mut tmp),
                };
                set_row(bits, i, &tmp, len * 8);
            }
        }

        // Rust note: `Box` puts the scratch buffer (~6.6 kB) on the heap,
        // made only when a Manchester decode is asked for.
        if self.decode_dm || self.decode_mc {
            let mut out = Box::new(BitBuffer::new());
            for i in 0..num_rows {
                out.clear();
                let n = bits.bits(i);
                if self.decode_dm {
                    bits.differential_manchester_decode(i, 0, &mut out, n);
                    let len = out.bits(0);
                    set_row(bits, i, out.row(0), len);
                }
                if self.decode_mc {
                    out.clear();
                    let n = bits.bits(i);
                    bits.manchester_decode(i, 0, &mut out, n);
                    let len = out.bits(0);
                    set_row(bits, i, out.row(0), len);
                }
            }
        }

        ctx.log(|| format!("{}: {bits:?}", self.name));

        // One message for the transmission: the first matching row.
        if self.unique {
            let n = bits.bits(r);
            let m = Message::new(self.name.clone())
                .int("count", match_count as i64)
                .int("num_rows", num_rows as i64)
                .int("len", n as i64)
                .str("data", row_hex(bits.row(r), n));
            let m = self.render_getters(m, bits.row(r));
            ctx.output(m);
            return 1;
        }

        if self.count_only {
            ctx.output(Message::new(self.name.clone()).int("count", match_count as i64));
            return 1;
        }

        let mut m = Message::new(self.name.clone())
            .int("count", match_count as i64)
            .int("num_rows", num_rows as i64);
        for i in 0..num_rows {
            let n = bits.bits(i);
            m = m.int("len", n as i64).str("data", row_hex(bits.row(i), n));
            m = self.render_getters(m, bits.row(i));
        }
        ctx.output(m);
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Value;

    fn run(spec: &str, code: &str) -> (i32, Vec<Message>) {
        let f = Flex::parse(spec).unwrap();
        let mut bits = BitBuffer::parse(code);
        let mut ctx = Ctx::default();
        let r = f.decode(&mut ctx, &mut bits);
        (r, ctx.out)
    }

    /// All values of `key`, as shown.
    fn all(m: &Message, key: &str) -> Vec<String> {
        m.fields
            .iter()
            .filter(|(k, _)| *k == key)
            .map(|(_, v)| v.to_string())
            .collect()
    }

    #[test]
    fn parses_rtl_433_example() {
        let f = Flex::parse(
            "n=doorbell,m=OOK_PWM,s=400,l=800,r=7000,g=1000,match={24}0xa9878c,repeats>=3",
        )
        .unwrap();
        assert_eq!(f.name, "doorbell");
        assert_eq!(f.device.modulation, Modulation::OokPwm);
        assert_eq!(f.device.group, Group::Flex);
        assert_eq!(f.device.name, "flex");
        assert_eq!((f.device.short_width, f.device.long_width), (400.0, 800.0));
        assert_eq!((f.device.reset_limit, f.device.gap_limit), (7000.0, 1000.0));
        assert_eq!(f.match_len, 24);
        assert_eq!(f.match_bits, vec![0xa9, 0x87, 0x8c]);
        assert_eq!(f.min_repeats, 3);
        // min_bits rises to the match length.
        assert_eq!(f.min_bits, 24);
    }

    #[test]
    fn parses_long_names_aliases_and_options() {
        let f = Flex::parse(
            " name = Remote X , modulation=ook_rz, short=200,long=600 ,reset=4000, sync=1200, tolerance=50, \
             priority=10, bits>=12, bits<=40, rows>=2, rows<=9, invert, reflect=0, unique, \
             get=@0:{8}:id, get=@8:{4}:button:[1:on 2:off], symbol_zero={3}8, symbol_one={3}c, decode_uart=8N1",
        )
        .unwrap();
        assert_eq!(f.name, "Remote X");
        assert_eq!(f.device.modulation, Modulation::OokPcm);
        assert_eq!(
            (f.device.sync_width, f.device.tolerance, f.device.priority),
            (1200.0, 50.0, 10)
        );
        assert_eq!(
            (
                f.min_bits,
                f.max_bits,
                f.min_rows,
                f.max_rows,
                f.min_repeats
            ),
            (12, 40, 2, 9, 1)
        );
        assert!(f.invert && !f.reflect && f.unique);
        assert_eq!(f.getters.len(), 2);
        assert_eq!(
            f.getters[1].map,
            vec![(1, "on".to_string()), (2, "off".to_string())]
        );
        assert_eq!(f.symbol_zero, 0x8000_0003);
        assert_eq!(f.symbol_one, 0xc000_0003);
        assert_eq!(f.decode_uart, Some(Uart::N1));
        for m in [
            "OOK_MC_ZEROBIT",
            "FSK_MC_ZEROBIT",
            "FSK_PCM",
            "FSK_PWM",
            "OOK_PPM",
            "OOK_NRZS",
        ] {
            assert!(
                Flex::parse(&format!("n=a,m={m},s=100,l=200,r=1000")).is_ok(),
                "{m}"
            );
        }
        // Manchester needs no long width.
        assert!(Flex::parse("n=a,m=OOK_MC_ZEROBIT,s=100,r=1000").is_ok());
        // Every modulation name DecLAB knows is accepted.
        for m in [
            Modulation::OokPiwmRaw,
            Modulation::OokPiwmDc,
            Modulation::OokDmc,
        ] {
            assert!(Flex::parse(&format!("n=a,m={},s=100,l=200,r=1000,t=20", m.name())).is_ok());
        }
    }

    #[test]
    fn refuses_bad_specs() {
        let bad = [
            ("", "Flex decoder spec"),
            ("help", "Flex decoder spec"),
            ("m=OOK_PWM,s=1,l=2,r=3", "missing name"),
            ("n=a,s=1,l=2,r=3", "missing modulation"),
            ("n=a,m=OOK_FOO,s=1,l=2,r=3", "unknown modulation"),
            ("n=a,m=OOK_MC_OSV1,s=1,l=2,r=3", "not supported"),
            ("n=a,m=OOK_PWM,l=2,r=3", "missing short"),
            ("n=a,m=OOK_PWM,s=1,r=3", "missing long"),
            ("n=a,m=OOK_PWM,s=1,l=2", "missing reset"),
            ("n=a,m=OOK_DMC,s=1,l=2,r=3", "missing tolerance"),
            ("n=a,m=OOK_PWM,s=abc,l=2,r=3", "invalid number"),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,s", "missing number"),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,bits>=x", "invalid number"),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,frobnicate=1", "unknown keyword"),
            (
                "n=a,m=OOK_PWM,s=1,l=2,r=3,match={8}aa/{8}bb",
                "exactly one bit row",
            ),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,match=", "missing bits"),
            (
                "n=a,m=OOK_PWM,s=1,l=2,r=3,symbol_zero={28}0000000",
                "27 bits",
            ),
            (
                "n=a,m=OOK_PWM,s=1,l=2,r=3,symbol_zero={2}8",
                "symbol_one missing",
            ),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,get=@0:{8}", "missing name"),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,get=@0:id", "missing bit count"),
            ("n=a,m=OOK_PWM,s=1,l=2,r=3,decode_uart=7e1", "uart mode"),
        ];
        for (spec, want) in bad {
            let e = Flex::parse(spec)
                .err()
                .unwrap_or_else(|| panic!("{spec} parsed"));
            assert!(e.contains(want), "{spec}: {e}");
        }
        let many: String = (0..13).map(|i| format!(",get=@{i}:{{1}}:g{i}")).collect();
        assert!(
            Flex::parse(&format!("n=a,m=OOK_PWM,s=1,l=2,r=3{many}"))
                .unwrap_err()
                .contains("at most 12")
        );
    }

    #[test]
    fn outputs_every_row() {
        let (r, out) = run("n=raw,m=OOK_PWM,s=1,l=2,r=3", "{12}abc {7}fe {0}");
        assert_eq!(r, 1);
        let m = &out[0];
        assert_eq!(m.model, "raw");
        assert_eq!(m.get("count"), Some(&Value::Int(3)));
        assert_eq!(m.get("num_rows"), Some(&Value::Int(3)));
        assert_eq!(all(m, "len"), ["12", "7", "0"]);
        assert_eq!(all(m, "data"), ["abc", "fe", ""]);
    }

    #[test]
    fn match_repeats_and_getters() {
        let spec = "n=doorbell,m=OOK_PWM,s=400,l=800,r=7000,g=1000,match={24}0xa9878c,repeats>=3,unique,\
                    get=@0:{8}:id:%02x,get=@20:{4}:button:[12:ding 3:dong],get=@0:{16}f00f:mix";
        let three = "{25}a9878c8 {25}a9878c8 {25}a9878c8";
        let (r, out) = run(spec, three);
        assert_eq!(r, 1);
        let m = &out[0];
        assert_eq!(m.get("count"), Some(&Value::Int(3)));
        assert_eq!(m.get("len"), Some(&Value::Int(25)));
        assert_eq!(m.get("data").unwrap().to_string(), "a9878c8");
        assert_eq!(m.get("id").unwrap().to_string(), "a9");
        assert_eq!(m.get("button").unwrap().to_string(), "ding");
        // The mask f00f takes the top and bottom nibbles of a987: a, 7.
        assert_eq!(m.get("mix"), Some(&Value::Int(0xa7)));
        // Only two repeats: refused.
        assert_eq!(run(spec, "{25}a9878c8 {25}a9878c8").0, DECODE_ABORT_EARLY);
        // Three repeats of something else: no match.
        assert_eq!(
            run(spec, "{25}a9878d8 {25}a9878d8 {25}a9878d8").0,
            DECODE_FAIL_SANITY
        );
        // Too short.
        assert_eq!(
            run(spec, "{16}a987 {16}a987 {16}a987").0,
            DECODE_ABORT_LENGTH
        );
    }

    #[test]
    fn preamble_aligns_and_invert() {
        // Inverted on the air: 0x5678 behind an inverted aa preamble.
        let (_, out) = run(
            "n=p,m=OOK_PWM,s=1,l=2,r=3,invert,preamble={8}aa,get=@0:{16}:word:%04X",
            "{28}f55a987 {6}00",
        );
        let m = &out[0];
        assert_eq!(m.get("count"), Some(&Value::Int(1)));
        // Inverted: 0aa5678 (28 bits) → after the preamble at bit 4: 5678.
        assert_eq!(all(m, "len"), ["16", "6"]);
        assert_eq!(all(m, "data"), ["5678", "fc"]);
        assert_eq!(all(m, "word")[0], "5678");
        // No preamble anywhere.
        assert_eq!(
            run("n=p,m=OOK_PWM,s=1,l=2,r=3,preamble={8}aa", "{16}1234").0,
            DECODE_FAIL_SANITY
        );
    }

    #[test]
    fn countonly_rows_reflect_symbols_manchester() {
        let (_, out) = run("n=c,m=OOK_PWM,s=1,l=2,r=3,countonly", "{8}01 {8}02");
        assert_eq!(out[0].fields.len(), 1);
        assert_eq!(out[0].get("count"), Some(&Value::Int(2)));
        assert_eq!(
            run("n=c,m=OOK_PWM,s=1,l=2,r=3,rows>=3", "{8}01 {8}02").0,
            DECODE_ABORT_LENGTH
        );
        assert_eq!(
            run("n=c,m=OOK_PWM,s=1,l=2,r=3,rows<=1", "{8}01 {8}02").0,
            DECODE_ABORT_LENGTH
        );
        // reflect: 0x01 → 0x80.
        let (_, out) = run("n=c,m=OOK_PWM,s=1,l=2,r=3,reflect", "{8}01");
        assert_eq!(out[0].get("data").unwrap().to_string(), "80");
        // Symbols: 100 = 0, 110 = 1: 100 110 110 100 → 0110.
        let (_, out) = run(
            "n=c,m=OOK_PWM,s=1,l=2,r=3,symbol_zero={3}8,symbol_one={3}c",
            "{12}9b4",
        );
        assert_eq!(out[0].get("len"), Some(&Value::Int(4)));
        assert_eq!(out[0].get("data").unwrap().to_string(), "6");
        // Manchester 01 = 1, 10 = 0: 01 10 01 01 → 1011.
        let (_, out) = run("n=c,m=OOK_PWM,s=1,l=2,r=3,decode_mc", "{8}65");
        assert_eq!(out[0].get("data").unwrap().to_string(), "b");
        // UART 8N1: start 0, 0x41 LSB first (10000010), stop 1 → "A".
        let (_, out) = run("n=c,m=OOK_PWM,s=1,l=2,r=3,decode_uart=8n1", "{10}414");
        assert_eq!(out[0].get("data").unwrap().to_string(), "41");
    }

    #[test]
    fn printf_formats() {
        assert_eq!(format_int("%d C", 21), "21 C");
        assert_eq!(format_int("0x%04X", 0xab), "0x00AB");
        assert_eq!(format_int("%#06x", 0xab), "0x00ab");
        assert_eq!(format_int("%5u|", 42), "   42|");
        assert_eq!(format_int("%-4o|", 42), "52  |");
        assert_eq!(format_int("100%%", 1), "100%");
    }

    #[test]
    fn decodes_a_simulated_remote_through_the_slicer() {
        use crate::sim::{Air, Sample};
        let f = Flex::parse("n=doorbell,m=OOK_PWM,s=400,l=800,r=7000,g=1000,match={24}0xa9878c,repeats>=3,unique,get=@0:{8}:id")
            .unwrap();
        let s = Sample {
            dev: 0,
            model: "doorbell",
            code: "{25}a9878c8 {25}a9878c8 {25}a9878c8 {25}a9878c8",
            expect: &[],
            air: Air::DEFAULT,
        };
        let lv = crate::sim::levels(&f.device, &s);
        let rate = 250_000.0;
        let mut p = crate::slicer::PulseData::default();
        crate::sim::levels_to_pulses(&lv, rate, &mut p);
        let mut bits = BitBuffer::new();
        let mut ctx = Ctx::default();
        let n =
            crate::slicer::slice_with(&p, &f.device, &|c, b| f.decode(c, b), &mut bits, &mut ctx);
        assert_eq!(n, 1, "{:?}", ctx.out);
        let m = &ctx.out[0];
        assert_eq!(m.model, "doorbell");
        assert_eq!(m.get("count"), Some(&Value::Int(4)));
        assert_eq!(m.get("data").unwrap().to_string(), "a9878c8");
        assert_eq!(m.get("id"), Some(&Value::Int(0xa9)));
    }
}
