// made by CasualArclamp
//! FLEX (Motorola; ARIB STD-43A), every speed: 1600 bit/s 2-level,
//! 3200 bit/s 2-level (3200 baud), 3200 bit/s 4-level (1600 baud) and
//! 6400 bit/s 4-level (3200 baud).
//!
//! A frame lasts 1.875 s: sync 1 and the frame information word (FIW) at
//! 1600 baud 2-level, sync 2 (25 ms) and eleven 160 ms blocks at the frame's
//! own speed. The blocks carry one to four interleaved *phases* (A–D) of 88
//! 32-bit words each: a block information word (BIW), the address field,
//! the vector field (one vector per address) and the message field.
//!
//! The layout follows multimon-ng's `demod_flex_next.c` (GPL-3.0-or-later;
//! sync codes, FIW/BIW fields, phase mapping and interleave, capcode ranges,
//! vector and header layouts), which is the reference that decodes real
//! networks. Where it differs from its own comments, the code is followed.
//!
//! Symbols are numbered 0–3 from the lowest frequency to the highest. The
//! sync pattern is read with 1 = low half (symbols 0, 1) and the data with
//! 1 = high half, as in multimon-ng; both polarities are tried.

use std::collections::HashMap;

use crate::{FlexSlot, Kind, Page, Protocol, bch, push_char};

/// The 32 bits between the A code and its complement in sync 1.
pub const MARKER: u32 = 0xA6C6_AAAA;

/// Sync 1 "A" codes → (symbol rate, levels). 0x4C7C is the ReFLEX A7 code.
pub const MODES: [(u16, u32, u8); 5] = [
    (0x870C, 1600, 2),
    (0xB068, 1600, 4),
    (0x7B18, 3200, 2),
    (0xDEA0, 3200, 4),
    (0x4C7C, 3200, 4),
];

/// Symbols after the sync match before the FIW (multimon-ng reads it at
/// symbols 16–47), and how far either side of that the FIW is looked for.
const FIW_OFFSET: usize = 15;
const FIW_SLACK: usize = 2;
const FIW_WINDOW: usize = FIW_OFFSET + FIW_SLACK + 32;

/// Words per phase.
pub const WORDS: usize = 88;

/// Numeric characters; 0xC (space) is the fill.
const BCD: &[u8; 16] = b"0123456789.U -][";
const FILL_DIGIT: u8 = 0xC;

/// FLEX group capcodes (short instruction targets): 2 029 568 + 0..15.
const GROUP_BASE: u64 = 2_029_568;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Sync1,
    Fiw,
    Sync2 { left: u32 },
    Data { count: u32 },
}

/// A message continuing in later frames.
struct Fragment {
    text: String,
    page: Page,
}

pub struct FlexDecoder {
    state: State,
    syncbuf: u64,
    /// Received with the opposite polarity to multimon-ng's convention.
    inverted: bool,
    baud: u32,
    levels: u8,
    fiw_syms: Vec<u8>,
    cycle: u8,
    frame: u8,
    phases: [[u32; WORDS]; 4],
    /// Reliability of each bit of each word (bit 0 first on air).
    rel: Box<[[[f32; 32]; WORDS]; 4]>,
    counter: usize,
    odd: bool,
    /// Group bit → capcodes enrolled by short instructions.
    groups: [Vec<u64>; 16],
    fragments: HashMap<u64, Fragment>,
    /// Report BIW date/time and system words as pages.
    pub system_words: bool,
    pages: Vec<Page>,
    /// Frames decoded, for the status line.
    pub frames: u64,
}

impl Default for FlexDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FlexDecoder {
    pub fn new() -> Self {
        FlexDecoder {
            state: State::Sync1,
            syncbuf: 0,
            inverted: false,
            baud: 1600,
            levels: 2,
            fiw_syms: Vec::with_capacity(FIW_WINDOW),
            cycle: 0,
            frame: 0,
            phases: [[0; WORDS]; 4],
            rel: Box::new([[[1.0; 32]; WORDS]; 4]),
            counter: 0,
            odd: false,
            groups: Default::default(),
            fragments: HashMap::new(),
            system_words: false,
            pages: Vec::new(),
            frames: 0,
        }
    }

    /// The symbol rate and number of levels the slicer should deliver now.
    pub fn wanted(&self) -> (u32, u8) {
        match self.state {
            State::Sync2 { .. } | State::Data { .. } => (self.baud, self.levels),
            _ => (1600, 2),
        }
    }

    pub fn in_sync(&self) -> bool {
        self.state != State::Sync1
    }

    /// The last frame's speed (symbol rate, levels) and its cycle and frame.
    pub fn mode(&self) -> (u32, u8, u8, u8) {
        (self.baud, self.levels, self.cycle, self.frame)
    }

    /// Feed one symbol, 0 (lowest frequency) to 3 (highest), with the
    /// reliability of its two bits. In 2-level modes only 0 and 3 are
    /// expected.
    pub fn push(&mut self, sym: u8, rel: [f32; 2]) {
        match self.state {
            State::Sync1 => {
                self.syncbuf = (self.syncbuf << 1) | u64::from(sym < 2);
                let found = match sync_code(self.syncbuf) {
                    Some(c) => Some((c, false)),
                    None => sync_code(!self.syncbuf).map(|c| (c, true)),
                };
                if let Some((code, inverted)) = found
                    && let Some(&(_, baud, levels)) =
                        MODES.iter().find(|m| (m.0 ^ code).count_ones() < 4)
                {
                    self.inverted = inverted;
                    self.baud = baud;
                    self.levels = levels;
                    self.fiw_syms.clear();
                    self.state = State::Fiw;
                }
            }
            State::Fiw => {
                self.fiw_syms.push(self.rectify(sym));
                if self.fiw_syms.len() == FIW_WINDOW {
                    self.decode_fiw();
                }
            }
            State::Sync2 { left } => {
                self.state = if left > 1 {
                    State::Sync2 { left: left - 1 }
                } else {
                    State::Data { count: 0 }
                };
            }
            State::Data { count } => {
                let s = self.rectify(sym);
                self.data_symbol(s, rel);
                let count = count + 1;
                if count >= self.baud * 1760 / 1000 {
                    self.decode_frame();
                    self.syncbuf = 0;
                    self.state = State::Sync1;
                } else {
                    self.state = State::Data { count };
                }
            }
        }
    }

    pub fn drain(&mut self, out: &mut Vec<Page>) {
        out.append(&mut self.pages);
    }

    /// End of input: report fragments still waiting for their ends.
    pub fn finish(&mut self) {
        let mut left: Vec<Fragment> = self.fragments.drain().map(|(_, f)| f).collect();
        left.sort_by_key(|f| f.page.capcode);
        for f in left {
            let mut p = f.page;
            p.text = f.text;
            p.flags.push_str(" incomplete");
            self.pages.push(p);
        }
        self.state = State::Sync1;
    }

    fn rectify(&self, sym: u8) -> u8 {
        if self.inverted { 3 - sym } else { sym }
    }

    fn decode_fiw(&mut self) {
        let mut tries = vec![FIW_OFFSET];
        for d in 1..=FIW_SLACK {
            tries.extend([FIW_OFFSET - d, FIW_OFFSET + d]);
        }
        for off in tries {
            let raw = self.fiw_syms[off..off + 32]
                .iter()
                .enumerate()
                .fold(0u32, |w, (k, &s)| w | (u32::from(s > 1) << k));
            let Some((fiw, _)) = fix_word(raw) else {
                continue;
            };
            if nibble_sum(fiw) != 0xF {
                continue;
            }
            self.cycle = ((fiw >> 4) & 0xF) as u8;
            self.frame = ((fiw >> 8) & 0x7F) as u8;
            // Symbols already taken past the FIW belong to sync 2, at 1600
            // baud: each is `baud / 1600` symbols of the frame's speed.
            let extra = (FIW_WINDOW - off - 32) as u32 * (self.baud / 1600);
            let left = self.baud * 25 / 1000 - extra;
            self.phases = [[0; WORDS]; 4];
            self.counter = 0;
            self.odd = false;
            self.state = if left > 0 {
                State::Sync2 { left }
            } else {
                State::Data { count: 0 }
            };
            return;
        }
        self.syncbuf = 0;
        self.state = State::Sync1;
    }

    /// Place one data symbol's bits in the phases (multimon-ng `read_data`).
    fn data_symbol(&mut self, s: u8, rel: [f32; 2]) {
        let a = s > 1;
        let b = self.levels == 4 && (s == 1 || s == 2);
        let c = self.counter;
        if c >= WORDS * 32 {
            return;
        }
        let idx = 8 * (c >> 8) + (c & 7);
        // The bit's place in its word once all 32 are in (first at bit 0).
        let m = (c >> 3) & 31;
        let (pa, pb) = match (self.baud, self.odd) {
            (1600, _) => (0, 2),
            (_, false) => (0, 1),
            (_, true) => (2, 3),
        };
        let put = |w: &mut u32, bit: bool| *w = (*w >> 1) | (u32::from(bit) << 31);
        put(&mut self.phases[pa][idx], a);
        put(&mut self.phases[pb][idx], b);
        self.rel[pa][idx][m] = rel[0];
        self.rel[pb][idx][m] = rel[1];
        if self.baud == 1600 || self.odd {
            self.counter += 1;
        }
        if self.baud != 1600 {
            self.odd = !self.odd;
        }
    }

    fn decode_frame(&mut self) {
        self.frames += 1;
        let active: &[usize] = match (self.baud, self.levels) {
            (1600, 2) => &[0],
            (1600, _) | (_, 2) => &[0, 2],
            _ => &[0, 1, 2, 3],
        };
        for &p in active {
            let words = self.phases[p];
            let rel = self.rel[p];
            self.decode_phase(&words, &rel, (b'A' + p as u8) as char);
        }
    }

    fn page(&self, capcode: u64, kind: Kind, phase: char) -> Page {
        let mut p = Page::new(Protocol::Flex, self.baud, self.levels, capcode, kind);
        p.slot = Some(FlexSlot {
            cycle: self.cycle,
            frame: self.frame,
            phase,
        });
        p
    }

    fn decode_phase(&mut self, raw: &[u32; WORDS], rel: &[[f32; 32]; WORDS], phase: char) {
        let mut w = [0u32; WORDS];
        let mut bad = [false; WORDS];
        let mut fixes = [0u32; WORDS];
        // Soft decoding beyond the hard decoder's two bits only in a phase
        // that is mostly sound: on a phase of noise (a frame found, its
        // words lost) it would turn every word into some codeword and the
        // address field into a list of false pages. A random word passes
        // the hard decoder about a quarter of the time.
        let hard = raw.iter().filter(|&&r| fix_word(r).is_some()).count();
        let trusted = hard * 10 >= WORDS * 6;
        for i in 0..WORDS {
            let soft = fix_word_soft(raw[i], &rel[i]).filter(|&(_, n)| n <= 2 || trusted);
            match soft {
                Some((v, n)) => {
                    w[i] = v;
                    fixes[i] = n;
                }
                None => {
                    w[i] = raw[i] & 0x1F_FFFF;
                    bad[i] = true;
                }
            }
        }
        let biw = w[0];
        if bad[0] || biw == 0 || biw == 0x1F_FFFF {
            return;
        }
        let aoffset = ((biw >> 8) & 3) as usize + 1;
        let voffset = ((biw >> 10) & 0x3F) as usize;
        if voffset < aoffset {
            return;
        }
        let priority = ((biw >> 4) & 0xF) as usize;
        for i in 1..aoffset {
            if !bad[i] {
                self.biw_word(w[i], phase);
            }
        }
        let n_vectors = (voffset - aoffset).min(WORDS - voffset);
        let mut used = 0;
        let mut i = aoffset;
        while i < voffset {
            let aw = w[i];
            let long = is_long(aw);
            let step = if long { 2 } else { 1 };
            let j = voffset + used;
            let mut fix = fixes[i];
            let addr = if bad[i] || aw == 0 || aw == 0x1F_FFFF {
                None
            } else if long {
                if i + 1 < voffset && !bad[i + 1] {
                    fix += fixes[i + 1];
                    long_capcode(aw, w[i + 1]).map(|c| (c, 'L'))
                } else {
                    None
                }
            } else {
                Some((u64::from(aw - 0x8000), short_type(aw)))
            };
            if let Some((capcode, atype)) = addr {
                let mut flags = atype.to_string();
                if i - aoffset < priority {
                    flags.push_str(" prio");
                }
                if used >= n_vectors || j >= WORDS {
                    if !long {
                        let mut p = self.page(capcode, Kind::Tone, phase);
                        p.flags = flags;
                        p.corrected_bits = fix;
                        self.pages.push(p);
                    }
                } else {
                    self.vector(&w, &bad, &fixes, j, long, capcode, flags, fix, phase);
                }
            }
            used += step;
            i += step;
        }
    }

    /// The extra BIW words: date, time, system information, SSIDs.
    fn biw_word(&mut self, v: u32, phase: char) {
        let text = match (v >> 4) & 7 {
            0 => format!(
                "SSID local id {} coverage {}",
                (v >> 7) & 0x1FF,
                (v >> 16) & 0x1F
            ),
            1 => {
                let (year, day, month) =
                    (1994 + ((v >> 7) & 0x1F), (v >> 12) & 0x1F, (v >> 17) & 0xF);
                if !(1..=12).contains(&month) || day == 0 {
                    return;
                }
                format!("date {year:04}-{month:02}-{day:02} (year mod 32)")
            }
            2 => {
                let (h, m, s) = ((v >> 7) & 0x1F, (v >> 12) & 0x3F, (v >> 18) & 7);
                if h > 23 || m > 59 {
                    return;
                }
                format!("time {h:02}:{m:02}:{:04.1}", s as f32 * 7.5)
            }
            5 => {
                let (a, info) = ((v >> 7) & 0xF, (v >> 11) & 0x3FF);
                if a == 4 || a == 8 {
                    format!("time zone {} dst {}", info & 0x1F, (info >> 5) & 1)
                } else {
                    format!("system info type {a} data 0x{info:03X}")
                }
            }
            7 => format!(
                "SSID traffic flags {:X} country {}",
                (v >> 7) & 0xF,
                (v >> 11) & 0x3FF
            ),
            t => format!("BIW reserved type {t} 0x{v:06X}"),
        };
        if self.system_words {
            let kind = if (v >> 4) & 7 == 1 || (v >> 4) & 7 == 2 {
                Kind::Time
            } else {
                Kind::System
            };
            let mut p = self.page(0, kind, phase);
            p.text = text;
            self.pages.push(p);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn vector(
        &mut self,
        w: &[u32; WORDS],
        bad: &[bool; WORDS],
        fixes: &[u32; WORDS],
        j: usize,
        long: bool,
        capcode: u64,
        mut flags: String,
        fix: u32,
        phase: char,
    ) {
        let viw = w[j];
        let vtype = (viw >> 4) & 7;
        let mut fix = fix + fixes[j];
        let group: Vec<u64> = if (GROUP_BASE..GROUP_BASE + 16).contains(&capcode) {
            flags.push_str(&format!(" G{}", capcode - GROUP_BASE));
            std::mem::take(&mut self.groups[(capcode - GROUP_BASE) as usize])
        } else {
            Vec::new()
        };
        // The second vector word of a long address.
        let vy = if long && j + 1 < WORDS {
            Some(j + 1)
        } else {
            None
        };
        let emit =
            |this: &mut Self, kind: Kind, text: String, flags: String, fix: u32, nbad: u32| {
                let mut p = this.page(capcode, kind, phase);
                p.text = text;
                p.flags = flags;
                p.corrected_bits = fix;
                p.bad_words = nbad + u32::from(bad[j]);
                p.group = group.clone();
                this.pages.push(p);
            };
        match vtype {
            1 => {
                let instr = (viw >> 7) & 7;
                let text = match instr {
                    0 => {
                        let (frame, g) = ((viw >> 10) & 0x7F, ((viw >> 17) & 0xF) as usize);
                        if !self.groups[g].contains(&capcode) {
                            self.groups[g].push(capcode);
                        }
                        format!("group {g} assigned, deliver in frame {frame}")
                    }
                    1 => format!("events 0x{:03X}", (viw >> 9) & 0x7FF),
                    t => format!("reserved instruction {t} 0x{:03X}", (viw >> 10) & 0x7FF),
                };
                emit(self, Kind::Instruction, text, flags, fix, 0);
            }
            2 => {
                let sub = (viw >> 7) & 3;
                match sub {
                    0 if flags.starts_with('N') => {
                        let text = format!(
                            "network id area {} multiplier {} traffic 0x{:X}",
                            (viw >> 9) & 0x1F,
                            (viw >> 14) & 7,
                            (viw >> 17) & 0xF
                        );
                        emit(self, Kind::ShortMessage, text, flags, fix, 0);
                    }
                    0 => {
                        let mut digits: Vec<u8> = [9, 13, 17]
                            .iter()
                            .map(|&s| ((viw >> s) & 0xF) as u8)
                            .collect();
                        let mut nbad = 0;
                        if let Some(y) = vy {
                            digits.extend((0..5).map(|k| ((w[y] >> (4 * k)) & 0xF) as u8));
                            fix += fixes[y];
                            nbad += u32::from(bad[y]);
                        }
                        let text: String = digits
                            .iter()
                            .filter(|&&d| d != FILL_DIGIT)
                            .map(|&d| BCD[d as usize] as char)
                            .collect();
                        let kind = if text.is_empty() {
                            Kind::Tone
                        } else {
                            Kind::ShortMessage
                        };
                        emit(self, kind, text, flags, fix, nbad);
                    }
                    1 => emit(
                        self,
                        Kind::ShortMessage,
                        format!("source {}", (viw >> 9) & 7),
                        flags,
                        fix,
                        0,
                    ),
                    2 => {
                        let text = format!(
                            "source {} message {} retrieval {}",
                            (viw >> 9) & 7,
                            (viw >> 12) & 0x3F,
                            (viw >> 18) & 1
                        );
                        emit(self, Kind::ShortMessage, text, flags, fix, 0);
                    }
                    _ => emit(
                        self,
                        Kind::ShortMessage,
                        format!("reserved 0x{:03X}", (viw >> 9) & 0xFFF),
                        flags,
                        fix,
                        0,
                    ),
                }
            }
            3 | 4 | 7 => {
                let start = ((viw >> 7) & 0x7F) as usize;
                let n = ((viw >> 14) & 7) as usize;
                let mut idx: Vec<usize> = Vec::new();
                match vy {
                    Some(y) => {
                        idx.push(y);
                        idx.extend(start..start + n);
                    }
                    None => idx.extend(start..=start + n),
                }
                idx.retain(|&k| k < WORDS);
                let skip = if vtype == 7 { 10 } else { 2 };
                let mut text = String::new();
                let mut digit = 0u8;
                let mut count = 4 + skip;
                let mut nbad = 0;
                for &k in &idx {
                    fix += fixes[k];
                    nbad += u32::from(bad[k]);
                    let mut dw = w[k];
                    for _ in 0..21 {
                        digit = (digit >> 1) | (((dw & 1) as u8) << 3);
                        dw >>= 1;
                        count -= 1;
                        if count == 0 {
                            if digit != FILL_DIGIT {
                                text.push(if bad[k] {
                                    '?'
                                } else {
                                    BCD[digit as usize] as char
                                });
                            }
                            count = 4;
                        }
                    }
                }
                let tag = match vtype {
                    4 => " special",
                    7 => " numbered",
                    _ => "",
                };
                flags.push_str(tag);
                emit(self, Kind::Numeric, text, flags, fix, nbad);
            }
            _ => {
                // Alphanumeric (5), secure (0) and binary (6): a header word,
                // then the message words.
                let mut mw1 = ((viw >> 7) & 0x7F) as usize;
                let mut len = ((viw >> 14) & 0x7F) as usize;
                let hdr = match vy {
                    Some(y) => y,
                    None => {
                        let h = mw1;
                        mw1 += 1;
                        h
                    }
                };
                len = len.saturating_sub(1);
                if mw1 >= WORDS || hdr >= WORDS {
                    return;
                }
                len = len.min(WORDS - mw1);
                let h = w[hdr];
                fix += fixes[hdr];
                let body = &w[mw1..mw1 + len];
                let body_bad = &bad[mw1..mw1 + len];
                fix += fixes[mw1..mw1 + len].iter().sum::<u32>();
                let nbad = body_bad.iter().filter(|&&b| b).count() as u32 + u32::from(bad[hdr]);
                let secure_bin = vtype == 0 && (h >> 19) & 3 != 0;
                if vtype == 6 || secure_bin {
                    let (frag, cont, num) = ((h >> 13) & 3, (h >> 12) & 1, (h >> 15) & 0x3F);
                    let data = if frag == 3 && !body.is_empty() { 1 } else { 0 };
                    let text = hex_text(&body[data..], &body_bad[data..], cont == 0);
                    flags.push_str(&format!(" N{num} {}", frag_flag(frag, cont)));
                    let kind = if vtype == 6 { Kind::Hex } else { Kind::Secure };
                    emit(self, kind, text, flags, fix, nbad);
                    return;
                }
                let (frag, cont, num) = ((h >> 11) & 3, (h >> 10) & 1, (h >> 13) & 0x3F);
                let k_ok = alpha_checksum(h, body) == h & 0x3FF;
                let mut text = String::new();
                for (n, (&dw, &b)) in body.iter().zip(body_bad).enumerate() {
                    if b {
                        text.push_str("???");
                        continue;
                    }
                    let first = if n == 0 && frag == 3 { 1 } else { 0 };
                    for s in first..3 {
                        let ch = ((dw >> (7 * s)) & 0x7F) as u8;
                        if ch != 0x03 && ch != 0 {
                            push_char(&mut text, ch);
                        }
                    }
                }
                flags.push_str(&format!(
                    " N{num} {} K{}",
                    frag_flag(frag, cont),
                    if k_ok { '+' } else { '-' }
                ));
                let kind = if vtype == 0 {
                    Kind::Secure
                } else {
                    Kind::Alpha
                };
                // Fragments: gather until the one with no continuation.
                let earlier = self.fragments.remove(&capcode);
                if frag == 3
                    && let Some(old) = earlier.as_ref()
                {
                    // A new message started before the old one finished.
                    let mut p = old.page.clone();
                    p.text = old.text.clone();
                    p.flags.push_str(" incomplete");
                    self.pages.push(p);
                }
                let text = match earlier {
                    Some(old) if frag != 3 => old.text + &text,
                    _ => text,
                };
                if cont == 1 {
                    let mut p = self.page(capcode, kind, phase);
                    p.flags = flags;
                    p.corrected_bits = fix;
                    p.bad_words = nbad;
                    p.group = group.clone();
                    self.fragments.insert(capcode, Fragment { text, page: p });
                } else {
                    emit(self, kind, text, flags, fix, nbad);
                }
            }
        }
    }
}

fn frag_flag(frag: u32, cont: u32) -> &'static str {
    match (frag, cont) {
        (3, 0) => "complete",
        (3, _) => "first",
        (_, 0) => "last",
        _ => "more",
    }
}

/// The alphanumeric check character (K, 10 bits) over the header (with its
/// K bits cleared) and the message words.
fn alpha_checksum(h: u32, body: &[u32]) -> u32 {
    let fold = |w: u32| (w & 0xFF) + ((w >> 8) & 0xFF) + ((w >> 16) & 0x1F);
    let sum = fold(h & !0x3FF) + body.iter().map(|&w| fold(w)).sum::<u32>();
    !sum & 0x3FF
}

/// Binary data: 21 bits a word, LSB first, as hex nibbles. A final word of
/// all zeros or all ones on the last fragment is fill.
fn hex_text(body: &[u32], bad: &[bool], last: bool) -> String {
    let mut n = body.len();
    if last && n > 1 && (body[n - 1] == 0 || body[n - 1] == 0x1F_FFFF) {
        n -= 1;
    }
    // (bit, word unreadable) for each bit of the stream.
    let mut bits: Vec<(bool, bool)> = Vec::with_capacity(n * 21);
    for (&w, &b) in body[..n].iter().zip(bad) {
        bits.extend((0..21).map(|k| ((w >> k) & 1 == 1, b)));
    }
    // Rust note: `as_chunks::<4>()` yields `&[_; 4]` arrays (the ragged
    // tail, under a nibble, is dropped).
    bits.as_chunks::<4>()
        .0
        .iter()
        .map(|c| {
            if c.iter().any(|x| x.1) {
                '?'
            } else {
                let v = c
                    .iter()
                    .enumerate()
                    .fold(0u32, |v, (i, x)| v | (u32::from(x.0) << i));
                char::from_digit(v, 16).unwrap_or('?').to_ascii_uppercase()
            }
        })
        .collect()
}

/// Decode a 64-bit sync 1 window: the A code if the marker and the A/Ā pair
/// both match to within three bits.
fn sync_code(buf: u64) -> Option<u16> {
    let marker = (buf >> 16) as u32;
    let hi = (buf >> 48) as u16;
    let lo = !(buf as u16);
    ((marker ^ MARKER).count_ones() < 4 && (hi ^ lo).count_ones() < 4).then_some(hi)
}

/// BCH-correct an LSB-first FLEX word; returns its 21 data bits.
pub fn fix_word(raw: u32) -> Option<(u32, u32)> {
    bch::decode(raw.reverse_bits()).map(|(c, n)| (c.reverse_bits() & 0x1F_FFFF, n))
}

/// As [`fix_word`], with each bit's reliability (bit 0 first on air):
/// soft-decision decoding.
pub fn fix_word_soft(raw: u32, rel: &[f32; 32]) -> Option<(u32, u32)> {
    let rel_tx: [f32; 32] = std::array::from_fn(|i| rel[31 - i]);
    bch::decode_soft(raw.reverse_bits(), &rel_tx).map(|(c, n)| (c.reverse_bits() & 0x1F_FFFF, n))
}

/// Build an LSB-first FLEX word from 21 data bits.
pub fn make_word(data: u32) -> u32 {
    bch::encode((data & 0x1F_FFFF).reverse_bits() >> 11).reverse_bits()
}

/// The FIW/BIW checksum: the nibbles of the 21 bits (the last is one bit)
/// add to 0xF.
fn nibble_sum(v: u32) -> u32 {
    ((v & 0xF)
        + ((v >> 4) & 0xF)
        + ((v >> 8) & 0xF)
        + ((v >> 12) & 0xF)
        + ((v >> 16) & 0xF)
        + ((v >> 20) & 1))
        & 0xF
}

/// Set bits 0–3 so the nibble sum is 0xF.
fn with_checksum(v: u32) -> u32 {
    let v = v & !0xF;
    v | ((0xF - nibble_sum(v)) & 0xF)
}

/// First word of a long address.
fn is_long(aw: u32) -> bool {
    (1..=0x8000).contains(&aw) || (0x1F_7FFF..=0x1F_FFFE).contains(&aw)
}

/// Short-address type letter by range: S short, R reserved, I information
/// service, N network, T temporary (group), O operator messaging.
fn short_type(aw: u32) -> char {
    match aw {
        0x1F_0001..=0x1F_27FF => 'R',
        0x1F_2800..=0x1F_67FF => 'I',
        0x1F_6800..=0x1F_77FF => 'N',
        0x1F_7800..=0x1F_780F => 'T',
        0x1F_7810..=0x1F_781F => 'O',
        0x1F_7820..=0x1F_7FFE => 'R',
        _ => 'S',
    }
}

/// Long-address capcode from its two address words (sets 1-2, 1-3/1-4 and
/// 2-3/2-4).
fn long_capcode(w1: u32, w2: u32) -> Option<u64> {
    let (w1, w2) = (u64::from(w1), u64::from(w2));
    let c = if (1..=32_768).contains(&w1) && (2_064_383..=2_097_150).contains(&w2) {
        w1 + (2_097_151 - w2) * 32_768 + 2_068_480
    } else if (1..=32_768).contains(&w1) && (1_966_081..=2_031_616).contains(&w2) {
        w1 + (w2 - 1_933_312) * 32_768 + 2_068_480
    } else if (2_064_383..=2_097_150).contains(&w1) && (1_966_081..=2_031_616).contains(&w2) {
        (w1 - 2_064_383) + (w2 - 1_867_776) * 32_768 + 2_068_479
    } else {
        return None;
    };
    (c <= 4_297_068_542).then_some(c)
}

// ---------------------------------------------------------------- encoder

/// The four FLEX speeds as (symbol rate, levels).
pub const SPEEDS: [(u32, u8); 4] = [(1600, 2), (3200, 2), (1600, 4), (3200, 4)];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlexBody {
    /// Short-message vector with all-fill digits.
    Tone,
    /// Short-message vector: up to 3 digits (8 with a long address).
    ShortNumeric(String),
    Numeric(String),
    Alpha(String),
    Hex(Vec<u8>),
    /// Short instruction: enrol this pager in group `group` for `frame`.
    GroupAssign {
        group: u8,
        frame: u8,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlexPage {
    pub capcode: u64,
    pub body: FlexBody,
}

/// A frame's symbols: sync 1 and FIW at 1600 baud 2-level, then sync 2 and
/// the data at `baud`. Symbols 0–3, lowest frequency first.
pub struct FlexFrame {
    pub head: Vec<u8>,
    pub baud: u32,
    pub levels: u8,
    pub data: Vec<u8>,
}

/// Address words for a capcode: one for a short address, two for a long.
fn address_words(capcode: u64) -> Vec<u32> {
    if (1..=1_933_312).contains(&capcode) || (GROUP_BASE..GROUP_BASE + 16).contains(&capcode) {
        vec![(capcode + 0x8000) as u32]
    } else {
        // Set 1-2.
        let c = capcode - 2_068_480;
        let w1 = (c - 1) % 32_768 + 1;
        let q = (c - w1) / 32_768;
        if (1..=32_768).contains(&q) {
            // Set 1-2.
            vec![w1 as u32, (2_097_151 - q) as u32]
        } else if (32_769..=98_304).contains(&q) {
            // Set 1-3/1-4.
            vec![w1 as u32, (q + 1_933_312) as u32]
        } else {
            // Set 2-3/2-4.
            let c = capcode - 2_068_479;
            let q = c / 32_768;
            assert!(
                (98_305..=163_840).contains(&q),
                "capcode {capcode} out of range"
            );
            vec![(c % 32_768 + 2_064_383) as u32, (q + 1_867_776) as u32]
        }
    }
}

fn vector_word(vtype: u32, rest: u32) -> u32 {
    with_checksum((vtype << 4) | (rest << 7))
}

/// Digits as a stream of 4-bit LSB-first groups after `skip` zero bits,
/// packed into 21-bit words, padded with fill digits.
fn digit_words(s: &str, skip: usize, words: usize) -> Vec<u32> {
    let mut bits: Vec<bool> = vec![false; skip];
    for c in s.bytes() {
        let d = BCD
            .iter()
            .position(|&b| b == c)
            .unwrap_or(FILL_DIGIT as usize) as u8;
        bits.extend((0..4).map(|i| (d >> i) & 1 == 1));
    }
    while bits.len() < words * 21 {
        bits.extend((0..4).map(|i| (FILL_DIGIT >> i) & 1 == 1));
    }
    bits.truncate(words * 21);
    bits.chunks(21)
        .map(|c| {
            c.iter()
                .enumerate()
                .fold(0, |v, (i, &b)| v | (u32::from(b) << i))
        })
        .collect()
}

/// Encode one phase: BIW, addresses, vectors, messages, idle fill. Returns
/// `None` if the pages do not fit.
pub fn encode_phase(pages: &[FlexPage]) -> Option<[u32; WORDS]> {
    let mut addrs: Vec<u32> = Vec::new();
    for p in pages {
        addrs.extend(address_words(p.capcode));
    }
    let voffset = 1 + addrs.len();
    let mut msg_at = voffset + addrs.len();
    let mut vectors: Vec<u32> = Vec::new();
    let mut messages: Vec<u32> = Vec::new();
    for p in pages {
        let long = address_words(p.capcode).len() == 2;
        // Second vector word of a long address, if it carries anything.
        let mut vy: Option<u32> = None;
        let v = match &p.body {
            FlexBody::Tone => vector_word(2, 0xC << 2 | 0xC << 6 | 0xC << 10),
            FlexBody::ShortNumeric(s) => {
                let mut d: Vec<u32> = s
                    .bytes()
                    .map(|c| BCD.iter().position(|&b| b == c).unwrap_or(12) as u32)
                    .collect();
                d.resize(8, u32::from(FILL_DIGIT));
                if long {
                    vy = Some((0..5).fold(0, |v, k| v | (d[3 + k] << (4 * k))));
                }
                vector_word(2, (d[0] | d[1] << 4 | d[2] << 8) << 2)
            }
            FlexBody::Numeric(s) => {
                let words = (2 + 4 * s.len()).div_ceil(21).clamp(1, 8);
                let mut d = digit_words(s, 2, words);
                let start = msg_at;
                if long {
                    vy = Some(d.remove(0));
                }
                let n = if long { d.len() } else { d.len() - 1 } as u32;
                msg_at += d.len();
                messages.extend(d);
                vector_word(3, start as u32 | (n << 7))
            }
            FlexBody::Alpha(s) => {
                let chars: Vec<u8> = s.bytes().map(|c| c & 0x7F).collect();
                let sig = !chars.iter().map(|&c| u32::from(c)).sum::<u32>() & 0x7F;
                let mut seq = vec![sig as u8];
                seq.extend(&chars);
                while !seq.len().is_multiple_of(3) {
                    seq.push(0x03);
                }
                let body: Vec<u32> = seq
                    .chunks(3)
                    .map(|c| u32::from(c[0]) | (u32::from(c[1]) << 7) | (u32::from(c[2]) << 14))
                    .collect();
                // Header: fragment 3 (initial), no continuation, message 0.
                let h0 = 3 << 11;
                let h = h0 | alpha_checksum(h0, &body);
                let start = msg_at;
                let v = if long {
                    vy = Some(h);
                    vector_word(5, start as u32 | ((body.len() as u32 + 1) << 7))
                } else {
                    messages.push(h);
                    msg_at += 1;
                    vector_word(5, start as u32 | ((body.len() as u32 + 1) << 7))
                };
                msg_at += body.len();
                messages.extend(body);
                v
            }
            FlexBody::Hex(bytes) => {
                let mut bits: Vec<bool> = Vec::new();
                for &b in bytes {
                    for nib in [b >> 4, b & 0xF] {
                        bits.extend((0..4).map(|i| (nib >> i) & 1 == 1));
                    }
                }
                let body: Vec<u32> = bits
                    .chunks(21)
                    .map(|c| {
                        c.iter()
                            .enumerate()
                            .fold(0, |v, (i, &b)| v | (u32::from(b) << i))
                    })
                    .collect();
                let h = 3 << 13;
                let hdr2 = 0u32;
                let start = msg_at;
                let count = body.len() as u32 + 2; // header, hdr2, data
                if long {
                    vy = Some(h);
                } else {
                    messages.push(h);
                    msg_at += 1;
                }
                messages.push(hdr2);
                messages.extend(&body);
                msg_at += 1 + body.len();
                vector_word(6, start as u32 | (count << 7))
            }
            FlexBody::GroupAssign { group, frame } => vector_word(
                1,
                (u32::from(*frame) & 0x7F) << 3 | (u32::from(*group) & 0xF) << 10,
            ),
        };
        vectors.push(v);
        if long {
            vectors.push(vy.unwrap_or(0));
        }
    }
    if msg_at > WORDS {
        return None;
    }
    let mut w = [0x1F_FFFFu32; WORDS];
    w[0] = with_checksum((voffset as u32) << 10);
    for (slot, v) in w[1..]
        .iter_mut()
        .zip(addrs.into_iter().chain(vectors).chain(messages))
    {
        *slot = v;
    }
    Some(w.map(make_word))
}

/// Encode a frame at one speed. Pages are dealt to the phases in turn.
pub fn encode_frame(
    baud: u32,
    levels: u8,
    cycle: u8,
    frame: u8,
    pages: &[FlexPage],
) -> Option<FlexFrame> {
    let code = MODES.iter().find(|m| m.1 == baud && m.2 == levels)?.0;
    let nphases = match (baud, levels) {
        (1600, 2) => 1,
        (3200, 4) => 4,
        _ => 2,
    };
    // Phase slots in use for this speed (A, or A+C, or A–D).
    let slots: &[usize] = match nphases {
        1 => &[0],
        2 => &[0, 2],
        _ => &[0, 1, 2, 3],
    };
    let mut per: Vec<Vec<FlexPage>> = vec![Vec::new(); 4];
    for (n, p) in pages.iter().enumerate() {
        per[slots[n % nphases]].push(p.clone());
    }
    let mut phases = [[make_word(0x1F_FFFF); WORDS]; 4];
    for &s in slots {
        if !per[s].is_empty() {
            phases[s] = encode_phase(&per[s])?;
        } else {
            // An idle phase: a BIW with no addresses.
            phases[s][0] = make_word(with_checksum(1 << 10));
        }
    }

    // Sync 1: bit sync, A code, marker, Ā; sync bits are 1 = low.
    let mut head: Vec<u8> = (0..33).map(|i| if i % 2 == 0 { 3 } else { 0 }).collect();
    let sync = (u64::from(code) << 48) | (u64::from(MARKER) << 16) | u64::from(!code);
    head.extend(
        (0..64)
            .rev()
            .map(|i| if (sync >> i) & 1 == 1 { 0 } else { 3 }),
    );
    head.extend((0..FIW_OFFSET).map(|i| if i % 2 == 0 { 3 } else { 0 }));
    let fiw = make_word(with_checksum(
        u32::from(cycle & 0xF) << 4 | u32::from(frame & 0x7F) << 8,
    ));
    head.extend((0..32).map(|k| if (fiw >> k) & 1 == 1 { 3 } else { 0 }));

    // Sync 2: comma patterns C then C̄ at the frame's speed.
    let s2 = (baud * 25 / 1000) as usize;
    let c: u16 = 0xED84;
    let mut data: Vec<u8> = (0..s2)
        .map(|i| {
            let bit = (c >> (15 - i % 16)) & 1 == 1;
            let bit = if i < s2 / 2 { bit } else { !bit };
            if bit { 3 } else { 0 }
        })
        .collect();

    let sym = |a: bool, b: bool| match (a, b) {
        (true, false) => 3,
        (true, true) => 2,
        (false, true) => 1,
        (false, false) => 0,
    };
    let bit = |p: usize, c: usize| (phases[p][8 * (c >> 8) + (c & 7)] >> ((c >> 3) & 31)) & 1 == 1;
    for c in 0..WORDS * 32 {
        match (baud, levels) {
            (1600, 2) => data.push(sym(bit(0, c), false)),
            (1600, _) => data.push(sym(bit(0, c), bit(2, c))),
            (_, 2) => {
                data.push(sym(bit(0, c), false));
                data.push(sym(bit(2, c), false));
            }
            _ => {
                data.push(sym(bit(0, c), bit(1, c)));
                data.push(sym(bit(2, c), bit(3, c)));
            }
        }
    }
    Some(FlexFrame {
        head,
        baud,
        levels,
        data,
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn sample_pages() -> Vec<FlexPage> {
        vec![
            FlexPage {
                capcode: 1_234_567,
                body: FlexBody::Alpha("FLEX alpha test, all modes.".into()),
            },
            FlexPage {
                capcode: 100_001,
                body: FlexBody::Numeric("0123456789-".into()),
            },
            FlexPage {
                capcode: 77,
                body: FlexBody::Tone,
            },
            FlexPage {
                capcode: 4_000_000_000,
                body: FlexBody::Alpha("long address".into()),
            },
            FlexPage {
                capcode: 555,
                body: FlexBody::ShortNumeric("42".into()),
            },
            FlexPage {
                capcode: 3_000_000,
                body: FlexBody::Numeric("31337".into()),
            },
            FlexPage {
                capcode: 999,
                body: FlexBody::Hex(vec![0xDE, 0xAD, 0xBE, 0xEF]),
            },
            FlexPage {
                capcode: 4242,
                body: FlexBody::GroupAssign { group: 5, frame: 3 },
            },
        ]
    }

    pub(crate) fn check(pages: &[Page]) {
        let find = |c: u64| {
            pages
                .iter()
                .find(|p| p.capcode == c)
                .unwrap_or_else(|| panic!("no {c} in {pages:#?}"))
        };
        assert_eq!(find(1_234_567).text, "FLEX alpha test, all modes.");
        assert!(
            find(1_234_567).flags.contains("K+"),
            "{:?}",
            find(1_234_567)
        );
        assert_eq!(find(100_001).text, "0123456789-");
        assert_eq!(find(77).kind, Kind::Tone);
        assert_eq!(find(4_000_000_000).text, "long address");
        assert_eq!(find(4_000_000_000).flags.chars().next(), Some('L'));
        assert_eq!(find(555).text, "42");
        assert_eq!(find(3_000_000).text, "31337");
        assert!(find(999).text.starts_with("DEADBEEF"), "{:?}", find(999));
        assert_eq!(find(4242).kind, Kind::Instruction);
    }

    /// Feed a frame as a slicer would: the symbols on a 3200-baud time
    /// grid, picked at the rate the decoder asks for.
    fn run(frame: &FlexFrame, invert: bool) -> Vec<Page> {
        let mut ticks: Vec<u8> = Vec::new();
        // Some noise first.
        for s in [0u8, 3, 3, 0, 3, 0, 0, 0, 3].iter().chain(&frame.head) {
            ticks.extend([*s, *s]);
        }
        for &s in &frame.data {
            ticks.extend(std::iter::repeat_n(s, (3200 / frame.baud) as usize));
        }
        let mut d = FlexDecoder::new();
        let mut t = 0;
        while t < ticks.len() {
            let step = (3200 / d.wanted().0) as usize;
            let s = ticks[t];
            d.push(if invert { 3 - s } else { s }, [1.0; 2]);
            t += step;
        }
        d.finish();
        let mut out = Vec::new();
        d.drain(&mut out);
        out
    }

    #[test]
    fn word_and_capcode_helpers() {
        for v in [0, 1, 0x1F_FFFF, 0x12_3456] {
            assert_eq!(fix_word(make_word(v)), Some((v, 0)));
            assert_eq!(fix_word(make_word(v) ^ 0x8001), Some((v, 2)));
        }
        for c in [
            2_101_249u64,
            3_000_000,
            1_500_000_000,
            4_000_000_000,
            4_297_068_542,
        ] {
            let w = address_words(c);
            assert!(is_long(w[0]));
            assert_eq!(long_capcode(w[0], w[1]), Some(c));
        }
    }

    #[test]
    fn every_speed_round_trips_at_symbol_level() {
        for (baud, levels) in SPEEDS {
            for invert in [false, true] {
                let f = encode_frame(baud, levels, 3, 77, &sample_pages()).unwrap();
                let pages = run(&f, invert);
                check(&pages);
                let s = pages[0].slot.unwrap();
                assert_eq!((s.cycle, s.frame), (3, 77));
                assert_eq!((pages[0].baud, pages[0].levels), (baud, levels));
            }
        }
    }

    #[test]
    fn group_call_lists_its_members() {
        let mut pages = sample_pages();
        pages.push(FlexPage {
            capcode: GROUP_BASE + 5,
            body: FlexBody::Alpha("to the group".into()),
        });
        let f = encode_frame(1600, 2, 0, 0, &pages).unwrap();
        let out = run(&f, false);
        let g = out.iter().find(|p| p.capcode == GROUP_BASE + 5).unwrap();
        assert_eq!(g.group, vec![4242]);
        assert_eq!(g.text, "to the group");
    }
}
