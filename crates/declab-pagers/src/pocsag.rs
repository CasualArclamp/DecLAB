// made by CasualArclamp
//! POCSAG, ITU-R M.584-2 Annex 1 ("Radiopaging Code No. 1"), at 512, 1200
//! and 2400 bit/s.
//!
//! A transmission is a preamble of at least 576 alternating bits, then
//! batches: a sync codeword and eight frames of two codewords. A pager with
//! address A listens in frame A mod 8. Address codewords (bit 31 = 0) carry
//! the top 18 address bits and two function bits; message codewords
//! (bit 31 = 1) carry 20 bits each and run on through frames and batches
//! until the next address or idle codeword.
//!
//! Binary 1 is the lower frequency (§4.1); the decoder accepts either
//! polarity, since receivers and discriminator taps differ.

use crate::{Kind, Page, Protocol, bch, push_char};

/// Sync codeword (§4.3).
pub const SYNC: u32 = 0x7CD2_15D8;
/// Idle codeword (§4.6).
pub const IDLE: u32 = 0x7A89_C197;
/// The bit rates in use.
pub const RATES: [u32; 3] = [512, 1200, 2400];

/// Bit errors allowed when looking for the first sync codeword, and when
/// checking that the next batch follows.
const SYNC_SEARCH_ERRS: u32 = 2;
const SYNC_KEEP_ERRS: u32 = 5;

/// Numeric characters (§4.5.1, Table 2): 0xA spare, 0xB urgent, 0xC space,
/// 0xD hyphen, 0xE and 0xF the brackets.
const BCD: &[u8; 16] = b"0123456789*U -][";

/// How to read message bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Charset {
    /// From the function bits (0 numeric, 3 alphanumeric), and by content
    /// for 1 and 2 — the usual network convention, not part of the code.
    #[default]
    Auto,
    Numeric,
    Alpha,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Looking for a sync codeword in either polarity.
    Search,
    /// Inside a batch: `word` codewords done, `bits` bits of the next.
    Batch { word: u8, bits: u8 },
    /// After a batch: the next 32 bits should be a sync codeword.
    NextSync { bits: u8 },
}

/// The message being gathered after an address codeword.
struct Pending {
    capcode: u64,
    function: u8,
    /// Message bits in order on air, 20 per codeword.
    bits: Vec<bool>,
    corrected: u32,
    bad: u32,
}

pub struct PocsagDecoder {
    baud: u32,
    charset: Charset,
    state: State,
    /// Last 32 bits as received (before polarity).
    raw: u32,
    /// Bits of the codeword in progress (after polarity), and how sure
    /// the demodulator was of each (index = bit, 31 first).
    word: u32,
    rel: [f32; 32],
    inverted: bool,
    pending: Option<Pending>,
    pages: Vec<Page>,
    /// Codewords since the transmission's first sync, and how many of them
    /// the hard decoder alone could repair: the test of whether the sync
    /// was real and whether soft decoding may be trusted.
    tx_words: u32,
    tx_hard: u32,
    /// Codewords seen since sync, and how many needed correcting, for the
    /// status line.
    pub codewords: u64,
    pub corrected_words: u64,
    pub bad_words: u64,
}

impl PocsagDecoder {
    pub fn new(baud: u32) -> Self {
        PocsagDecoder {
            baud,
            charset: Charset::Auto,
            state: State::Search,
            raw: 0,
            word: 0,
            rel: [1.0; 32],
            inverted: false,
            pending: None,
            pages: Vec::new(),
            tx_words: 0,
            tx_hard: 0,
            codewords: 0,
            corrected_words: 0,
            bad_words: 0,
        }
    }

    pub fn set_charset(&mut self, c: Charset) {
        self.charset = c;
    }

    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// In a batch, so the slicer may track slowly.
    pub fn in_sync(&self) -> bool {
        self.state != State::Search
    }

    /// Feed one symbol decision: `high` is the upper frequency, `rel` how
    /// sure the demodulator was (any positive scale).
    pub fn push(&mut self, high: bool, rel: f32) {
        // Binary 1 is the lower frequency.
        let raw_bit = !high;
        self.raw = (self.raw << 1) | u32::from(raw_bit);
        let bit = raw_bit ^ self.inverted;
        match self.state {
            State::Search => {
                if let Some(inv) = self.sync_here() {
                    self.start(inv);
                }
            }
            // A sync where none is due, while what came since the last one
            // does not decode: the last one was noise, this may be real.
            State::Batch { .. } | State::NextSync { .. }
                if !self.trusted() && self.sync_here().is_some() =>
            {
                self.pending = None;
                let inv = self.sync_here().unwrap_or(false);
                self.start(inv);
            }
            State::Batch { word, bits } => {
                self.word = (self.word << 1) | u32::from(bit);
                self.rel[31 - bits as usize] = rel;
                if bits < 31 {
                    self.state = State::Batch {
                        word,
                        bits: bits + 1,
                    };
                } else if !self.codeword(word, self.word) {
                    // Too little decodes: a false sync.
                    self.pending = None;
                    self.state = State::Search;
                } else {
                    self.state = if word < 15 {
                        State::Batch {
                            word: word + 1,
                            bits: 0,
                        }
                    } else {
                        State::NextSync { bits: 0 }
                    };
                }
            }
            State::NextSync { bits } => {
                self.word = (self.word << 1) | u32::from(bit);
                if bits < 31 {
                    self.state = State::NextSync { bits: bits + 1 };
                } else if (self.word ^ SYNC).count_ones() <= SYNC_KEEP_ERRS {
                    self.state = State::Batch { word: 0, bits: 0 };
                } else {
                    // End of the transmission.
                    self.flush();
                    self.state = State::Search;
                }
            }
        }
    }

    /// The sync codeword ends here: in which polarity.
    fn sync_here(&self) -> Option<bool> {
        if (self.raw ^ SYNC).count_ones() <= SYNC_SEARCH_ERRS {
            Some(false)
        } else if (!self.raw ^ SYNC).count_ones() <= SYNC_SEARCH_ERRS {
            Some(true)
        } else {
            None
        }
    }

    fn start(&mut self, inverted: bool) {
        self.inverted = inverted;
        self.state = State::Batch { word: 0, bits: 0 };
        self.tx_words = 0;
        self.tx_hard = 0;
    }

    /// Enough of this transmission hard-decodes that it is real (a random
    /// word is within two bits of a codeword about a quarter of the time).
    fn trusted(&self) -> bool {
        self.tx_words >= 3 && self.tx_hard * 10 >= self.tx_words * 7
    }

    /// Take the pages finished so far.
    pub fn drain(&mut self, out: &mut Vec<Page>) {
        out.append(&mut self.pages);
    }

    /// End of input: finish any message in progress.
    pub fn finish(&mut self) {
        self.flush();
        self.state = State::Search;
    }

    /// Handle one codeword; `false` when the sync it follows looks false.
    fn codeword(&mut self, index: u8, w: u32) -> bool {
        self.codewords += 1;
        self.tx_words += 1;
        if bch::decode(w).is_some() {
            self.tx_hard += 1;
        }
        if self.tx_words == 3 && self.tx_hard < 2 {
            return false;
        }
        // Soft decoding past the hard decoder's two bits only inside a
        // transmission that is clearly real: on noise it would always find
        // a codeword.
        let soft = bch::decode_soft(w, &self.rel).filter(|&(_, n)| n <= 2 || self.trusted());
        let Some((w, fixed)) = soft else {
            self.bad_words += 1;
            // A message word that cannot be repaired still holds its place;
            // an address word is lost.
            if let Some(p) = &mut self.pending
                && w >> 31 == 1
            {
                p.bad += 1;
                p.bits.extend((0..20).map(|i| (w >> (30 - i)) & 1 == 1));
            }
            return true;
        };
        if fixed > 0 {
            self.corrected_words += 1;
        }
        if w == IDLE {
            self.flush();
        } else if w >> 31 == 0 {
            self.flush();
            let frame = u64::from(index / 2);
            self.pending = Some(Pending {
                capcode: (u64::from((w >> 13) & 0x3_FFFF) << 3) | frame,
                function: ((w >> 11) & 3) as u8,
                bits: Vec::new(),
                corrected: fixed,
                bad: 0,
            });
        } else if let Some(p) = &mut self.pending {
            p.corrected += fixed;
            p.bits.extend((0..20).map(|i| (w >> (30 - i)) & 1 == 1));
        }
        true
    }

    fn flush(&mut self) {
        let Some(p) = self.pending.take() else { return };
        let numeric = numeric_text(&p.bits);
        let kind = if p.bits.is_empty() {
            Kind::Tone
        } else {
            match self.charset {
                Charset::Numeric => Kind::Numeric,
                Charset::Alpha => Kind::Alpha,
                Charset::Auto => match p.function {
                    0 => Kind::Numeric,
                    3 => Kind::Alpha,
                    // Function 1 and 2 are tone or numeric on most networks;
                    // a long message is alphanumeric.
                    _ if p.bits.len() <= 40 && numeric.is_some() => Kind::Numeric,
                    _ => Kind::Alpha,
                },
            }
        };
        let mut page = Page::new(Protocol::Pocsag, self.baud, 2, p.capcode, kind);
        page.function = Some(p.function);
        page.corrected_bits = p.corrected;
        page.bad_words = p.bad;
        page.text = match kind {
            Kind::Numeric => numeric.unwrap_or_else(|| numeric_lossy(&p.bits)),
            Kind::Alpha => alpha_text(&p.bits),
            _ => String::new(),
        };
        self.pages.push(page);
    }
}

/// Bits taken `n` at a time, the first on air as the least significant.
fn lsb_first(bits: &[bool], n: usize) -> impl Iterator<Item = u8> + '_ {
    bits.chunks_exact(n).map(|c| {
        c.iter()
            .enumerate()
            .fold(0u8, |v, (i, &b)| v | (u8::from(b) << i))
    })
}

fn numeric_lossy(bits: &[bool]) -> String {
    let s: String = lsb_first(bits, 4)
        .map(|d| BCD[d as usize] as char)
        .collect();
    s.trim_end().to_string()
}

/// Numeric text, or `None` if it uses the spare code (a sign that the page
/// is really alphanumeric).
fn numeric_text(bits: &[bool]) -> Option<String> {
    if lsb_first(bits, 4).any(|d| d == 0xA) {
        return None;
    }
    Some(numeric_lossy(bits))
}

fn alpha_text(bits: &[bool]) -> String {
    let mut s = String::new();
    let chars: Vec<u8> = lsb_first(bits, 7).collect();
    // Trailing NUL/ETX/EOT padding ends the message.
    let end = chars
        .iter()
        .rposition(|&c| !matches!(c, 0 | 3 | 4))
        .map_or(0, |i| i + 1);
    for &c in &chars[..end] {
        push_char(&mut s, c);
    }
    s
}

// ---------------------------------------------------------------- encoder

/// One page to send.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PocsagMessage {
    /// 21-bit address (0..=2_097_151).
    pub capcode: u32,
    pub function: u8,
    pub body: Body,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Body {
    Tone,
    Numeric(String),
    Alpha(String),
}

fn message_bits(body: &Body) -> Vec<bool> {
    let mut bits = Vec::new();
    let mut push = |v: u8, n: usize| bits.extend((0..n).map(|i| (v >> i) & 1 == 1));
    match body {
        Body::Tone => {}
        Body::Numeric(s) => {
            for c in s.bytes() {
                let d = BCD.iter().position(|&b| b == c).unwrap_or(0xC) as u8;
                push(d, 4);
            }
        }
        Body::Alpha(s) => {
            for c in s.bytes() {
                push(c & 0x7F, 7);
            }
            push(0x04, 7); // EOT
        }
    }
    // Pad to whole codewords: spaces for numeric, ETX/NUL for alpha.
    let pad = match body {
        Body::Numeric(_) => [false, false, true, true],
        _ => [false; 4],
    };
    let mut i = 0;
    while !bits.len().is_multiple_of(20) {
        bits.push(pad[i % 4]);
        i += 1;
    }
    bits
}

/// The bits of one transmission (preamble and batches), first on air first,
/// with binary 1 as `true`.
pub fn encode(messages: &[PocsagMessage]) -> Vec<bool> {
    // Codeword slots, 16 per batch.
    let mut words: Vec<u32> = Vec::new();
    for m in messages {
        let frame = (m.capcode & 7) as usize;
        // Idle until the pager's frame comes round.
        while words.len() % 16 != frame * 2 {
            words.push(IDLE);
        }
        let addr = ((m.capcode >> 3) & 0x3_FFFF) << 2 | u32::from(m.function & 3);
        words.push(bch::encode(addr));
        for chunk in message_bits(&m.body).chunks(20) {
            let data = chunk.iter().fold(0u32, |v, &b| (v << 1) | u32::from(b));
            words.push(bch::encode((1 << 20) | data));
        }
    }
    // Separate the last message from what follows, then fill the batch.
    words.push(IDLE);
    while !words.len().is_multiple_of(16) {
        words.push(IDLE);
    }
    let mut bits: Vec<bool> = (0..576).map(|i| i % 2 == 0).collect();
    for batch in words.chunks(16) {
        for &w in std::iter::once(&SYNC).chain(batch) {
            bits.extend((0..32).rev().map(|i| (w >> i) & 1 == 1));
        }
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_bits(bits: &[bool], invert: bool) -> Vec<Page> {
        let mut d = PocsagDecoder::new(1200);
        for &b in bits {
            // On air a 1 is the lower frequency.
            d.push(!b ^ invert, 1.0);
        }
        d.finish();
        let mut out = Vec::new();
        d.drain(&mut out);
        out
    }

    fn sample() -> Vec<PocsagMessage> {
        vec![
            PocsagMessage {
                capcode: 1_234_567,
                function: 3,
                body: Body::Alpha("Hello, pager world! 42".into()),
            },
            PocsagMessage {
                capcode: 8,
                function: 0,
                body: Body::Numeric("0123456789-U []".into()),
            },
            PocsagMessage {
                capcode: 2_097_151,
                function: 1,
                body: Body::Tone,
            },
        ]
    }

    #[test]
    fn round_trip_both_polarities() {
        for invert in [false, true] {
            let pages = decode_bits(&encode(&sample()), invert);
            assert_eq!(pages.len(), 3, "{pages:?}");
            assert_eq!(pages[0].capcode, 1_234_567);
            assert_eq!(pages[0].kind, Kind::Alpha);
            assert_eq!(pages[0].text, "Hello, pager world! 42");
            assert_eq!(pages[1].capcode, 8);
            assert_eq!(pages[1].text, "0123456789-U []");
            assert_eq!(pages[2].kind, Kind::Tone);
            assert_eq!(pages[2].function, Some(1));
        }
    }

    #[test]
    fn two_errors_per_word_are_corrected() {
        let mut bits = encode(&sample());
        // Two errors in every codeword after the preamble and first sync.
        let mut i = 576 + 32 + 3;
        while i + 20 < bits.len() {
            bits[i] ^= true;
            bits[i + 17] ^= true;
            i += 32;
        }
        let pages = decode_bits(&bits, false);
        assert_eq!(pages[0].text, "Hello, pager world! 42");
        assert!(pages[0].corrected_bits > 0);
    }
}
