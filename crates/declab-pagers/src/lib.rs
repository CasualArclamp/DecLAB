// made by CasualArclamp
//! Pager protocols, copied from DecPagers (`..\DecPagers`, crate
//! `decpagers-proto`); change them there first and copy them back.
//!
//! - [`bch`]: the BCH(31,21)+parity codeword code both protocols use.
//! - [`pocsag`]: POCSAG (ITU-R M.584-2) at 512, 1200 and 2400 bit/s —
//!   numeric, alphanumeric and tone-only pages. Decoder and encoder.
//! - [`flex`]: FLEX at 1600/2, 3200/2, 3200/4 and 6400/4 — every vector type
//!   (alphanumeric, standard/special/numbered numeric, tone and short
//!   message, binary, secure, short instruction), long addresses, group
//!   calls, fragments, and the BIW date/time/system words. Decoder and
//!   encoder.
//! - [`fsk`]: the 2/4-level FSK symbol slicer with timing recovery.
//! - [`tones`]: non-coherent tone-correlator decisions with per-tone AFC.
//! - [`receiver`]: one channel: FM discriminator → slicers → every decoder.
//! - [`modulate`]: FSK modulator for test signals.

pub mod bch;
pub mod flex;
pub mod fsk;
pub mod modulate;
pub mod pocsag;
pub mod receiver;
pub mod tones;

use std::fmt;

pub use receiver::{Receiver, ReceiverConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Protocol {
    Pocsag,
    Flex,
}

/// What a page carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Address only: the pager beeps.
    Tone,
    Numeric,
    Alpha,
    /// FLEX binary (hex) vector.
    Hex,
    /// FLEX secure vector.
    Secure,
    /// FLEX short message / tone vector (3 or 8 digits, or a source code).
    ShortMessage,
    /// FLEX short instruction (group assignment, events).
    Instruction,
    /// FLEX BIW date and time.
    Time,
    /// FLEX BIW system information.
    System,
}

impl Kind {
    pub fn tag(self) -> &'static str {
        match self {
            Kind::Tone => "TON",
            Kind::Numeric => "NUM",
            Kind::Alpha => "ALN",
            Kind::Hex => "HEX",
            Kind::Secure => "SEC",
            Kind::ShortMessage => "SMSG",
            Kind::Instruction => "INS",
            Kind::Time => "TIME",
            Kind::System => "SYS",
        }
    }
}

/// Where in the FLEX cycle a page came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlexSlot {
    pub cycle: u8,
    pub frame: u8,
    /// 'A'..='D'.
    pub phase: char,
}

/// One decoded page (or FLEX system word).
#[derive(Debug, Clone, PartialEq)]
pub struct Page {
    pub protocol: Protocol,
    /// Symbol rate, baud.
    pub baud: u32,
    /// FSK levels: 2 or 4.
    pub levels: u8,
    pub capcode: u64,
    /// POCSAG function bits (0–3).
    pub function: Option<u8>,
    pub kind: Kind,
    pub text: String,
    /// Protocol details: FLEX address type, fragment, checksum results.
    pub flags: String,
    pub slot: Option<FlexSlot>,
    /// Members of a FLEX group call, from earlier short instructions.
    pub group: Vec<u64>,
    /// Bits the BCH code put right in this page's words.
    pub corrected_bits: u32,
    /// Words beyond repair (their characters are shown as '?').
    pub bad_words: u32,
    /// Seconds into the stream, set by the receiver.
    pub time_s: f64,
    /// RF channel, Hz (0 when unknown), set by the caller.
    pub channel_hz: f64,
}

impl Page {
    pub(crate) fn new(protocol: Protocol, baud: u32, levels: u8, capcode: u64, kind: Kind) -> Self {
        Page {
            protocol,
            baud,
            levels,
            capcode,
            function: None,
            kind,
            text: String::new(),
            flags: String::new(),
            slot: None,
            group: Vec::new(),
            corrected_bits: 0,
            bad_words: 0,
            time_s: 0.0,
            channel_hz: 0.0,
        }
    }

    /// "POCSAG1200" or "FLEX3200/4".
    pub fn mode(&self) -> String {
        match self.protocol {
            Protocol::Pocsag => format!("POCSAG{}", self.baud),
            Protocol::Flex => format!(
                "FLEX{}/{}",
                self.baud * u32::from(self.levels / 2),
                self.levels
            ),
        }
    }
}

impl fmt::Display for Page {
    /// One line: `time  mode  [cycle.frame.phase]  capcode  type  flags | text`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:9.3} ", self.time_s)?;
        if self.channel_hz > 0.0 {
            write!(f, "{:.4}M ", self.channel_hz / 1e6)?;
        }
        write!(f, "{:<11}", self.mode())?;
        match (self.slot, self.function) {
            (Some(s), _) => write!(f, " {:02}.{:03}.{}", s.cycle, s.frame, s.phase)?,
            (None, Some(func)) => write!(f, " F{func}       ")?,
            (None, None) => write!(f, "          ")?,
        }
        write!(f, " {:>10} {:<4}", self.capcode, self.kind.tag())?;
        if !self.flags.is_empty() {
            write!(f, " {}", self.flags)?;
        }
        if self.corrected_bits > 0 || self.bad_words > 0 {
            write!(f, " [fix {} bad {}]", self.corrected_bits, self.bad_words)?;
        }
        if !self.group.is_empty() {
            let g: Vec<String> = self.group.iter().map(u64::to_string).collect();
            write!(f, " group({})", g.join(","))?;
        }
        if !self.text.is_empty() {
            write!(f, " | {}", self.text)?;
        }
        Ok(())
    }
}

/// Text from 7-bit characters: printable ASCII kept, tab/CR/LF written as
/// escapes so a page stays on one line, other controls shown as `<XX>`.
pub(crate) fn push_char(out: &mut String, c: u8) {
    match c {
        0x20..=0x7E => out.push(c as char),
        b'\n' => out.push_str("\\n"),
        b'\r' => out.push_str("\\r"),
        b'\t' => out.push_str("\\t"),
        _ => out.push_str(&format!("<{c:02X}>")),
    }
}
