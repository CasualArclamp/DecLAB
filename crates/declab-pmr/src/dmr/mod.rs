// made by CasualArclamp
//! DMR (ETSI TS 102 361): Tier I and II conventional, and Tier III
//! trunking's control channel.
//!
//! - [`receiver`]: 4FSK samples in, bursts found and decoded, [`Event`]s out.
//! - [`bptc`], [`ambe`]: the burst codes and the voice frame's FEC.
//! - [`data`], [`lrrp`]: packet data (headers, rate ½/¾ blocks, reassembly),
//!   IPv4/UDP and Motorola's LRRP locations.
//! - [`pdu`]: Full LC, CSBK (with Tier III's grants, Aloha, broadcasts) and
//!   Short LC.
//! - [`sim`]: a base-station modulator, for the tests and the sweep.
//!
//! A burst is 264 bits (132 symbols, 27.5 ms): 108 payload bits, a 48-bit
//! centre (SYNC, or EMB + embedded signalling, or slot type + SYNC for data)
//! and 108 more payload bits. A base station puts a 24-bit CACH before each
//! burst, so its outbound slots are 144 symbols (30 ms) back to back.

pub mod ambe;
pub mod bptc;
pub mod data;
pub mod lrrp;
pub mod pdu;
pub mod receiver;
pub mod sim;

pub use receiver::{Event, Receiver, Stats};

/// Symbols a second, clause 10.2.1.
pub const SYMBOL_RATE: f64 = 4800.0;
/// The outer symbols' deviation (±3), clause 10.2.2.1.
pub const OUTER_DEV_HZ: f64 = 1944.0;
/// The root-raised-cosine roll-off: flat to 1 920 Hz, zero at 2 880 Hz
/// (clause 10.2.2.2), α = 960 / 4 800.
pub const ALPHA: f64 = 0.2;
/// Symbols from one burst's start to the next's (30 ms).
pub const SLOT_SYMBOLS: usize = 144;

/// The 48-bit SYNC patterns, ETSI TS 102 361-1 table 9.2. Each data pattern
/// is the symbol-wise negative of its voice pattern, so one correlator per
/// pair finds both (a negative peak is data).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// A repeater's outbound (with CACH).
    Bs,
    /// A radio's transmission to a repeater.
    Ms,
    /// Direct mode (radio to radio), time slot 1.
    Ts1,
    /// Direct mode, time slot 2.
    Ts2,
}

/// The voice SYNC of each source; data is its negative.
pub const VOICE_SYNC: [(Source, u64); 4] = [
    (Source::Bs, 0x755F_D7DF_75F7),
    (Source::Ms, 0x7F7D_5DD5_7DFD),
    (Source::Ts1, 0x5D57_7F77_57FF),
    (Source::Ts2, 0x7DFF_D5F5_5D5F),
];
/// The data SYNCs, for the modulator.
pub const DATA_SYNC: [(Source, u64); 4] = [
    (Source::Bs, 0xDFF5_7D75_DF5D),
    (Source::Ms, 0xD5D7_F77F_D757),
    (Source::Ts1, 0xF7FD_D5DD_FD55),
    (Source::Ts2, 0xD755_7F5F_F7F5),
];

/// The symbol a dibit is sent as, table 10.3: 01 → +3, 00 → +1, 10 → −1,
/// 11 → −3.
pub fn symbol(dibit: u8) -> f32 {
    match dibit & 3 {
        0b01 => 3.0,
        0b00 => 1.0,
        0b10 => -1.0,
        _ => -3.0,
    }
}

/// A 48-bit pattern's 24 symbols.
pub fn pattern_symbols(p: u64) -> [f32; 24] {
    let mut s = [0.0; 24];
    for (i, v) in s.iter_mut().enumerate() {
        *v = symbol(((p >> (46 - 2 * i)) & 3) as u8);
    }
    s
}

/// The slot type's Data Type, table 9.22.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    PiHeader,
    VoiceLcHeader,
    TerminatorLc,
    Csbk,
    MbcHeader,
    MbcContinuation,
    DataHeader,
    Rate12,
    Rate34,
    Idle,
    Rate1,
    Usbd,
    Reserved(u8),
}

impl DataType {
    pub fn from_u8(v: u8) -> DataType {
        use DataType::*;
        match v {
            0 => PiHeader,
            1 => VoiceLcHeader,
            2 => TerminatorLc,
            3 => Csbk,
            4 => MbcHeader,
            5 => MbcContinuation,
            6 => DataHeader,
            7 => Rate12,
            8 => Rate34,
            9 => Idle,
            10 => Rate1,
            11 => Usbd,
            v => Reserved(v),
        }
    }

    pub fn to_u8(self) -> u8 {
        use DataType::*;
        match self {
            PiHeader => 0,
            VoiceLcHeader => 1,
            TerminatorLc => 2,
            Csbk => 3,
            MbcHeader => 4,
            MbcContinuation => 5,
            DataHeader => 6,
            Rate12 => 7,
            Rate34 => 8,
            Idle => 9,
            Rate1 => 10,
            Usbd => 11,
            Reserved(v) => v,
        }
    }
}

/// The CRC masks of table B.21.
pub mod mask {
    pub const PI_HEADER: u16 = 0x6969;
    pub const VOICE_LC_HEADER: [u8; 3] = [0x96, 0x96, 0x96];
    pub const TERMINATOR_LC: [u8; 3] = [0x99, 0x99, 0x99];
    pub const CSBK: u16 = 0xA5A5;
    pub const MBC_HEADER: u16 = 0xAAAA;
    pub const DATA_HEADER: u16 = 0xCCCC;
    pub const USBD: u16 = 0x3333;
}

/// Bit positions in a 264-bit burst (clause 6): the info halves of a data
/// burst, its slot type, and a voice burst's EMB and embedded signalling.
pub mod layout {
    /// The 196 BPTC bits: 0..98 and 166..264.
    pub const INFO_A: std::ops::Range<usize> = 0..98;
    pub const INFO_B: std::ops::Range<usize> = 166..264;
    /// Slot type: 10 bits each side of the SYNC.
    pub const SLOT_A: std::ops::Range<usize> = 98..108;
    pub const SLOT_B: std::ops::Range<usize> = 156..166;
    /// The 48-bit centre.
    pub const CENTRE: std::ops::Range<usize> = 108..156;
    /// EMB: 8 bits each side of the 32 embedded bits.
    pub const EMB_A: std::ops::Range<usize> = 108..116;
    pub const EMBEDDED: std::ops::Range<usize> = 116..148;
    pub const EMB_B: std::ops::Range<usize> = 148..156;
    /// The CACH's TACT bits (AT, TC, LS1, LS0, H2, H1, H0) among its 24,
    /// first sent first (figure B.9); the other 17 carry the payload.
    pub const TACT: [usize; 7] = [0, 4, 8, 12, 14, 18, 22];
    pub const CACH_PAYLOAD: [usize; 17] =
        [1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 15, 16, 17, 19, 20, 21, 23];
}
