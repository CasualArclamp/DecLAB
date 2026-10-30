// made by CasualArclamp
//! DecDAB's audio codecs.
//!
//! - [`aac`]: DAB+'s HE-AAC v2 (AAC-LC with SBR and PS, 960-sample frames)
//!   through libxaac (DecDAB used FDK-AAC, whose licence does not mix with
//!   DecLAB's GPL).
//! - [`mp2`]: DAB's MPEG-1/2 Audio Layer II, in Rust.

pub mod aac;
pub mod mp2;
mod mp2_window;

pub use aac::AacDecoder;
pub use mp2::Mp2Decoder;

#[derive(Debug, thiserror::Error)]
pub enum CodecError {
    #[error("libxaac: {1} failed (0x{0:08X})")]
    Xaac(i32, &'static str),
    #[error("{0}")]
    Bad(&'static str),
}

/// Decoded audio: interleaved 16-bit samples.
#[derive(Debug, Clone)]
pub struct Pcm {
    pub rate: u32,
    pub channels: u16,
    pub samples: Vec<i16>,
    /// The decoder concealed (filled in) this frame.
    pub concealed: bool,
}
