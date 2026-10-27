// made by CasualArclamp
//! The DecLAB engine: the host every plug-in decoder runs in.
//!
//! - [`host`]: the source thread, the down-converter and a worker per decoder.
//! - [`waterfall`] / [`spectrum`]: the waterfall's rows, on a thread of their own.
//! - [`frontend`]: DC offset and I/Q image taken off the radio's samples.
//! - [`audio`]: decoded audio to the sound card.
//! - [`meter`]: CPU meters.

pub mod audio;
pub mod frontend;
pub mod host;
pub mod meter;
pub mod spectrum;
pub mod waterfall;

pub use host::{ChannelInfo, DecoderView, Host, HostOptions, Row, Stats};
