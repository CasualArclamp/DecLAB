// made by CasualArclamp
//! DecDAB's Main Service Channel.
//!
//! - [`subchannel`]: a sub-channel's CIFs to logical frames.
//! - [`superframe`]: DAB+ superframes to access units (and their PAD).

pub mod subchannel;
pub mod superframe;

pub use subchannel::SubChannelDecoder;
pub use superframe::{AudioParams, SuperframeDecoder, au_pad};
