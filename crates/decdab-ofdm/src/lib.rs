// made by CasualArclamp
//! DecDAB's OFDM layer (EN 300 401 §14): transmission mode I, the phase
//! reference symbol, frequency interleaving, and the demodulator that turns
//! 2.048 MS/s samples into each frame's soft bits.
//!
//! - [`params`]: mode I's numbers.
//! - [`tables`]: the phase reference and the frequency interleaver.
//! - [`frontend`]: decimation from the radio's rate to 2.048 MS/s.
//! - [`demod`]: acquisition, tracking and DQPSK demodulation.

pub mod demod;
pub mod frontend;
pub mod params;
pub mod tables;
pub mod tii;

pub use demod::{
    Demod, Echo, Frame, FrameStats, SOUND_ROW_S, TII_EVERY, SOUND_ROWS, SOUND_SPACING_HZ, Scope,
};
pub use frontend::Frontend;
pub use tii::{TiiReport, Transmitter};
