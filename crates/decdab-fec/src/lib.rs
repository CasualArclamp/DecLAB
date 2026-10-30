// made by CasualArclamp
//! DecDAB's channel coding (EN 300 401 §10–12, TS 102 563 §6).
//!
//! - [`conv`]: the rate-1/4 mother code, puncturing vectors, and the FIC,
//!   EEP and UEP plans.
//! - [`viterbi`]: the block Viterbi decoder.
//! - [`prbs`]: energy dispersal.
//! - [`timeint`]: the MSC's time interleaving.
//! - [`rs`]: Reed–Solomon, DAB+'s RS(120,110).

pub mod conv;
pub mod prbs;
pub mod rs;
pub mod timeint;
pub mod viterbi;

pub use conv::{EepSet, Plan};
pub use viterbi::Viterbi;
