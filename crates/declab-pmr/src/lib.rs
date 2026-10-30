// made by CasualArclamp
//! Digital land-mobile radio for DecLAB: professional and amateur two-way
//! radio's digital voice and trunking.
//!
//! - [`fsk4`]: the 4FSK front end DMR, NXDN and P25 phase 1 share.
//! - [`fec`]: the block codes, CRCs and Reed-Solomon code, soft-decoded.
//! - [`conv`]: the K = 5 convolutional code (NXDN), soft Viterbi.
//! - [`dmr`]: DMR Tier I/II/III (ETSI TS 102 361).
//! - [`nxdn`]: NXDN48 conventional voice and signalling.
//! - [`vocoder`]: AMBE+2 speech, ported from mbelib.
//!
//! P25 and TETRA follow, on the same front end where they can.
//!
//! Voice comes out as vocoder parameter frames (AMBE+2's 49 bits), which
//! [`vocoder`] turns into 8 kHz speech.

pub mod conv;
pub mod dmr;
pub mod fec;
pub mod fsk4;
pub mod nxdn;
pub mod p25;
pub mod tetra;
pub mod vocoder;
