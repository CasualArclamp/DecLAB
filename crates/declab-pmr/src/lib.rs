// made by CasualArclamp
//! Digital land-mobile radio for DecLAB: professional and amateur two-way
//! radio's digital voice and trunking.
//!
//! - [`fsk4`]: the 4FSK front end DMR, NXDN and P25 phase 1 share.
//! - [`fec`]: the block codes, CRCs and Reed-Solomon code, soft-decoded.
//! - [`dmr`]: DMR Tier I/II/III (ETSI TS 102 361).
//!
//! NXDN, P25 and TETRA follow, on the same front end where they can.
//!
//! Voice comes out as vocoder parameter frames (AMBE+2's 49 bits); turning
//! them into speech needs an AMBE+2 synthesiser, which is not here yet.

pub mod dmr;
pub mod fec;
pub mod fsk4;
