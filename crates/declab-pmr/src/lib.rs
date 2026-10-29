// made by CasualArclamp
//! Digital land-mobile radio for DecLAB: professional and amateur two-way
//! radio's digital voice and trunking.
//!
//! - [`fsk4`]: the 4FSK front end DMR, NXDN and P25 phase 1 share.
//! - [`fec`]: the block codes, CRCs and Reed-Solomon code, soft-decoded.
//! - [`dmr`]: DMR Tier I/II/III (ETSI TS 102 361).
//! - [`vocoder`]: AMBE+2 speech, ported from mbelib.
//!
//! NXDN, P25 and TETRA follow, on the same front end where they can.
//!
//! Voice comes out as vocoder parameter frames (AMBE+2's 49 bits), which
//! [`vocoder`] turns into 8 kHz speech.

pub mod dmr;
pub mod fec;
pub mod fsk4;
pub mod vocoder;
