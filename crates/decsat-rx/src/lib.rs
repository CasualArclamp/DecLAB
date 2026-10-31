// made by CasualArclamp
//! DVB-S/S2/S2X receiver core, from DecSAT's engine: the carrier and
//! symbol identification, the PL demodulator, the FEC stage and the generic
//! PSK demodulator that DVB-S uses.

pub mod demod;
pub mod estimate;
pub mod fec;
pub mod identify;
pub mod psk;
pub mod spectrum;
pub mod streams;
pub mod superframe;

pub use estimate::{BandEstimate, estimate_band};
pub use spectrum::Spectrum;
pub mod beacon;
pub mod period;

/// Seconds since the Unix epoch, for file names.
pub(crate) fn unix_stamp() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}
