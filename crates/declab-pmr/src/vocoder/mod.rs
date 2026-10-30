// made by CasualArclamp
//! Speech decoders for the digital voice modes.
//!
//! - [`ambe2`]: AMBE+2 3600×2450 (DMR, NXDN, P25 phase 2), ported from
//!   mbelib.
//!
//! - [`imbe`]: IMBE 7200×4400 (P25 phase 1), ported from mbelib.
//!
//! Both share mbelib's synthesiser (`mbe.rs`).
//!
//! - [`acelp`]: TETRA's ACELP, loaded at run time from a separate library
//!   (ETSI's code is not GPL).

pub mod acelp;
pub mod ambe2;
mod ambe_tables;
pub mod imbe;
mod imbe_tables;
mod mbe;

pub use ambe2::{Ambe2Decoder, FrameKind};
pub use imbe::ImbeDecoder;

/// 8 kHz speech to 48 kHz, ×6 through a polyphase low-pass (3.6 kHz,
/// Blackman-windowed sinc), so the host's linear resampler has no images
/// of the 8 kHz spectrum left to pass.
pub struct Upsample6 {
    /// `phases[p][k]`: tap `6k + p` of the 6-times-oversampled filter.
    phases: [[f32; TAPS_PER_PHASE]; 6],
    hist: [f32; TAPS_PER_PHASE],
}

const TAPS_PER_PHASE: usize = 12;

impl Default for Upsample6 {
    fn default() -> Self {
        Self::new()
    }
}

impl Upsample6 {
    pub fn new() -> Upsample6 {
        let n = 6 * TAPS_PER_PHASE;
        let h = crate::fsk4::lowpass(3_600.0, 48_000.0, n - 1);
        let mut phases = [[0f32; TAPS_PER_PHASE]; 6];
        for (i, &x) in h.iter().enumerate() {
            // ×6: each phase passes a sixth of the filter's DC gain.
            phases[i % 6][i / 6] = x * 6.0;
        }
        Upsample6 {
            phases,
            hist: [0.0; TAPS_PER_PHASE],
        }
    }

    pub fn reset(&mut self) {
        self.hist = [0.0; TAPS_PER_PHASE];
    }

    /// Append six output samples per input sample to `out`.
    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        out.reserve(input.len() * 6);
        for &x in input {
            self.hist.copy_within(0..TAPS_PER_PHASE - 1, 1);
            self.hist[0] = x;
            for ph in &self.phases {
                out.push(ph.iter().zip(&self.hist).map(|(a, b)| a * b).sum());
            }
        }
    }
}
