// made by CasualArclamp
//! DecDAB's Fast Information Channel (EN 300 401 §5.2, §6, §8).
//!
//! [`FicDecoder`] takes each frame's FIC soft bits (three symbols in mode
//! I), decodes the four FIC blocks (depuncture, Viterbi, energy dispersal),
//! checks the twelve FIBs' CRCs and parses their FIGs into an
//! [`Ensemble`].
//!
//! - [`ensemble`]: the model: sub-channels, services, labels, time.
//! - [`figs`]: the FIG parser.

pub mod ensemble;
pub mod figs;

pub use ensemble::{
    Component, DabTime, Ensemble, Label, PacketComponent, Protection, Service, SubChannel,
    Transport, UserApp, pty_name,
};
pub use figs::FigCounts;

use decdab_core::crc::crc16_ok;
use decdab_fec::prbs::Prbs;
use decdab_fec::{Plan, Viterbi};

/// Bits of one FIC block in mode I (one per CIF): three FIBs, coded.
pub const FIC_BLOCK_BITS: usize = 2304;

#[derive(Debug, Clone, Copy, Default)]
pub struct FicStats {
    pub fibs: u64,
    pub fibs_ok: u64,
    /// FIBs good in the last frame, of 12.
    pub last_frame_ok: u8,
}

pub struct FicDecoder {
    plan: Plan,
    vit: Viterbi,
    prbs: Prbs,
    mother: Vec<i8>,
    bits: Vec<u8>,
    pub ensemble: Ensemble,
    pub counts: FigCounts,
    pub stats: FicStats,
    /// Bumped whenever a FIB parses, so a UI can tell when to look again.
    pub generation: u64,
}

impl Default for FicDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FicDecoder {
    pub fn new() -> FicDecoder {
        FicDecoder {
            plan: Plan::fic(),
            vit: Viterbi::new(),
            prbs: Prbs::default(),
            mother: Vec::new(),
            bits: Vec::new(),
            ensemble: Ensemble::default(),
            counts: FigCounts::default(),
            stats: FicStats::default(),
            generation: 0,
        }
    }

    /// Forget the ensemble (after a retune).
    pub fn reset(&mut self) {
        self.ensemble = Ensemble::default();
        self.counts = FigCounts::default();
        self.stats = FicStats::default();
        self.generation += 1;
    }

    /// Decode a frame's FIC: `soft` holds its 4 × 2304 soft bits.
    pub fn push_frame(&mut self, soft: &[i8]) {
        let mut ok = 0u8;
        for blk in soft.as_chunks::<FIC_BLOCK_BITS>().0 {
            self.plan.depuncture(blk, &mut self.mother);
            self.bits.clear();
            self.vit.decode(&self.mother, &mut self.bits);
            self.prbs.apply(&mut self.bits);
            for fib in self.bits.as_chunks::<256>().0 {
                let mut bytes = [0u8; 32];
                for (o, b) in bytes.iter_mut().zip(fib.as_chunks::<8>().0) {
                    *o = b.iter().fold(0u8, |a, &x| (a << 1) | x);
                }
                self.stats.fibs += 1;
                if crc16_ok(&bytes) {
                    ok += 1;
                    self.stats.fibs_ok += 1;
                    figs::parse_fib(&bytes[..30], &mut self.ensemble, &mut self.counts);
                    self.generation += 1;
                }
            }
        }
        self.stats.last_frame_ok = ok;
    }
}
