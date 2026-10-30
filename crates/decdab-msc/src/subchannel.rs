// made by CasualArclamp
//! One sub-channel of the MSC, from each CIF's soft bits to its logical
//! frames (EN 300 401 §11–12): cut the sub-channel's capacity units out of
//! the CIF, time-deinterleave, depuncture, Viterbi, energy dispersal.
//!
//! A logical frame is the sub-channel's 24 ms of data: bit rate × 3 bytes.
//! The first comes out 15 CIFs (360 ms) after the first CIF goes in.

use decdab_fec::prbs::Prbs;
use decdab_fec::timeint::TimeDeinterleaver;
use decdab_fec::{Plan, Viterbi};
use decdab_fic::SubChannel;

/// Bits in a capacity unit.
pub const CU_BITS: usize = 64;

pub struct SubChannelDecoder {
    sub: SubChannel,
    plan: Plan,
    ti: TimeDeinterleaver,
    vit: Viterbi,
    prbs: Prbs,
    soft: Vec<i8>,
    mother: Vec<i8>,
    bits: Vec<u8>,
    frame: Vec<u8>,
    /// Logical frames decoded, and the Viterbi's margins (a weak one hints
    /// at errors).
    pub frames: u64,
    pub last_margin: i32,
}

impl SubChannelDecoder {
    /// `None` when FIG 0/1 described a protection that does not exist.
    pub fn new(sub: SubChannel) -> Option<Self> {
        let plan = sub.plan()?;
        let len = sub.size as usize * CU_BITS;
        if plan.coded_bits() != len {
            return None;
        }
        Some(SubChannelDecoder {
            sub,
            plan,
            ti: TimeDeinterleaver::new(len),
            vit: Viterbi::new(),
            prbs: Prbs::default(),
            soft: Vec::with_capacity(len),
            mother: Vec::new(),
            bits: Vec::new(),
            frame: Vec::new(),
            frames: 0,
            last_margin: 0,
        })
    }

    pub fn subchannel(&self) -> &SubChannel {
        &self.sub
    }

    /// Bytes a logical frame holds.
    pub fn frame_bytes(&self) -> usize {
        self.plan.info_bits() / 8
    }

    /// Take one CIF's soft bits (all 55 296); returns the next logical frame
    /// once the time interleaver has filled.
    pub fn push_cif(&mut self, cif: &[i8]) -> Option<&[u8]> {
        let a = self.sub.start as usize * CU_BITS;
        let b = a + self.sub.size as usize * CU_BITS;
        if !self.ti.push(cif.get(a..b)?, &mut self.soft) {
            return None;
        }
        self.plan.depuncture(&self.soft, &mut self.mother);
        self.bits.clear();
        self.last_margin = self.vit.decode(&self.mother, &mut self.bits);
        self.prbs.apply(&mut self.bits);
        self.frame.clear();
        self.frame.extend(
            self.bits
                .as_chunks::<8>()
                .0
                .iter()
                .map(|b| b.iter().fold(0u8, |a, &x| (a << 1) | x)),
        );
        self.frames += 1;
        Some(&self.frame)
    }
}
