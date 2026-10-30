// made by CasualArclamp
//! The MSC's time interleaving (EN 300 401 §12): bit i of a sub-channel's
//! logical frame is sent d(i mod 16) CIFs late, d = 0, 8, 4, 12, 2, 10, 6,
//! 14, 1, 9, 5, 13, 3, 11, 7, 15 (Table 34; the bit reversal of i mod 16).
//! The deinterleaver delays bit i by 15 − d, so every bit comes out 15 CIFs
//! (360 ms) after it went in.

/// Table 34.
pub const DELAY: [usize; 16] = [0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15];

/// Deinterleaver for one sub-channel's soft bits.
pub struct TimeDeinterleaver {
    /// The last 16 CIFs' bits, `ring[r]` the CIF r mod 16.
    ring: Vec<Vec<i8>>,
    len: usize,
    /// CIFs taken so far.
    count: u64,
}

impl TimeDeinterleaver {
    /// For logical frames of `len` coded bits.
    pub fn new(len: usize) -> Self {
        TimeDeinterleaver {
            ring: vec![vec![0; len]; 16],
            len,
            count: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Take one CIF's bits of the sub-channel; once 15 have gone in,
    /// `out` receives a whole deinterleaved frame and `true` is returned.
    pub fn push(&mut self, cif: &[i8], out: &mut Vec<i8>) -> bool {
        let r = (self.count % 16) as usize;
        self.ring[r].copy_from_slice(&cif[..self.len]);
        self.count += 1;
        if self.count < 16 {
            return false;
        }
        out.clear();
        out.reserve(self.len);
        // Bit i of the frame that went in 15 CIFs ago was sent d CIFs after
        // that, i.e. in CIF (r − 15 + d).
        for i in 0..self.len {
            let src = (r + 1 + DELAY[i % 16]) % 16;
            out.push(self.ring[src][i]);
        }
        true
    }

    pub fn reset(&mut self) {
        self.count = 0;
    }
}

/// The transmitter's side, for the modulator: delays bit i by d(i mod 16).
pub struct TimeInterleaver {
    ring: Vec<Vec<u8>>,
    count: u64,
}

impl TimeInterleaver {
    pub fn new(len: usize) -> Self {
        TimeInterleaver {
            ring: vec![vec![0; len]; 16],
            count: 0,
        }
    }

    /// One logical frame in, one CIF's worth out (zeros until the delays
    /// fill).
    pub fn push(&mut self, frame: &[u8], out: &mut Vec<u8>) {
        let r = (self.count % 16) as usize;
        self.ring[r].copy_from_slice(frame);
        self.count += 1;
        out.clear();
        for i in 0..frame.len() {
            let src = (r + 16 - DELAY[i % 16]) % 16;
            out.push(self.ring[src][i]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interleave_then_deinterleave_is_a_15_cif_delay() {
        let len = 64;
        let mut ti = TimeInterleaver::new(len);
        let mut td = TimeDeinterleaver::new(len);
        let frames: Vec<Vec<u8>> = (0..40)
            .map(|f| (0..len).map(|i| ((f * 31 + i * 7) % 5 == 0) as u8).collect())
            .collect();
        let (mut cif, mut out) = (Vec::new(), Vec::new());
        let mut got = Vec::new();
        for f in &frames {
            ti.push(f, &mut cif);
            let soft: Vec<i8> = cif.iter().map(|&b| if b == 0 { 1 } else { -1 }).collect();
            if td.push(&soft, &mut out) {
                got.push(out.iter().map(|&s| (s < 0) as u8).collect::<Vec<u8>>());
            }
        }
        // The first deinterleaved frame is frame 0, once 16 CIFs are in.
        assert_eq!(got.len(), frames.len() - 15);
        for (g, f) in got.iter().zip(&frames) {
            assert_eq!(g, f);
        }
    }
}
