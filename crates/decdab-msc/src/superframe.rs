// made by CasualArclamp
//! DAB+ audio superframes (TS 102 563): five logical frames (120 ms) of a
//! sub-channel, protected by RS(120,110) and opened by a Fire-code-checked
//! header that says how the access units (AUs) lie.
//!
//! **Sync.** The five frames of a superframe start at a CIF count that is a
//! multiple of five, but a receiver joining late does not know which; the
//! last five logical frames are tried as a superframe after each new one
//! until one passes RS and the Fire code, and then every fifth frame.
//!
//! **Layout** (§5.2): bytes 0–1 the Fire code over bytes 2–10; byte 2:
//! rfa, dac_rate, sbr_flag, aac_channel_mode, ps_flag, mpeg_surround (3);
//! then the AU start addresses, 12 bits each, for AUs 1…n−1. Each AU ends
//! in a CRC-16 (EN 300 401's). The RS parity (§6) fills the last 10·s
//! bytes; codeword j takes bytes j, j + s, j + 2s … (s = bit rate / 8).

use decdab_core::crc::crc16_ok;
use decdab_fec::rs::ReedSolomon;

/// The audio parameters a superframe header gives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AudioParams {
    /// DAC rate 48 kHz (else 32 kHz).
    pub dac_48k: bool,
    pub sbr: bool,
    pub stereo: bool,
    pub ps: bool,
    pub mpeg_surround: u8,
}

impl AudioParams {
    /// AUs per superframe (§5.2 Table 2).
    pub fn au_count(&self) -> usize {
        match (self.dac_48k, self.sbr) {
            (false, true) => 2,
            (true, true) => 3,
            (false, false) => 4,
            (true, false) => 6,
        }
    }

    /// The AAC core's sampling rate.
    pub fn core_rate(&self) -> u32 {
        let r = if self.dac_48k { 48_000 } else { 32_000 };
        if self.sbr { r / 2 } else { r }
    }

    /// The output (DAC) rate.
    pub fn output_rate(&self) -> u32 {
        if self.dac_48k { 48_000 } else { 32_000 }
    }

    /// "HE-AAC v2", "HE-AAC", "AAC-LC".
    pub fn codec_name(&self) -> &'static str {
        match (self.sbr, self.ps) {
            (true, true) => "HE-AAC v2",
            (true, false) => "HE-AAC",
            _ => "AAC-LC",
        }
    }

    /// The MPEG-4 AudioSpecificConfig (ISO/IEC 14496-3 §1.6.2.1) for these
    /// parameters, with the 960-sample frame DAB+ uses: explicit
    /// hierarchical signalling of SBR (AOT 5) or PS (AOT 29) over AAC-LC.
    pub fn audio_specific_config(&self) -> Vec<u8> {
        fn sf_index(hz: u32) -> u32 {
            match hz {
                48_000 => 3,
                32_000 => 5,
                24_000 => 6,
                16_000 => 8,
                _ => 15,
            }
        }
        let mut w = BitWriter::default();
        let channels = if self.stereo { 2 } else { 1 };
        if self.sbr {
            w.put(if self.ps { 29 } else { 5 }, 5);
            w.put(sf_index(self.core_rate()), 4);
            w.put(channels, 4);
            w.put(sf_index(self.output_rate()), 4);
            w.put(2, 5); // AAC-LC underneath
        } else {
            w.put(2, 5);
            w.put(sf_index(self.core_rate()), 4);
            w.put(channels, 4);
        }
        // GASpecificConfig: frameLengthFlag (960), dependsOnCoreCoder,
        // extensionFlag.
        w.put(1, 1);
        w.put(0, 1);
        w.put(0, 1);
        w.finish()
    }
}

#[derive(Default)]
struct BitWriter {
    out: Vec<u8>,
    acc: u32,
    n: u32,
}

impl BitWriter {
    fn put(&mut self, v: u32, bits: u32) {
        for i in (0..bits).rev() {
            self.acc = (self.acc << 1) | ((v >> i) & 1);
            self.n += 1;
            if self.n == 8 {
                self.out.push(self.acc as u8);
                self.acc = 0;
                self.n = 0;
            }
        }
    }

    fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            self.out.push((self.acc << (8 - self.n)) as u8);
        }
        self.out
    }
}

/// The Fire code (TS 102 563 §5.2): g(x) = (x¹¹ + 1)(x⁵ + x³ + x² + x + 1),
/// register from zero, over the 9 bytes after it.
pub fn fire_code(bytes: &[u8]) -> u16 {
    const G: u16 = 0x782F;
    let mut r = 0u16;
    for &b in bytes {
        for i in (0..8).rev() {
            let fb = ((r >> 15) as u8 ^ (b >> i)) & 1;
            r <<= 1;
            if fb == 1 {
                r ^= G;
            }
        }
    }
    r
}

/// What a superframe gave.
#[derive(Debug, Default, Clone, Copy)]
pub struct SuperframeStats {
    pub superframes: u64,
    /// Bytes RS corrected, and superframes RS could not.
    pub rs_corrected: u64,
    pub rs_failed: u64,
    pub au_ok: u64,
    pub au_bad: u64,
    /// Whether the decoder is in step with the superframes.
    pub synced: bool,
}

pub struct SuperframeDecoder {
    /// Bytes per logical frame (bit rate × 3).
    frame_bytes: usize,
    s: usize,
    rs: ReedSolomon,
    /// The last five logical frames, oldest first.
    buf: Vec<u8>,
    frames_in: usize,
    synced: bool,
    misses: u32,
    word: Vec<u8>,
    pub params: Option<AudioParams>,
    pub stats: SuperframeStats,
}

/// One access unit: its bytes (without the CRC) and whether its CRC held.
pub struct Au<'a> {
    pub data: &'a [u8],
    pub ok: bool,
}

impl SuperframeDecoder {
    /// For a sub-channel of `frame_bytes` bytes a logical frame (a multiple
    /// of 24: DAB+ bit rates are multiples of 8 kbit/s).
    pub fn new(frame_bytes: usize) -> Self {
        let s = frame_bytes / 24;
        SuperframeDecoder {
            frame_bytes,
            s,
            rs: ReedSolomon::dab_plus(),
            buf: Vec::with_capacity(5 * frame_bytes),
            frames_in: 0,
            synced: false,
            misses: 0,
            word: vec![0; 120],
            params: None,
            stats: SuperframeStats::default(),
        }
    }

    /// Take a logical frame; `on_au` gets each AU of a superframe completed
    /// by it, with the superframe's parameters.
    pub fn push(&mut self, frame: &[u8], mut on_au: impl FnMut(&AudioParams, Au)) {
        debug_assert_eq!(frame.len(), self.frame_bytes);
        if self.buf.len() == 5 * self.frame_bytes {
            self.buf.drain(..self.frame_bytes);
        }
        self.buf.extend_from_slice(frame);
        self.frames_in += 1;
        if self.buf.len() < 5 * self.frame_bytes {
            return;
        }
        if self.synced && !self.frames_in.is_multiple_of(5) {
            return;
        }
        let mut sf = std::mem::take(&mut self.buf);
        let good = self.try_superframe(&mut sf, &mut on_au);
        if good {
            sf.clear();
            self.frames_in = 0;
            self.misses = 0;
            self.synced = true;
        } else if self.synced {
            self.misses += 1;
            if self.misses >= 3 {
                self.synced = false;
            } else {
                // Keep the rhythm through a bad superframe.
                sf.clear();
                self.frames_in = 0;
            }
        }
        self.buf = sf;
        self.stats.synced = self.synced;
    }

    fn try_superframe(&mut self, sf: &mut [u8], on_au: &mut impl FnMut(&AudioParams, Au)) -> bool {
        let s = self.s;
        // RS: codeword j is bytes j, j + s, … (§6.1).
        let mut corrected = 0u64;
        let mut failed = false;
        for j in 0..s {
            for i in 0..120 {
                self.word[i] = sf[j + s * i];
            }
            match self.rs.decode(&mut self.word) {
                Ok(n) => {
                    corrected += n as u64;
                    if n > 0 {
                        for i in 0..120 {
                            sf[j + s * i] = self.word[i];
                        }
                    }
                }
                Err(_) => failed = true,
            }
        }
        let fire = u16::from_be_bytes([sf[0], sf[1]]);
        if fire_code(&sf[2..11]) != fire {
            return false;
        }
        self.stats.superframes += 1;
        self.stats.rs_corrected += corrected;
        if failed {
            self.stats.rs_failed += 1;
        }
        let h = sf[2];
        let p = AudioParams {
            dac_48k: h & 0x40 != 0,
            sbr: h & 0x20 != 0,
            stereo: h & 0x10 != 0,
            ps: h & 0x08 != 0,
            mpeg_surround: h & 7,
        };
        let n = p.au_count();
        let mut starts = [0usize; 7];
        starts[0] = match n {
            2 => 5,
            3 => 6,
            4 => 8,
            _ => 11,
        };
        for (i, st) in starts.iter_mut().enumerate().take(n).skip(1) {
            // 12-bit fields from byte 3, packed.
            let bit = 24 + 12 * (i - 1);
            let (b0, b1) = (sf[bit / 8] as usize, sf[bit / 8 + 1] as usize);
            *st = if bit % 8 == 0 {
                (b0 << 4) | (b1 >> 4)
            } else {
                ((b0 & 0x0F) << 8) | b1
            };
        }
        starts[n] = 110 * s;
        self.params = Some(p);
        for i in 0..n {
            let (a, b) = (starts[i], starts[i + 1]);
            if a + 2 > b || b > 110 * s {
                self.stats.au_bad += 1;
                continue;
            }
            let ok = crc16_ok(&sf[a..b]);
            if ok {
                self.stats.au_ok += 1;
            } else {
                self.stats.au_bad += 1;
            }
            on_au(
                &p,
                Au {
                    data: &sf[a..b - 2],
                    ok,
                },
            );
        }
        true
    }
}

/// The PAD of a DAB+ AU (TS 102 563 §5.4): a data_stream_element at the
/// AU's start (ISO/IEC 14496-3 §4.4.2.7). Returns its bytes.
pub fn au_pad(au: &[u8]) -> Option<&[u8]> {
    // id_syn_ele (3) = 4 (DSE), element_instance_tag (4),
    // data_byte_align_flag (1), count (8), [esc_count (8)].
    let b0 = *au.first()?;
    if b0 >> 5 != 4 {
        return None;
    }
    let align = b0 & 1 != 0;
    let mut count = *au.get(1)? as usize;
    let mut pos = 2usize; // in bytes, with the bit position after `count`
    if count == 255 {
        count += *au.get(2)? as usize;
        pos = 3;
    }
    // The header is 3 + 4 + 1 + 8 (+ 8) bits: 16 or 24, byte-aligned
    // already, so the alignment flag changes nothing here.
    let _ = align;
    au.get(pos..pos + count)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asc_for_he_aac_v2_and_lc() {
        let p = AudioParams {
            dac_48k: true,
            sbr: true,
            stereo: false,
            ps: true,
            mpeg_surround: 0,
        };
        // AOT 29, 24 kHz core (6), mono, 48 kHz ext (3), AOT 2, 960:
        // 11101 0110 0001 0011 00010 100, zero-padded.
        assert_eq!(p.audio_specific_config(), [0xEB, 0x09, 0x8A, 0x00]);
        let lc = AudioParams {
            dac_48k: true,
            sbr: false,
            stereo: true,
            ps: false,
            mpeg_surround: 0,
        };
        // AOT 2, 48 kHz (3), stereo, 960: 00010 0011 0010 100.
        assert_eq!(lc.audio_specific_config(), [0x11, 0x94]);
        assert_eq!(p.au_count(), 3);
        assert_eq!(p.core_rate(), 24_000);
    }

    #[test]
    fn fire_code_is_linear_and_detects_errors() {
        let a = [1u8, 2, 3, 4, 5, 6, 7, 8, 9];
        let mut b = a;
        b[4] ^= 0x10;
        assert_ne!(fire_code(&a), fire_code(&b));
        assert_eq!(fire_code(&[0u8; 9]), 0);
    }

    #[test]
    fn dse_pad_is_found() {
        let au = [0x80, 3, 0xAA, 0xBB, 0xCC, 0x21];
        assert_eq!(au_pad(&au), Some(&[0xAA, 0xBB, 0xCC][..]));
        assert_eq!(au_pad(&[0x21, 0]), None);
    }
}
