// made by CasualArclamp
//! Shared types for DecDAB: sample formats, IQ conversion, the Band III
//! channel plan, CRCs and error types. No DSP or I/O lives here: this crate
//! is the vocabulary every other crate speaks.
//!
//! - [`channel`]: the Band III DAB block plan (5A–13F), for tuning by block.
//! - [`charset`]: DAB text (EBU Latin, UTF-8, UCS-2) to Unicode.
//! - [`crc`]: the CRCs DAB uses.
//!
//! See `docs/DESIGN.md` for scope.

pub mod channel;
pub mod charset;
pub mod crc;

use num_complex::Complex32;

/// Baseband complex sample. `f32` is plenty for 8-bit HackRF input and keeps the
/// FFTs cache-friendly on the target CPU.
pub type Iq = Complex32;

/// Sample format of an interleaved-IQ source or file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFormat {
    /// Signed 8-bit I, 8-bit Q — the HackRF's native format.
    Cs8,
    /// Unsigned 8-bit I/Q, offset 128 — the RTL-SDR's native format, and
    /// 8-bit WAV.
    Cu8,
    /// Signed 16-bit I/Q (little-endian).
    Cs16,
    /// Signed 24-bit I/Q (little-endian, packed) — 24-bit WAV.
    Cs24,
    /// 32-bit float I/Q.
    Cf32,
}

impl SampleFormat {
    /// Bytes per complex sample.
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            SampleFormat::Cs8 | SampleFormat::Cu8 => 2,
            SampleFormat::Cs16 => 4,
            SampleFormat::Cs24 => 6,
            SampleFormat::Cf32 => 8,
        }
    }
}

/// Errors surfaced across the DecDAB crates.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("not yet implemented: {0}")]
    Unimplemented(&'static str),
    #[error("{0}")]
    Other(String),
}

/// Crate-wide result alias.
pub type Result<T> = std::result::Result<T, Error>;

/// Convert one interleaved-IQ byte buffer into complex samples, normalised to
/// roughly unit scale. Used by the file reader and (later) the HackRF source.
pub fn bytes_to_iq(bytes: &[u8], fmt: SampleFormat, out: &mut Vec<Iq>) {
    out.clear();
    // Rust note: `as_chunks::<N>()` gives fixed-size `&[u8; N]` chunks plus a
    // remainder, which lets `from_le_bytes` take the array without a runtime
    // length check — and keeps clippy happy about constant chunk sizes.
    match fmt {
        SampleFormat::Cs8 => {
            let (chunks, _rest) = bytes.as_chunks::<2>();
            for ch in chunks {
                let i = (ch[0] as i8) as f32 / 128.0;
                let q = (ch[1] as i8) as f32 / 128.0;
                out.push(Iq::new(i, q));
            }
        }
        SampleFormat::Cu8 => {
            let (chunks, _rest) = bytes.as_chunks::<2>();
            for ch in chunks {
                let i = (ch[0] as f32 - 127.5) / 128.0;
                let q = (ch[1] as f32 - 127.5) / 128.0;
                out.push(Iq::new(i, q));
            }
        }
        SampleFormat::Cs16 => {
            let (chunks, _rest) = bytes.as_chunks::<4>();
            for ch in chunks {
                let i = i16::from_le_bytes([ch[0], ch[1]]) as f32 / 32768.0;
                let q = i16::from_le_bytes([ch[2], ch[3]]) as f32 / 32768.0;
                out.push(Iq::new(i, q));
            }
        }
        SampleFormat::Cs24 => {
            // Each 24-bit value placed in the top of an i32, then scaled: the
            // shift keeps the sign.
            let s24 =
                |b: &[u8]| (i32::from_le_bytes([0, b[0], b[1], b[2]]) >> 8) as f32 / 8_388_608.0;
            let (chunks, _rest) = bytes.as_chunks::<6>();
            for ch in chunks {
                out.push(Iq::new(s24(&ch[0..3]), s24(&ch[3..6])));
            }
        }
        SampleFormat::Cf32 => {
            let (chunks, _rest) = bytes.as_chunks::<8>();
            for ch in chunks {
                let i = f32::from_le_bytes([ch[0], ch[1], ch[2], ch[3]]);
                let q = f32::from_le_bytes([ch[4], ch[5], ch[6], ch[7]]);
                out.push(Iq::new(i, q));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cs8_round_trips_scale() {
        let bytes = [0u8, 0, 127, 0, 0x80, 0x80];
        let mut out = Vec::new();
        bytes_to_iq(&bytes, SampleFormat::Cs8, &mut out);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0], Iq::new(0.0, 0.0));
        assert!((out[1].re - 127.0 / 128.0).abs() < 1e-6);
        // 0x80 = -128
        assert!((out[2].re + 1.0).abs() < 1e-6);
    }

    #[test]
    fn unsigned_8_and_24_bit_samples() {
        let mut out = Vec::new();
        bytes_to_iq(&[128, 127, 255, 0], SampleFormat::Cu8, &mut out);
        assert!(out[0].re.abs() < 0.01 && out[0].im.abs() < 0.01);
        assert!((out[1].re - 1.0).abs() < 0.01 && (out[1].im + 1.0).abs() < 0.01);
        // 24-bit: 0x400000 = +0.5, 0xC00000 = -0.5
        bytes_to_iq(&[0, 0, 0x40, 0, 0, 0xC0], SampleFormat::Cs24, &mut out);
        assert_eq!(out, vec![Iq::new(0.5, -0.5)]);
    }
}
