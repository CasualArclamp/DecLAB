// made by CasualArclamp
//! Transmission mode I (EN 300 401 §14.2, Table 38), the mode every Band III
//! ensemble uses. Times are in samples of the system clock, 2.048 MHz.
//!
//! Modes II–IV (L band and below) are not supported: no UK transmitter uses
//! them, and Mode I's long guard is what SFN networks need.

/// The system clock: every DAB time is a multiple of 1/2.048 MHz.
pub const SAMPLE_RATE: f64 = 2_048_000.0;
/// Transmission frame, 96 ms.
pub const FRAME: usize = 196_608;
/// Null symbol.
pub const NULL: usize = 2656;
/// OFDM symbol, guard included.
pub const SYMBOL: usize = 2552;
/// Useful part of a symbol: the FFT size; carriers are 1 kHz apart.
pub const FFT: usize = 2048;
/// Guard interval (cyclic prefix), 246 µs.
pub const GUARD: usize = SYMBOL - FFT;
/// Symbols after the null: the phase reference, then 75 carrying data.
pub const SYMBOLS: usize = 76;
/// Data symbols: three of FIC, then 72 of MSC.
pub const DATA_SYMBOLS: usize = SYMBOLS - 1;
/// Active carriers, k = −768…768 less 0.
pub const CARRIERS: usize = 1536;
/// Bits a data symbol carries (2 per carrier).
pub const BITS_PER_SYMBOL: usize = 2 * CARRIERS;
/// FIC symbols (§14.2: symbols 2–4 in mode I).
pub const FIC_SYMBOLS: usize = 3;
/// CIFs a frame carries, 18 symbols each, every 24 ms.
pub const CIFS: usize = 4;
pub const SYMBOLS_PER_CIF: usize = 18;
/// Bits in a CIF: 864 capacity units of 64 bits.
pub const CIF_BITS: usize = SYMBOLS_PER_CIF * BITS_PER_SYMBOL;
/// FIC bits per frame, and per FIC block (one block per CIF).
pub const FIC_BITS: usize = FIC_SYMBOLS * BITS_PER_SYMBOL;
pub const FIC_BLOCK_BITS: usize = FIC_BITS / CIFS;

/// The FFT bin of carrier `k` (−768…768).
#[inline]
pub fn bin(k: i32) -> usize {
    k.rem_euclid(FFT as i32) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_i_adds_up() {
        assert_eq!(NULL + SYMBOLS * SYMBOL, FRAME);
        assert_eq!(GUARD, 504);
        assert_eq!(CIF_BITS, 864 * 64);
        assert_eq!(FIC_BLOCK_BITS, 2304);
        assert_eq!(FIC_SYMBOLS + CIFS * SYMBOLS_PER_CIF, DATA_SYMBOLS);
    }
}
