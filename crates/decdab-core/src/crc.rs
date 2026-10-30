// made by CasualArclamp
//! The CRCs DAB uses.
//!
//! - [`crc16_ccitt`]: CRC-16 with g(x) = x^16 + x^12 + x^5 + 1, register
//!   starting at all ones, the result inverted (EN 300 401 §5.2.1 and
//!   Annex E). It guards FIBs, MSC data groups, packets, PAD data groups,
//!   DAB+ access units (TS 102 563 §5.2) and MP2's ScF-CRC area's
//!   neighbours. Transmitted MSB first.

use std::sync::OnceLock;

fn table() -> &'static [u16; 256] {
    static TABLE: OnceLock<[u16; 256]> = OnceLock::new();
    TABLE.get_or_init(|| {
        let mut t = [0u16; 256];
        for (v, e) in t.iter_mut().enumerate() {
            let mut c = (v as u16) << 8;
            for _ in 0..8 {
                c = if c & 0x8000 != 0 {
                    (c << 1) ^ 0x1021
                } else {
                    c << 1
                };
            }
            *e = c;
        }
        t
    })
}

/// The CRC-16 register after `bytes`, from `init`, not inverted.
pub fn crc16_raw(bytes: &[u8], init: u16) -> u16 {
    let t = table();
    bytes
        .iter()
        .fold(init, |c, &b| (c << 8) ^ t[((c >> 8) as u8 ^ b) as usize])
}

/// DAB's CRC-16 over `bytes` (EN 300 401 Annex E): the value to send.
pub fn crc16_ccitt(bytes: &[u8]) -> u16 {
    !crc16_raw(bytes, 0xFFFF)
}

/// Whether `frame` ends in a correct DAB CRC-16 over the bytes before it.
pub fn crc16_ok(frame: &[u8]) -> bool {
    if frame.len() < 2 {
        return false;
    }
    let (data, crc) = frame.split_at(frame.len() - 2);
    crc16_ccitt(data) == u16::from_be_bytes([crc[0], crc[1]])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value() {
        // CRC-16/GENIBUS in the CRC catalogue: init FFFF, inverted output.
        assert_eq!(crc16_ccitt(b"123456789"), 0xD64E);
        let mut f = b"123456789".to_vec();
        f.extend_from_slice(&0xD64Eu16.to_be_bytes());
        assert!(crc16_ok(&f));
        f[3] ^= 1;
        assert!(!crc16_ok(&f));
    }
}
