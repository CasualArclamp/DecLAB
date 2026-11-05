// made by CasualArclamp
//! Efergy e2 classic electricity meter (current clamp), ~433.55 MHz.
//!
//! Ported from rtl_433's `src/devices/efergy_e2_classic.c` (Copyright (C)
//! 2015 Robert Högberg, GPL-2.0-or-later). Power calculations from
//! Nathaniel Elijah's EfergyRPI_001.
//!
//! FSK PWM, 64 bits (sometimes a spurious 65th), 8 bytes:
//! `[00] [id lo] [id hi] [L B II ....] [cur hi] [cur lo] [exp] [sum]`:
//! learn mode, battery, sending interval ((I + 1) × 6 s), a 16-bit current
//! scaled by 2^-(15 − exp), and the sum of the first 7 bytes.

use crate::bits::{BitBuffer, add_bytes};
use crate::device::*;
use crate::sim::{Air, Sample};

fn efergy_e2_classic_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    if bb.num_rows != 1 {
        return DECODE_ABORT_EARLY;
    }
    let num_bits = bb.bits(0);
    // Real captures are 64 or 65 bits.
    if !(64..=65).contains(&num_bits) {
        return DECODE_ABORT_LENGTH;
    }
    // The first nibble is 0000, or 1111 if pulses and gaps are mixed up.
    let mut bytes = [0u8; 8];
    bytes.copy_from_slice(&bb.row(0)[..8]);
    if bytes[0] & 0xf0 != 0xf0 && bytes[0] & 0xf0 != 0x00 {
        return DECODE_ABORT_EARLY;
    }
    if bytes[0] & 0xf0 != 0 {
        for b in bytes.iter_mut() {
            *b = !*b;
        }
    }
    let zero_count = bytes.iter().filter(|&&b| b == 0).count();
    if zero_count > 5 {
        return DECODE_FAIL_SANITY; // too many null bytes
    }
    let checksum = add_bytes(&bytes[..7]);
    if checksum == 0 {
        return DECODE_FAIL_SANITY; // reduce false positives
    }
    if (checksum & 0xff) as u8 != bytes[7] {
        ctx.log(|| {
            format!(
                "efergy_e2_classic: checksum {:02x} != {:02x}",
                checksum & 0xff,
                bytes[7]
            )
        });
        return DECODE_FAIL_MIC;
    }

    let address = ((bytes[2] as u16) << 8) | bytes[1] as u16;
    let learn = (bytes[3] & 0x80) >> 7;
    let interval = (((bytes[3] & 0x30) >> 4) + 1) * 6;
    let battery = (bytes[3] & 0x40) >> 6;
    // `-(int8_t)b + 15` in uint8_t arithmetic: wrapping ops do the same.
    let fact = (bytes[6] as i8).wrapping_neg().wrapping_add(15) as u8;
    if !(7..=23).contains(&fact) {
        return DECODE_FAIL_SANITY; // invalid exponent
    }
    let current = (((bytes[4] as u32) << 8) | bytes[5] as u32) as f32 / (1u32 << fact) as f32;

    let m = Message::new("Efergy-e2CT")
        .int("id", address)
        .int("battery_ok", (battery != 0) as u8)
        .float("current", current as f64, 2)
        .int("interval", interval)
        .str("learn", if learn != 0 { "YES" } else { "NO" })
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Efergy e2 classic",
    group: Group::Meter,
    modulation: Modulation::FskPwm,
    short_width: 64.0,
    long_width: 136.0,
    sync_width: 500.0,
    gap_limit: 200.0,
    reset_limit: 400.0,
    decode: efergy_e2_classic_decode,
    ..Device::DEFAULT
}];

/// rtl_433's test codes (4.64 A and 185.16 A).
pub static SAMPLES: &[Sample] = &[
    Sample {
        dev: 0,
        model: "Efergy-e2CT",
        code: "{65}0cc055604a41030f8",
        expect: &[
            ("id", "21952"),
            ("battery_ok", "1"),
            ("current", "4.64"),
            ("interval", "18"),
            ("learn", "NO"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 0,
        model: "Efergy-e2CT",
        code: "{65}0cc055605c9408798",
        expect: &[("id", "21952"), ("current", "185.16")],
        air: Air::DEFAULT,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_decode() {
        crate::devices::check_samples(DEVICES, SAMPLES);
    }

    #[test]
    fn inverted_and_bad_checksum() {
        // The first test code with every bit inverted (pulses and gaps
        // mixed up) still decodes.
        let m = decode_rows(&DEVICES[0], "{65}f33faa9fb5befcf00");
        assert_eq!(m[0].get("current").unwrap().to_string(), "4.64");
        // A changed checksum byte is refused.
        assert!(decode_rows(&DEVICES[0], "{65}0cc055604a41030e8").is_empty());
    }
}
