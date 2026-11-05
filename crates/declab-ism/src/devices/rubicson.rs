// made by CasualArclamp
//! Rubicson temperature sensor, also older TFA 30.3197, inFactory
//! ZX-7382-675 / ZX-7614-675 and the InFactory PT-310 pool sensor.
//!
//! Ported from rtl_433's `src/devices/rubicson.c` (GPL-2.0-or-later).
//!
//! 36 bits sent 12 times, PPM: a ~500 µs pulse, then a ~1000 µs gap for 0
//! or ~2000 µs for 1; ~4000 µs between rows (the PT-310 sends 37 or 38).
//! Nine nibbles: `[id0] [id1] [bat|0|chan] [temp0] [temp1] [temp2] [f]
//! [crc0] [crc1]`, temperature 12-bit signed in 0.1 °C, the CRC-8
//! (poly 0x31, init 0x6c) over the first seven nibbles.

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn rubicson_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let Some(r) = bb.find_repeated_row(3, 36) else {
        return DECODE_ABORT_EARLY;
    };
    // InFactory sensors send 38 bits (37 on the last repeat).
    if bb.bits(r) < 36 || bb.bits(r) > 38 {
        return DECODE_ABORT_LENGTH;
    }
    let b = bb.row(r);
    if b[3] & 0xf0 != 0xf0 {
        return DECODE_ABORT_EARLY; // const not 1111
    }
    // The seven data nibbles, a 0 nibble of padding, then the CRC nibbles:
    // a CRC over data and CRC together comes out 0.
    let tmp = [
        b[0],
        b[1],
        b[2],
        b[3] & 0xf0,
        ((b[3] & 0x0f) << 4) | ((b[4] & 0xf0) >> 4),
    ];
    if crc8(&tmp, 0x31, 0x6c) != 0 {
        return DECODE_FAIL_MIC;
    }
    let id = b[0];
    let battery = b[1] & 0x80 != 0;
    let channel = ((b[1] & 0x30) >> 4) + 1;
    // 12-bit signed: put it at the top of an i16 and shift back down, so
    // the sign bit is copied (an arithmetic shift).
    let temp_raw = (((b[1] as u16) << 12) | ((b[2] as u16) << 4)) as i16;
    let temp_c = (temp_raw >> 4) as f32 * 0.1;
    let m = Message::new("Rubicson-Temperature")
        .int("id", id)
        .int("channel", channel)
        .int("battery_ok", battery as u8)
        .float("temperature_C", temp_c as f64, 1)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Rubicson, TFA 30.3197 or InFactory PT-310 Temperature Sensor",
    group: Group::Weather,
    modulation: Modulation::OokPpm,
    short_width: 1000.0,
    long_width: 2000.0,
    gap_limit: 3000.0,
    reset_limit: 4800.0,
    decode: rubicson_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: id 0xa3, channel 1, battery ok, 21.7 °C
/// (0xa3, flags 1000, 0x0d9, f, CRC 0xcf), sent 4 times.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Rubicson-Temperature",
    code: "{36}a380d9fcf0 {36}a380d9fcf0 {36}a380d9fcf0 {36}a380d9fcf0",
    expect: &[
        ("id", "163"),
        ("channel", "1"),
        ("battery_ok", "1"),
        ("temperature_C", "21.7"),
        ("mic", "CRC"),
    ],
    air: Air {
        pulse_us: 500.0,
        row_gap_us: 3900.0,
        ..Air::DEFAULT
    },
}];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_decode() {
        crate::devices::check_samples(DEVICES, SAMPLES);
    }

    #[test]
    fn bad_crc_fails() {
        let m = decode_rows(&DEVICES[0], "{36}a380d9fce0 {36}a380d9fce0 {36}a380d9fce0");
        assert!(m.is_empty());
    }

    #[test]
    fn pt310_38_bits() {
        // The pool sensor's longer rows decode the same.
        let m = decode_rows(&DEVICES[0], "{38}a380d9fcf0 {38}a380d9fcf0 {38}a380d9fcf0");
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "21.7");
    }
}
