// made by CasualArclamp
//! ThermoPro TX2 temperature/humidity sensor: the Prologue protocol with
//! the battery flag inverted (1 = low). Off by default, as in rtl_433
//! (turn Prologue off and this on to use it).
//!
//! Ported from rtl_433's `src/devices/thermopro_tx2.c` (Copyright (C) 2020
//! Christian W. Zuckschwerdt, GPL-2.0-or-later).
//!
//! 36 bits sent 7 times after a sync pulse, PPM: a ~500 µs pulse, then a
//! ~2000 µs gap for 0 or ~4000 µs for 1; ~9000 µs between rows. Nine
//! nibbles: `[type] [id0] [id1] [flags] [temp0] [temp1] [temp2] [humi0]
//! [humi1]`, type 9 or 5, flags `L b C C` (battery low, button,
//! channel − 1), temperature 12-bit signed in 0.1 °C, humidity 0xcc when
//! there is no humidity sensor.

use crate::bits::BitBuffer;
use crate::device::*;
use crate::sim::{Air, Sample};

fn thermopro_tx2_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Alecto/Auriol-v2 sends 8 sync bits first: not one of ours.
    if bb.bits(0) <= 8 && bb.bits(0) != 0 {
        return DECODE_ABORT_EARLY;
    }
    // Only 3 repeats gives false positives on Alecto/Auriol-v2.
    let Some(r) = bb.find_repeated_row(4, 36) else {
        return DECODE_ABORT_EARLY;
    };
    if bb.bits(r) > 37 {
        return DECODE_ABORT_LENGTH;
    }
    let b = bb.row(r);
    if b[0] & 0xf0 != 0x90 && b[0] & 0xf0 != 0x50 {
        return DECODE_FAIL_SANITY;
    }
    let kind = b[0] >> 4;
    let id = ((b[0] & 0x0f) << 4) | ((b[1] & 0xf0) >> 4);
    let battery_low = b[1] & 0x08 != 0;
    let button = (b[1] & 0x04) >> 2;
    let channel = (b[1] & 0x03) + 1;
    // 12-bit signed at the top of an i16, shifted down with its sign.
    let temp_raw = (((b[2] as u16) << 8) | (b[3] & 0xf0) as u16) as i16 >> 4;
    let humidity = ((b[3] & 0x0f) << 4) | (b[4] >> 4);
    let m = Message::new("Thermopro-TX2")
        .int("subtype", kind)
        .int("id", id)
        .int("channel", channel)
        .int("battery_ok", !battery_low as u8)
        .float("temperature_C", temp_raw as f64 * 0.1, 2)
        .cond(humidity != 0xcc, |m| m.int("humidity", humidity))
        .int("button", button);
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "ThermoPro-TX2 temperature sensor",
    group: Group::Weather,
    modulation: Modulation::OokPpm,
    short_width: 2000.0,
    long_width: 4000.0,
    gap_limit: 7000.0,
    reset_limit: 10000.0,
    decode: thermopro_tx2_decode,
    disabled: true,
    // Alecto collision: if Alecto's checksum is right it is not this.
    priority: 10,
    ..Device::DEFAULT
}];

/// Made up for the simulator: type 5, id 0x3a, battery ok, button, channel
/// 2, -3.5 °C, no humidity sensor (5, 0x3a, flags 0101, 0xfdd, 0xcc), sent
/// 5 times.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Thermopro-TX2",
    code: "{36}53a5fddcc0 {36}53a5fddcc0 {36}53a5fddcc0 {36}53a5fddcc0 {36}53a5fddcc0",
    expect: &[
        ("subtype", "5"),
        ("id", "58"),
        ("channel", "2"),
        ("battery_ok", "1"),
        ("temperature_C", "-3.50"),
        ("button", "1"),
    ],
    air: Air {
        pulse_us: 500.0,
        row_gap_us: 8500.0,
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
    fn battery_low_and_humidity() {
        let m = decode_rows(
            &DEVICES[0],
            "{36}92c80eb340 {36}92c80eb340 {36}92c80eb340 {36}92c80eb340",
        );
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("battery_ok").unwrap().to_string(), "0");
        assert_eq!(m[0].get("humidity").unwrap().to_string(), "52");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "23.50");
    }
}
