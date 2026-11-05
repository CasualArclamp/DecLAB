// made by CasualArclamp
//! Bresser Weather Center 5-in-1 (and the Bresser Professional Rain Gauge).
//!
//! Ported from rtl_433's `src/devices/bresser_5in1.c` (GPL-2.0-or-later;
//! Copyright (C) 2018 Daniel Krueger, (C) 2019 Christian W. Zuckschwerdt).
//!
//! 868.3 MHz FSK PCM, 124 µs bits, a transmission every 12 s: preamble
//! `aa aa aa aa aa 2d d4`, then 26 bytes. The first 13 bytes are the
//! bitwise inverse of the last 13 (the integrity check). In the second
//! half: `uu II sS GG DG WW W TT T HH RR RR Bt` — checksum (unused), ID,
//! startup/sensor type, wind gust (binary, 0.1 m/s, its MSB nibble out of
//! order), direction (22.5° steps), wind average (BCD, 0.1 m/s),
//! temperature (BCD, 0.1 °C, sign in byte 25), humidity (BCD), rain (BCD,
//! 0.1 mm), battery (0x80 low).

use crate::bits::BitBuffer;
use crate::device::*;
use crate::sim::{Air, Sample};

fn bresser_5in1_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 5] = [0xaa, 0xaa, 0xaa, 0x2d, 0xd4];
    let mut msg = [0u8; 26];

    if bb.num_rows != 1 || bb.bits(0) < 248 || bb.bits(0) > 440 {
        ctx.log(|| format!("bresser_5in1: bit_per_row {} out of range", bb.bits(0)));
        return DECODE_ABORT_EARLY;
    }
    let mut start_pos = bb.search(0, 0, &PREAMBLE, PREAMBLE.len() * 8);
    if start_pos == bb.bits(0) {
        return DECODE_ABORT_LENGTH;
    }
    start_pos += PREAMBLE.len() * 8;
    let mut len = bb.bits(0) - start_pos;
    if len.div_ceil(8) < msg.len() {
        ctx.log(|| format!("bresser_5in1: {len} too short"));
        return DECODE_ABORT_LENGTH;
    }
    // Truncate any excess bits.
    len = len.min(msg.len() * 8);
    bb.extract_bytes(0, start_pos, &mut msg, len);

    // The first 13 bytes must be the inverse of the last 13.
    for col in 0..msg.len() / 2 {
        if msg[col] ^ msg[col + 13] != 0xff {
            ctx.log(|| format!("bresser_5in1: parity wrong at {col}"));
            return DECODE_FAIL_MIC;
        }
    }

    let sensor_id = msg[14];

    // BCD digits; a temperature digit of 0xf means a sensor error.
    let temp_ok = msg[20] & 0x0f <= 9;
    let mut temp_raw = (msg[20] & 0x0f) as i32
        + ((msg[20] & 0xf0) >> 4) as i32 * 10
        + (msg[21] & 0x0f) as i32 * 100;
    if msg[25] & 0x0f != 0 {
        temp_raw = -temp_raw;
    }
    // rtl_433 computes in C `float`: f32 here gives the same rounding.
    let temperature = temp_raw as f32 * 0.1;

    let humidity_ok = msg[22] & 0x0f <= 9;
    let humidity = (msg[22] & 0x0f) as i32 + ((msg[22] & 0xf0) >> 4) as i32 * 10;

    let wind_direction_deg = ((msg[17] & 0xf0) >> 4) as f32 * 22.5;

    let gust_raw = (((msg[17] & 0x0f) as i32) << 8) + msg[16] as i32;
    let wind_gust = gust_raw as f32 * 0.1;

    let wind_raw = (msg[18] & 0x0f) as i32
        + ((msg[18] & 0xf0) >> 4) as i32 * 10
        + (msg[19] & 0x0f) as i32 * 100;
    let wind_avg = wind_raw as f32 * 0.1;

    let rain_raw = (msg[23] & 0x0f) as i32
        + ((msg[23] & 0xf0) >> 4) as i32 * 10
        + (msg[24] & 0x0f) as i32 * 100
        + ((msg[24] & 0xf0) >> 4) as i32 * 1000;
    let mut rain = rain_raw as f32 * 0.1;

    let battery_low = msg[25] & 0x80 != 0;
    let sensor_type = msg[15] & 0x7f;

    let m = if (0x39..=0x3b).contains(&sensor_type) {
        // A Bresser Professional Rain Gauge: rescale the rain reading.
        rain *= 2.5;
        Message::new("Bresser-ProRainGauge")
            .int("id", sensor_id)
            .int("battery_ok", !battery_low as u8)
            .cond(temp_ok, |m| m.float("temperature_C", temperature as f64, 1))
            .float("rain_mm", rain as f64, 1)
            .str("mic", "CHECKSUM")
    } else {
        Message::new("Bresser-5in1")
            .int("id", sensor_id)
            .int("battery_ok", !battery_low as u8)
            .cond(temp_ok, |m| m.float("temperature_C", temperature as f64, 1))
            .cond(humidity_ok, |m| m.int("humidity", humidity))
            .float("wind_max_m_s", wind_gust as f64, 1)
            .float("wind_avg_m_s", wind_avg as f64, 1)
            .float("wind_dir_deg", wind_direction_deg as f64, 1)
            .float("rain_mm", rain as f64, 1)
            .str("mic", "CHECKSUM")
    };
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Bresser Weather Center 5-in-1",
    group: Group::Weather,
    modulation: Modulation::FskPcm,
    short_width: 124.0,
    long_width: 124.0,
    reset_limit: 25000.0,
    decode: bresser_5in1_decode,
    ..Device::DEFAULT
}];

/// Made up (after rtl_433's `eb 93 7f eb …` example), behind a short
/// preamble. The slicer pads the row with zeros after its last 1 (up to
/// the reset gap's worth, ~221 bits), and the decoder takes at most 440
/// bits, so the message's last 1 has to come early: humidity 0x80 and
/// the rain, sign and battery bytes all 0.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Bresser-5in1",
    code: "{248}aaaaaa2dd4ed937feb9feeefcefd7fffffff126c8014601110310280000000",
    expect: &[
        ("id", "108"),
        ("battery_ok", "1"),
        ("temperature_C", "23.1"),
        ("humidity", "80"),
        ("wind_max_m_s", "2.0"),
        ("wind_avg_m_s", "1.1"),
        ("wind_dir_deg", "135.0"),
        ("rain_mm", "0.0"),
    ],
    air: Air::DEFAULT,
}];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_decode() {
        crate::devices::check_samples(DEVICES, SAMPLES);
    }

    #[test]
    fn rtl_433_examples() {
        // Large wind values from rtl_433 issue #1315: gust 37.4 m/s, average 27.5 m/s.
        let m = decode_rows(
            &DEVICES[0],
            "{264}aaaaaaaaaa2dd4e3fd7f897e8aed68feaf9bfdff1c0280768175129701506402000000",
        );
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("wind_max_m_s").unwrap().to_string(), "37.4");
        assert_eq!(m[0].get("wind_avg_m_s").unwrap().to_string(), "27.5");
        // Low battery, -7.0 °C.
        let m = decode_rows(
            &DEVICES[0],
            "{264}aaaaaaaaaa2dd4eda1ffff1fffef8fffd6dfff77125e0000e00010700029200088 0000",
        );
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "-7.0");
        assert_eq!(m[0].get("battery_ok").unwrap().to_string(), "0");
        // A Professional Rain Gauge (sensor type 0x39).
        let m = decode_rows(
            &DEVICES[0],
            "{264}aaaaaaaaaa2dd4edee46ffffffef9fff8b7debff1211b90000001060007482140000 00",
        );
        assert_eq!(m[0].model, "Bresser-ProRainGauge");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "6.0");
        // A changed bit fails the inverse check.
        let m = decode_rows(
            &DEVICES[0],
            "{248}aaaaaa2dd4eb937feb9feeeffcfcd6bfffff146c80146011100303294100 00",
        );
        assert!(m.is_empty());
    }
}
