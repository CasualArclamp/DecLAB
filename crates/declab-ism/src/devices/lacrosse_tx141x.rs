// made by CasualArclamp
//! LaCrosse TX141-Bv2, TX141TH-Bv2, TX141-Bv3 (and -Bv4), TX141W and
//! TX145wsdth sensors, also TFA 30.3221.02, 30.3222.02, 30.3243.02,
//! 30.3249.02, 30.3251.10 and the ORIA WA50B.
//!
//! Ported from rtl_433's `src/devices/lacrosse_tx141x.c` (Copyright (C)
//! 2017 Robert Fraczkiewicz, changes by Andrew Rivett; GPL-2.0-or-later).
//!
//! OOK PWM with a fixed 625 µs bit period (417 µs pulse for 1, 208 µs for
//! 0, but the slicer reads short = 1, so the rows are inverted first),
//! after four 833 µs sync pulses; 12 (or 4) packets per burst. The packet
//! length tells the model:
//! - 40 bits (41 for TX141TH-Bv3), TX141TH-Bv2: `[id8] [B T CC temp12]
//!   [humi8] [digest8]`, temperature in 0.1 °C + 50 °C, an LFSR digest
//!   (0x31, key 0xf4, reflected) over the first four bytes;
//! - 37 bits TX141-Bv2, 33 to 36 bits TX141-Bv3, 32 bits TX141B: as above
//!   without humidity or check, and the battery bit means ok;
//! - 65 bits TX141W / TX145wsdth: `PRE5 ID19 BAT1 TEST1 CH2 TYPE4
//!   TEMP_WIND12 HUM_DIR12 CRC8 x`, type 1 temperature/humidity, type 2
//!   wind speed (0.1 km/h) and direction, CRC-8 (0x31, init 0) over seven
//!   bytes.

use crate::bits::{BitBuffer, crc8, lfsr_digest8_reflect};
use crate::device::*;
use crate::sim::{Air, Sample};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Tx141B,
    Tx141,
    Tx141Th,
    Tx141Bv3,
    Tx141W,
}

/// rtl_433's `bitbuffer_invert`. Private for now: `BitBuffer::invert` in
/// `bits.rs` computes `0xff >> 8` for a row that ends on a byte boundary,
/// which (shift overflow, masked to `>> 0` in release) flips the last byte
/// back; C's `0xFF >> 8` on an int is 0. Move here into bits.rs once fixed.
fn invert(bb: &mut BitBuffer) {
    for r in 0..bb.num_rows {
        let n = bb.bits(r);
        if n == 0 {
            continue;
        }
        let last_col = (n - 1) / 8;
        let last_bits = (n - 1) % 8 + 1;
        for c in 0..=last_col {
            bb.bb[r][c] = !bb.bb[r][c];
        }
        // Clear the bits past the row's end again (none when it ends on a
        // byte boundary: `checked_shr` gives None for a shift of 8).
        bb.bb[r][last_col] &= !0xffu8.checked_shr(last_bits as u32).unwrap_or(0);
    }
}

fn lacrosse_tx141x_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    invert(bb);

    // The most frequent packet: at least 5 of 12 or 3 of 4 repeats, so a
    // 4-repeat burst may carry a bogus extra row.
    let min_repeats = if bb.num_rows > 5 { 5 } else { 3 };
    // Rust note: `Option::or_else` tries the second search only when the
    // first found nothing (rtl_433's `if (r < 0) r = …`).
    let mut r = bb
        .find_repeated_row(min_repeats, 32)
        // TX141W/TX145wsdth: at least 2 of 3-7 repeats.
        .or_else(|| bb.find_repeated_row(2, 64));

    // TX141TH has its own check, so a single passing row is enough.
    if r.is_none() && bb.num_rows <= 4 {
        r = (0..bb.num_rows).find(|&row| {
            let n = bb.bits(row);
            let b = bb.row(row);
            (n == 40 || n == 41) && lfsr_digest8_reflect(&b[..4], 0x31, 0xf4) == b[4]
        });
    }
    let Some(r) = r else {
        return DECODE_ABORT_LENGTH;
    };

    let n = bb.bits(r);
    let kind = if n >= 64 {
        Kind::Tx141W
    } else if n > 41 {
        return DECODE_ABORT_LENGTH;
    } else if n >= 41 {
        if bb.num_rows > 12 {
            return DECODE_ABORT_LENGTH; // false positive with GT-WT03
        }
        Kind::Tx141Th // actually TX141TH-Bv3
    } else if n >= 40 {
        Kind::Tx141Th
    } else if n >= 37 {
        Kind::Tx141
    } else if n == 32 {
        Kind::Tx141B
    } else {
        Kind::Tx141Bv3
    };

    let b = bb.row(r);

    if kind == Kind::Tx141W {
        if b[0] >> 3 != 0x01 {
            return DECODE_ABORT_EARLY;
        }
        if crc8(&b[..8], 0x31, 0x00) != 0 {
            return DECODE_FAIL_MIC;
        }
        let id = (((b[0] & 0x07) as u64) << 16) | ((b[1] as u64) << 8) | b[2] as u64;
        let battery_low = b[3] >> 7;
        let test = (b[3] & 0x40) >> 6;
        let channel = ((b[3] & 0x30) >> 4) as u64;
        let subtype = b[3] & 0x0f;
        let temp_raw = ((b[4] as i32) << 4) | (b[5] >> 4) as i32;
        let humidity = (((b[5] & 0x0f) as i32) << 8) | b[6] as i32;
        let m = Message::new("LaCrosse-TX141W")
            .hex("id", id, 5)
            .hex("channel", channel, 1)
            .int("battery_ok", (battery_low == 0) as u8);
        let m = match subtype {
            1 => {
                // Temperature/humidity.
                let temp_c = (temp_raw - 500) as f32 * 0.1;
                m.float("temperature_C", temp_c as f64, 2)
                    .int("humidity", humidity)
            }
            2 => {
                // Wind: speed in the temperature field, direction in the
                // humidity field.
                let speed_kmh = temp_raw as f32 * 0.1;
                m.float("wind_avg_km_h", speed_kmh as f64, 1)
                    .int("wind_dir_deg", humidity)
            }
            _ => {
                ctx.log(|| format!("lacrosse_tx141x: unknown subtype: {subtype}"));
                return 0; // rtl_433's DECODE_FAIL_OTHER
            }
        };
        ctx.output(m.int("test", test).str("mic", "CRC"));
        return 1;
    }

    let id = b[0] as u64;
    let battery_low = if kind == Kind::Tx141Th {
        b[1] >> 7 != 0
    } else {
        // TX141 and TX141-Bv3: the bit means ok.
        b[1] >> 7 == 0
    };
    let test = (b[1] & 0x40) >> 6;
    let test_str = if test != 0 { "Yes" } else { "No" };
    let channel = (b[1] & 0x30) >> 4;
    let temp_raw = (((b[1] & 0x0f) as i32) << 8) | b[2] as i32;
    let temp_c = (temp_raw - 500) as f32 * 0.1;
    let humidity = if kind == Kind::Tx141Th { b[3] } else { 0 };

    if id == 0
        || (kind == Kind::Tx141Th && (humidity == 0 || humidity > 100))
        || !(-40.0..=140.0).contains(&temp_c)
    {
        ctx.log(|| {
            format!("lacrosse_tx141x: data error, id: {id}, humidity: {humidity}, temp: {temp_c}")
        });
        return DECODE_FAIL_SANITY;
    }

    let m = match kind {
        Kind::Tx141B => Message::new("LaCrosse-TX141B")
            .hex("id", id, 2)
            .float("temperature_C", temp_c as f64, 2)
            .int("battery_ok", !battery_low as u8)
            .str("test", test_str),
        Kind::Tx141 => Message::new("LaCrosse-TX141Bv2")
            .hex("id", id, 2)
            .int("channel", channel)
            .float("temperature_C", temp_c as f64, 2)
            .int("battery_ok", !battery_low as u8)
            .str("test", test_str),
        Kind::Tx141Bv3 => Message::new("LaCrosse-TX141Bv3")
            .hex("id", id, 2)
            .int("channel", channel)
            .int("battery_ok", !battery_low as u8)
            .float("temperature_C", temp_c as f64, 2)
            .str("test", test_str),
        _ => {
            // TX141TH-Bv2: check the digest.
            if lfsr_digest8_reflect(&b[..4], 0x31, 0xf4) != b[4] {
                ctx.log(|| "lacrosse_tx141x: checksum digest TX141TH failed".into());
                return DECODE_FAIL_MIC;
            }
            Message::new("LaCrosse-TX141THBv2")
                .hex("id", id, 2)
                .int("channel", channel)
                .int("battery_ok", !battery_low as u8)
                .float("temperature_C", temp_c as f64, 2)
                .int("humidity", humidity)
                .str("test", test_str)
                .str("mic", "CRC")
        }
    };
    ctx.output(m);
    1
}

// TX141W, TX145wsdth alone would be m=OOK_PWM, s=256, l=500, r=1888, y=748.
pub static DEVICES: &[Device] = &[Device {
    name: "LaCrosse TX141-Bv2, TX141TH-Bv2, TX141-Bv3, TX141W, TX145wsdth, (TFA, ORIA) sensor",
    group: Group::Weather,
    modulation: Modulation::OokPwm,
    short_width: 208.0, // short pulse 208 µs + 417 µs gap
    long_width: 417.0,  // long pulse 417 µs + 208 µs gap
    sync_width: 833.0,  // sync pulse 833 µs + 833 µs gap
    gap_limit: 625.0,   // a long gap (after a short pulse) is ~417 µs, a sync gap ~833 µs
    reset_limit: 1700.0,
    decode: lacrosse_tx141x_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator. The rows are as the slicer outputs them,
/// i.e. the inverse of the data (the decoder inverts them back).
pub static SAMPLES: &[Sample] = &[
    // TX141TH-Bv2: id 0x5c, channel 0, 22.3 °C, 48 %; data 5c 02 d3 30,
    // digest 0x60.
    Sample {
        dev: 0,
        model: "LaCrosse-TX141THBv2",
        code: "{40}a3fd2ccf9f {40}a3fd2ccf9f {40}a3fd2ccf9f {40}a3fd2ccf9f",
        expect: &[
            ("id", "0x5c"),
            ("channel", "0"),
            ("battery_ok", "1"),
            ("temperature_C", "22.30"),
            ("humidity", "48"),
            ("test", "No"),
            ("mic", "CRC"),
        ],
        air: Air::DEFAULT,
    },
    // TX141-Bv2 (37 bits): id 0x6d, channel 1, 18.5 °C.
    Sample {
        dev: 0,
        model: "LaCrosse-TX141Bv2",
        code: "{37}926d52fff8 {37}926d52fff8 {37}926d52fff8 {37}926d52fff8",
        expect: &[
            ("id", "0x6d"),
            ("channel", "1"),
            ("temperature_C", "18.50"),
            ("battery_ok", "1"),
        ],
        air: Air::DEFAULT,
    },
    // TX141B (32 bits): id 0x4e, -5.0 °C.
    Sample {
        dev: 0,
        model: "LaCrosse-TX141B",
        code: "{32}b17e3dff {32}b17e3dff {32}b17e3dff {32}b17e3dff",
        expect: &[
            ("id", "0x4e"),
            ("temperature_C", "-5.00"),
            ("battery_ok", "1"),
        ],
        air: Air::DEFAULT,
    },
    // TX141-Bv3 (33 bits): id 0x91, channel 2, 30.1 °C.
    Sample {
        dev: 0,
        model: "LaCrosse-TX141Bv3",
        code: "{33}6e5cdeff80 {33}6e5cdeff80 {33}6e5cdeff80 {33}6e5cdeff80",
        expect: &[
            ("id", "0x91"),
            ("channel", "2"),
            ("battery_ok", "1"),
            ("temperature_C", "30.10"),
        ],
        air: Air::DEFAULT,
    },
    // TX141W temperature/humidity: id 0x2a5c3, 19.6 °C, 63 %.
    Sample {
        dev: 0,
        model: "LaCrosse-TX141W",
        code: "{65}f55a3cfed47fc0aa00 {65}f55a3cfed47fc0aa00 {65}f55a3cfed47fc0aa00",
        expect: &[
            ("id", "0x2a5c3"),
            ("channel", "0x0"),
            ("battery_ok", "1"),
            ("temperature_C", "19.60"),
            ("humidity", "63"),
            ("test", "0"),
            ("mic", "CRC"),
        ],
        air: Air::DEFAULT,
    },
    // TX145wsdth wind: id 0x2a5c3, 12.3 km/h from 270°.
    Sample {
        dev: 0,
        model: "LaCrosse-TX141W",
        code: "{65}f55a3cfdf84ef18600 {65}f55a3cfdf84ef18600 {65}f55a3cfdf84ef18600",
        expect: &[
            ("id", "0x2a5c3"),
            ("wind_avg_km_h", "12.3"),
            ("wind_dir_deg", "270"),
            ("mic", "CRC"),
        ],
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
    fn tx141th_single_row_by_digest() {
        // No repeats, but the digest passes.
        let m = decode_rows(&DEVICES[0], "{40}a3fd2ccf9f");
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("humidity").unwrap().to_string(), "48");
    }

    #[test]
    fn tx141th_bad_digest() {
        assert!(
            decode_rows(&DEVICES[0], "{40}a3fd2ccf9e {40}a3fd2ccf9e {40}a3fd2ccf9e").is_empty()
        );
    }
}
