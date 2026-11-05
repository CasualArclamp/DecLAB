// made by CasualArclamp
//! Oregon Scientific weather sensors, protocols v2.1 and v3 (THGR122N,
//! THGR228N, THGR968, THGR328N, WGR968, BHTR968, BTHR918, BTHGN129, RGR968,
//! THR228N, THN132N, AWR129, RTGN129, RTGN318, RTGR328N, THN129, RTHN129,
//! UVR128, THGR810, THN802, UV800, PCR800, WGR800) and the Owl CM130,
//! CM160, CM180 and CM180i energy monitors that share the v3 framing.
//!
//! Ported from rtl_433's `src/devices/oregon_scientific.c` (Copyright (C)
//! 2015 Helge Weissig, Denis Bodor, Tommy Vestermark, Karl Lattimer,
//! deennoo, pclov3r, onlinux, Pasquale Fiorillo; GPL-2.0-or-later). The
//! protocols are described in "Oregon Scientific RF Protocols"
//! (wmrx00.sourceforge.net). The v1 and SL109H sensors are other files.
//!
//! Manchester, ~488 µs half-bits (1024 Hz). Each 4-bit nibble is sent LSB
//! first, so the decoder reflects the nibbles. v2.1 sends every bit twice
//! (inverted, then as is): after a 0x55/0xaa preamble and the sync nibble
//! 0xa (0x99 doubled), the rest is Manchester-decoded once more. v3 sends
//! bits once, after a run of 0s (or 1s) and the sync nibble 0xa.
//!
//! After reflection: `[id16] [chan4 dev_lo4] [dev_hi4 flags4] [data…]
//! [sum8]`: a 16-bit sensor type, channel, rolling device id, battery low
//! (flags bit 2), BCD readings, and a sum-of-nibbles checksum with its two
//! nibbles swapped, at a nibble position that depends on the sensor.

use crate::bits::{BitBuffer, crc8, reflect_nibbles};
use crate::device::*;
use crate::sim::{Air, Sample};

// Sensor types (the first 16 bits after reflection).
const ID_THGR122N: u16 = 0x1d20; // also THGR228N
const ID_THGR968: u16 = 0x1d30;
const ID_BTHR918: u16 = 0x5d50;
const ID_BHTR968: u16 = 0x5d60;
const ID_RGR968: u16 = 0x2d10;
const ID_THR228N: u16 = 0xec40;
const ID_THN132N: u16 = 0xec40; // as THR228N, a different length
const ID_AWR129: u16 = 0xec41; // as THR228N with a hundreds digit
const ID_RTGN318: u16 = 0x0cc3; // only the low 12 bits are compared
const ID_RTGN129: u16 = 0x0cc3; // as RTGN318, a different length
// Rolling ids 0xf024, 0xf224, 0xf824, 0xfa24 (rebranded versions): three
// of the four nibbles are compared.
const ID_THGR810: u16 = 0xf024;
const ID_THGR810A: u16 = 0xf8b4; // unconfirmed version
const ID_THN802: u16 = 0xc844;
const ID_PCR800: u16 = 0x2914;
const ID_PCR800A: u16 = 0x2d14; // AU version?
const ID_WGR800: u16 = 0x1984;
const ID_WGR800A: u16 = 0x1994; // unconfirmed version
const ID_WGR968: u16 = 0x3d00;
const ID_UV800: u16 = 0xd874;
const ID_THN129: u16 = 0xcc43; // temperature only
const ID_RTHN129: u16 = 0x0cd3; // temperature and clock; low 12 bits
const ID_BTHGN129: u16 = 0x5d53; // pressure, temperature, humidity
const ID_UVR128: u16 = 0xec70;
const ID_THGR328N: u16 = 0xcc23; // as THR228N with 5 channels
// RTGR328N temperature/humidity part, channels 1 to 5.
const ID_RTGR328N_1: u16 = 0xdcc3;
const ID_RTGR328N_2: u16 = 0xccc3;
const ID_RTGR328N_3: u16 = 0xbcc3;
const ID_RTGR328N_4: u16 = 0xacc3;
const ID_RTGR328N_5: u16 = 0x9cc3;
// RTGR328N radio-clock part.
const ID_RTGR328N_6: u16 = 0x8ce3;
const ID_RTGR328N_7: u16 = 0x8ae3;

/// BCD temperature: tens in nibble 11, units in 8, tenths in 9 (0.1 °C);
/// the AWR129 BBQ thermometer adds hundreds in nibble 10's low 3 bits,
/// whose top bit is the sign. A C float, as rtl_433 computes it.
fn os_temperature(m: &[u8]) -> f32 {
    let mut t = (((m[5] >> 4) as i32 * 100)
        + ((m[4] & 0x0f) as i32 * 10)
        + ((m[4] >> 4) & 0x0f) as i32) as f32
        / 10.0;
    t += (m[5] & 0x07) as f32 * 100.0;
    if m[5] & 0x08 != 0 {
        t = -t;
    }
    t
}

/// Nibbles 11..8: rain rate, BCD.
fn os_rain_rate(m: &[u8]) -> f32 {
    (((m[5] & 0x0f) as i32 * 1000)
        + ((m[5] >> 4) as i32 * 100)
        + ((m[4] & 0x0f) as i32 * 10)
        + (m[4] >> 4) as i32) as f32
        / 100.0
}

/// Nibbles 17..12: total rain, BCD, last digit 0.001.
fn os_total_rain(m: &[u8]) -> f32 {
    (m[8] & 0x0f) as f32 * 100.0
        + ((m[8] >> 4) & 0x0f) as f32 * 10.0
        + (m[7] & 0x0f) as f32
        + ((m[7] >> 4) & 0x0f) as f32 / 10.0
        + (m[6] & 0x0f) as f32 / 100.0
        + ((m[6] >> 4) & 0x0f) as f32 / 1000.0
}

fn os_humidity(m: &[u8]) -> u32 {
    ((m[6] & 0x0f) as u32 * 10) + (m[6] >> 4) as u32
}

fn os_uv(m: &[u8]) -> u32 {
    ((m[4] & 0x0f) as u32 * 10) + (m[4] >> 4) as u32
}

fn swap_nibbles(b: u8) -> u8 {
    b.rotate_left(4)
}

fn cm180i_power(m: &[u8], offset: usize) -> u32 {
    let val = ((m[4 + offset * 2] as u32) << 8) | (m[3 + offset * 2] & 0xf0) as u32;
    // 1 + 1/160 matches the CM180's own display (rtl_433's note); C
    // converts the float product back to unsigned by truncation.
    (val as f32 * 1.00625) as u32
}

fn cm180i_total(m: &[u8]) -> u64 {
    // Sent only when nibble 3 is 0.
    if m[1] & 0x0f != 0 {
        return 0;
    }
    ((m[14] as u64) << 40)
        | ((m[13] as u64) << 32)
        | ((m[12] as u64) << 24)
        | ((m[11] as u64) << 16)
        | ((m[10] as u64) << 8)
        | m[9] as u64
}

fn cm180_power(m: &[u8]) -> u32 {
    let val = ((m[4] as u32) << 8) | (m[3] & 0xf0) as u32;
    (val as f32 * 1.00625) as u32
}

fn cm180_total(m: &[u8]) -> u64 {
    if m[1] & 0x0f != 0 {
        return 0;
    }
    ((m[10] as u64) << 40)
        + ((m[9] as u64) << 32)
        + ((m[8] as u64) << 24)
        + ((m[7] as u64) << 16)
        + ((m[6] as u64) << 8)
        + m[5] as u64
}

/// The v2.1/v3 checksum: the sum of nibbles `0..idx`, low byte, against
/// the byte whose low nibble is nibble `idx` and high nibble `idx + 1`.
/// True when it matches (rtl_433's `validate_os_checksum` returns 0 then).
fn checksum_ok(ctx: &mut Ctx, msg: &[u8], idx: usize) -> bool {
    let mut sum: u32 = 0;
    let mut i = 0;
    // Rust note: `idx - 1` would underflow for idx 0 (usize is unsigned);
    // no sensor uses it, but `i + 1 < idx` is the same test without that.
    while i + 1 < idx {
        let v = msg[i >> 1];
        sum += ((v >> 4) + (v & 0x0f)) as u32;
        i += 2;
    }
    let checksum = if idx & 1 != 0 {
        sum += (msg[idx >> 1] >> 4) as u32;
        (msg[idx >> 1] & 0x0f) as u32 | (msg[(idx + 1) >> 1] & 0xf0) as u32
    } else {
        ((msg[idx >> 1] >> 4) | ((msg[idx >> 1] & 0x0f) << 4)) as u32
    };
    sum &= 0xff;
    if sum == checksum {
        true
    } else {
        ctx.log(|| {
            format!(
                "oregon_scientific: checksum error, expected {checksum:02x} calculated {sum:02x}"
            )
        });
        false
    }
}

/// rtl_433's `validate_os_v2_message`: the message has the expected
/// length and its checksum is right.
fn v2_ok(
    ctx: &mut Ctx,
    msg: &[u8],
    bits_expected: usize,
    msg_bits: usize,
    checksum_idx: usize,
) -> bool {
    if bits_expected == msg_bits {
        return checksum_ok(ctx, msg, checksum_idx);
    }
    ctx.log(|| {
        format!(
            "oregon_scientific: bit validation error, expected {bits_expected} bits, got {msg_bits}"
        )
    });
    false
}

/// The fields every v2.1/v3 sensor starts with.
fn head(model: &str, device_id: u8, channel: u8, battery_low: bool) -> Message {
    Message::new(model)
        .int("id", device_id)
        .int("channel", channel)
        .int("battery_ok", !battery_low as u8)
}

fn v2_1_decode(ctx: &mut Ctx, bb: &BitBuffer) -> i32 {
    let b = bb.row(0);

    // Bytes 1 and 2 must be preamble (byte 0 may hold sync/start-up bit
    // errors).
    if (b[1] != 0x55 || b[2] != 0x55) && (b[1] != 0xaa || b[2] != 0xaa) {
        if b[3] != 0 {
            ctx.log(|| {
                format!(
                    "oregon_scientific: badly formatted OS v2.1 message {}",
                    bb.row_string(0)
                )
            });
        }
        return DECODE_ABORT_EARLY;
    }

    let mut databits = BitBuffer::new();
    // There may be extra or dropped bits: look for the sync byte at the
    // expected place, up to 7 bits later.
    let sync_test_val = u32::from_be_bytes([b[3], b[4], b[5], b[6]]);
    for pattern_index in 0..8 {
        let mask = 0xffff_0000u32 >> pattern_index;
        let pattern = 0x5599_0000u32 >> pattern_index;
        let pattern2 = 0xaa99_0000u32 >> pattern_index;
        if sync_test_val & mask != pattern && sync_test_val & mask != pattern2 {
            continue;
        }
        // The payload starts after the sync nibble: decode it out of the
        // doubled bits.
        bb.manchester_decode(0, pattern_index + 40, &mut databits, 173);
        break;
    }
    let msg_bits = databits.bits(0);
    // A copy of the (up to 173-bit = 22-byte) message, nibbles reflected.
    let mut msg = [0u8; 24];
    msg.copy_from_slice(&databits.row(0)[..24]);
    reflect_nibbles(&mut msg[..msg_bits.div_ceil(8)]);
    // Rust note: from here on `msg` is only read, so rebind it as a shared
    // slice the helpers take.
    let msg = &msg[..];

    let sensor_id = ((msg[0] as u16) << 8) | msg[1] as u16;
    let channel = (msg[2] >> 4) & 0x0f;
    let device_id = (msg[2] & 0x0f) | (msg[3] & 0xf0);
    let battery_low = (msg[3] >> 2) & 0x01 != 0;
    ctx.log(|| format!("oregon_scientific: found sensor type ({sensor_id:04x})"));

    let head = |model: &str| head(model, device_id, channel, battery_low);

    if sensor_id == ID_THGR122N || sensor_id == ID_THGR968 {
        if !v2_ok(ctx, msg, 68, msg_bits, 15) && !v2_ok(ctx, msg, 76, msg_bits, 15) {
            return 0;
        }
        // THGR122N sends a last byte outside the checksum.
        let model = if sensor_id == ID_THGR968 {
            "Oregon-THGR968"
        } else if msg_bits == 76 {
            "Oregon-THGR122N"
        } else {
            "Oregon-THGR228N"
        };
        ctx.output(
            head(model)
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_WGR968 {
        if !v2_ok(ctx, msg, 94, msg_bits, 17) {
            return 0;
        }
        let quadrant = ((msg[4] & 0x0f) as i32 * 10
            + ((msg[4] >> 4) & 0x0f) as i32
            + ((msg[5] >> 4) & 0x0f) as i32 * 100) as f32;
        let avg = ((msg[7] >> 4) & 0x0f) as f32 / 10.0
            + (msg[7] & 0x0f) as f32
            + ((msg[8] >> 4) & 0x0f) as f32 / 10.0;
        let gust = (msg[5] & 0x0f) as f32 / 10.0
            + ((msg[6] >> 4) & 0x0f) as f32
            + (msg[6] & 0x0f) as f32 / 10.0;
        ctx.output(
            head("Oregon-WGR968")
                .float("wind_max_m_s", gust as f64, 1)
                .float("wind_avg_m_s", avg as f64, 1)
                .float("wind_dir_deg", quadrant as f64, 1)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_BHTR968 {
        if !v2_ok(ctx, msg, 92, msg_bits, 19) {
            return 0;
        }
        let pressure = (((msg[7] & 0x0f) | (msg[8] & 0xf0)) as i32 + 856) as f32;
        ctx.output(
            head("Oregon-BHTR968")
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .float("pressure_hPa", pressure as f64, 0)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_BTHR918 {
        // As the BHTR968, a shorter message and another pressure offset.
        if !v2_ok(ctx, msg, 84, msg_bits, 19) {
            return 0;
        }
        let pressure = (((msg[7] & 0x0f) | (msg[8] & 0xf0)) as i32 + 795) as f32;
        ctx.output(
            head("Oregon-BTHR918")
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .float("pressure_hPa", pressure as f64, 0)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_RGR968 {
        if !v2_ok(ctx, msg, 80, msg_bits, 16) {
            return 0;
        }
        let rain_rate = ((msg[4] & 0x0f) as i32 * 100
            + (msg[4] >> 4) as i32 * 10
            + ((msg[5] >> 4) & 0x0f) as i32) as f32
            / 10.0;
        let total_rain = ((msg[7] & 0x0f) as i32 * 10000
            + (msg[7] >> 4) as i32 * 1000
            + (msg[6] & 0x0f) as i32 * 100
            + (msg[6] >> 4) as i32 * 10
            + (msg[5] & 0x0f) as i32) as f32
            / 10.0;
        ctx.output(
            head("Oregon-RGR968")
                .float("rain_rate_mm_h", rain_rate as f64, 2)
                .float("rain_mm", total_rain as f64, 2)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if (sensor_id == ID_THR228N || sensor_id == ID_AWR129) && msg_bits == 76 {
        if !v2_ok(ctx, msg, 76, msg_bits, 12) {
            return 0;
        }
        let model = if sensor_id == ID_THR228N {
            "Oregon-THR228N"
        } else {
            "Oregon-AWR129"
        };
        ctx.output(
            head(model)
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_THN132N && msg_bits == 64 {
        if !v2_ok(ctx, msg, 64, msg_bits, 12) {
            return 0;
        }
        if ((msg[5] >> 4) & 0x0f) > 9 || (msg[4] & 0x0f) > 9 || ((msg[4] >> 4) & 0x0f) > 9 {
            ctx.log(|| "oregon_scientific: THN132N message failed BCD sanity check".into());
            return DECODE_FAIL_SANITY;
        }
        let temp_c = os_temperature(msg);
        if !(-50.0..=70.0).contains(&temp_c) {
            ctx.log(|| {
                format!("oregon_scientific: THN132N failed value sanity check: temp {temp_c:.1} C")
            });
            return DECODE_FAIL_SANITY;
        }
        ctx.output(
            head("Oregon-THN132N")
                .float("temperature_C", temp_c as f64, 2)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if (sensor_id & 0x0fff) == ID_RTGN129 && msg_bits == 80 {
        if !v2_ok(ctx, msg, 80, msg_bits, 15) {
            return 0;
        }
        ctx.output(
            head("Oregon-RTGN129")
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if [
        ID_RTGR328N_1,
        ID_RTGR328N_2,
        ID_RTGR328N_3,
        ID_RTGR328N_4,
        ID_RTGR328N_5,
    ]
    .contains(&sensor_id)
        && msg_bits == 173
    {
        if !v2_ok(ctx, msg, 173, msg_bits, 15) {
            return 0;
        }
        ctx.output(
            head("Oregon-RTGR328N")
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_RTGR328N_6 || sensor_id == ID_RTGR328N_7 {
        if !v2_ok(ctx, msg, 100, msg_bits, 21) {
            return 0;
        }
        // BCD, each byte's low nibble the tens.
        let bcd = |v: u8| (v & 0x0f) as u32 * 10 + ((v & 0xf0) >> 4) as u32;
        let year = bcd(msg[9]) + 2000;
        let month = (msg[8] & 0xf0) >> 4;
        let day = bcd(msg[7]);
        let hours = bcd(msg[6]);
        let minutes = bcd(msg[5]);
        let seconds = bcd(msg[4]);
        let clock = format!("{year:04}-{month:02}-{day:02}T{hours:02}:{minutes:02}:{seconds:02}");
        ctx.output(
            head("Oregon-RTGR328N")
                .str("radio_clock", clock)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if (sensor_id & 0x0fff) == ID_RTGN318 {
        if msg_bits == 76 && v2_ok(ctx, msg, 76, msg_bits, 15) {
            ctx.output(
                head("Oregon-RTGN318")
                    .float("temperature_C", os_temperature(msg) as f64, 2)
                    .int("humidity", os_humidity(msg))
                    .str("mic", "CHECKSUM"),
            );
            return 1;
        } else if msg_bits == 100 && v2_ok(ctx, msg, 100, msg_bits, 21) {
            // An RF clock message? Not decoded.
            return 0;
        }
    } else if sensor_id == ID_THN129 || (sensor_id & 0x0fff) == ID_RTHN129 {
        if v2_ok(ctx, msg, 68, msg_bits, 12) {
            let model = if sensor_id == ID_THN129 {
                "Oregon-THN129"
            } else {
                "Oregon-RTHN129"
            };
            ctx.output(
                head(model)
                    .float("temperature_C", os_temperature(msg) as f64, 2)
                    .str("mic", "CHECKSUM"),
            );
            return 1;
        } else if msg_bits == 104 && v2_ok(ctx, msg, 104, msg_bits, 18) {
            // An RF clock message: not decoded.
            return 0;
        }
    } else if sensor_id == ID_BTHGN129 {
        if !v2_ok(ctx, msg, 92, msg_bits, 19) {
            return 0;
        }
        // In hPa; the offset may need adjusting for altitude (rtl_433's
        // note).
        let pressure = ((((msg[7] & 0x0f) | (msg[8] & 0xf0)) as i32) * 2
            + (msg[8] & 0x01) as i32
            + 600) as f32;
        ctx.output(
            head("Oregon-BTHGN129")
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .float("pressure_hPa", pressure as f64, 2)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_UVR128 && msg_bits == 148 {
        if !v2_ok(ctx, msg, 148, msg_bits, 12) {
            return 0;
        }
        if ((msg[4] >> 4) & 0x0f) > 9 || (msg[4] & 0x0f) > 9 {
            ctx.log(|| "oregon_scientific: UVR128 message failed BCD sanity check".into());
            return DECODE_FAIL_SANITY;
        }
        let uvidx = os_uv(msg);
        if uvidx > 25 {
            ctx.log(|| format!("oregon_scientific: UVR128 failed values sanity check: uv {uvidx}"));
            return DECODE_FAIL_SANITY;
        }
        // No channel field for this one.
        ctx.output(
            Message::new("Oregon-UVR128")
                .int("id", device_id)
                .float("uvi", uvidx as f64, 0)
                .int("battery_ok", !battery_low as u8)
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if sensor_id == ID_THGR328N {
        if !v2_ok(ctx, msg, 173, msg_bits, 15) {
            return 0;
        }
        ctx.output(
            head("Oregon-THGR328N")
                .float("temperature_C", os_temperature(msg) as f64, 2)
                .int("humidity", os_humidity(msg))
                .str("mic", "CHECKSUM"),
        );
        return 1;
    } else if msg_bits > 16 {
        ctx.log(|| {
            format!("oregon_scientific: unrecognized OS v2.1 message (type {sensor_id:04x})")
        });
    } else {
        ctx.log(|| "oregon_scientific: possible OS v2.1 message, but sync nibble not found".into());
    }
    0
}

/// ceil((335 + 11) / 8): the longest v3 message rtl_433 takes.
const EXPECTED_NUM_BYTES: usize = 44;

fn v3_decode(ctx: &mut Ctx, bb: &BitBuffer) -> i32 {
    let b = bb.row(0);

    // A v3 preamble of 1s or of 0s.
    if (b[0] & 0x0f != 0x0f || b[1] != 0xff || b[2] & 0xc0 != 0xc0)
        && (b[0] & 0x0f != 0x00 || b[1] != 0x00 || b[2] & 0xc0 != 0x00)
    {
        if b[3] != 0 {
            ctx.log(|| {
                format!(
                    "oregon_scientific: unrecognized msg in OS v3 {}",
                    bb.row_string(0)
                )
            });
        }
        return DECODE_ABORT_EARLY;
    }

    // e.g. WGR800X has {335} 00 00 00 b1 22 40 0e 00 06 00 00 00 19 7c
    // three times; aligned (at 11) and reflected that is 3 packets of
    // 00 0a 19 84 00 e0 00 c0 00 00 00 3d 70.

    // Full preamble 00 00 00 5 (shorter for the WGR800X).
    const OS_PATTERN: [u8; 2] = [0x00, 0x05];
    // CM180: 00 00 00 46, the 0x46 already data; CM180i likewise 0x4a.
    const CM180_PATTERN: [u8; 2] = [0x00, 0x46];
    const CM180I_PATTERN: [u8; 2] = [0x00, 0x4a];
    // CM130: 00 00 00 60, the 0x60 already data.
    const CM130_PATTERN: [u8; 4] = [0x00, 0x00, 0x00, 0x60];
    // A broken Manchester demodulator: CM160's preamble may look like
    // 7f ff ff aa, i.e. ff ff f5.
    const ALT_PATTERN: [u8; 2] = [0xff, 0xf5];

    // Rust note: positions in i32, as C's int, so "bits after the
    // pattern" can go negative when the search found nothing (it returns
    // the row length).
    let bits = bb.bits(0) as i32;
    let os_pos = bb.search(0, 0, &OS_PATTERN, 16) as i32 + 16;
    let cm180_pos = bb.search(0, 0, &CM180_PATTERN, 16) as i32 + 8; // keep the 0x46
    let cm180i_pos = bb.search(0, 0, &CM180I_PATTERN, 16) as i32 + 8; // keep the 0x4a
    let cm130_pos = bb.search(0, 0, &CM130_PATTERN, 32) as i32 + 24; // keep the 0x60
    let alt_pos = bb.search(0, 0, &ALT_PATTERN, 16) as i32 + 16;

    let (msg_pos, msg_len) = if bits - os_pos >= 7 * 8 {
        (os_pos, bits - os_pos)
    } else if bits - cm180_pos >= 52 {
        // 52 bits: secondary frame (instant watts only); 108 bits: primary
        // frame (instant watts and cumulative watt-hours).
        (cm180_pos, bits - cm180_pos)
    } else if bits - cm180i_pos >= 84 {
        (cm180i_pos, bits - cm180i_pos)
    } else if bits - cm130_pos >= 96 {
        (cm130_pos, bits - cm130_pos)
    } else if bits - alt_pos >= 7 * 8 {
        (alt_pos, bits - alt_pos)
    } else {
        (0, 0)
    };
    if msg_len == 0 || msg_len as usize > EXPECTED_NUM_BYTES * 8 {
        return DECODE_ABORT_EARLY;
    }
    let msg_len = msg_len as usize;
    let mut msg = [0u8; EXPECTED_NUM_BYTES];
    bb.extract_bytes(0, msg_pos as usize, &mut msg, msg_len);
    reflect_nibbles(&mut msg[..msg_len.div_ceil(8)]);

    // Not for the CM sensors.
    let sensor_id = ((msg[0] as u16) << 8) | msg[1] as u16;
    let channel = (msg[2] >> 4) & 0x0f;
    let device_id = (msg[2] & 0x0f) | (msg[3] & 0xf0);
    let battery_low = (msg[3] >> 2) & 0x01 != 0;
    let head = |model: &str| head(model, device_id, channel, battery_low);

    if (sensor_id & 0xf0ff) == ID_THGR810 || sensor_id == ID_THGR810A {
        if !checksum_ok(ctx, &msg, 15) {
            return DECODE_FAIL_MIC;
        }
        if ((msg[5] >> 4) & 0x0f) > 9
            || (msg[4] & 0x0f) > 9
            || ((msg[4] >> 4) & 0x0f) > 9
            || (msg[6] & 0x0f) > 9
            || ((msg[6] >> 4) & 0x0f) > 9
        {
            ctx.log(|| "oregon_scientific: THGR810 message failed BCD sanity check".into());
            return DECODE_FAIL_SANITY;
        }
        let temp_c = os_temperature(&msg);
        let humidity = os_humidity(&msg);
        if !(-50.0..=70.0).contains(&temp_c) {
            ctx.log(|| format!("oregon_scientific: THGR810 failed value sanity check: temp {temp_c:.1} C hum {humidity}%"));
            return DECODE_FAIL_SANITY;
        }
        let tx_button = msg[0] & 1; // an unused sensor id bit
        let m = Message::new("Oregon-THGR810")
            .int("id", device_id)
            .int("channel", channel)
            .cond(tx_button != 0, |m| m.int("button", tx_button))
            .int("battery_ok", !battery_low as u8)
            .float("temperature_C", temp_c as f64, 2)
            .int("humidity", humidity)
            .str("mic", "CHECKSUM");
        ctx.output(m);
        1
    } else if sensor_id == ID_THN802 {
        if !checksum_ok(ctx, &msg, 12) {
            return DECODE_FAIL_MIC;
        }
        ctx.output(
            head("Oregon-THN802")
                .float("temperature_C", os_temperature(&msg) as f64, 2)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if sensor_id == ID_UV800 {
        if !checksum_ok(ctx, &msg, 13) {
            return DECODE_FAIL_MIC;
        }
        ctx.output(
            head("Oregon-UV800")
                .float("uvi", os_uv(&msg) as f64, 0)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if sensor_id == ID_PCR800 {
        if !checksum_ok(ctx, &msg, 18) {
            return DECODE_FAIL_MIC;
        }
        // Every nibble of 8..17 must be a BCD digit.
        if msg[4..9].iter().any(|&v| v & 0x0f > 9 || v >> 4 > 9) {
            ctx.log(|| "oregon_scientific: PCR800 message failed BCD sanity check".into());
            return DECODE_FAIL_SANITY;
        }
        ctx.output(
            head("Oregon-PCR800")
                .float("rain_rate_in_h", os_rain_rate(&msg) as f64, 1)
                .float("rain_in", os_total_rain(&msg) as f64, 3)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if sensor_id == ID_PCR800A {
        if !checksum_ok(ctx, &msg, 18) {
            return DECODE_FAIL_MIC;
        }
        ctx.output(
            head("Oregon-PCR800a")
                .float("rain_rate_in_h", os_rain_rate(&msg) as f64, 1)
                .float("rain_in", os_total_rain(&msg) as f64, 1)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if sensor_id == ID_WGR800 || sensor_id == ID_WGR800A {
        if !checksum_ok(ctx, &msg, 17) {
            return DECODE_FAIL_MIC;
        }
        if (msg[5] & 0x0f) > 9
            || ((msg[6] >> 4) & 0x0f) > 9
            || (msg[6] & 0x0f) > 9
            || ((msg[7] >> 4) & 0x0f) > 9
            || (msg[7] & 0x0f) > 9
            || ((msg[8] >> 4) & 0x0f) > 9
        {
            ctx.log(|| "oregon_scientific: WGR800 failed BCD sanity check".into());
            return DECODE_FAIL_SANITY;
        }
        let gust = (msg[5] & 0x0f) as f32 / 10.0
            + ((msg[6] >> 4) & 0x0f) as f32
            + (msg[6] & 0x0f) as f32 * 10.0;
        let avg = ((msg[7] >> 4) & 0x0f) as f32 / 10.0
            + (msg[7] & 0x0f) as f32
            + ((msg[8] >> 4) & 0x0f) as f32 * 10.0;
        let quadrant = ((msg[4] >> 4) & 0x0f) as f32 * 22.5;
        if !(0.0..=56.0).contains(&gust) || !(0.0..=56.0).contains(&avg) {
            ctx.log(|| format!("oregon_scientific: WGR800 failed value sanity check: gust {gust:.1} avg {avg:.1}"));
            return DECODE_FAIL_SANITY;
        }
        ctx.output(
            head("Oregon-WGR800")
                .float("wind_max_m_s", gust as f64, 1)
                .float("wind_avg_m_s", avg as f64, 1)
                .float("wind_dir_deg", quadrant as f64, 1)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if (0x20..=0x24).contains(&msg[0]) {
        // Owl CM160.
        msg[0] &= 0x0f;
        if !checksum_ok(ctx, &msg, 22) {
            return DECODE_FAIL_MIC;
        }
        let id = msg[1] & 0x0f;
        let current_amps = swap_nibbles(msg[3]) as u32 | (((msg[4] >> 4) as u32) << 8);
        // Assuming 230 V mains, as rtl_433 does.
        let current_watts = current_amps as f64 * 0.07 * 230.0;
        let total_amps = (((swap_nibbles(msg[10]) as u64) << 36)
            | ((swap_nibbles(msg[9]) as u64) << 28)
            | ((swap_nibbles(msg[8]) as u64) << 20)
            | ((swap_nibbles(msg[7]) as u64) << 12)
            | ((swap_nibbles(msg[6]) as u64) << 4)
            | (msg[5] & 0x0f) as u64) as f64;
        // × 1.12 matches the CM160's display between readings.
        let total_kwh = total_amps * 230.0 / 3600.0 / 1000.0 * 1.12;
        ctx.output(
            Message::new("Oregon-CM160")
                .int("id", id)
                .float("power_W", current_watts, 4)
                .float("energy_kWh", total_kwh, 4)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if msg[0] == 0x26 {
        // Owl CM180.
        msg[0] &= 0x0f;
        if !checksum_ok(ctx, &msg, 23) {
            return DECODE_FAIL_MIC;
        }
        for v in msg.iter_mut() {
            *v = swap_nibbles(*v);
        }
        let sequence = msg[1] & 0x0f;
        let id = ((msg[2] as u32) << 8) | (msg[1] & 0xf0) as u32;
        let batt_low = msg[3] & 0x1; // bit 8, not bit 6 as the others
        let ipower = cm180_power(&msg);
        let itotal = cm180_total(&msg);
        let total_energy = itotal as f32 / 3600.0 / 1000.0;
        ctx.output(
            Message::new("Oregon-CM180")
                .int("id", id)
                .int("battery_ok", (batt_low == 0) as u8)
                .int("power_W", ipower)
                .cond(itotal != 0, |m| {
                    m.float("energy_kWh", total_energy as f64, 2)
                })
                .int("sequence", sequence)
                .str("mic", "CHECKSUM"),
        );
        1
    } else if msg[0] == 0x25 {
        // Owl CM180i: rtl_433 does not check its checksum yet.
        msg[0] &= 0x0f;
        for v in msg.iter_mut() {
            *v = swap_nibbles(*v);
        }
        let sequence = msg[1] & 0x0f;
        let id = ((msg[2] as u32) << 8) | (msg[1] & 0xf0) as u32;
        let batt_low = msg[3] & 0x40 != 0;
        let ipower1 = cm180i_power(&msg, 0);
        let ipower2 = cm180i_power(&msg, 1);
        let ipower3 = cm180i_power(&msg, 2);
        let itotal = if msg_len >= 140 {
            cm180i_total(&msg)
        } else {
            0
        };
        // Ws (J) to kWh.
        let total_energy = itotal as f32 / 3600.0 / 1000.0;
        ctx.output(
            Message::new("Oregon-CM180i")
                .int("id", id)
                .int("battery_ok", !batt_low as u8)
                .int("power1_W", ipower1)
                .int("power2_W", ipower2)
                .int("power3_W", ipower3)
                .cond(itotal != 0, |m| {
                    m.float("energy_kWh", total_energy as f64, 2)
                })
                .int("sequence", sequence),
        );
        1
    } else if msg[0] == 0x60 {
        // Owl CM130: 12 bytes `60 II II PP 00 P. EE EE EE EE XX`, a CRC-8
        // (poly 0x07) over bytes 1..10 against byte 11 nibble-swapped.
        if crc8(&msg[1..11], 0x07, 0x00) != swap_nibbles(msg[11]) {
            ctx.log(|| "oregon_scientific: CM130 checksum fail".into());
            return DECODE_FAIL_MIC;
        }
        for v in msg[..12].iter_mut() {
            *v = swap_nibbles(*v);
        }
        let id = msg[2];
        // 16-bit little-endian count, 16 W each.
        let power_w = (((msg[4] as u32) << 8) | msg[3] as u32) * 16;
        // 32-bit little-endian lifetime energy, 1/8192 kWh each (fitted
        // against the console, rtl_433 #1493).
        let energy_cnt = u32::from_le_bytes([msg[6], msg[7], msg[8], msg[9]]);
        let energy_kwh = energy_cnt as f32 / 8192.0;
        ctx.output(
            Message::new("Oregon-CM130")
                .int("id", id)
                .int("power_W", power_w)
                .float("energy_kWh", energy_kwh as f64, 2)
                .str("mic", "CRC"),
        );
        1
    } else {
        if msg[0] != 0 && msg[1] != 0 {
            ctx.log(|| {
                "oregon_scientific: message received from unrecognized Oregon Scientific v3 sensor"
                    .into()
            });
        } else if b[3] != 0 {
            ctx.log(|| {
                "oregon_scientific: possible OS v3 message, but sync nibble not found".into()
            });
        }
        DECODE_FAIL_SANITY
    }
}

fn oregon_scientific_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let ret = v2_1_decode(ctx, bb);
    if ret > 0 {
        return ret;
    }
    v3_decode(ctx, bb)
}

pub static DEVICES: &[Device] = &[Device {
    name: "Oregon Scientific Weather Sensor",
    group: Group::Weather,
    modulation: Modulation::OokManchesterZeroBit,
    // Nominally 1024 Hz (488 µs), but pulses are shorter than pauses.
    short_width: 440.0,
    long_width: 0.0,
    reset_limit: 2400.0,
    decode: oregon_scientific_decode,
    ..Device::DEFAULT
}];

/// Shorthand for the samples: they all go on the air the same way.
const fn s(
    model: &'static str,
    code: &'static str,
    expect: &'static [(&'static str, &'static str)],
) -> Sample {
    Sample {
        dev: 0,
        model,
        code,
        expect,
        air: Air::DEFAULT,
    }
}

/// Made up for the simulator, one per sensor type, each with a valid
/// checksum. Rows as the slicer outputs them (the first bit is its
/// hard-coded 0). v2.1: preamble 55 55 55 55, sync 99, then each message
/// bit doubled; v3: 00 00 00, sync 05 (or the CM sensors' 46, 4a, 60),
/// then the message bits.
pub static SAMPLES: &[Sample] = &[
    // --- v2.1
    s(
        "Oregon-THGR122N",
        "{192}55555555996a659aaa6a6699aaa66a9aaa9666aa9a5aaaaa",
        &[
            ("id", "165"),
            ("channel", "1"),
            ("battery_ok", "1"),
            ("temperature_C", "21.40"),
            ("humidity", "56"),
        ],
    ),
    s(
        "Oregon-THGR228N",
        "{176}55555555996a659aaa9aa55aaa5a66aaa96aa9aa995a",
        &[
            ("id", "60"),
            ("channel", "2"),
            ("temperature_C", "-5.30"),
            ("humidity", "81"),
        ],
    ),
    s(
        "Oregon-THGR968",
        "{176}55555555996a655aaaa65656aa69696aaaaaa6aa995a",
        &[
            ("id", "119"),
            ("channel", "4"),
            ("temperature_C", "19.90"),
            ("humidity", "40"),
        ],
    ),
    s(
        "Oregon-WGR968",
        "{228}55555555995a65aaaaaa9a6aaa669a9a5aa6aa9a6aaa969aaaaaaaaaa0",
        &[
            ("id", "18"),
            ("wind_max_m_s", "4.3"),
            ("wind_avg_m_s", "1.2"),
            ("wind_dir_deg", "225.0"),
        ],
    ),
    s(
        "Oregon-BHTR968",
        "{224}5555555599666596aaaaa9a9aa669a9aaa66a6aa9969aaaa65a6aaaa",
        &[
            ("id", "136"),
            ("temperature_C", "22.50"),
            ("humidity", "45"),
            ("pressure_hPa", "1010"),
        ],
    ),
    s(
        "Oregon-BTHR918",
        "{208}5555555599666566aaaa6969aa56a96aaa9a66aa6565aaaa9966",
        &[
            ("id", "153"),
            ("temperature_C", "18.70"),
            ("humidity", "52"),
            ("pressure_hPa", "1016"),
        ],
    ),
    s(
        "Oregon-RGR968",
        "{200}55555555999a656aaaaa6a9aaa9aaa665aa66aaaaa9a9aaaaa",
        &[
            ("id", "33"),
            ("rain_rate_mm_h", "2.50"),
            ("rain_mm", "14.30"),
        ],
    ),
    s(
        "Oregon-THR228N",
        "{192}555555559995a5a6aa6a5966aa5a9a6aaa665aaaaaaaaaaa",
        &[("id", "91"), ("channel", "1"), ("temperature_C", "12.30")],
    ),
    s(
        "Oregon-AWR129",
        "{192}555555559995a5a66a6aa566aaa666a96a5aa6aaaaaaaaaa",
        &[("id", "92"), ("temperature_C", "185.40")],
    ),
    s(
        "Oregon-THN132N",
        "{168}555555559995a5a6aa9a9596aa56a9aaa959a6aaaa",
        &[("id", "110"), ("channel", "2"), ("temperature_C", "-8.70")],
    ),
    s(
        "Oregon-RTGN129",
        "{200}5555555599aaa5a55a5a6aa6aa66aa9aaa6a96aa6a5aaaaaaa",
        &[
            ("id", "65"),
            ("channel", "3"),
            ("temperature_C", "20.50"),
            ("humidity", "61"),
        ],
    ),
    s(
        "Oregon-RTGR328N",
        "{386}5555555599a5a5a55a9a9aa6aa96566aaa69a6aa99a6aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa80",
        &[
            ("id", "66"),
            ("channel", "2"),
            ("temperature_C", "17.60"),
            ("humidity", "49"),
        ],
    ),
    s(
        "Oregon-RTGR328N",
        "{240}5555555599a9a5955a9a9aa6aaaa5a66a69a6a666a965a969aaa5a66aaaa",
        &[
            ("id", "66"),
            ("channel", "2"),
            ("radio_clock", "2026-06-15T12:45:30"),
        ],
    ),
    s(
        "Oregon-RTGN318",
        "{192}5555555599aaa5a55a6a5aa6aa9a6a6aaa9a56aaaa5aaaaa",
        &[
            ("id", "67"),
            ("channel", "1"),
            ("temperature_C", "11.20"),
            ("humidity", "72"),
        ],
    ),
    s(
        "Oregon-THN129",
        "{176}5555555599a5a5a65a6aa6a6aaa696aaaa9a5aaaaaaa",
        &[("id", "68"), ("temperature_C", "6.40")],
    ),
    s(
        "Oregon-RTHN129",
        "{176}5555555599aaa5655a6a66a6aaa95a9aaa5a5aaaaaaa",
        &[("id", "69"), ("temperature_C", "23.80")],
    ),
    s(
        "Oregon-BTHGN129",
        "{224}55555555996665665a6a96a6aa6a6a9aaaaa66aaa56a6aaaa55aaaaa",
        &[
            ("id", "70"),
            ("temperature_C", "21.10"),
            ("humidity", "50"),
            ("pressure_hPa", "657.00"),
        ],
    ),
    s(
        "Oregon-UVR128",
        "{336}555555559995a556aaaa56a6aa56aaaaaa5a5aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        &[("id", "71"), ("uvi", "7")],
    ),
    s(
        "Oregon-THGR328N",
        "{386}5555555599a5a59a5a5aa9a6aa6669aaaa5696aa56a6aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa80",
        &[
            ("id", "72"),
            ("channel", "3"),
            ("temperature_C", "9.50"),
            ("humidity", "67"),
        ],
    ),
    // --- v3
    s(
        "Oregon-THGR810",
        "{112}00000005f0428b90c2401c002000",
        &[
            ("id", "157"),
            ("channel", "1"),
            ("temperature_C", "24.30"),
            ("humidity", "38"),
        ],
    ),
    s(
        "Oregon-THN802",
        "{96}00000005312247904c014200",
        &[("id", "158"), ("channel", "2"), ("temperature_C", "-3.20")],
    ),
    s(
        "Oregon-UV800",
        "{96}00000005b1e28f90a00007c0",
        &[("id", "159"), ("channel", "1"), ("uvi", "5")],
    ),
    s(
        "Oregon-PCR800",
        "{120}00000005498200504c800a04081400",
        &[
            ("id", "160"),
            ("rain_rate_in_h", "1.3"),
            ("rain_in", "102.050"),
        ],
    ),
    s(
        "Oregon-PCR800a",
        "{120}000000054b8208504c800a0408b400",
        &[
            ("id", "161"),
            ("rain_rate_in_h", "1.3"),
            ("rain_in", "102.1"),
        ],
    ),
    s(
        "Oregon-WGR800",
        "{112}0000000589120450200ae0c209c0",
        &[
            ("id", "162"),
            ("wind_max_m_s", "7.5"),
            ("wind_avg_m_s", "4.3"),
            ("wind_dir_deg", "90.0"),
        ],
    ),
    s(
        "Oregon-CM160",
        "{128}00000005440c002d000e482c000000a4",
        &[
            ("id", "3"),
            ("power_W", "2898.0000"),
            ("energy_kWh", "15.2619"),
        ],
    ),
    s(
        "Oregon-CM180",
        "{128}000000460c5a016002eed80000000220",
        &[
            ("id", "23088"),
            ("battery_ok", "1"),
            ("power_W", "1674"),
            ("energy_kWh", "0.50"),
            ("sequence", "0"),
        ],
    ),
    s(
        "Oregon-CM180i",
        "{120}0000004a0adc0840028000c0000000",
        &[
            ("id", "15184"),
            ("power1_W", "531"),
            ("power2_W", "322"),
            ("power3_W", "772"),
        ],
    ),
    s(
        "Oregon-CM130",
        "{120}0000006000340a00000002000000db",
        &[("id", "44"), ("power_W", "1280"), ("energy_kWh", "2.00")],
    ),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_decode() {
        crate::devices::check_samples(DEVICES, SAMPLES);
    }

    #[test]
    fn wgr800x_example() {
        // rtl_433's WGR800X example: three packets in one row, calm.
        let m = decode_rows(
            &DEVICES[0],
            "{335}000000b122400e0006000000197c000000b122400e0006000000197c000000b122400e0006000000197c",
        );
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].model, "Oregon-WGR800");
        assert_eq!(m[0].get("id").unwrap().to_string(), "224");
        assert_eq!(m[0].get("channel").unwrap().to_string(), "0");
        assert_eq!(m[0].get("wind_avg_m_s").unwrap().to_string(), "0.0");
    }

    #[test]
    fn bad_checksum_rejected() {
        // The THGR122N sample with one data bit flipped.
        let m = decode_rows(
            &DEVICES[0],
            "{192}55555555996a659aaa9a6699aaa66a9aaa9666aa9a5aaaaa",
        );
        assert!(m.is_empty());
        // THGR810 with its checksum byte changed.
        assert!(decode_rows(&DEVICES[0], "{112}00000005f0428b90c2401c003000").is_empty());
    }
}
