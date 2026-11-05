// made by CasualArclamp
//! Fine Offset Electronics sensors and their rebrands: WH2 / WH2A / WH5 /
//! Telldus / TFA 30.3225 (and the disabled Rosenborg 66796 that collides
//! with WH5), WH25 / WH32 / WH32B / WN32B, WH24 / WH65B / WS69 / HP1000 /
//! Misol WS2320, WH0290 / WH41 PM2.5, WH51 soil moisture, WH0530 and the
//! Alecto WS-1200 v1 / v2 / DCF77.
//!
//! Ported from rtl_433's `src/devices/fineoffset.c` (GPL-2.0-or-later;
//! Copyright (C) 2017 Tommy Vestermark, (C) 2019 Christian W. Zuckschwerdt,
//! WH51 (C) 2019 Marco Di Leo).
//!
//! The OOK ones are PWM (~500 µs 1, ~1500 µs 0, fixed ~1000 µs gap) with
//! an 0xff or 7-bit 0xfe preamble and a CRC-8 (poly 0x31, init 0). The FSK
//! ones are 58 µs PCM with preamble `aa 2d d4`, then a family code, the
//! readings and a CRC-8 and/or add-sum. The WH25 device also hands rows
//! of other lengths to the WH24 and WH0290 decoders, as rtl_433 does.

use crate::bits::{BitBuffer, add_bytes, crc8, xor_bytes};
use crate::device::*;
use crate::sim::{Air, Sample};

const MODEL_WH2: i32 = 2;
const MODEL_WH2A: i32 = 3;
const MODEL_WH5: i32 = 5;
const MODEL_TP: i32 = 7;
const MODEL_TFA303225: i32 = 8;

/// WH2 / WH2A / WH5 / Telldus / TFA 30.3225: `[pre] [type id] [id temp]
/// [temp] [humi] [crc]`, 48 / 55 / 47 / 49 bits.
fn fineoffset_wh2_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let mut b = [0u8; 6];
    let r0 = bb.row(0);
    // Rust note: `match` is an expression, so each arm's last value is
    // the model number.
    let model_num = match bb.bits(0) {
        48 if r0[0] == 0xff => {
            bb.extract_bytes(0, 8, &mut b, 40);
            MODEL_WH2
        }
        55 if r0[0] == 0xfe => {
            bb.extract_bytes(0, 7, &mut b, 48);
            // The TFA 30.3225 always sends humidity 0xff; a WH2A sends 0-100 %.
            if b[3] == 0xff {
                MODEL_TFA303225
            } else {
                MODEL_WH2A
            }
        }
        47 if r0[0] == 0xfe => {
            bb.extract_bytes(0, 7, &mut b, 40);
            MODEL_WH5 // could also be a Rosenborg-66796
        }
        49 if r0[0] == 0xff && r0[1] & 0x80 == 0x80 => {
            bb.extract_bytes(0, 9, &mut b, 40);
            MODEL_TP
        }
        _ => return DECODE_ABORT_LENGTH,
    };

    if b[4] != crc8(&b[..4], 0x31, 0) {
        return DECODE_FAIL_MIC;
    }
    if model_num == MODEL_TFA303225 && (add_bytes(&b[..5]) & 0xff) != b[5] as i32 {
        return DECODE_FAIL_MIC;
    }

    // The type nibble must be 4.
    let typ = b[0] >> 4;
    if typ != 4 {
        ctx.log(|| format!("fineoffset_wh2: unknown type ({model_num}) {typ}"));
        return DECODE_FAIL_SANITY;
    }
    let id = ((b[0] & 0x0f) << 4) | ((b[1] & 0xf0) >> 4);

    let mut temp = (((b[1] & 0x0f) as i32) << 8) | b[2] as i32;
    let mut low_battery = false;
    if model_num == MODEL_TFA303225 {
        // The top bit is a low-battery flag; unsigned, offset by 40 °C.
        low_battery = temp & 0x800 != 0;
        temp &= 0x7ff;
        temp -= 400;
    } else if model_num == MODEL_WH5 {
        temp -= 400;
    } else if temp & 0x800 != 0 {
        // WH2, Telldus, WH2A: sign and magnitude.
        temp &= 0x7ff;
        temp = -temp;
    }
    let temperature = temp as f32 * 0.1;

    // WH5 shares its format with the Rosenborg-66796, which decodes here
    // as nonsense temperatures: reject those.
    if model_num == MODEL_WH5 && !(-40.0..=60.0).contains(&temperature) {
        ctx.log(|| format!("fineoffset_wh2: implausible temperature {temperature:.1} C"));
        return DECODE_FAIL_SANITY;
    }
    let humidity = b[3];

    let model = match model_num {
        MODEL_WH2 => "Fineoffset-WH2",
        MODEL_WH2A => "Fineoffset-WH2A",
        MODEL_WH5 => "Fineoffset-WH5",
        MODEL_TP => "Fineoffset-TelldusProove",
        _ => "TFA-303225",
    };
    let m = Message::new(model)
        .int("id", id)
        .cond(model_num == MODEL_TFA303225, |m| {
            m.int("battery_ok", !low_battery as u8)
        })
        .float("temperature_C", temperature as f64, 1)
        .cond(humidity != 0xff, |m| m.int("humidity", humidity))
        .str("mic", "CRC");
    ctx.output(m);
    1
}

/// Agimex Rosenborg 66796: the WH5 frame with a signed-magnitude
/// temperature.
fn fineoffset_wh5rb_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    if bb.bits(0) != 47 || bb.row(0)[0] != 0xfe {
        return DECODE_ABORT_EARLY;
    }
    let mut b = [0u8; 6];
    bb.extract_bytes(0, 7, &mut b, 40);
    if b[4] != crc8(&b[..4], 0x31, 0) {
        return DECODE_FAIL_MIC;
    }
    let typ = b[0] >> 4;
    if typ != 4 {
        ctx.log(|| format!("rosenborg: unknown type ({typ})"));
        return DECODE_FAIL_SANITY;
    }
    let id = ((b[0] & 0x0f) << 4) | ((b[1] & 0xf0) >> 4);
    let mut temp_raw = (((b[1] & 0x0f) as i32) << 8) | b[2] as i32;
    if temp_raw & 0x800 != 0 {
        temp_raw &= 0x7ff;
        temp_raw = -temp_raw;
    }
    let temperature = temp_raw as f32 * 0.1;
    let humidity = b[3];
    let m = Message::new("Rosenborg-66796")
        .int("id", id)
        .float("temperature_C", temperature as f64, 1)
        .cond(humidity != 0xff, |m| m.int("humidity", humidity))
        .str("mic", "CRC");
    ctx.output(m);
    1
}

const MODEL_WH24: i32 = 24;
const MODEL_WH65: i32 = 65;
const MODEL_WS69: i32 = 69;

/// WH24 / WH65 / WS69 / HP1000: family 0x24, `FF II DD VT TT HH WW GG RR
/// RR UU UU LL LL LL CC BB` (+ 8 bytes of pressure on the WS69).
fn fineoffset_wh24_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
    let mut b = [0u8; 25];
    let bits = bb.bits(0);

    // WH24 nominal 196 bit periods, WH65 209, WS69 260.
    if !(190..=268).contains(&bits) {
        ctx.log(|| format!("fineoffset_wh24: wrong package size {bits}"));
        return DECODE_ABORT_LENGTH;
    }
    let bit_offset = bb.search(0, 0, &PREAMBLE, 24) + 24;
    if bit_offset + 17 * 8 > bits {
        ctx.log(|| format!("fineoffset_wh24: short package, header index {bit_offset}"));
        return DECODE_ABORT_LENGTH;
    }

    // Classification by the lengths around the message.
    let mut typ = if bits - bit_offset - 17 * 8 < 8 && bit_offset < 61 {
        MODEL_WH24 // nominal 3 bits postamble
    } else {
        MODEL_WH65 // nominal 12 bits postamble
    };
    if bits > 215 {
        typ = MODEL_WS69; // nominal 260 bits
    }

    bb.extract_bytes(0, bit_offset, &mut b, 25 * 8);

    if b[0] != 0x24 {
        ctx.log(|| format!("fineoffset_wh24: unknown family code 0x{:02x}", b[0]));
        return DECODE_FAIL_SANITY;
    }
    let crc = crc8(&b[..16], 0x31, 0x00);
    let sum = (add_bytes(&b[..16]) & 0xff) as u8;
    if crc != 0 || sum != b[16] {
        ctx.log(|| format!("fineoffset_wh24: checksum error {crc:02x} {sum:02x}"));
        return DECODE_FAIL_MIC;
    }

    let mut pressure_hpa = -1.0f32;
    if typ == MODEL_WS69 {
        let pressure_raw = ((b[17] as i32) << 16) | ((b[18] as i32) << 8) | b[19] as i32; // 0x01ffff if invalid
        let p_crc = crc8(&b[..24], 0x31, 0x00);
        let p_sum = (add_bytes(&b[..24]) & 0xff) as u8;
        if p_crc != 0 || p_sum != b[24] {
            ctx.log(|| {
                format!("fineoffset_wh24: checksum error in pressure {p_crc:02x} {p_sum:02x}")
            });
        } else if pressure_raw < 0x01ffff {
            pressure_hpa = pressure_raw as f32 * 0.01;
        }
    }

    let id = b[1];
    let wind_dir = b[2] as i32 | (((b[3] & 0x80) as i32) << 1); // 0x1ff if invalid
    let low_battery = (b[3] & 0x08) >> 3;
    let temp_raw = (((b[3] & 0x07) as i32) << 8) | b[4] as i32; // 0x7ff if invalid
    let temperature = (temp_raw - 400) as f32 * 0.1;
    let humidity = b[5]; // 0xff if invalid
    let wind_speed_raw = b[6] as i32 | (((b[3] & 0x10) as i32) << 4); // 0x1ff if invalid
    // WH24: 1.12 m/s per count, 0.3 mm per tip; WH65: 0.51 m/s, 0.254 mm.
    let (wind_speed_factor, rain_cup_count) = if typ == MODEL_WH24 {
        (1.12f32, 0.3f32)
    } else {
        (0.51f32, 0.254f32)
    };
    // Wind speed is scaled by 8.
    let wind_speed_ms = wind_speed_raw as f32 * 0.125 * wind_speed_factor;
    let gust_speed_raw = b[7] as i32; // 0xff if invalid
    let gust_speed_ms = gust_speed_raw as f32 * wind_speed_factor;
    let rainfall_raw = ((b[8] as i32) << 8) | b[9] as i32;
    let rainfall_mm = rainfall_raw as f32 * rain_cup_count;
    let uv_raw = ((b[10] as i32) << 8) | b[11] as i32; // 0xffff if invalid
    let light_raw = ((b[12] as i32) << 16) | ((b[13] as i32) << 8) | b[14] as i32; // 0xffffff if invalid
    let light_lux = light_raw as f64 * 0.1;

    // UV raw value to UV index: the upper bound of each index.
    const UVI_UPPER: [i32; 13] = [
        432, 851, 1210, 1570, 2017, 2450, 2761, 3100, 3512, 3918, 4277, 4650, 5029,
    ];
    let mut uv_index = 0;
    while uv_index < 13 && UVI_UPPER[uv_index] < uv_raw {
        uv_index += 1;
    }

    let model = match typ {
        MODEL_WH24 => "Fineoffset-WH24",
        MODEL_WH65 => "Fineoffset-WH65B",
        _ => "Fineoffset-WS69",
    };
    let m = Message::new(model)
        .int("id", id)
        .int("battery_ok", (low_battery == 0) as u8)
        .cond(temp_raw != 0x7ff, |m| {
            m.float("temperature_C", temperature as f64, 1)
        })
        .cond(humidity != 0xff, |m| m.int("humidity", humidity))
        .cond(pressure_hpa >= 0.0, |m| {
            m.float("pressure_hPa", pressure_hpa as f64, 2)
        })
        .cond(wind_dir != 0x1ff, |m| m.int("wind_dir_deg", wind_dir))
        .cond(wind_speed_raw != 0x1ff, |m| {
            m.float("wind_avg_m_s", wind_speed_ms as f64, 1)
        })
        .cond(gust_speed_raw != 0xff, |m| {
            m.float("wind_max_m_s", gust_speed_ms as f64, 1)
        })
        .float("rain_mm", rainfall_mm as f64, 1)
        .cond(uv_raw != 0xffff, |m| m.int("uv", uv_raw))
        .cond(uv_raw != 0xffff, |m| m.float("uvi", uv_index as f64, 0))
        .cond(light_raw != 0xffffff, |m| {
            m.float("light_lux", light_lux, 1)
        })
        .str("mic", "CRC");
    ctx.output(m);
    1
}

/// WH0290 / WH41 / Ambient Weather PM25: `FF DD ?P PP ?A AA CC BB`.
fn fineoffset_wh0290_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
    let mut b = [0u8; 8];
    let bit_offset = bb.search(0, 0, &PREAMBLE, 24) + 24;
    if bit_offset + b.len() * 8 > bb.bits(0) {
        ctx.log(|| {
            format!(
                "fineoffset_wh0290: short package, row length {}, header index {bit_offset}",
                bb.bits(0)
            )
        });
        return DECODE_ABORT_LENGTH;
    }
    bb.extract_bytes(0, bit_offset, &mut b, 64);

    let crc = crc8(&b[..6], 0x31, 0x00);
    let checksum = (add_bytes(&b[..7]) & 0xff) as u8;
    if crc != b[6] || checksum != b[7] {
        ctx.log(|| format!("fineoffset_wh0290: checksum error {crc:02x} {checksum:02x}"));
        return DECODE_FAIL_MIC;
    }

    let family = b[0];
    let id = b[1];
    let unknown1 = (b[2] & 0x80 != 0) as u8;
    let pm25 = (((b[2] & 0x3f) as i32) << 8) | b[3] as i32;
    let pm100 = (((b[4] & 0x3f) as i32) << 8) | b[5] as i32;
    let battery_bars = ((b[2] & 0x40) >> 4) | ((b[4] & 0xc0) >> 6); // out of 5
    let battery_ok = battery_bars as f32 * 0.2;

    let m = Message::new("Fineoffset-WH0290")
        .int("id", id)
        .float("battery_ok", battery_ok as f64, 1)
        .int("pm2_5_ug_m3", pm25 / 10)
        .int("estimated_pm10_0_ug_m3", pm100 / 10)
        .int("family", family)
        .int("unknown1", unknown1)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

/// WH25 / WH32 / WH32B / WN32B: `MI IT TT HH PP PP CC XX`; other row
/// lengths go to the WH24 or WH0290 decoder.
fn fineoffset_wh25_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
    let mut b = [0u8; 8];
    let mut typ = 25;
    let bits = bb.bits(0);

    if bits < 160 {
        // Nominal WH0290 length is 129 bits.
        return fineoffset_wh0290_callback(ctx, bb);
    } else if bits < 190 {
        typ = 32; // new WN32B, nominal 173 bits
    } else if bits < 440 {
        // Nominal WH25 size is 488 bit periods.
        return fineoffset_wh24_callback(ctx, bb);
    }
    if bits > 510 {
        typ = 32; // WH32B, nominal 971 bit periods
    }

    let bit_offset = bb.search(0, 0, &PREAMBLE, 24) + 24;
    if bit_offset + b.len() * 8 > bits {
        ctx.log(|| format!("fineoffset_wh25: short package, header index {bit_offset}"));
        return DECODE_ABORT_LENGTH;
    }
    bb.extract_bytes(0, bit_offset, &mut b, 64);

    let msg_type = b[0] & 0xf0;
    if typ == 32 && msg_type == 0xd0 {
        typ = 31; // an older "WH32", no barometer
    } else if msg_type != 0xe0 {
        ctx.log(|| format!("fineoffset_wh25: msg type unknown 0x{:02x}", b[0]));
        if b[0] == 0x41 {
            return fineoffset_wh0290_callback(ctx, bb);
        }
        return DECODE_ABORT_EARLY;
    }

    if (add_bytes(&b[..6]) & 0xff) - b[6] as i32 != 0 {
        ctx.log(|| "fineoffset_wh25: checksum error".into());
        return DECODE_FAIL_MIC;
    }
    // Xor of the 6 data bytes, nibbles swapped; checked for the WH25 only.
    let bitsum = xor_bytes(&b[..6]).rotate_left(4);
    if typ == 25 && bitsum != b[7] {
        ctx.log(|| "fineoffset_wh25: bitsum error".into());
        return DECODE_FAIL_MIC;
    }

    let id = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
    let low_battery = (b[1] & 0x08) >> 3;
    let temp_raw = (((b[1] & 0x03) as i32) << 8) | b[2] as i32;
    let temperature = (temp_raw - 400) as f32 * 0.1;
    let humidity = b[3];
    let pressure_raw = ((b[4] as i32) << 8) | b[5] as i32;
    let pressure = pressure_raw as f32 * 0.1;

    let model = match typ {
        31 => "Fineoffset-WH32",
        32 => "Fineoffset-WH32B",
        _ => "Fineoffset-WH25",
    };
    let m = Message::new(model)
        .int("id", id)
        .int("battery_ok", (low_battery == 0) as u8)
        .float("temperature_C", temperature as f64, 1)
        .int("humidity", humidity)
        .cond(pressure_raw != 0xffff, |m| {
            m.float("pressure_hPa", pressure as f64, 1)
        })
        .str("mic", "CRC");
    ctx.output(m);
    1
}

/// WH51 / WN31 / SwitchDoc SM23 soil moisture: `FF II II II TB YY MM ZA
/// AA XX XX XX CC SS`.
fn fineoffset_wh51_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
    let mut b = [0u8; 14];

    if bb.bits(0) < 120 {
        return DECODE_ABORT_LENGTH;
    }
    let bit_offset = bb.search(0, 0, &PREAMBLE, 24) + 24;
    if bit_offset + b.len() * 8 > bb.bits(0) {
        ctx.log(|| format!("fineoffset_wh51: short package, header index {bit_offset}"));
        return DECODE_ABORT_LENGTH;
    }
    bb.extract_bytes(0, bit_offset, &mut b, 14 * 8);

    if b[0] != 0x51 {
        ctx.log(|| format!("fineoffset_wh51: msg family unknown 0x{:02x}", b[0]));
        return DECODE_ABORT_EARLY;
    }
    if (add_bytes(&b[..13]) & 0xff) != b[13] as i32 {
        ctx.log(|| "fineoffset_wh51: checksum error".into());
        return DECODE_FAIL_MIC;
    }
    if crc8(&b[..12], 0x31, 0) != b[12] {
        ctx.log(|| "fineoffset_wh51: bitsum error".into());
        return DECODE_FAIL_MIC;
    }

    let id = format!("{:02x}{:02x}{:02x}", b[1], b[2], b[3]);
    let boost = (b[4] & 0xe0) >> 5;
    let battery_mv_bits = b[4] & 0x1f;
    let battery_mv = battery_mv_bits as i32 * 100;
    let ad_raw = (((b[7] & 0x01) as i32) << 8) | b[8] as i32;
    let moisture = b[6];
    // rtl_433's mapping for an alkaline AA cell: 1.6 V new … 1.2 V flat.
    let battery_level = match battery_mv_bits {
        16.. => 1.0,
        15 => 0.9,
        14 => 0.5,
        13 => 0.1,
        _ => 0.0,
    };

    let m = Message::new("Fineoffset-WH51")
        .str("id", id)
        // DATA_DOUBLE without a format: rtl_433's default "%.3f".
        .float("battery_ok", battery_level, 3)
        .int("battery_mV", battery_mv)
        .int("moisture", moisture)
        .int("boost", boost)
        .int("ad_raw", ad_raw)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

/// The 7 bits of preamble (all ones) and the message type nibble or byte
/// that the Alecto/WH0530 frames start with.
fn alecto_head(bb: &BitBuffer) -> (u8, u8) {
    let r = bb.row(0);
    (r[0] >> 1, r[1])
}

/// Alecto WS-1200 v1.0: 63 bits, `1111111 FFFFIIII IIIIB?TT TTTTTTTT
/// RRRRRRRR RRRRRRRR 11111111 CCCCCCCC`.
fn alecto_ws1200v1_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let (pre, b1) = alecto_head(bb);
    if bb.bits(0) != 63 || pre != 0x7f || (b1 >> 5) != 0x3 {
        return DECODE_ABORT_LENGTH;
    }
    let mut b = [0u8; 7];
    bb.extract_bytes(0, 7, &mut b, 56); // skip the 7 preamble bits
    if crc8(&b, 0x31, 0) != 0 {
        ctx.log(|| "Alecto WS-1200 v1.0: CRC error".into());
        return DECODE_FAIL_MIC;
    }
    output_alecto_t_rain(ctx, &b, "Alecto-WS1200v1");
    1
}

/// The temperature/rain fields shared by the WH0530 and the Alecto v1/v2.
fn output_alecto_t_rain(ctx: &mut Ctx, b: &[u8], model: &str) {
    let id = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
    let battery_low = (b[1] >> 3) & 0x1;
    let temp_raw = (((b[1] & 0x7) as i32) << 8) | b[2] as i32;
    let temperature = (temp_raw - 400) as f32 * 0.1;
    let rainfall_raw = ((b[4] as i32) << 8) | b[3] as i32; // little endian tip counter
    let rainfall = rainfall_raw as f32 * 0.3; // 0.3 mm per tip
    let m = Message::new(model)
        .int("id", id)
        .int("battery_ok", (battery_low == 0) as u8)
        .float("temperature_C", temperature as f64, 1)
        .float("rain_mm", rainfall as f64, 1)
        .str("mic", "CRC");
    ctx.output(m);
}

/// Alecto WS-1200 v2.0 DCF77 time: 95 bits, type byte 0x52.
fn alecto_ws1200v2_dcf_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let (pre, b1) = alecto_head(bb);
    if bb.bits(0) != 95 || pre != 0x7f || (b1 >> 1) != 0x52 {
        return DECODE_ABORT_LENGTH;
    }
    let mut b = [0u8; 11];
    bb.extract_bytes(0, 7, &mut b, 88);
    if crc8(&b[..10], 0x31, 0) != 0 {
        return DECODE_FAIL_MIC;
    }
    if (add_bytes(&b[..10]) - b[10] as i32) & 0xff != 0 {
        ctx.log(|| "Alecto WS-1200 v2.0 DCF77: checksum error".into());
        return DECODE_FAIL_MIC;
    }
    let id = b[1];
    let battery_low = (b[2] >> 7) & 0x1;
    // The date and time are BCD: rtl_433 prints them as hex.
    let clock = format!(
        "{:04x}-{:02x}-{:02x}T{:02x}:{:02x}:{:02x}",
        b[4] as u32 + 0x2000,
        b[5],
        b[6],
        b[7],
        b[8],
        b[9]
    );
    let m = Message::new("Alecto-WS1200v2")
        .int("id", id)
        .int("battery_ok", (battery_low == 0) as u8)
        .str("radio_clock", clock)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

/// Alecto WS-1200 v2.0: the v1 frame plus an add-checksum and 24 bits of
/// DCF77 time (95 bits); anything else of 95 bits may be the DCF77 frame.
fn alecto_ws1200v2_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let (pre, b1) = alecto_head(bb);
    if bb.bits(0) != 95 || pre != 0x7f || (b1 >> 5) != 0x3 {
        return alecto_ws1200v2_dcf_callback(ctx, bb);
    }
    let mut b = [0u8; 11];
    bb.extract_bytes(0, 7, &mut b, 88);
    if crc8(&b[..7], 0x31, 0) != 0 {
        ctx.log(|| "Alecto WS-1200 v2.0: CRC error".into());
        return DECODE_FAIL_MIC;
    }
    if (add_bytes(&b[..7]) - b[7] as i32) & 0xff != 0 {
        ctx.log(|| "Alecto WS-1200 v2.0: checksum error".into());
        return DECODE_FAIL_MIC;
    }
    output_alecto_t_rain(ctx, &b, "Alecto-WS1200v2");
    1
}

/// WH0530 / Rosenborg 35926 temperature and rain: 71 bits, `1111111` then
/// `FI IT TT RR RR ?? CC AA`; 63 and 95 bit rows are tried as Alecto.
fn fineoffset_wh0530_callback(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    match bb.bits(0) {
        63 => return alecto_ws1200v1_callback(ctx, bb),
        95 => return alecto_ws1200v2_callback(ctx, bb),
        71 => {}
        _ => return DECODE_ABORT_LENGTH,
    }
    let (pre, b1) = alecto_head(bb);
    if pre != 0x7f || (b1 >> 5) != 0x3 {
        return DECODE_ABORT_EARLY;
    }
    let mut b = [0u8; 8];
    bb.extract_bytes(0, 7, &mut b, 64);
    let crc = crc8(&b[..7], 0x31, 0);
    let sum = (add_bytes(&b[..7]) & 0xff) - b[7] as i32;
    if crc != 0 || sum != 0 {
        ctx.log(|| "fineoffset_wh0530: checksum error".into());
        return DECODE_FAIL_MIC;
    }
    output_alecto_t_rain(ctx, &b, "Fineoffset-WH0530");
    1
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "Fine Offset Electronics, WH2, WH5, Telldus Temperature/Humidity/Rain Sensor",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 500.0,
        long_width: 1500.0,
        reset_limit: 1200.0,
        tolerance: 160.0,
        decode: fineoffset_wh2_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Agimex Rosenborg 66796 (collides with Fine Offset Electronics WH5) Temperature/Humidity Sensor",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 500.0,
        long_width: 1500.0,
        reset_limit: 1200.0,
        tolerance: 160.0,
        decode: fineoffset_wh5rb_decode,
        disabled: true, // collides with WH5
        ..Device::DEFAULT
    },
    Device {
        name: "Fine Offset Electronics, WH25, WH32, WH32B, WN32B, WH24, WH65, WS69, HP1000, Misol WS2320 Temperature/Humidity/Pressure Sensor",
        group: Group::Weather,
        modulation: Modulation::FskPcm,
        short_width: 58.0,
        long_width: 58.0,
        reset_limit: 20000.0, // the package starts with a gap of ~18900 µs
        decode: fineoffset_wh25_callback,
        ..Device::DEFAULT
    },
    Device {
        name: "Fine Offset Electronics/Ecowitt WH51, WN31, SwitchDoc Labs SM23 Soil Moisture Sensor",
        group: Group::Weather,
        modulation: Modulation::FskPcm,
        short_width: 58.0,
        long_width: 58.0,
        reset_limit: 5000.0,
        decode: fineoffset_wh51_callback,
        ..Device::DEFAULT
    },
    Device {
        name: "Fine Offset Electronics, WH0530 Temperature/Rain Sensor",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 504.0,
        long_width: 1480.0,
        reset_limit: 1200.0,
        sync_width: 0.0,
        tolerance: 160.0,
        decode: fineoffset_wh0530_callback,
        ..Device::DEFAULT
    },
];

/// The PWM ones have a fixed gap after each pulse (~1000 µs).
const FIXED_GAP: Air = Air {
    pulse_us: 1000.0,
    ..Air::DEFAULT
};

pub static SAMPLES: &[Sample] = &[
    // Made up (CRC computed): WH2, id 91, 21.5 °C, 55 %.
    Sample {
        dev: 0,
        model: "Fineoffset-WH2",
        code: "{48}ff45b0d73745",
        expect: &[("id", "91"), ("temperature_C", "21.5"), ("humidity", "55")],
        air: FIXED_GAP,
    },
    // Made up: WH5 (7-bit preamble, offset temperature), 18.5 °C, 65 %.
    Sample {
        dev: 0,
        model: "Fineoffset-WH5",
        code: "{47}fe942492832a",
        expect: &[("id", "161"), ("temperature_C", "18.5"), ("humidity", "65")],
        air: FIXED_GAP,
    },
    // Made up: Rosenborg, -3.5 °C (sign and magnitude), 65 %.
    Sample {
        dev: 1,
        model: "Rosenborg-66796",
        code: "{47}fe94304683e0",
        expect: &[("id", "161"), ("temperature_C", "-3.5"), ("humidity", "65")],
        air: FIXED_GAP,
    },
    // rtl_433's WH25 example (22.6 °C, 40 %, 1001.7 hPa). The row is as
    // the slicer gives it: the trailing zeros are the reset gap's (several
    // hundred of them), and they make it a WH25-length row. For the same
    // reason the shorter WH24 / WH65 / WS69 / WH0290 / WN32B rows cannot
    // survive the simulator: they are checked in the tests below instead.
    Sample {
        dev: 2,
        model: "Fineoffset-WH25",
        code: "{448}aaaaaa2dd4e50272282721c9bb00000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
        expect: &[
            ("id", "80"),
            ("battery_ok", "1"),
            ("temperature_C", "22.6"),
            ("humidity", "40"),
            ("pressure_hPa", "1001.7"),
        ],
        air: Air::DEFAULT,
    },
    // rtl_433's WH51 example.
    Sample {
        dev: 3,
        model: "Fineoffset-WH51",
        code: "{152}aaaaaa2dd451006b586e7f24f8d2ffffff3c28",
        expect: &[
            ("id", "006b58"),
            ("battery_ok", "0.500"),
            ("battery_mV", "1400"),
            ("moisture", "36"),
            ("boost", "3"),
            ("ad_raw", "210"),
        ],
        air: Air::DEFAULT,
    },
    // rtl_433's WH0530 example `38 a2 8f 02 00 ff e7 51`.
    Sample {
        dev: 4,
        model: "Fineoffset-WH0530",
        code: "{71}fe71451e0401ffcea2",
        expect: &[
            ("id", "138"),
            ("battery_ok", "1"),
            ("temperature_C", "25.5"),
            ("rain_mm", "0.6"),
        ],
        air: Air {
            pulse_us: 960.0,
            ..Air::DEFAULT
        },
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    fn field(m: &Message, k: &str) -> String {
        m.get(k)
            .unwrap_or_else(|| panic!("no {k} in {m:?}"))
            .to_string()
    }

    #[test]
    fn samples_decode() {
        crate::devices::check_samples(DEVICES, SAMPLES);
    }

    #[test]
    fn wh24_family() {
        // rtl_433's WH24 example: id 191, 11.8 °C, 78 %, 266°, 1.12 m/s,
        // gust 2.24 m/s, 22.2 mm.
        let m = decode_rows(
            &DEVICES[2],
            "{196}d55555555516ea125f857103270401002500008000004783 9",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].model, "Fineoffset-WH24");
        assert_eq!(field(&m[0], "id"), "191");
        assert_eq!(field(&m[0], "temperature_C"), "11.8");
        assert_eq!(field(&m[0], "humidity"), "78");
        assert_eq!(field(&m[0], "wind_dir_deg"), "266");
        assert_eq!(field(&m[0], "wind_avg_m_s"), "1.1");
        assert_eq!(field(&m[0], "wind_max_m_s"), "2.2");
        assert_eq!(field(&m[0], "rain_mm"), "22.2");
        // WH65 example (longer preamble and postamble).
        let m = decode_rows(
            &DEVICES[2],
            "{209}5555555555516ea122833f143a0800000008001000000460a1008",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].model, "Fineoffset-WH65B");
        assert_eq!(field(&m[0], "temperature_C"), "24.7");
        assert_eq!(field(&m[0], "wind_dir_deg"), "359");
        // WS69 example: no pressure module (0x1ffff).
        let m = decode_rows(
            &DEVICES[2],
            "{263}aaaaaaaaaaa2dd4241c8801ca63000000330000000000acd501ffffff002f835a0",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].model, "Fineoffset-WS69");
        assert_eq!(field(&m[0], "temperature_C"), "5.8");
        assert!(m[0].get("pressure_hPa").is_none());
    }

    #[test]
    fn wh0290_and_wn32b() {
        // rtl_433's examples: pm2.5 9, pm10 10, id 151, 4/5 bars.
        let m = decode_rows(&DEVICES[2], "{129}5555555555516ea20cba02d0032513c000");
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].model, "Fineoffset-WH0290");
        assert_eq!(field(&m[0], "id"), "151");
        assert_eq!(field(&m[0], "pm2_5_ug_m3"), "9");
        assert_eq!(field(&m[0], "estimated_pm10_0_ug_m3"), "10");
        assert_eq!(field(&m[0], "battery_ok"), "0.8");
        let m = decode_rows(&DEVICES[2], "{129}5555555555516ea20cb80146019 49c2c00");
        assert_eq!(field(&m[0], "pm2_5_ug_m3"), "4");
        assert_eq!(field(&m[0], "battery_ok"), "0.6");
        // The WH25 message in a WN32B-length row, and as an old WH32.
        let m = decode_rows(
            &DEVICES[2],
            "{170}aaaaaa2dd4e50272282721c9bb000000000000000000",
        );
        assert_eq!(m[0].model, "Fineoffset-WH32B");
        let m = decode_rows(
            &DEVICES[2],
            "{170}aaaaaa2dd4d50272282721b900000000000000000000",
        );
        assert_eq!(m[0].model, "Fineoffset-WH32");
        assert_eq!(field(&m[0], "temperature_C"), "22.6");
    }

    #[test]
    fn alecto() {
        // Made up (CRCs computed): id 165, 10.0 °C, 16 tips = 4.8 mm.
        let m = decode_rows(&DEVICES[4], "{63}fe74a3e82001fff6");
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].model, "Alecto-WS1200v1");
        assert_eq!(field(&m[0], "id"), "165");
        assert_eq!(field(&m[0], "temperature_C"), "10.0");
        assert_eq!(field(&m[0], "rain_mm"), "4.8");
        let m = decode_rows(&DEVICES[4], "{95}fe74a3e82001fff712000000");
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].model, "Alecto-WS1200v2");
        assert_eq!(field(&m[0], "rain_mm"), "4.8");
        // DCF77 time 2026-11-05 12:34, the seconds byte being the CRC.
        let m = decode_rows(&DEVICES[4], "{95}fea46600004c220a24688c9a");
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(field(&m[0], "radio_clock"), "2026-11-05T12:34:46");
    }
}
