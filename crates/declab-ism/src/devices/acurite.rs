// made by CasualArclamp
//! Acurite weather stations and sensors: the 592TXR "tower" family
//! (592TXR / 592TX / 06002RM / 6044M, Iris 5-in-1, Notos 3-in-1, Atlas
//! 7-in-1, Optimus 6-in-1, 6045M lightning, 899 rain, 515 fridge/freezer,
//! 1190/1192 leak), the 609TXC, 606TX / Technoline TX960, 986
//! fridge/freezer, 00275rm/00276rm room monitors and the 896 rain gauge.
//!
//! Ported from rtl_433's `src/devices/acurite.c` (GPL-2.0-or-later;
//! Copyright (c) 2015 Jens Jenson, Helge Weissig, David Ray Thompson,
//! Robert Terzi; 606TX by Boing). Not ported: the 985 and 590TX decoders.
//!
//! The tower family is OOK PWM (220 µs 1, 408 µs 0, 620 µs sync), sent
//! inverted: after inverting, byte 0 is `CC` channel + ID bits, byte 1 ID,
//! byte 2 `pB MMMMMM` (parity, battery ok, message type), data bytes with
//! an even-parity top bit, and a last byte that is the sum of the others.

use crate::bits::{BitBuffer, add_bytes, crc8le, crc16lsb, lfsr_digest8, parity_bytes, reverse8};
use crate::device::*;
use crate::sim::{Air, Sample};

const ACURITE_TXR_BYTELEN: usize = 7;
const ACURITE_515_BYTELEN: usize = 6;
const ACURITE_1190_BYTELEN: usize = 7;
const ACURITE_3N1_BYTELEN: usize = 8;
const ACURITE_5N1_BYTELEN: usize = 8;
const ACURITE_899_BYTELEN: usize = 8;
const ACURITE_ATLAS_BYTELEN: usize = 8;
const ACURITE_6045_BYTELEN: usize = 9;
const ACURITE_ATLAS_LTNG_BYTELEN: usize = 10;
const ACURITE_OPTIMUS_BYTELEN: usize = 10;

// Message types (byte 2, low 6 bits).
const MSGTYPE_1190_DETECTOR: u8 = 0x01;
const MSGTYPE_TOWER_SENSOR: u8 = 0x04;
const MSGTYPE_ATLAS_WNDSPD_TEMP_HUM: u8 = 0x05;
const MSGTYPE_ATLAS_WNDSPD_RAIN: u8 = 0x06;
const MSGTYPE_ATLAS_WNDSPD_UV_LUX: u8 = 0x07;
const MSGTYPE_515_REFRIGERATOR: u8 = 0x08;
const MSGTYPE_515_FREEZER: u8 = 0x09;
const MSGTYPE_3N1_WINDSPEED_TEMP_HUMIDITY: u8 = 0x20;
const MSGTYPE_ATLAS_WNDSPD_TEMP_HUM_LTNG: u8 = 0x25;
const MSGTYPE_ATLAS_WNDSPD_RAIN_LTNG: u8 = 0x26;
const MSGTYPE_ATLAS_WNDSPD_UV_LUX_LTNG: u8 = 0x27;
const MSGTYPE_6045M: u8 = 0x2f;
const MSGTYPE_899_RAINFALL: u8 = 0x30;
const MSGTYPE_5N1_WINDSPEED_WINDDIR_RAINFALL: u8 = 0x31;
const MSGTYPE_5N1_WINDSPEED_TEMP_HUMIDITY: u8 = 0x38;
const MSGTYPE_OPTIMUS_WIND_RAIN: u8 = 0x3a;
const MSGTYPE_OPTIMUS_TEMP_HUM: u8 = 0x3b;

/// 5-in-1 raw wind direction to AcuLink's direction, in 22.5° units (the
/// mapping jumps around; from draythomp/Desert-home-rtl_433).
const ACURITE_5N1_WINDDIRECTIONS: [i32; 16] =
    [14, 11, 13, 12, 15, 10, 0, 9, 3, 6, 4, 5, 2, 7, 1, 8];

/// The channel in the top two bits of byte 0: 00 C, 10 B, 11 A (01 is
/// invalid: "E").
fn acurite_get_channel(byte: u8) -> &'static str {
    ["C", "E", "B", "A"][((byte & 0xc0) >> 6) as usize]
}

/// rtl_433's `data_hex`: bytes as lowercase hex.
fn hex_str(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Acurite 896 rain gauge: id, then 12 bits of bucket tips (0.5 mm each).
fn acurite_rain_896_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let b = bb.row(0);
    if bb.bits(0) < 24 {
        return DECODE_ABORT_LENGTH;
    }
    // Nominally 16 repeats: require 12 (fewer is likely Oregon V1).
    if bb.num_rows < 12 {
        return DECODE_ABORT_EARLY;
    }
    if b[0] == 0 || b[1] == 0 || b[2] == 0 || b[3] != 0 || b[4] != 0 {
        return DECODE_ABORT_EARLY;
    }
    let id = b[0];
    let total_rain = ((((b[1] & 0xf) as i32) << 8) | b[2] as i32) as f32 * 0.5;
    let m = Message::new("Acurite-Rain")
        .int("id", id)
        .float("rain_mm", total_rain as f64, 1);
    ctx.output(m);
    1
}

/// Acurite 609TXC: `II ST TT HH CC`, temperature 12-bit signed 0.1 °C,
/// CC the sum of the first four bytes.
fn acurite_th_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let mut valid = 0;
    let mut result = 0;
    for brow in 0..bb.num_rows {
        if bb.bits(brow) != 40 {
            result = DECODE_ABORT_LENGTH;
            continue;
        }
        let b = bb.row(brow);
        let cksum = b[0] as i32 + b[1] as i32 + b[2] as i32 + b[3] as i32;
        if cksum == 0 || (cksum & 0xff) != b[4] as i32 {
            result = DECODE_FAIL_MIC;
            continue;
        }
        // 12 bits, sign-extended through an i16.
        let temp_raw = ((((b[1] & 0x0f) as u16) << 12) | ((b[2] as u16) << 4)) as i16;
        let tempc = (temp_raw >> 4) as f32 * 0.1;
        let id = b[0];
        let status = (b[1] & 0xf0) >> 4;
        let battery_low = status & 0x8 != 0;
        let humidity = b[3];
        if humidity > 100 {
            ctx.log(|| format!("609txc 0x{id:04X}: invalid humidity: {humidity} %rH"));
            return DECODE_FAIL_SANITY;
        }
        let m = Message::new("Acurite-609TXC")
            .int("id", id)
            .int("battery_ok", !battery_low as u8)
            .float("temperature_C", tempc as f64, 1)
            .int("humidity", humidity)
            .int("status", status)
            .str("mic", "CHECKSUM");
        ctx.output(m);
        valid += 1;
    }
    if valid > 0 { 1 } else { result }
}

/// Acurite 6045M lightning detector: `CCIIIIII IIIIIIII pB101111 pHHHHHHH
/// pA?TTTTT pTTTTTTT pLLLLLLL pLRDDDDD KKKKKKKK`; temperature in 0.1 °F
/// + 1480.
fn acurite_6045_decode(ctx: &mut Ctx, bb: &[u8], browlen: usize) -> i32 {
    let mut exception = 0;
    let channel_str = acurite_get_channel(bb[0]);
    let sensor_id = (((bb[0] & 0x3f) as u16) << 8) | bb[1] as u16;
    let battery_low = (bb[2] & 0x40) == 0;
    let humidity = bb[3] & 0x7f;
    if humidity > 100 {
        ctx.log(|| {
            format!("6045m 0x{sensor_id:04X} Ch {channel_str}: invalid humidity: {humidity} %rH")
        });
        return DECODE_FAIL_SANITY;
    }
    let active = (bb[4] & 0x40) == 0x40;
    let temp_raw = (((bb[4] & 0x1f) as i32) << 7) | (bb[5] & 0x7f) as i32;
    let tempf = (temp_raw - 1480) as f32 * 0.1;
    if !(-40.0..=158.0).contains(&tempf) {
        ctx.log(|| {
            format!("6045m 0x{sensor_id:04X} Ch {channel_str}, invalid temperature: {tempf:.1} F")
        });
        return DECODE_FAIL_SANITY;
    }
    if temp_raw & 0x3000 != 0 {
        exception += 1;
    }
    // 8-bit strike count, its LSB in the next byte.
    let strike_count = ((bb[6] & 0x7f) << 1) | ((bb[7] & 0x40) >> 6);
    let strike_distance = bb[7] & 0x1f;
    let rfi_detect = (bb[7] & 0x20) == 0x20;
    if bb[4] & 0x20 != 0 {
        exception += 1; // unknown status bit, always off
    }
    let m = Message::new("Acurite-6045M")
        .int("id", sensor_id)
        .str("channel", channel_str)
        .int("battery_ok", !battery_low as u8)
        .float("temperature_F", tempf as f64, 1)
        .int("humidity", humidity)
        .int("strike_count", strike_count)
        .int("storm_dist", strike_distance)
        .int("active", active as u8)
        .int("rfi", rfi_detect as u8)
        .int("exception", exception)
        .str("raw_msg", hex_str(&bb[..browlen.min(15)]));
    ctx.output(m);
    1
}

/// Acurite 899 rain gauge: 14 bits of 0.01 inch tips (output in mm).
fn acurite_899_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let sensor_id = (((bb[0] & 0x3f) as u16) << 8) | bb[1] as u16;
    let battery_low = (bb[2] & 0x40) == 0;
    // rtl_433 keeps this numeric channel (00 A, 01 B, 10 C) for compatibility.
    let channel = bb[0] >> 6;
    let raincounter = (((bb[5] & 0x7f) as i32) << 7) | (bb[6] & 0x7f) as i32;
    let m = Message::new("Acurite-Rain899")
        .int("id", sensor_id)
        .int("channel", channel)
        .int("battery_ok", !battery_low as u8)
        .float("rain_mm", raincounter as f64 * 0.254, 2)
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

/// Acurite Notos 3-in-1: wind speed (mph), temperature (0.1 °F + 1480),
/// humidity.
fn acurite_3n1_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let channel_str = acurite_get_channel(bb[0]);
    let sensor_id = (((bb[0] & 0x3f) as u16) << 8) | bb[1] as u16;
    let message_type = bb[2] & 0x3f;
    if channel_str == "E" {
        ctx.log(|| format!("3n1: bad channel Ch {channel_str}, msg type 0x{message_type:02x}"));
        return DECODE_FAIL_SANITY;
    }
    // rtl_433 notes these bits are also part of the ID; kept for compatibility.
    let sequence_num = (bb[0] & 0x30) >> 4;
    let battery_low = (bb[2] & 0x40) == 0;
    let humidity = bb[3] & 0x7f;
    if humidity > 100 {
        ctx.log(|| {
            format!("3n1 0x{sensor_id:04X} Ch {channel_str}: invalid humidity: {humidity} %rH")
        });
        return DECODE_FAIL_SANITY;
    }
    let temp_raw = (((bb[4] & 0x1f) as i32) << 7) | (bb[5] & 0x7f) as i32;
    let tempf = (temp_raw - 1480) as f32 * 0.1;
    if !(-40.0..=158.0).contains(&tempf) {
        ctx.log(|| {
            format!("3n1 0x{sensor_id:04X} Ch {channel_str}, invalid temperature: {tempf:.1} F")
        });
        return DECODE_FAIL_SANITY;
    }
    let wind_speed_mph = (bb[6] & 0x7f) as f64;
    let m = Message::new("Acurite-3n1")
        .int("message_type", message_type)
        // rtl_433 prints "0x%02X" (upper case); ours shows lower case.
        .hex("id", sensor_id as u64, 2)
        .str("channel", channel_str)
        .int("sequence_num", sequence_num)
        .int("battery_ok", !battery_low as u8)
        .float("wind_avg_mi_h", wind_speed_mph, 1)
        .float("temperature_F", tempf as f64, 1)
        .int("humidity", humidity)
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

/// Acurite Iris 5-in-1: two alternating messages, wind + direction + rain
/// (0x31) or wind + temperature + humidity (0x38).
fn acurite_5n1_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let channel_str = acurite_get_channel(bb[0]);
    let sensor_id = (((bb[0] & 0x0f) as u16) << 8) | bb[1] as u16;
    let sequence_num = (bb[0] & 0x30) >> 4;
    let battery_low = (bb[2] & 0x40) == 0;
    let message_type = bb[2] & 0x3f;

    // Cup rotations per 4 seconds.
    let wind_speed_raw = (((bb[3] & 0x1f) as i32) << 3) | ((bb[4] & 0x70) >> 4) as i32;
    let wind_speed_kph = if wind_speed_raw > 0 {
        wind_speed_raw as f32 * 0.8278 + 1.0
    } else {
        0.0
    };

    if message_type == MSGTYPE_5N1_WINDSPEED_WINDDIR_RAINFALL {
        let wind_dir = ACURITE_5N1_WINDDIRECTIONS[(bb[4] & 0x0f) as usize] as f32 * 22.5;
        // 0.01 inch increments, accumulated.
        let raincounter = (((bb[5] & 0x7f) as i32) << 7) | (bb[6] & 0x7f) as i32;
        let m = Message::new("Acurite-5n1")
            .int("message_type", message_type)
            .int("id", sensor_id)
            .str("channel", channel_str)
            .int("sequence_num", sequence_num)
            .int("battery_ok", !battery_low as u8)
            .float("wind_avg_km_h", wind_speed_kph as f64, 1)
            .float("wind_dir_deg", wind_dir as f64, 1)
            .float("rain_in", (raincounter as f32 * 0.01) as f64, 2)
            .str("mic", "CHECKSUM");
        ctx.output(m);
    } else if message_type == MSGTYPE_5N1_WINDSPEED_TEMP_HUMIDITY {
        let temp_raw = (((bb[4] & 0x0f) as i32) << 7) | (bb[5] & 0x7f) as i32;
        let tempf = (temp_raw - 400) as f32 * 0.1;
        if !(-40.0..=158.0).contains(&tempf) {
            ctx.log(|| {
                format!("5n1 0x{sensor_id:04X} Ch {channel_str}, invalid temperature: {tempf:.1} F")
            });
            return DECODE_FAIL_SANITY;
        }
        let humidity = bb[6] & 0x7f;
        if humidity > 100 {
            ctx.log(|| {
                format!("5n1 0x{sensor_id:04X} Ch {channel_str}: invalid humidity: {humidity} %rH")
            });
            return DECODE_FAIL_SANITY;
        }
        let m = Message::new("Acurite-5n1")
            .int("message_type", message_type)
            .int("id", sensor_id)
            .str("channel", channel_str)
            .int("sequence_num", sequence_num)
            .int("battery_ok", !battery_low as u8)
            .float("wind_avg_km_h", wind_speed_kph as f64, 1)
            .float("temperature_F", tempf as f64, 1)
            .int("humidity", humidity)
            .str("mic", "CHECKSUM");
        ctx.output(m);
    } else {
        ctx.log(|| format!("5n1: unknown message type 0x{message_type:02x}"));
        return DECODE_FAIL_SANITY;
    }
    1
}

/// Acurite Atlas 7-in-1: wind speed in every message, then temperature +
/// humidity (0x05/0x25), direction + rain (0x06/0x26) or UV + lux
/// (0x07/0x27); the 0x2x types add lightning count and distance.
fn acurite_atlas_decode(ctx: &mut Ctx, bb: &[u8], browlen: usize) -> i32 {
    let mut exception = 0;
    let message_type = bb[2] & 0x3f;
    let sensor_id = (((bb[0] & 0x03) as u16) << 8) | bb[1] as u16;
    let channel_str = acurite_get_channel(bb[0]);
    // Which of the three copies this is.
    let sequence_num = (bb[0] & 0x0c) >> 2;
    let battery_low = (bb[2] & 0x40) == 0;
    // 8 bits of raw mph (spec 0-200).
    let wind_speed_mph = ((((bb[3] & 0x7f) as i32) << 1) | ((bb[4] & 0x40) >> 6) as i32) as f64;
    if wind_speed_mph > 200.0 {
        ctx.log(|| format!("Atlas 0x{sensor_id:04X} Ch {channel_str}, invalid wind speed: {wind_speed_mph:.1} MPH"));
        return DECODE_FAIL_SANITY;
    }

    let mut m = Message::new("Acurite-Atlas")
        .int("id", sensor_id)
        .str("channel", channel_str)
        .int("sequence_num", sequence_num)
        .int("battery_ok", !battery_low as u8)
        .int("message_type", message_type)
        .float("wind_avg_mi_h", wind_speed_mph, 1);

    if message_type == MSGTYPE_ATLAS_WNDSPD_TEMP_HUM
        || message_type == MSGTYPE_ATLAS_WNDSPD_TEMP_HUM_LTNG
    {
        // 11 bits decoded; flag the two spare bits if ever set.
        let temp_raw = (((bb[4] & 0x0f) as i32) << 7) | (bb[5] & 0x7f) as i32;
        if bb[4] & 0x30 != 0 {
            exception += 1;
        }
        let tempf = (temp_raw - 400) as f32 * 0.1;
        if !(-40.0..=158.0).contains(&tempf) {
            ctx.log(|| {
                format!(
                    "Atlas 0x{sensor_id:04X} Ch {channel_str}, invalid temperature: {tempf:.1} F"
                )
            });
            return DECODE_FAIL_SANITY;
        }
        let humidity = bb[6] & 0x7f;
        if humidity > 100 {
            ctx.log(|| {
                format!("0x{sensor_id:04X} Ch {channel_str}: impossible humidity: {humidity} %rH")
            });
            return DECODE_FAIL_SANITY;
        }
        if humidity == 0 {
            exception += 1;
        }
        m = m
            .float("temperature_F", tempf as f64, 1)
            .int("humidity", humidity);
    }

    if message_type == MSGTYPE_ATLAS_WNDSPD_RAIN || message_type == MSGTYPE_ATLAS_WNDSPD_RAIN_LTNG {
        let wind_dir = ((((bb[4] & 0x1f) as i32) << 5) | ((bb[5] & 0x7c) >> 2) as i32) as f64;
        if bb[4] & 0x30 != 0 {
            exception += 1;
        }
        if wind_dir > 360.0 {
            ctx.log(|| format!("Atlas 0x{sensor_id:04X} Ch {channel_str}, invalid wind direction: {wind_dir:.1}"));
            return DECODE_FAIL_SANITY;
        }
        // 9 bits, rolls over after 5.11 inches.
        let raincounter = (((bb[5] & 0x03) as i32) << 7) | (bb[6] & 0x7f) as i32;
        m = m.float("wind_dir_deg", wind_dir, 1).float(
            "rain_in",
            (raincounter as f32 * 0.01) as f64,
            2,
        );
    }

    if message_type == MSGTYPE_ATLAS_WNDSPD_UV_LUX
        || message_type == MSGTYPE_ATLAS_WNDSPD_UV_LUX_LTNG
    {
        let uv = bb[4] & 0x0f;
        // Lux / 10, 14 bits.
        let lux = (((bb[5] & 0x7f) as i32) << 7) | (bb[6] & 0x7f) as i32;
        if lux > 12000 {
            ctx.log(|| format!("Atlas 0x{sensor_id:04X} Ch {channel_str}, invalid lux {lux}"));
            return DECODE_FAIL_SANITY;
        }
        m = m.float("uvi", uv as f64, 0).int("lux", lux * 10);
    }

    if message_type == MSGTYPE_ATLAS_WNDSPD_TEMP_HUM_LTNG
        || message_type == MSGTYPE_ATLAS_WNDSPD_RAIN_LTNG
        || message_type == MSGTYPE_ATLAS_WNDSPD_UV_LUX_LTNG
    {
        let strike_count = (((bb[7] & 0x7f) as i32) << 2) | ((bb[8] & 0x60) >> 5) as i32;
        let strike_distance = bb[8] & 0x1f;
        m = m
            .int("strike_count", strike_count)
            .int("strike_distance", strike_distance);
    }

    m = m
        .int("exception", exception)
        .str("raw_msg", hex_str(&bb[..browlen.min(15)]));
    ctx.output(m);
    1
}

/// Acurite 592TXR tower: `CCII IIII | IIII IIII | pB00 0100 | pHHH HHHH |
/// p??T TTTT | pTTT TTTT | KKKK KKKK`, temperature 0.1 °C + 1000.
fn acurite_tower_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let mut exception = 0;
    let channel_str = acurite_get_channel(bb[0]);
    let sensor_id = (((bb[0] & 0x3f) as i32) << 8) | bb[1] as i32;
    let battery_low = (bb[2] & 0x40) == 0;
    // 0 is allowed (low battery or broken sensor); 127 means none (592TX).
    let humidity = (bb[3] & 0x7f) as i32;
    if humidity > 100 && humidity != 127 {
        ctx.log(|| format!("0x{sensor_id:04X} Ch {channel_str}: invalid humidity: {humidity} %rH"));
        return DECODE_FAIL_SANITY;
    }
    let temp_raw = (((bb[4] & 0x7f) as i32) << 7) | (bb[5] & 0x7f) as i32;
    let tempc = (temp_raw - 1000) as f32 * 0.1;
    if !(-40.0..=70.0).contains(&tempc) {
        ctx.log(|| {
            format!("0x{sensor_id:04X} Ch {channel_str}: invalid temperature: {tempc:.2} C")
        });
        return DECODE_FAIL_SANITY;
    }
    if temp_raw & 0x3800 != 0 {
        exception += 1;
    }
    let mut m = Message::new("Acurite-Tower")
        .int("id", sensor_id)
        .str("channel", channel_str)
        .int("battery_ok", !battery_low as u8)
        .float("temperature_C", tempc as f64, 1)
        .cond(humidity != 127, |m| m.int("humidity", humidity))
        .str("mic", "CHECKSUM");
    if exception != 0 {
        m = m
            .int("exception", exception)
            .str("raw_msg", hex_str(&bb[..ACURITE_TXR_BYTELEN]));
    }
    ctx.output(m);
    1
}

/// Acurite 1190/1192 leak detector: bit 0x10 of byte 3 is "wet".
fn acurite_1190_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let channel_str = acurite_get_channel(bb[0]);
    let sensor_id = (((bb[0] & 0x3f) as i32) << 8) | bb[1] as i32;
    let battery_low = (bb[2] & 0x40) == 0;
    let is_wet = (bb[3] & 0x10) >> 4;
    let m = Message::new("Acurite-Leak")
        .int("id", sensor_id)
        .str("channel", channel_str)
        .int("battery_ok", !battery_low as u8)
        .int("leak_detected", is_wet)
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

/// Acurite 515 refrigerator (0x08) / freezer (0x09): temperature 0.1 °F
/// + 1480 in 14 bits.
fn acurite_515_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let mut exception = 0;
    let message_type = bb[2] & 0x3f;
    let channel_str = acurite_get_channel(bb[0]);
    let kind = match message_type {
        MSGTYPE_515_REFRIGERATOR => 'R',
        MSGTYPE_515_FREEZER => 'F',
        _ => {
            ctx.log(|| format!("515: unknown message type 0x{message_type:02x}"));
            return DECODE_FAIL_SANITY;
        }
    };
    let channel_type_str = format!("{channel_str}{kind}");
    let sensor_id = (((bb[0] & 0x3f) as u16) << 8) | bb[1] as u16;
    let temp_raw = (((bb[3] & 0x7f) as i32) << 7) | (bb[4] & 0x7f) as i32;
    let tempf = (temp_raw - 1480) as f32 * 0.1;
    if !(-40.0..=158.0).contains(&tempf) {
        ctx.log(|| {
            format!("515 0x{sensor_id:04X} Ch {channel_str}, invalid temperature: {tempf:.1} F")
        });
        return DECODE_FAIL_SANITY;
    }
    if temp_raw & 0x3000 != 0 {
        exception += 1;
    }
    let battery_low = (bb[2] & 0x40) == 0;
    let mut m = Message::new("Acurite-515")
        .int("id", sensor_id)
        .str("channel", channel_type_str)
        .int("battery_ok", !battery_low as u8)
        .float("temperature_F", tempf as f64, 1)
        .str("mic", "CHECKSUM");
    if exception != 0 {
        m = m
            .int("exception", exception)
            .str("raw_msg", hex_str(&bb[..ACURITE_515_BYTELEN]));
    }
    ctx.output(m);
    1
}

/// AcuRite Optimus 6-in-1 (06188M): 10 bytes, wind speed in each, then
/// temperature + humidity (0x3b) or direction + rain (0x3a).
fn acurite_optimus_decode(ctx: &mut Ctx, bb: &[u8]) -> i32 {
    let channel_str = acurite_get_channel(bb[0]);
    let sensor_id = (((bb[0] & 0x0f) as u16) << 8) | bb[1] as u16;
    let sequence_num = (bb[0] & 0x30) >> 4;
    let battery_low = (bb[2] & 0x40) == 0;
    let message_type = bb[2] & 0x3f;

    let wind_speed_mph = ((((bb[3] & 0x7f) as i32) << 1) | ((bb[4] & 0x40) >> 6) as i32) as f32;
    if wind_speed_mph > 200.0 {
        ctx.log(|| format!("Optimus 0x{sensor_id:04X} Ch {channel_str}, invalid wind speed: {wind_speed_mph:.1} MPH"));
        return DECODE_FAIL_SANITY;
    }
    let wind_speed_kmh = wind_speed_mph * 1.609344;

    let mut m = Message::new("Acurite-Optimus")
        .int("id", sensor_id)
        .str("channel", channel_str)
        .int("sequence_num", sequence_num)
        .int("battery_ok", !battery_low as u8)
        .float("wind_avg_mi_h", wind_speed_mph as f64, 1)
        .float("wind_avg_km_h", wind_speed_kmh as f64, 1);

    if message_type == MSGTYPE_OPTIMUS_TEMP_HUM {
        let temp_raw = (((bb[4] & 0x0f) as i32) << 7) | (bb[5] & 0x7f) as i32;
        let tempf = (temp_raw - 400) as f32 * 0.1;
        if !(-40.0..=158.0).contains(&tempf) {
            ctx.log(|| {
                format!(
                    "Optimus 0x{sensor_id:04X} Ch {channel_str}, invalid temperature: {tempf:.1} F"
                )
            });
            return DECODE_FAIL_SANITY;
        }
        let humidity = bb[6] & 0x7f;
        if humidity > 100 {
            ctx.log(|| {
                format!(
                    "Optimus 0x{sensor_id:04X} Ch {channel_str}, invalid humidity: {humidity} %rH"
                )
            });
            return DECODE_FAIL_SANITY;
        }
        m = m
            .float("temperature_F", tempf as f64, 1)
            .int("humidity", humidity);
    } else if message_type == MSGTYPE_OPTIMUS_WIND_RAIN {
        let wind_dir = ACURITE_5N1_WINDDIRECTIONS[(bb[4] & 0x0f) as usize] as f32 * 22.5;
        let raincounter = (((bb[5] & 0x03) as i32) << 7) | (bb[6] & 0x7f) as i32;
        let rain_in = raincounter as f32 * 0.01;
        m = m
            .float("wind_dir_deg", wind_dir as f64, 1)
            .float("rain_in", rain_in as f64, 2);
    }
    m = m.str("raw_msg", hex_str(&bb[..ACURITE_OPTIMUS_BYTELEN]));
    ctx.output(m);
    1
}

/// Check a tower-family row: length, the add-checksum in byte
/// `explen - 1`, even parity of bytes 2..explen-1, and (unless
/// `check_chan` is off) a valid channel. 0 if good, else a decode error.
/// Longer rows pass if their first `explen` bytes do.
fn acurite_txr_check(
    ctx: &mut Ctx,
    bb: &[u8],
    browlen: usize,
    explen: usize,
    check_chan: bool,
) -> i32 {
    if browlen < 6 {
        return DECODE_ABORT_LENGTH;
    }
    if browlen < explen {
        ctx.log(|| {
            format!(
                "acurite_txr: wrong length for msg type: {}",
                hex_str(&bb[..browlen])
            )
        });
        return DECODE_ABORT_LENGTH;
    }
    if (add_bytes(&bb[..explen - 1]) & 0xff) != bb[explen - 1] as i32 {
        ctx.log(|| format!("acurite_txr: bad checksum: {}", hex_str(&bb[..browlen])));
        return DECODE_FAIL_MIC;
    }
    if parity_bytes(&bb[2..explen - 1]) != 0 {
        ctx.log(|| format!("acurite_txr: bad parity: {}", hex_str(&bb[..browlen])));
        return DECODE_FAIL_MIC;
    }
    if check_chan && acurite_get_channel(bb[0]) == "E" {
        let message_type = bb[2] & 0x3f;
        ctx.log(|| {
            format!("acurite_txr: bad channel, msg type 0x{message_type:02x}, msg len {browlen}")
        });
        return DECODE_FAIL_SANITY;
    }
    0
}

/// The tower family's dispatcher: invert the rows, then for each row of 6
/// to 10 bytes check it as its message type needs and decode it.
fn acurite_txr_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let mut decoded = 0;
    let mut error_ret = 0;

    bb.invert();

    for brow in 0..bb.num_rows {
        let row_bit_cnt = bb.bits(brow);
        // Rounding down is safe: extra bits are spurious.
        let browlen = row_bit_cnt / 8;
        if browlen < 6 {
            continue; // quietly skip short rows
        }
        // Longest known message is 10 bytes (Atlas with lightning).
        if browlen > 10 {
            ctx.log(|| {
                format!(
                    "acurite_txr: skipping wrong len row {brow} bits {row_bit_cnt}, bytes {browlen}"
                )
            });
            error_ret = DECODE_ABORT_LENGTH;
            continue;
        }
        // A copy of the row's first bytes, so the sub-decoders can take a
        // plain slice while `bb` stays borrowed only here.
        let mut row = [0u8; 16];
        row.copy_from_slice(&bb.row(brow)[..16]);
        let b = &row[..];

        // Quietly ignore rows of zeros.
        if b[0] == 0 && b[1] == 0 && b[2] == 0 && b[browlen - 1] == 0 {
            continue;
        }

        let message_type = b[2] & 0x3f;
        if !matches!(
            message_type,
            MSGTYPE_1190_DETECTOR
                | MSGTYPE_TOWER_SENSOR
                | MSGTYPE_6045M
                | MSGTYPE_5N1_WINDSPEED_WINDDIR_RAINFALL
                | MSGTYPE_5N1_WINDSPEED_TEMP_HUMIDITY
                | MSGTYPE_OPTIMUS_WIND_RAIN
                | MSGTYPE_OPTIMUS_TEMP_HUM
                | MSGTYPE_ATLAS_WNDSPD_TEMP_HUM
                | MSGTYPE_ATLAS_WNDSPD_RAIN
                | MSGTYPE_ATLAS_WNDSPD_UV_LUX
                | MSGTYPE_ATLAS_WNDSPD_TEMP_HUM_LTNG
                | MSGTYPE_ATLAS_WNDSPD_RAIN_LTNG
                | MSGTYPE_ATLAS_WNDSPD_UV_LUX_LTNG
                | MSGTYPE_515_REFRIGERATOR
                | MSGTYPE_515_FREEZER
                | MSGTYPE_3N1_WINDSPEED_TEMP_HUMIDITY
                | MSGTYPE_899_RAINFALL
        ) {
            ctx.log(|| format!("acurite_txr: unknown message type {}", bb.row_string(brow)));
            error_ret = DECODE_FAIL_SANITY;
            continue;
        }

        // Check, then decode; record the error and go on to the next row
        // otherwise. Rust note: a closure can't borrow `ctx` while we also
        // pass it, so this is a small local macro instead.
        macro_rules! run {
            ($explen:expr, $chan:expr, $decode:expr) => {{
                let ret = acurite_txr_check(ctx, b, browlen, $explen, $chan);
                if ret != 0 {
                    error_ret = ret;
                } else {
                    let ret = $decode;
                    if ret > 0 {
                        decoded += ret;
                    } else if ret < 0 {
                        error_ret = ret;
                    }
                }
            }};
        }

        match message_type {
            MSGTYPE_TOWER_SENSOR => run!(ACURITE_TXR_BYTELEN, true, acurite_tower_decode(ctx, b)),
            MSGTYPE_1190_DETECTOR => run!(ACURITE_1190_BYTELEN, true, acurite_1190_decode(ctx, b)),
            MSGTYPE_6045M => {
                let browlen_full = row_bit_cnt.div_ceil(8);
                run!(
                    ACURITE_6045_BYTELEN,
                    true,
                    acurite_6045_decode(ctx, b, browlen_full)
                )
            }
            MSGTYPE_515_REFRIGERATOR | MSGTYPE_515_FREEZER => {
                run!(ACURITE_515_BYTELEN, true, acurite_515_decode(ctx, b))
            }
            MSGTYPE_5N1_WINDSPEED_TEMP_HUMIDITY | MSGTYPE_5N1_WINDSPEED_WINDDIR_RAINFALL => {
                run!(ACURITE_5N1_BYTELEN, true, acurite_5n1_decode(ctx, b))
            }
            MSGTYPE_OPTIMUS_TEMP_HUM | MSGTYPE_OPTIMUS_WIND_RAIN => {
                run!(
                    ACURITE_OPTIMUS_BYTELEN,
                    true,
                    acurite_optimus_decode(ctx, b)
                )
            }
            MSGTYPE_3N1_WINDSPEED_TEMP_HUMIDITY => {
                // rtl_433 checks only the checksum here, not parity.
                if browlen < ACURITE_3N1_BYTELEN {
                    ctx.log(|| "acurite_txr: 3n1 wrong length".into());
                    error_ret = DECODE_ABORT_LENGTH;
                    continue;
                }
                if (add_bytes(&b[..ACURITE_3N1_BYTELEN - 1]) & 0xff)
                    != b[ACURITE_3N1_BYTELEN - 1] as i32
                {
                    ctx.log(|| "acurite_txr: 3n1 bad checksum".into());
                    error_ret = DECODE_FAIL_MIC;
                    continue;
                }
                let ret = acurite_3n1_decode(ctx, b);
                if ret > 0 {
                    decoded += ret;
                } else if ret < 0 {
                    error_ret = ret;
                }
            }
            // The 899 numbers its channels differently: no channel check.
            MSGTYPE_899_RAINFALL => run!(ACURITE_899_BYTELEN, false, acurite_899_decode(ctx, b)),
            MSGTYPE_ATLAS_WNDSPD_TEMP_HUM
            | MSGTYPE_ATLAS_WNDSPD_RAIN
            | MSGTYPE_ATLAS_WNDSPD_UV_LUX => {
                let browlen_full = row_bit_cnt.div_ceil(8);
                run!(
                    ACURITE_ATLAS_BYTELEN,
                    true,
                    acurite_atlas_decode(ctx, b, browlen_full)
                )
            }
            _ => {
                // The three Atlas types with lightning (10 bytes).
                let browlen_full = row_bit_cnt.div_ceil(8);
                run!(
                    ACURITE_ATLAS_LTNG_BYTELEN,
                    true,
                    acurite_atlas_decode(ctx, b, browlen_full)
                )
            }
        }
    }

    if decoded > 0 { decoded } else { error_ret }
}

/// Acurite 986 refrigerator/freezer: 5 bytes sent LSB first
/// (`TT II II SS CC`), temperature in °F sign and magnitude, CRC-8 poly
/// 0x07 little-endian. The last bit is often lost: rtl_433 accepts that.
fn acurite_986_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const BROWLEN: usize = 5;
    let mut valid_cnt = 0;
    let mut result = 0;

    for brow in 0..bb.num_rows {
        let bits = bb.bits(brow);
        if !(39..=43).contains(&bits) {
            if bits > 16 {
                ctx.log(|| "acurite_986: skipping wrong len".into());
            }
            result = DECODE_ABORT_LENGTH;
            continue;
        }
        let b = bb.row(brow);
        // Reduce false positives.
        if (b[0] == 0xff && b[1] == 0xff && b[2] == 0xff) || (b[0] == 0 && b[1] == 0 && b[2] == 0) {
            result = DECODE_ABORT_EARLY;
            continue;
        }
        // Sent LSB first: reverse each byte.
        let mut br = [0u8; BROWLEN];
        for (o, &i) in br.iter_mut().zip(b) {
            *o = reverse8(i);
        }

        let sensor_id = ((br[1] as u16) << 8) + br[2] as u16;
        let mut status = br[3];
        let sensor_num = (status & 0x01) + 1;
        status >>= 1;
        let battery_low = (status & 1) == 1;
        // By default sensor 1 is the refrigerator, 2 the freezer.
        let channel_str = if sensor_num == 2 { "2F" } else { "1R" };

        let crc = br[4];
        let crcc = crc8le(&br[..4], 0x07, 0);
        if crcc != crc {
            // The message often lacks its last 1 bit (0x80, sent LSB
            // first): accept the CRC with it put back.
            if crcc == (crc | 0x80) {
                ctx.log(|| format!("acurite_986: CRC fix {crc:02x} - {crcc:02x}"));
            } else {
                continue; // DECODE_FAIL_MIC
            }
        }

        // 8 bits, sign and magnitude.
        let mut tempf = br[0] as i8 as i32;
        if br[0] & 0x80 != 0 {
            tempf = -((br[0] & 0x7f) as i32);
        }

        let m = Message::new("Acurite-986")
            .int("id", sensor_id)
            .str("channel", channel_str)
            .int("battery_ok", !battery_low as u8)
            // rtl_433 prints "%f F": 6 decimals.
            .float("temperature_F", tempf as f64, 6)
            .int("status", status)
            .str("mic", "CRC");
        ctx.output(m);
        valid_cnt += 1;
    }
    if valid_cnt > 0 { 1 } else { result }
}

/// Acurite 606TX / Technoline TX960: `IIIIIIII BbCCTTTT TTTTTTTT KKKKKKKK`
/// (battery, button, channel, 12-bit signed 0.1 °C), K an LFSR digest
/// (generator 0x98, key 0xf1).
fn acurite_606_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Six rows expected.
    let Some(row) = bb.find_repeated_row(3, 32) else {
        return DECODE_ABORT_EARLY;
    };
    if bb.bits(row) > 33 {
        return DECODE_ABORT_LENGTH;
    }
    let b = bb.row(row);
    // Reject blank messages.
    if b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0 {
        return DECODE_FAIL_SANITY;
    }
    if lfsr_digest8(&b[..3], 0x98, 0xf1) != b[3] {
        return DECODE_FAIL_MIC;
    }
    let sensor_id = b[0];
    let battery_ok = (b[1] & 0x80) >> 7;
    let channel = ((b[1] & 0x30) >> 4) + 1;
    let button = (b[1] & 0x40) >> 6;
    // 12 bits sign-extended: the high nibble of b[1] falls off the i16.
    let temp_raw = ((((b[1] as u16) << 12) | ((b[2] as u16) << 4)) as i16) >> 4;
    let temp_c = temp_raw as f32 * 0.1;
    let m = Message::new("Acurite-606TX")
        .int("id", sensor_id)
        .int("channel", channel)
        .int("battery_ok", battery_ok)
        .int("button", button)
        .float("temperature_C", temp_c as f64, 1)
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

/// Acurite 00275rm / 00276rm room monitors with optional probe: 88 bits
/// sent inverted three times, CRC-16 (LSB first, poly 0x00b2, init
/// 0x00d0). Three good-length rows are also combined bit by bit (majority
/// vote) as a fallback.
fn acurite_00275rm_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let mut result = 0;
    bb.invert();

    let mut rows = [0usize; 3];
    let mut n_rows = 0;
    for row in 0..bb.num_rows {
        if n_rows < 3 && bb.bits(row) == 88 {
            rows[n_rows] = row;
            n_rows += 1;
        }
    }
    if n_rows == 3 {
        let mut b = [0u8; 11];
        for (i, o) in b.iter_mut().enumerate() {
            let (x, y, z) = (bb.bb[rows[0]][i], bb.bb[rows[1]][i], bb.bb[rows[2]][i]);
            *o = (x & y) | (y & z) | (z & x);
        }
        bb.add_row();
        let last = bb.num_rows - 1;
        bb.bb[last][..11].copy_from_slice(&b);
        bb.bits_per_row[last] = 88;
    }

    // Output the first valid row.
    for row in 0..bb.num_rows {
        if bb.bits(row) != 88 {
            result = DECODE_ABORT_LENGTH;
            continue;
        }
        let b = bb.row(row);
        if crc16lsb(&b[..11], 0x00b2, 0x00d0) != 0 {
            ctx.log(|| format!("acurite_00275rm: sensor bad CRC {}", hex_str(&b[..11])));
            result = DECODE_FAIL_MIC;
            continue;
        }
        let id = ((b[0] as i32) << 16) | ((b[1] as i32) << 8) | b[3] as i32;
        let battery_low = (b[2] & 0x40) == 0;
        let model_flag = b[2] & 1 != 0;
        let temp_raw = ((b[4] as i32) << 4) | (b[5] >> 4) as i32;
        let tempc = (temp_raw - 1000) as f32 * 0.1;
        let probe = b[5] & 3;
        let humidity = (((b[6] & 0x1f) as i32) << 2) | (b[7] >> 6) as i32;
        // Water probe: valid only if probe == 1.
        let water = (b[7] & 0x0f) == 15;
        // Soil / spot probe temperature: probe 2 or 3.
        let ptemp_raw = (((b[7] & 0x0f) as i32) << 8) | b[8] as i32;
        let ptempc = (ptemp_raw - 1000) as f32 * 0.1;
        // Spot probe humidity: probe 3.
        let phumidity = b[9] & 0x7f;

        let m = Message::new(if model_flag {
            "Acurite-00275rm"
        } else {
            "Acurite-00276rm"
        })
        .int("subtype", probe)
        .int("id", id)
        .int("battery_ok", !battery_low as u8)
        .float("temperature_C", tempc as f64, 1)
        .int("humidity", humidity)
        .cond(probe == 1, |m| m.int("water", water as u8))
        .cond(probe == 2 || probe == 3, |m| {
            m.float("temperature_1_C", ptempc as f64, 1)
        })
        .cond(probe == 3, |m| m.int("humidity_1", phumidity))
        .str("mic", "CRC");
        ctx.output(m);
        return 1;
    }
    result
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "Acurite 896 Rain Gauge",
        group: Group::Weather,
        modulation: Modulation::OokPpm,
        short_width: 1000.0,
        long_width: 2000.0,
        gap_limit: 3500.0,
        reset_limit: 5000.0,
        decode: acurite_rain_896_decode,
        // Let Oregon Scientific v1 go first: fewer false positives.
        priority: 10,
        ..Device::DEFAULT
    },
    Device {
        name: "Acurite 609TXC Temperature and Humidity Sensor",
        group: Group::Weather,
        modulation: Modulation::OokPpm,
        short_width: 1000.0,
        long_width: 2000.0,
        gap_limit: 3000.0,
        reset_limit: 10000.0,
        decode: acurite_th_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Acurite 592TXR temp/humidity, 592TX temp, 5n1, 3n1, Atlas weather station, 515 fridge/freezer, 6045 lightning, 899 rain, 1190/1192 leak",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 220.0,  // 220 µs pulse + 392 µs gap
        long_width: 408.0,   // 408 µs pulse + 204 µs gap
        sync_width: 620.0,   // 620 µs pulse + 596 µs gap
        gap_limit: 500.0,    // longest data gap 392 µs, sync gap 596 µs
        reset_limit: 4000.0, // packet gap 2192 µs
        decode: acurite_txr_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Acurite 986 Refrigerator / Freezer Thermometer",
        group: Group::Weather,
        modulation: Modulation::OokPpm,
        short_width: 520.0,
        long_width: 880.0,
        gap_limit: 1280.0,
        reset_limit: 4000.0,
        decode: acurite_986_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Acurite 606TX / Technoline TX960 Temperature Sensor",
        group: Group::Weather,
        modulation: Modulation::OokPpm,
        short_width: 2000.0,
        long_width: 4000.0,
        gap_limit: 7000.0,
        reset_limit: 10000.0,
        decode: acurite_606_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Acurite 00275rm,00276rm Temp/Humidity with optional probe",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 232.0, // short pulse 232 µs
        long_width: 420.0,  // long pulse 420 µs
        gap_limit: 520.0,   // long gap 384 µs, sync gap 592 µs
        reset_limit: 708.0, // no packet gap, sync gap 592 µs
        sync_width: 632.0,  // sync pulse 632 µs
        decode: acurite_00275rm_decode,
        ..Device::DEFAULT
    },
];

/// The tower family's rows are sent three times.
const TXR_AIR: Air = Air::DEFAULT;

/// All made up for the simulator, checksums, parities and CRCs computed.
/// The tower-family rows are as the slicer gives them (inverted).
pub static SAMPLES: &[Sample] = &[
    // 896: id 0x5a, 42 tips = 21.0 mm, 12 repeats.
    Sample {
        dev: 0,
        model: "Acurite-Rain",
        code: "{24}5a802a {24}5a802a {24}5a802a {24}5a802a {24}5a802a {24}5a802a \
               {24}5a802a {24}5a802a {24}5a802a {24}5a802a {24}5a802a {24}5a802a",
        expect: &[("id", "90"), ("rain_mm", "21.0")],
        air: Air::DEFAULT,
    },
    // 609TXC: id 0x3c, status 2, 21.3 °C, 45 %.
    Sample {
        dev: 1,
        model: "Acurite-609TXC",
        code: "{40}3c20d52d5e {40}3c20d52d5e",
        expect: &[
            ("id", "60"),
            ("battery_ok", "1"),
            ("temperature_C", "21.3"),
            ("humidity", "45"),
            ("status", "2"),
        ],
        air: Air::DEFAULT,
    },
    // Tower: channel A, id 0x1234, battery ok, 21.5 °C, 50 %.
    Sample {
        dev: 2,
        model: "Acurite-Tower",
        code: "{56}2dcbbb4df6c0bb {56}2dcbbb4df6c0bb {56}2dcbbb4df6c0bb",
        expect: &[
            ("id", "4660"),
            ("channel", "A"),
            ("battery_ok", "1"),
            ("temperature_C", "21.5"),
            ("humidity", "50"),
        ],
        air: TXR_AIR,
    },
    // 5-in-1 wind/direction/rain: channel B, id 0x567, wind raw 0,
    // direction index 0xa (E), 0.01 in × 172.
    Sample {
        dev: 2,
        model: "Acurite-5n1",
        code: "{64}6a988efa557e53b6 {64}6a988efa557e53b6",
        expect: &[
            ("message_type", "49"),
            ("id", "1383"),
            ("channel", "B"),
            ("sequence_num", "1"),
            ("wind_dir_deg", "90.0"),
            ("rain_in", "1.72"),
        ],
        air: TXR_AIR,
    },
    // 5-in-1 wind/temperature/humidity: wind raw 10, 72.5 °F, 45 %.
    Sample {
        dev: 2,
        model: "Acurite-5n1",
        code: "{64}6a98877ed79ad250 {64}6a98877ed79ad250",
        expect: &[
            ("message_type", "56"),
            ("wind_avg_km_h", "9.3"),
            ("temperature_F", "72.5"),
            ("humidity", "45"),
        ],
        air: TXR_AIR,
    },
    // 3-in-1: channel C, id 0xa2b, 68.0 °F, 40 %, 7 mph.
    Sample {
        dev: 2,
        model: "Acurite-3n1",
        code: "{64}f5d49fd76f0f783b {64}f5d49fd76f0f783b",
        expect: &[
            ("id", "0xa2b"),
            ("channel", "C"),
            ("temperature_F", "68.0"),
            ("humidity", "40"),
            ("wind_avg_mi_h", "7.0"),
        ],
        air: TXR_AIR,
    },
    // Atlas (no lightning): channel A, id 0x2f3, 12 mph, 70.0 °F, 33 %.
    Sample {
        dev: 2,
        model: "Acurite-Atlas",
        code: "{64}3d0c3af97733de0a {64}3d0c3af97733de0a",
        expect: &[
            ("id", "755"),
            ("channel", "A"),
            ("message_type", "5"),
            ("wind_avg_mi_h", "12.0"),
            ("temperature_F", "70.0"),
            ("humidity", "33"),
            ("exception", "0"),
        ],
        air: TXR_AIR,
    },
    // Atlas with lightning, UV/lux: sequence 1, 4 mph, UV 5, 25000 lux,
    // 3 strikes at distance 10.
    Sample {
        dev: 2,
        model: "Acurite-Atlas",
        code: "{80}390c187dfa6cbbff9597 {80}390c187dfa6cbbff9597",
        expect: &[
            ("sequence_num", "1"),
            ("message_type", "39"),
            ("wind_avg_mi_h", "4.0"),
            ("uvi", "5"),
            ("lux", "25000"),
            ("strike_count", "3"),
            ("strike_distance", "10"),
        ],
        air: TXR_AIR,
    },
    // 6045M: channel B, id 0x1abc, 75.0 °F, 60 %, active, 5 strikes at 8.
    Sample {
        dev: 2,
        model: "Acurite-6045M",
        code: "{72}654390c32ec97db72d {72}654390c32ec97db72d",
        expect: &[
            ("id", "6844"),
            ("channel", "B"),
            ("battery_ok", "1"),
            ("temperature_F", "75.0"),
            ("humidity", "60"),
            ("strike_count", "5"),
            ("storm_dist", "8"),
            ("active", "1"),
            ("rfi", "0"),
        ],
        air: TXR_AIR,
    },
    // 515 freezer: channel A, id 0x123, 5.0 °F.
    Sample {
        dev: 2,
        model: "Acurite-515",
        code: "{48}3edc367405cd {48}3edc367405cd",
        expect: &[("id", "291"), ("channel", "AF"), ("temperature_F", "5.0")],
        air: TXR_AIR,
    },
    // 1190 leak: channel C, id 0x456, wet.
    Sample {
        dev: 2,
        model: "Acurite-Leak",
        code: "{56}fba9be6fffffd4 {56}fba9be6fffffd4",
        expect: &[("id", "1110"), ("channel", "C"), ("leak_detected", "1")],
        air: TXR_AIR,
    },
    // 899 rain: id 0x789, numeric channel 1, 123 tips = 31.24 mm.
    Sample {
        dev: 2,
        model: "Acurite-Rain899",
        code: "{64}b8760fffffff84c4 {64}b8760fffffff84c4",
        expect: &[("id", "1929"), ("channel", "1"), ("rain_mm", "31.24")],
        air: TXR_AIR,
    },
    // Optimus temperature/humidity: id 0x345, 6 mph, 68.5 °F, 52 %.
    Sample {
        dev: 2,
        model: "Acurite-Optimus",
        code: "{80}3cba84fc77424bff30b1 {80}3cba84fc77424bff30b1",
        expect: &[
            ("id", "837"),
            ("channel", "A"),
            ("wind_avg_mi_h", "6.0"),
            ("wind_avg_km_h", "9.7"),
            ("temperature_F", "68.5"),
            ("humidity", "52"),
        ],
        air: TXR_AIR,
    },
    // 986: id 0x1234, sensor 1 (refrigerator), 38 °F.
    Sample {
        dev: 3,
        model: "Acurite-986",
        code: "{40}64482c0088 {40}64482c0088",
        expect: &[
            ("id", "4660"),
            ("channel", "1R"),
            ("battery_ok", "1"),
            ("temperature_F", "38.000000"),
        ],
        air: Air {
            pulse_us: 220.0,
            ..Air::DEFAULT
        },
    },
    // 606TX: id 0x5a, channel 1, battery ok, 22.4 °C, 6 repeats.
    Sample {
        dev: 4,
        model: "Acurite-606TX",
        code: "{32}5a80e060 {32}5a80e060 {32}5a80e060 {32}5a80e060 {32}5a80e060 {32}5a80e060",
        expect: &[
            ("id", "90"),
            ("channel", "1"),
            ("battery_ok", "1"),
            ("button", "0"),
            ("temperature_C", "22.4"),
        ],
        air: Air {
            pulse_us: 500.0,
            ..Air::DEFAULT
        },
    },
    // 00275rm: id 0x123456, battery ok, 22.0 °C, 48 %, no probe.
    Sample {
        dev: 5,
        model: "Acurite-00275rm",
        code: "{88}edcbbea9b3bff3ffff8aff {88}edcbbea9b3bff3ffff8aff {88}edcbbea9b3bff3ffff8aff",
        expect: &[
            ("subtype", "0"),
            ("id", "1193046"),
            ("battery_ok", "1"),
            ("temperature_C", "22.0"),
            ("humidity", "48"),
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
    fn txr_rejects_bad_rows() {
        // A changed bit fails the checksum.
        assert!(decode_rows(&DEVICES[2], "{56}2dcbbb4df6c0ba").is_empty());
        // Too short a row for a tower message.
        assert!(decode_rows(&DEVICES[2], "{48}2dcbbb4df6c0").is_empty());
    }

    #[test]
    fn acurite_986_missing_last_bit() {
        // The CRC's last bit (sent last) lost: rtl_433 still accepts it.
        // 30 °F, CRC 0xe0: its top bit (sent last) dropped.
        let m = decode_rows(&DEVICES[3], "{39}78482c0007");
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("temperature_F").unwrap().to_string(), "30.000000");
    }

    #[test]
    fn acurite_00275rm_majority_vote() {
        // One bad bit in each of three copies (different places): the
        // vote repairs it.
        let m = decode_rows(
            &DEVICES[5],
            "{88}ecdbbea9b3bff3ffff8aff {88}edcbbfa9b3bff3ffff8aff {88}edcbbea9b3bff3fffe8aff",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "22.0");
    }
}
