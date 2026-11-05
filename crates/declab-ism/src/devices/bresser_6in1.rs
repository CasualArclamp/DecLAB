// made by CasualArclamp
//! Bresser Weather Center 6-in-1, also the 7-in-1 indoor sensor, the new
//! 5-in-1, the 3-in-1 wind gauge, soil probe, thermo/hygro and pool
//! sensors, Froggit WH6000, Ventus C8488A, SENCOR SWS 9898, Ambient
//! Weather TX-3110B.
//!
//! Ported from rtl_433's `src/devices/bresser_6in1.c` (GPL-2.0-or-later).
//!
//! 868 MHz FSK PCM, 124 µs bits. Preamble `aa aa 2d d4`, then 18 bytes:
//! an LFSR-16 digest (generator 0x8810, key 0x5412) over bytes 2..17, a
//! 32-bit ID, sensor type / startup / channel, inverted BCD wind speeds,
//! BCD wind direction, then either temperature + humidity or (when the
//! rain flag in byte 16 is set) an inverted BCD rain counter, inverted BCD
//! UV, and an 8-bit add-checksum making bytes 2..17 sum to 0xff.

use crate::bits::{BitBuffer, add_bytes, lfsr_digest16};
use crate::device::*;
use crate::sim::{Air, Sample};

fn bresser_6in1_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 4] = [0xaa, 0xaa, 0x2d, 0xd4];
    // Moisture is sent as an index 1-16 in the humidity field.
    const MOISTURE_MAP: [i32; 16] = [0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99];
    let mut msg = [0u8; 18];

    if bb.num_rows != 1 || bb.bits(0) < 160 || bb.bits(0) > 440 {
        ctx.log(|| format!("bresser_6in1: bit_per_row {} out of range", bb.bits(0)));
        return DECODE_ABORT_EARLY;
    }
    let start_pos = bb.search(0, 0, &PREAMBLE, PREAMBLE.len() * 8) + PREAMBLE.len() * 8;
    if start_pos >= bb.bits(0) {
        return DECODE_ABORT_LENGTH;
    }
    let len = bb.bits(0) - start_pos;
    if len < msg.len() * 8 {
        ctx.log(|| format!("bresser_6in1: {len} too short"));
        return DECODE_ABORT_LENGTH;
    }
    let n = msg.len() * 8;
    bb.extract_bytes(0, start_pos, &mut msg, n);

    let chkdgst = ((msg[0] as u16) << 8) | msg[1] as u16;
    let digest = lfsr_digest16(&msg[2..17], 0x8810, 0x5412);
    if chkdgst != digest {
        ctx.log(|| format!("bresser_6in1: digest check failed {chkdgst:04x} vs {digest:04x}"));
        return DECODE_FAIL_MIC;
    }
    // Add with carry over bytes 2..=17 must give 0xff.
    let sum = add_bytes(&msg[2..18]);
    if sum & 0xff != 0xff {
        ctx.log(|| format!("bresser_6in1: checksum failed {sum:04x}"));
        return DECODE_FAIL_MIC;
    }

    let id = u32::from_be_bytes([msg[2], msg[3], msg[4], msg[5]]);
    let s_type = (msg[6] >> 4) as i32; // 1 weather station, 2 thermo/hygro, 3 pool, 4 soil
    let startup = ((msg[6] >> 3) & 1) as i32;
    let chan = (msg[6] & 0x7) as i32;
    let battery = ((msg[13] >> 1) & 1) as i32;

    // Temperature and humidity share their bytes with the rain counter;
    // only valid if they are valid BCD.
    let temp_ok = msg[12] <= 0x99 && (msg[13] & 0xf0) <= 0x90;
    let temp_raw =
        (msg[12] >> 4) as i32 * 100 + (msg[12] & 0x0f) as i32 * 10 + (msg[13] >> 4) as i32;
    let temp_sign = (msg[13] >> 3) & 1;
    let mut temp_c = temp_raw as f32 * 0.1;
    if temp_sign != 0 {
        temp_c = (temp_raw - 1000) as f32 * 0.1;
    }
    // Correction for the 3-in-1 Professional Wind Gauge, PN 7002531.
    if temp_c < -50.0 {
        temp_c = -temp_raw as f32 * 0.1;
    }

    let humidity = (msg[14] >> 4) as i32 * 10 + (msg[14] & 0x0f) as i32;

    // Inverted BCD; ff01 or 0000 when not available.
    let (n15, n16) = (!msg[15], !msg[16]);
    let mut uv_ok = (msg[16] & 0x0f) == 0 && n15 <= 0x99 && (n16 & 0xf0) <= 0x90;
    let uv_raw =
        ((n15 & 0xf0) >> 4) as i32 * 100 + (n15 & 0x0f) as i32 * 10 + ((n16 & 0xf0) >> 4) as i32;
    let uvi = uv_raw as f32 * 0.1;
    let flags = (msg[16] & 0x0f) as i32;

    // The three wind bytes are inverted.
    msg[7] ^= 0xff;
    msg[8] ^= 0xff;
    msg[9] ^= 0xff;
    let mut wind_ok = msg[7] <= 0x99 && msg[8] <= 0x99 && msg[9] <= 0x99;
    let gust_raw = (msg[7] >> 4) as i32 * 100 + (msg[7] & 0x0f) as i32 * 10 + (msg[8] >> 4) as i32;
    let wind_gust = gust_raw as f32 * 0.1;
    let wavg_raw =
        (msg[9] >> 4) as i32 * 100 + (msg[9] & 0x0f) as i32 * 10 + (msg[8] & 0x0f) as i32;
    let wind_avg = wavg_raw as f32 * 0.1;
    let wind_dir = ((msg[10] & 0xf0) >> 4) as i32 * 100
        + (msg[10] & 0x0f) as i32 * 10
        + ((msg[11] & 0xf0) >> 4) as i32;

    // Rain counter: inverted 3-byte BCD in the temperature/humidity bytes.
    msg[12] ^= 0xff;
    msg[13] ^= 0xff;
    msg[14] ^= 0xff;
    let rain_ok = msg[16] & 1 != 0;
    let rain_raw = (msg[12] >> 4) as i32 * 100000
        + (msg[12] & 0x0f) as i32 * 10000
        + (msg[13] >> 4) as i32 * 1000
        + (msg[13] & 0x0f) as i32 * 100
        + (msg[14] >> 4) as i32 * 10
        + (msg[14] & 0x0f) as i32;
    let rain_mm = rain_raw as f32 * 0.1;

    // The thermo/hygro and soil sensors may send plausible wind and UV but
    // have no such hardware.
    if s_type == 2 || s_type == 4 {
        wind_ok = false;
        uv_ok = false;
    }

    let mut moisture = -1;
    if s_type == 4 && temp_ok && (1..=16).contains(&humidity) {
        moisture = MOISTURE_MAP[(humidity - 1) as usize];
    }

    let m = Message::new("Bresser-6in1")
        .hex("id", id as u64, 8)
        .int("channel", chan)
        .cond(!rain_ok, |m| m.int("battery_ok", battery))
        .cond(temp_ok, |m| m.float("temperature_C", temp_c as f64, 1))
        .cond(temp_ok && moisture < 0, |m| m.int("humidity", humidity))
        .int("sensor_type", s_type)
        .cond(moisture >= 0, |m| m.int("moisture", moisture))
        .cond(wind_ok, |m| m.float("wind_max_m_s", wind_gust as f64, 1))
        .cond(wind_ok, |m| m.float("wind_avg_m_s", wind_avg as f64, 1))
        .cond(wind_ok, |m| m.int("wind_dir_deg", wind_dir))
        .cond(rain_ok, |m| m.float("rain_mm", rain_mm as f64, 1))
        .cond(uv_ok, |m| m.float("uvi", uvi as f64, 1))
        .cond(startup != 0, |m| m.int("startup", startup))
        .int("flags", flags)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Bresser Weather Center 6-in-1, 7-in-1 indoor, soil, new 5-in-1, 3-in-1 wind gauge, Froggit WH6000, Ventus C8488A",
    group: Group::Weather,
    modulation: Modulation::FskPcm,
    short_width: 124.0,
    long_width: 124.0,
    reset_limit: 25000.0,
    decode: bresser_6in1_decode,
    ..Device::DEFAULT
}];

/// From rtl_433's comment: a soil moisture sensor, battery ok.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Bresser-6in1",
    code: "{176}aaaa2dd4e3ae1870079341ffffff0000221201fff279",
    expect: &[
        ("id", "0x18700793"),
        ("channel", "1"),
        ("battery_ok", "1"),
        ("temperature_C", "22.1"),
        ("sensor_type", "4"),
        ("moisture", "0"),
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
        // Weather station: 11.8 °C, 81 %, gust 5.7 m/s, average 4.0 m/s
        // (rtl_433 example, the preamble added back).
        let m = decode_rows(
            &DEVICES[0],
            "aaaa2dd45eaa188002c318fa8ffb2768118481fff07200",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("humidity").unwrap().to_string(), "81");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "11.8");
        assert_eq!(m[0].get("wind_avg_m_s").unwrap().to_string(), "4.0");
        assert_eq!(m[0].get("wind_max_m_s").unwrap().to_string(), "5.7");
        // Same station: 3.0 °C, 95 %, no wind.
        let m = decode_rows(
            &DEVICES[0],
            "{205}55555555545ba999263100058631fffffe66d006092bffe0cff8",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "3.0");
        assert_eq!(m[0].get("humidity").unwrap().to_string(), "95");
        assert_eq!(m[0].get("wind_max_m_s").unwrap().to_string(), "0.0");
        // Battery low.
        let m = decode_rows(
            &DEVICES[0],
            "{176}aaaa2dd43d2c1870079341ffffff0000219001fff2fc",
        );
        assert_eq!(m[0].get("battery_ok").unwrap().to_string(), "0");
    }
}
