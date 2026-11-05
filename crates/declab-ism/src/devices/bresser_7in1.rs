// made by CasualArclamp
//! Bresser Weather Center 7-in-1 (also the 3-in-1 and 8-in-1 stations)
//! and the Bresser air quality sensors: PM2.5/PM10 (PN 7009970), CO2
//! (PN 7009977), HCHO/VOC (PN 7009978).
//!
//! Ported from rtl_433's `src/devices/bresser_7in1.c` (GPL-2.0-or-later).
//!
//! 868 MHz FSK PCM, 124 µs bits. Preamble `aa aa aa aa aa 2d d4`, then 25
//! bytes whitened with 0xaa (the sensor type / startup / channel byte is
//! read before de-whitening). The first two bytes are an LFSR-16 digest
//! (generator 0x8810, key 0xba95) of bytes 2..24, xored with 0x6df1.
//! Readings are BCD.

use crate::bits::{BitBuffer, lfsr_digest16};
use crate::device::*;
use crate::sim::{Air, Sample};

const SENSOR_TYPE_WEATHER: u8 = 1;
const SENSOR_TYPE_AIR_PM: u8 = 8;
const SENSOR_TYPE_CO2: u8 = 10;
const SENSOR_TYPE_HCHO_VOC: u8 = 11;
const SENSOR_TYPE_WEATHER3: u8 = 12;
const SENSOR_TYPE_WEATHER8: u8 = 13;

/// A nibble's value (BCD digit).
fn hi(b: u8) -> i32 {
    (b >> 4) as i32
}

fn lo(b: u8) -> i32 {
    (b & 0x0f) as i32
}

fn bresser_7in1_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 5] = [0xaa, 0xaa, 0xaa, 0x2d, 0xd4];
    let mut msg = [0u8; 25];

    if bb.num_rows != 1 || bb.bits(0) < 240 - 80 {
        ctx.log(|| format!("bresser_7in1: too few bits ({})", bb.bits(0)));
        return DECODE_ABORT_LENGTH;
    }
    let start_pos = bb.search(0, 0, &PREAMBLE, PREAMBLE.len() * 8) + PREAMBLE.len() * 8;
    if start_pos >= bb.bits(0) {
        ctx.log(|| "bresser_7in1: preamble not found".into());
        return DECODE_ABORT_EARLY;
    }
    if start_pos + 21 * 8 >= bb.bits(0) {
        ctx.log(|| {
            format!(
                "bresser_7in1: message too short ({})",
                bb.bits(0) - start_pos
            )
        });
        return DECODE_ABORT_LENGTH;
    }
    let n = msg.len() * 8;
    bb.extract_bytes(0, start_pos, &mut msg, n);

    if msg[21] == 0x00 {
        return DECODE_FAIL_SANITY;
    }

    let s_type = msg[6] >> 4;
    let nstartup = (msg[6] & 0x08) >> 3;
    let chan = (msg[6] & 0x07) as i32;

    // Data whitening.
    for b in msg.iter_mut() {
        *b ^= 0xaa;
    }

    let chk = ((msg[0] as u16) << 8) | msg[1] as u16;
    let digest = lfsr_digest16(&msg[2..25], 0x8810, 0xba95);
    if chk ^ digest != 0x6df1 {
        ctx.log(|| format!("bresser_7in1: digest check failed {chk:04x} vs {digest:04x}"));
        return DECODE_FAIL_MIC;
    }

    let id = ((msg[2] as i32) << 8) | msg[3] as i32;
    let flags = msg[15] & 0x0f;
    let battery_low = (flags & 0x06) == 0x06;
    let startup = (nstartup == 0) as i32;

    if s_type == SENSOR_TYPE_WEATHER
        || s_type == SENSOR_TYPE_WEATHER3
        || s_type == SENSOR_TYPE_WEATHER8
    {
        let wdir = hi(msg[4]) * 100 + lo(msg[4]) * 10 + hi(msg[5]);
        let wgst_raw = hi(msg[7]) * 100 + lo(msg[7]) * 10 + hi(msg[8]);
        let wavg_raw = lo(msg[8]) * 100 + hi(msg[9]) * 10 + lo(msg[9]);
        let rain_raw = hi(msg[10]) * 100000
            + lo(msg[10]) * 10000
            + hi(msg[11]) * 1000
            + lo(msg[11]) * 100
            + hi(msg[12]) * 10
            + lo(msg[12]);
        let rain_mm = rain_raw as f32 * 0.1;
        let temp_raw = hi(msg[14]) * 100 + lo(msg[14]) * 10 + hi(msg[15]);
        let mut temp_c = temp_raw as f32 * 0.1;
        if temp_raw > 600 {
            temp_c = (temp_raw - 1000) as f32 * 0.1;
        }
        let humidity = hi(msg[16]) * 10 + lo(msg[16]);
        let lght_raw = hi(msg[17]) * 100000
            + lo(msg[17]) * 10000
            + hi(msg[18]) * 1000
            + lo(msg[18]) * 100
            + hi(msg[19]) * 10
            + lo(msg[19]);
        let uv_raw = hi(msg[20]) * 100 + lo(msg[20]) * 10 + hi(msg[21]);

        let light_klx = lght_raw as f32 * 0.001;
        let light_lux = lght_raw as f32;
        let uv_index = uv_raw as f32 * 0.1;

        // The 3-in-1 (type 12) has temperature, humidity and rain only.
        let wind_light_ok = s_type != SENSOR_TYPE_WEATHER3;

        let mut tglobe_ok = false;
        let mut tglobe_c = 0.0f32;
        if s_type == SENSOR_TYPE_WEATHER8 && hi(msg[23]) < 10 {
            // Globe thermometer, only in the 8-in-1.
            tglobe_ok = true;
            tglobe_c = (hi(msg[22]) * 10 + lo(msg[22])) as f32 + hi(msg[23]) as f32 * 0.1;
        }

        let m = Message::new("Bresser-7in1")
            .int("id", id)
            .cond(startup != 0, |m| m.int("startup", startup))
            .float("temperature_C", temp_c as f64, 1)
            .int("humidity", humidity)
            .cond(wind_light_ok, |m| {
                m.float("wind_max_m_s", (wgst_raw as f32 * 0.1) as f64, 1)
            })
            .cond(wind_light_ok, |m| {
                m.float("wind_avg_m_s", (wavg_raw as f32 * 0.1) as f64, 1)
            })
            .cond(wind_light_ok, |m| m.int("wind_dir_deg", wdir))
            .float("rain_mm", rain_mm as f64, 1)
            .cond(wind_light_ok, |m| m.float("light_klx", light_klx as f64, 3))
            .cond(wind_light_ok, |m| m.float("light_lux", light_lux as f64, 3))
            .cond(wind_light_ok, |m| m.float("uvi", uv_index as f64, 1))
            .cond(tglobe_ok, |m| {
                m.float("temperature_1_C", tglobe_c as f64, 1)
            })
            .int("battery_ok", !battery_low as u8)
            .str("mic", "CRC");
        ctx.output(m);
        1
    } else if s_type == SENSOR_TYPE_AIR_PM {
        let pm_2_5 = lo(msg[10]) * 1000 + hi(msg[11]) * 100 + lo(msg[11]) * 10 + hi(msg[12]);
        let pm_10 = lo(msg[12]) * 1000 + hi(msg[13]) * 100 + lo(msg[13]) * 10 + hi(msg[14]);
        // All 0xf in the first packet after power-up.
        let pm_2_5_init = lo(msg[10]) == 0x0f;
        let pm_10_init = lo(msg[12]) == 0x0f;
        let m = Message::new("Bresser-7in1")
            .int("id", id)
            .int("channel", chan)
            .cond(startup != 0, |m| m.int("startup", startup))
            .int("battery_ok", !battery_low as u8)
            .cond(!pm_2_5_init, |m| m.int("pm2_5_ug_m3", pm_2_5))
            .cond(!pm_10_init, |m| m.int("pm10_0_ug_m3", pm_10))
            .str("mic", "CRC");
        ctx.output(m);
        1
    } else if s_type == SENSOR_TYPE_CO2 {
        let co2 = hi(msg[4]) * 1000 + lo(msg[4]) * 100 + hi(msg[5]) * 10 + lo(msg[5]);
        let co2_init = lo(msg[5]) == 0x0f;
        let m = Message::new("Bresser-CO2")
            .int("id", id)
            .int("channel", chan)
            .cond(startup != 0, |m| m.int("startup", startup))
            .int("battery_ok", !battery_low as u8)
            .cond(!co2_init, |m| m.int("co2_ppm", co2))
            .str("mic", "CRC");
        ctx.output(m);
        1
    } else if s_type == SENSOR_TYPE_HCHO_VOC {
        let hcho = hi(msg[4]) * 1000 + lo(msg[4]) * 100 + hi(msg[5]) * 10 + lo(msg[5]);
        let voc = lo(msg[22]);
        let hcho_init = lo(msg[5]) == 0x0f;
        let voc_init = voc == 0x0f;
        let m = Message::new("Bresser-HCHOVOC")
            .int("id", id)
            .int("channel", chan)
            .cond(startup != 0, |m| m.int("startup", startup))
            .int("battery_ok", !battery_low as u8)
            .cond(!hcho_init, |m| m.int("hcho_ppb", hcho))
            .cond(!voc_init, |m| m.int("voc_level", voc))
            .str("mic", "CRC");
        ctx.output(m);
        1
    } else {
        ctx.log(|| format!("bresser_7in1: s_type={s_type} not implemented"));
        DECODE_FAIL_SANITY
    }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Bresser Weather Center 7-in-1, Air Quality PM2.5/PM10 7009970, CO2 7009977, HCHO/VOC 7009978 sensors",
    group: Group::Weather,
    modulation: Modulation::FskPcm,
    short_width: 124.0,
    long_width: 124.0,
    reset_limit: 25000.0,
    decode: bresser_7in1_decode,
    ..Device::DEFAULT
}];

pub static SAMPLES: &[Sample] = &[
    // From rtl_433's comment: a 3-in-1 (temperature, humidity, rain only).
    Sample {
        dev: 0,
        model: "Bresser-7in1",
        code: "{256}aaaaaaaaaa2dd4a6b2bf4b55aac8555555aa3e8f3eabca3355555555555555aa00",
        expect: &[
            ("id", "5601"),
            ("temperature_C", "1.6"),
            ("humidity", "99"),
            ("rain_mm", "942.5"),
            ("battery_ok", "1"),
        ],
        air: Air::DEFAULT,
    },
    // Made up: a 7-in-1, id 0x1234, wind dir 270°, gust 4.5 m/s, average
    // 3.2 m/s, rain 123.4 mm, 21.5 °C, 55 %, 12345 lux, UV 3.5.
    Sample {
        dev: 0,
        model: "Bresser-7in1",
        code: "{256}aaaaaaaaaa2dd42c38b89e8daa19aefa98aab89eaa8bfaffab89efa9faaaaaaa00",
        expect: &[
            ("id", "4660"),
            ("temperature_C", "21.5"),
            ("humidity", "55"),
            ("wind_dir_deg", "270"),
            ("wind_max_m_s", "4.5"),
            ("wind_avg_m_s", "3.2"),
            ("rain_mm", "123.4"),
            ("light_lux", "12345.000"),
            ("light_klx", "12.345"),
            ("uvi", "3.5"),
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
    fn rtl_433_examples() {
        // From rtl_433's comment, the preamble added back: 0.08 klx.
        let m = decode_rows(
            &DEVICES[0],
            "aaaaaaaaaa2dd410b8b4a5a3ca10aaaaaaaaaaaaaa8bcacbaaaa2aaaaaaaaaaa0000000000000000",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("light_klx").unwrap().to_string(), "0.080");
        // "54.0 klx", UV 2.6.
        let m = decode_rows(
            &DEVICES[0],
            "aaaaaaaaaa2dd49a59b4a5a3da10aaaaaaaaaaaaaa8bdac8afea28a8caaaaaaa000000000000000000",
        );
        assert_eq!(m[0].get("light_klx").unwrap().to_string(), "54.082");
        assert_eq!(m[0].get("uvi").unwrap().to_string(), "2.6");
    }
}
