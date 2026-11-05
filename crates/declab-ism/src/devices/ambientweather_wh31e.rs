// made by CasualArclamp
//! Ambient Weather WH31E / WH31B thermo-hygrometers (and their radio
//! clock packets), EcoWitt WH40 and WN20 rain gauges, EcoWitt WS68
//! anemometer.
//!
//! Ported from rtl_433's `src/devices/ambientweather_wh31e.c`
//! (GPL-2.0-or-later).
//!
//! 915/868/433 MHz FSK PCM, 56 µs bits: a warm-up, a preamble of
//! 0xaa…, sync word `2d d4`, then a type byte that sets the layout. Every
//! layout ends in a CRC-8 (poly 0x31, init 0) and an 8-bit add-sum.
//! WH31E (0x30) / WH31B (0x37): `YY II CT TT HH XX AA`, C = `1CCC0B..`,
//! temperature 10 bits, 0.1 °C + 40 °C offset. 0x52: radio clock (BCD
//! date and time). 0x40: WH40 rain. 0x20: WN20 rain. 0x68: WS68 wind,
//! light, UV.

use crate::bits::{BitBuffer, add_bytes, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn ambientweather_whx_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // (Partial) preamble and sync word.
    const PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
    const WH31E_TYPE_CODE: u8 = 0x30;
    const WH31B_TYPE_CODE: u8 = 0x37;
    let mut events = 0;

    for row in 0..bb.num_rows {
        let start_pos = bb.search(row, 0, &PREAMBLE, 24);
        if start_pos == bb.bits(row) {
            continue; // DECODE_ABORT_EARLY
        }
        ctx.log(|| {
            format!(
                "WH31E/WH31B/WH40/WN20 detected, buffer is {} bits",
                bb.bits(row)
            )
        });

        // 18 bytes: only 6/9/17.5 are used, the rest is shown as extra data.
        let mut b = [0u8; 18];
        bb.extract_bytes(row, start_pos + 24, &mut b, 18 * 8);
        let msg_type = b[0];

        // The add-sum of the first `n` bytes must equal byte `n` (as u8).
        let sum_ok = |n: usize| (add_bytes(&b[..n]) - b[n] as i32) & 0xff == 0;

        if msg_type == WH31E_TYPE_CODE || msg_type == WH31B_TYPE_CODE {
            if crc8(&b[..6], 0x31, 0x00) != 0 {
                ctx.log(|| format!("WH31E/WH31B ({msg_type}) bad CRC"));
                continue; // DECODE_FAIL_MIC
            }
            if !sum_ok(6) {
                ctx.log(|| format!("WH31E/WH31B ({msg_type}) bad SUM"));
                continue;
            }
            let id = b[1];
            let batt_low = (b[2] & 0x04) >> 2;
            let channel = ((b[2] & 0x70) >> 4) + 1;
            let temp_raw = (((b[2] & 0x03) as i32) << 8) | b[3] as i32;
            let temp_c = (temp_raw - 400) as f32 * 0.1;
            let humidity = b[4];
            let extra = format!(
                "{:02x}{:02x}{:02x}{:02x}{:02x}",
                b[6], b[7], b[8], b[9], b[10]
            );
            let model = if msg_type == 0x30 {
                "AmbientWeather-WH31E"
            } else {
                "AmbientWeather-WH31B"
            };
            let m = Message::new(model)
                .int("id", id)
                .int("channel", channel)
                .int("battery_ok", (batt_low == 0) as u8)
                .float("temperature_C", temp_c as f64, 1)
                .int("humidity", humidity)
                .str("data", extra)
                .str("mic", "CRC");
            ctx.output(m);
            events += 1;
        } else if msg_type == 0x52 {
            // WH31E radio clock (WWVB) packet.
            if crc8(&b[..10], 0x31, 0x00) != 0 {
                ctx.log(|| "WH31E RCC bad CRC".into());
                continue;
            }
            if !sum_ok(10) {
                ctx.log(|| "WH31E RCC bad SUM".into());
                continue;
            }
            let id = b[1];
            let unknown = b[2];
            let year = ((b[3] & 0xf0) >> 4) as i32 * 10 + (b[3] & 0x0f) as i32 + 2000;
            let month = ((b[4] & 0x10) >> 4) * 10 + (b[4] & 0x0f);
            let day = ((b[5] & 0x30) >> 4) * 10 + (b[5] & 0x0f);
            let hours = ((b[6] & 0x30) >> 4) * 10 + (b[6] & 0x0f);
            let minutes = ((b[7] & 0x70) >> 4) * 10 + (b[7] & 0x0f);
            let seconds = ((b[8] & 0x70) >> 4) * 10 + (b[8] & 0x0f);
            let clock =
                format!("{year:04}-{month:02}-{day:02}T{hours:02}:{minutes:02}:{seconds:02}Z");
            let m = Message::new("AmbientWeather-WH31E")
                .int("id", id)
                .int("data", unknown)
                .str("radio_clock", clock)
                .str("mic", "CRC");
            ctx.output(m);
            events += 1;
        } else if msg_type == 0x40 {
            // WH40.
            if crc8(&b[..8], 0x31, 0x00) != 0 {
                ctx.log(|| "WH40 bad CRC".into());
                continue;
            }
            if !sum_ok(8) {
                ctx.log(|| "WH40 bad SUM".into());
                continue;
            }
            let id = (((b[1] & 0x0f) as u64) << 16) | ((b[2] as u64) << 8) | b[3] as u64;
            let battery_v = (b[4] & 0x1f) as i32;
            // 0.9 V – 1.5 V is 0 – 100 %.
            let battery_lvl = if battery_v <= 9 {
                0
            } else {
                (100 * (battery_v - 9) / 6).min(100)
            };
            let rain_raw = ((b[5] as i32) << 8) | b[6] as i32;
            let extra = format!(
                "{:02x}{:02x}{:02x}{:02x}{:02x}",
                b[9], b[10], b[11], b[12], b[13]
            );
            let m = Message::new("EcoWitt-WH40")
                .hex("id", id, 5)
                // rtl_433 prints these with "%f" and the default "%.3f".
                .cond(battery_v != 0, |m| {
                    m.float("battery_V", (battery_v as f32 * 0.1) as f64, 6)
                })
                .cond(battery_v != 0, |m| {
                    m.float("battery_ok", (battery_lvl as f32 * 0.01) as f64, 3)
                })
                .float("rain_mm", rain_raw as f64 * 0.1, 1)
                .str("data", extra)
                .str("mic", "CRC");
            ctx.output(m);
            events += 1;
        } else if msg_type == 0x20 {
            // WN20: TYPE PAD ID ID FV RAIN RAIN UNK CRC SUM.
            if crc8(&b[..9], 0x31, 0x00) != 0 {
                ctx.log(|| "WN20 bad CRC".into());
                continue;
            }
            if !sum_ok(9) {
                ctx.log(|| "WN20 bad SUM".into());
                continue;
            }
            let id = ((b[2] as i32) << 8) | b[3] as i32;
            let battery_raw = b[4] as i32;
            let battery_v = battery_raw as f32 * 0.02; // 2xAAA, nominal ~3.0 V
            // 1.8 V – 3.0 V is 0 – 100 %.
            let battery_lvl = if battery_raw <= 90 {
                0
            } else {
                (100 * (battery_raw - 90) / 60).min(100)
            };
            let rain_raw = ((b[5] as i32) << 8) | b[6] as i32;
            // Bytes 10..14 lie past the WN20's own 10 bytes.
            let extra = format!(
                "{:02x}{:02x}{:02x}{:02x}{:02x}",
                b[10], b[11], b[12], b[13], b[14]
            );
            let m = Message::new("EcoWitt-WN20")
                .int("id", id)
                .float("battery_V", battery_v as f64, 2)
                .int("battery_ok", (battery_lvl > 0) as u8)
                .int("battery_pct", battery_lvl)
                .float("rain_mm", rain_raw as f64 * 0.1, 1)
                .str("data", extra)
                .str("mic", "CRC");
            ctx.output(m);
            events += 1;
        } else if msg_type == 0x68 {
            // WS68.
            if crc8(&b[..15], 0x31, 0x00) != 0 {
                ctx.log(|| "WS68 bad CRC".into());
                continue;
            }
            if !sum_ok(15) {
                ctx.log(|| "WS68 bad SUM".into());
                continue;
            }
            let id = ((b[2] as i32) << 8) | b[3] as i32;
            let lux_raw = ((b[4] as i32) << 8) | b[5] as i32;
            let light_lux = lux_raw * 10;
            let batt = b[6];
            let batt_ok = batt > 0x20; // rtl_433: "wild guess"
            let wspeed = (((b[7] & 0x10) as i32) << 4) | b[10] as i32;
            let wdir = (((b[7] & 0x20) as i32) << 3) | b[11] as i32;
            let wgust = (((b[7] & 0x40) as i32) << 2) | b[12] as i32;
            // C's `(int)b[13] * 0.1f` truncated to int.
            let uvindex = (b[13] as f32 * 0.1) as i32;
            let extra = format!("{:02x}{:01x}", b[16], b[17] >> 4);
            let m = Message::new("EcoWitt-WS68")
                .int("id", id)
                .int("battery_raw", batt)
                .int("battery_ok", batt_ok as u8)
                .int("light_lux", light_lux)
                .float("wind_avg_m_s", (wspeed as f32 * 0.1) as f64, 1)
                .float("wind_max_m_s", (wgust as f32 * 0.1) as f64, 1)
                .float("uvi", uvindex as f64, 0)
                .int("wind_dir_deg", wdir)
                .str("data", extra)
                .str("mic", "CRC");
            ctx.output(m);
            events += 1;
        } else {
            ctx.log(|| {
                format!("unknown message type {msg_type:02x} (expected 0x20/0x30/0x40/0x68)")
            });
        }
    }
    events
}

pub static DEVICES: &[Device] = &[Device {
    name: "Ambient Weather WH31E Thermo-Hygrometer Sensor, EcoWitt WH40/WN20 rain gauge, WS68 weather station",
    group: Group::Weather,
    modulation: Modulation::FskPcm,
    short_width: 56.0,
    long_width: 56.0,
    reset_limit: 1500.0,
    gap_limit: 1800.0,
    decode: ambientweather_whx_decode,
    ..Device::DEFAULT
}];

/// All from rtl_433's comments, behind a 6-byte preamble and `2d d4`.
pub static SAMPLES: &[Sample] = &[
    Sample {
        dev: 0,
        model: "AmbientWeather-WH31E",
        code: "{177}aaaaaaaaaaaa2dd430c3820a5edfbc0756a7ae00000000",
        expect: &[
            ("id", "195"),
            ("channel", "1"),
            ("battery_ok", "1"),
            ("temperature_C", "12.2"),
            ("humidity", "94"),
            ("data", "bc0756a7ae"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 0,
        model: "EcoWitt-WH40",
        code: "{176}aaaaaaaaaaaa2dd44000cd6f10000064f000027b0000",
        expect: &[
            ("id", "0x0cd6f"),
            ("battery_V", "1.600000"),
            ("battery_ok", "1.000"),
            ("rain_mm", "0.0"),
            ("data", "00027b0000"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 0,
        model: "EcoWitt-WN20",
        code: "{144}aaaaaaaaaaaa2dd420002c87980013049e20",
        expect: &[
            ("id", "11399"),
            ("battery_V", "3.04"),
            ("battery_ok", "1"),
            ("battery_pct", "100"),
            ("rain_mm", "1.9"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 0,
        model: "EcoWitt-WS68",
        code: "{204}aaaaaaaaaaaa2dd4680000c500004b0fffff005a0000d0af1040",
        expect: &[
            ("id", "197"),
            ("battery_raw", "75"),
            ("battery_ok", "1"),
            ("light_lux", "0"),
            ("wind_avg_m_s", "0.0"),
            ("wind_dir_deg", "90"),
            ("uvi", "0"),
            ("data", "104"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 0,
        model: "AmbientWeather-WH31E",
        code: "{152}aaaaaaaaaaaa2dd452274a2010200206550575",
        expect: &[
            ("id", "39"),
            ("data", "74"),
            ("radio_clock", "2020-10-20T02:06:55Z"),
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
        // The second example packet: channel 2, 13.8 °C (0x21a = 538, less 400), 57 %.
        let m = decode_rows(
            &DEVICES[0],
            "{178}aaaaaaaaaaaa2dd43044921a395ab3074504 5f00000000",
        );
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("channel").unwrap().to_string(), "2");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "13.8");
        assert_eq!(m[0].get("humidity").unwrap().to_string(), "57");
        // A WH40 bucket tip: rain counter 1 = 0.1 mm.
        let m = decode_rows(
            &DEVICES[0],
            "{176}aaaaaaaaaaaa2dd44000cd6f10000155e2 0002f60000",
        );
        assert_eq!(m[0].get("rain_mm").unwrap().to_string(), "0.1");
        // A changed bit fails the CRC.
        let m = decode_rows(
            &DEVICES[0],
            "{177}aaaaaaaaaaaa2dd430c3820b5edfbc0756a7ae00000000",
        );
        assert!(m.is_empty());
    }
}
