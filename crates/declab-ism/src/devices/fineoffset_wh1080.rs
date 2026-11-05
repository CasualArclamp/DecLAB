// made by CasualArclamp
//! Fine Offset WH1080 / WH3080 weather stations (Watson W-8681, Digitech
//! XC0348, PCE-FWS 20, Elecsa AstroTouch 6975, Froggit WH1080, …): weather,
//! DCF77/WWVB time and (WH3080) UV/light messages, OOK and FSK variants.
//!
//! Ported from rtl_433's `src/devices/fineoffset_wh1080.c`
//! (GPL-2.0-or-later; 2016-2017 Nicola Quiriti).
//!
//! OOK PWM (544 µs 1, 1524 µs 0, fixed 1036 µs gap) or FSK PCM (58 µs,
//! preamble `aa 2d d4`). Weather/time messages are 0xff then 10 bytes
//! (`ff FI IT TT HH SS GG ?R RR BD CC`), UV/light 0xff then 7 bytes; a
//! CRC-8 (poly 0x31, init 0xff, over the 0xff too) ends both. Newer OOK
//! units send only 7 preamble bits (87 or 63 bits). The message format
//! nibble F is 0xa weather, 0xb time, 0x7 UV/light.

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

const WIND_DIR_DEGR: [i32; 16] = [
    0, 23, 45, 68, 90, 113, 135, 158, 180, 203, 225, 248, 270, 293, 315, 338,
];

/// 8 preamble bits (EPB) or 7 (SPB).
const EPB: i32 = 8;
const SPB: i32 = 7;

#[derive(PartialEq, Clone, Copy)]
enum Kind {
    Ook,
    Fsk,
}

fn fineoffset_wh1080_callback(ctx: &mut Ctx, bb: &mut BitBuffer, kind: Kind) -> i32 {
    const FSK_PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
    // Our own copy of the message (rtl_433 points into the row or a
    // shifted copy; a copy keeps the borrow checker out of it).
    let mut br = [0u8; 11];
    let preamble;
    let mut sens_msg = 10; // 10 weather/time, 7 UV/light

    if bb.num_rows != 1 {
        return DECODE_ABORT_EARLY;
    }
    let bits = bb.bits(0);

    if kind == Kind::Fsk {
        let bit_offset = bb.search(0, 0, &FSK_PREAMBLE, 24) + 24;
        if bit_offset + br.len() * 8 > bits {
            ctx.log(|| format!("wh1080: short package, header index {bit_offset}"));
            return DECODE_ABORT_LENGTH;
        }
        bb.extract_bytes(0, bit_offset - 8, &mut br, 88);
        br[0] = 0xff; // emulate the OOK payload
        preamble = EPB;
    } else if (88..100).contains(&bits) {
        preamble = EPB;
        br.copy_from_slice(&bb.row(0)[..11]);
    } else if bits == 87 {
        // 7 preamble bits: shift the bytes into place.
        preamble = SPB;
        bb.extract_bytes(0, 7, &mut br[1..], 80);
        br[0] = (bb.row(0)[0] >> 1) | 0x80;
    } else if bits == 64 {
        preamble = EPB;
        sens_msg = 7;
        br.copy_from_slice(&bb.row(0)[..11]);
    } else if bits == 63 {
        preamble = SPB;
        sens_msg = 7;
        bb.extract_bytes(0, 7, &mut br[1..8], 56);
        br[0] = (bb.row(0)[0] >> 1) | 0x80;
    } else {
        return DECODE_ABORT_LENGTH;
    }

    if br[0] != 0xff {
        return DECODE_FAIL_SANITY; // preamble missing
    }
    let crc_len = if sens_msg == 10 { 11 } else { 8 };
    if crc8(&br[..crc_len], 0x31, 0xff) != 0 {
        return DECODE_FAIL_MIC;
    }

    let msg_type = match br[1] >> 4 {
        0x0a => 0, // weather
        0x0b => 1, // date/time
        0x07 => 2, // WH3080 UV/light
        // 0x03 is WH0530 / Alecto WS-1200, 0x05 Alecto WS-1200 DCF77.
        _ => return DECODE_FAIL_SANITY,
    };

    // Weather.
    let temperature = if kind == Kind::Ook {
        // Only 10 bits, discard the top bits.
        let temp_raw = (((br[2] & 0x03) as i32) << 8) | br[3] as i32;
        (temp_raw - 400) as f32 * 0.1
    } else {
        let mut temp_raw = (((br[2] & 0x0f) as i32) << 8) | br[3] as i32;
        if temp_raw & 0x800 != 0 {
            temp_raw &= 0x7ff; // sign and magnitude
            temp_raw = -temp_raw;
        }
        temp_raw as f32 * 0.1
    };
    let humidity = br[4];
    let direction_deg = WIND_DIR_DEGR[(br[9] & 0x0f) as usize];
    let speed = (br[5] as f32 * 0.34) * 3.6; // m/s -> km/h
    let gust = (br[6] as f32 * 0.34) * 3.6;
    let rain_raw = (((br[7] & 0x0f) as i32) << 8) | br[8] as i32;
    let rain = rain_raw as f32 * 0.3;
    let device_id = ((br[1] << 4) & 0xf0) | (br[2] >> 4);
    let battery_low = (br[9] >> 4) == 1;

    // UV.
    let uv_sensor_id = device_id;
    let uv_status_ok = br[3] == 85;
    let uv_index = br[2] & 0x0f;

    // Light.
    let light = ((br[4] as i32) << 16) | ((br[5] as i32) << 8) | br[6] as i32;
    let lux = light as f64 * 0.1;
    let wm = if preamble == SPB {
        light as f32 / 1265.8
    } else {
        light as f32 / 6830.0
    };

    // Time.
    let signal_type = (br[2] & 0x0f) == 10;
    let signal_type_str = if signal_type { "DCF77" } else { "WWVB/MSF" };
    let bcd = |b: u8, hi_mask: u8| ((b & hi_mask) >> 4) as i32 * 10 + (b & 0x0f) as i32;
    let hours = bcd(br[3], 0x30);
    let minutes = bcd(br[4], 0xf0);
    let seconds = bcd(br[5], 0xf0);
    let year = bcd(br[6], 0xf0) + 2000;
    let month = bcd(br[7], 0x10);
    let day = bcd(br[8], 0xf0);

    let m = match msg_type {
        0 => Message::new("Fineoffset-WHx080")
            .int("subtype", msg_type)
            .int("id", device_id)
            .int("battery_ok", !battery_low as u8)
            .float("temperature_C", temperature as f64, 1)
            .int("humidity", humidity)
            .int("wind_dir_deg", direction_deg)
            .float("wind_avg_km_h", speed as f64, 2)
            .float("wind_max_km_h", gust as f64, 2)
            .float("rain_mm", rain as f64, 1)
            .str("mic", "CRC"),
        1 => Message::new("Fineoffset-WHx080")
            .int("subtype", msg_type)
            .int("id", device_id)
            .str("signal", signal_type_str)
            .str(
                "radio_clock",
                format!("{year:04}-{month:02}-{day:02}T{hours:02}:{minutes:02}:{seconds:02}"),
            )
            .str("mic", "CRC"),
        _ => Message::new("Fineoffset-WHx080")
            .int("subtype", msg_type)
            .int("uv_sensor_id", uv_sensor_id)
            .str("uv_status", if uv_status_ok { "OK" } else { "ERROR" })
            .int("uv_index", uv_index)
            .float("lux", lux, 1)
            .float("wm", wm as f64, 2)
            .str("mic", "CRC"),
    };
    ctx.output(m);
    1
}

fn fineoffset_wh1080_callback_ook(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    fineoffset_wh1080_callback(ctx, bb, Kind::Ook)
}

fn fineoffset_wh1080_callback_fsk(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    fineoffset_wh1080_callback(ctx, bb, Kind::Fsk)
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "Fine Offset Electronics WH1080/WH3080 Weather Station",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 544.0,
        long_width: 1524.0,
        reset_limit: 2800.0,
        decode: fineoffset_wh1080_callback_ook,
        ..Device::DEFAULT
    },
    Device {
        name: "Fine Offset Electronics WH1080/WH3080 Weather Station (FSK)",
        group: Group::Weather,
        modulation: Modulation::FskPcm,
        short_width: 58.0,
        long_width: 58.0,
        reset_limit: 5800.0,
        decode: fineoffset_wh1080_callback_fsk,
        ..Device::DEFAULT
    },
];

/// Made up for the simulator (CRCs computed): station 0x5b, 21.5 °C, 55 %,
/// wind 5 × 0.34 m/s from the east, gust 10 × 0.34 m/s, rain 42 × 0.3 mm.
pub static SAMPLES: &[Sample] = &[
    Sample {
        dev: 0,
        model: "Fineoffset-WHx080",
        code: "{88}ffa5b26737050a002a0448",
        expect: &[
            ("subtype", "0"),
            ("id", "91"),
            ("battery_ok", "1"),
            ("temperature_C", "21.5"),
            ("humidity", "55"),
            ("wind_dir_deg", "90"),
            ("wind_avg_km_h", "6.12"),
            ("wind_max_km_h", "12.24"),
            ("rain_mm", "12.6"),
        ],
        // The fixed gap after each pulse.
        air: Air {
            pulse_us: 1036.0,
            ..Air::DEFAULT
        },
    },
    Sample {
        dev: 1,
        model: "Fineoffset-WHx080",
        code: "{128}aaaaaa2dd4a5b0d737050a002a04c600",
        expect: &[
            ("id", "91"),
            ("temperature_C", "21.5"),
            ("humidity", "55"),
            ("wind_dir_deg", "90"),
            ("rain_mm", "12.6"),
        ],
        air: Air::DEFAULT,
    },
    // Time: DCF77, 2026-11-05 12:34:56.
    Sample {
        dev: 0,
        model: "Fineoffset-WHx080",
        code: "{88}ffb5ba1234562611050039",
        expect: &[
            ("subtype", "1"),
            ("signal", "DCF77"),
            ("radio_clock", "2026-11-05T12:34:56"),
        ],
        air: Air {
            pulse_us: 1036.0,
            ..Air::DEFAULT
        },
    },
    // WH3080 UV/light: UV 3, 12345.6 lux.
    Sample {
        dev: 0,
        model: "Fineoffset-WHx080",
        code: "{64}ff75b35501e24018",
        expect: &[
            ("subtype", "2"),
            ("uv_sensor_id", "91"),
            ("uv_status", "OK"),
            ("uv_index", "3"),
            ("lux", "12345.6"),
            ("wm", "18.08"),
        ],
        air: Air {
            pulse_us: 1036.0,
            ..Air::DEFAULT
        },
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
    fn seven_bit_preamble() {
        // The same weather message with only 7 preamble bits (87 bits).
        let m = decode_rows(&DEVICES[0], "{87}ff4b64ce6e0a1400540890");
        assert_eq!(m.len(), 1, "{m:?}");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "21.5");
        // A changed bit fails the CRC.
        assert!(decode_rows(&DEVICES[0], "{88}ffa5b26737050a002a0449").is_empty());
    }
}
