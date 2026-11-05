// made by CasualArclamp
//! LaCrosse "IT+" (instant transmission) sensors: LaCrosse/StarMeteo/Conrad
//! TX35DTH-IT and TFA Dostmann 30.3155 (temperature/humidity), TX29-IT and
//! TFA Dostmann 30.3159.IT (temperature), and the TX25U temperature/probe
//! sensor at 915 MHz.
//!
//! Ported from rtl_433's `src/devices/lacrosse_tx35.c` (GPL-2.0-or-later).
//!
//! FSK NRZ, 55 µs bits (TX29, TX25U) or 105 µs (TX35); otherwise the same
//! protocol, so two devices differing only in bit width. After a 1010…
//! preamble and the sync word 0x2dd4, five bytes:
//! `MMMM IIII | II N x TTTT | TTTT TTTT | B HHHHHHH | CCCCCCCC`: model 9,
//! a 6-bit id, new-battery flag, temperature in BCD (0.1 °C, + 40 °C),
//! weak battery, humidity (0x6a: no humidity sensor; 0x7d: the TX25U's
//! probe channel), and a CRC-8 (poly 0x31, init 0) over the first four.

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

/// The humidity value of a sensor without humidity.
const TX29_NOHUMIDSENSOR: u8 = 0x6a;
/// The humidity value that flags the TX25U's probe channel.
const TX25_PROBE_FLAG: u8 = 0x7d;

/// rtl_433's `lacrosse_it`: `tx29` picks the model name.
fn lacrosse_it(ctx: &mut Ctx, bb: &BitBuffer, tx29: bool) -> i32 {
    // 4 bits of preamble, sync word 2dd4, sensor model 9: 24 bits.
    const PREAMBLE: [u8; 3] = [0xa2, 0xdd, 0x49];
    let model = if tx29 {
        "LaCrosse-TX29IT"
    } else {
        "LaCrosse-TX35DTHIT"
    };
    let mut events = 0;
    for row in 0..bb.num_rows {
        let start = bb.search(row, 0, &PREAMBLE, 24);
        if start >= bb.bits(row) {
            continue; // no preamble in this row
        }
        // Skip the preamble and sync (20 bits) and keep five bytes from
        // the model nibble on.
        let mut b = [0u8; 5];
        bb.extract_bytes(row, start + 20, &mut b, 40);
        let c_crc = crc8(&b[..4], 0x31, 0x00);
        if b[4] != c_crc {
            ctx.log(|| {
                format!(
                    "lacrosse_tx35: bad CRC: calculated {c_crc:02x}, received {:02x}",
                    b[4]
                )
            });
            continue;
        }
        let mut sensor_id = ((b[0] & 0x0f) << 2) | (b[1] >> 6);
        // BCD tens, units and tenths, in a C float as rtl_433 has it.
        let temp_c =
            10.0 * (b[1] & 0x0f) as f32 + ((b[2] >> 4) & 0x0f) as f32 + 0.1 * (b[2] & 0x0f) as f32
                - 40.0;
        let new_batt = (b[1] >> 5) & 1;
        let battery_low = b[3] >> 7;
        let humidity = b[3] & 0x7f;
        let no_humidity = humidity == TX29_NOHUMIDSENSOR || humidity == TX25_PROBE_FLAG;
        if humidity == TX25_PROBE_FLAG {
            sensor_id += 0x40; // the probe channel gets its own id
        }
        let m = Message::new(model)
            .int("id", sensor_id)
            .int("battery_ok", (battery_low == 0) as u8)
            .int("newbattery", new_batt)
            .float("temperature_C", temp_c as f64, 1)
            .cond(!no_humidity, |m| m.int("humidity", humidity))
            .str("mic", "CRC");
        ctx.output(m);
        events += 1;
    }
    events
}

fn tx29_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    lacrosse_it(ctx, bb, true)
}

fn tx35_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    lacrosse_it(ctx, bb, false)
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "LaCrosse TX29IT, TFA Dostmann 30.3159.IT Temperature sensor",
        group: Group::Weather,
        modulation: Modulation::FskPcm,
        short_width: 55.0, // 58 µs for the TX34-IT
        long_width: 55.0,
        reset_limit: 4000.0,
        decode: tx29_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "LaCrosse TX35DTH-IT, TFA Dostmann 30.3155 Temperature/Humidity sensor",
        group: Group::Weather,
        modulation: Modulation::FskPcm,
        short_width: 105.0,
        long_width: 105.0,
        reset_limit: 4000.0,
        decode: tx35_decode,
        ..Device::DEFAULT
    },
];

pub static SAMPLES: &[Sample] = &[
    // rtl_433's documented TX29 example (aa 2dd4 9284486a ec) with a
    // longer preamble: id 10, 4.8 °C, no humidity sensor.
    Sample {
        dev: 0,
        model: "LaCrosse-TX29IT",
        code: "{80}aaaaaa2dd49284486aec",
        expect: &[
            ("id", "10"),
            ("battery_ok", "1"),
            ("newbattery", "0"),
            ("temperature_C", "4.8"),
            ("mic", "CRC"),
        ],
        air: Air::DEFAULT,
    },
    // Made up: id 43, 21.3 °C, 55 %, battery ok (9a c6 13 37, CRC 0x41).
    Sample {
        dev: 1,
        model: "LaCrosse-TX35DTHIT",
        code: "{72}aaaa2dd49ac6133741",
        expect: &[
            ("id", "43"),
            ("battery_ok", "1"),
            ("newbattery", "0"),
            ("temperature_C", "21.3"),
            ("humidity", "55"),
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
    fn rtl_433_example() {
        // The example from rtl_433's comment, exactly as given.
        let m = decode_rows(&DEVICES[0], "{64}aa2dd49284486aec");
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].model, "LaCrosse-TX29IT");
        assert_eq!(m[0].get("id").unwrap().to_string(), "10");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "4.8");
        assert!(m[0].get("humidity").is_none());
    }

    #[test]
    fn bad_crc_rejected() {
        assert!(decode_rows(&DEVICES[1], "{72}aaaa2dd49ac6133740").is_empty());
    }
}
