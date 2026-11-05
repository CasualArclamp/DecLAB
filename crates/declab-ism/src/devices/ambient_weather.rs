// made by CasualArclamp
//! Ambient Weather F007TH and F012TH thermo-hygrometers, TFA 30.3208.02
//! (for the TFA "Klima-Monitor" 30.3054), SwitchDoc Labs F016TH and
//! Suomen Lämpömittari 7411.
//!
//! Ported from rtl_433's `src/devices/ambient_weather.c` (contributed by
//! David Ediger, discovered by Ron C. Lewis; GPL-2.0-or-later).
//!
//! Manchester, 500 µs half-bits, three repeats without a gap, each after
//! the preamble `0x00145`. Six bytes from the preamble's last byte:
//! `xxxxMMMM IIIIIIII BCCCTTTT TTTTTTTT HHHHHHHH DDDDDDDD`: model 5, id,
//! battery low, channel − 1, temperature in 0.1 °F + 400, humidity, and an
//! LFSR digest-8 (gen 0x98, key 0x3e) xor 0x64.

use crate::bits::{BitBuffer, lfsr_digest8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut b = [0u8; 6];
    bb.extract_bytes(row, bitpos, &mut b, 6 * 8);
    let expected = b[5];
    let calculated = lfsr_digest8(&b[..5], 0x98, 0x3e) ^ 0x64;
    if expected != calculated {
        ctx.log(|| format!("ambient_weather: checksum error, expected {expected:02x} calculated {calculated:02x}"));
        return DECODE_FAIL_MIC;
    }
    let device_id = b[1];
    let battery_low = b[2] & 0x80 != 0;
    let channel = ((b[2] & 0x70) >> 4) + 1;
    let temp_raw = (((b[2] & 0x0f) as i32) << 8) | b[3] as i32;
    // rtl_433 works in a C float: f32 here too, so the rounding matches.
    let temp_f = (temp_raw - 400) as f32 * 0.1;
    let humidity = b[4];
    // Bad packets often pass the digest: humidity over 100 (such as 255)
    // and 344 °F or more (raw 0xf00..0xfff) are rejected.
    if humidity > 100 {
        ctx.log(|| format!("ambient_weather: humidity failed sanity check 0x{humidity:02x}"));
        return DECODE_FAIL_SANITY;
    }
    if !(-40.0..344.0).contains(&temp_f) {
        ctx.log(|| format!("ambient_weather: temperature failed sanity check 0x{temp_raw:03x}"));
        return DECODE_FAIL_SANITY;
    }
    let m = Message::new("Ambientweather-F007TH")
        .int("id", device_id)
        .int("channel", channel)
        .int("battery_ok", !battery_low as u8)
        .float("temperature_F", temp_f as f64, 1)
        .int("humidity", humidity)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn ambient_weather_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // The full preamble is 0x00145 (the last bits may vary, 0x00146), and
    // 0xffd45 when the receiver locked on the other phase.
    const PREAMBLE: [u8; 2] = [0x01, 0x45]; // 12 bits
    const PREAMBLE_INVERTED: [u8; 2] = [0xfd, 0x45]; // 12 bits
    let mut ret = 0;
    for row in 0..bb.num_rows {
        let len = bb.bits(row);
        // A preamble with enough bits after it to hold a whole packet.
        let mut bitpos = 0;
        loop {
            bitpos = bb.search(row, bitpos, &PREAMBLE, 12);
            if bitpos + 8 + 6 * 8 > len {
                break;
            }
            ret = decode_at(ctx, bb, row, bitpos + 8);
            if ret > 0 {
                return ret; // the first good message is enough
            }
            bitpos += 16;
        }
        bitpos = 0;
        loop {
            bitpos = bb.search(row, bitpos, &PREAMBLE_INVERTED, 12);
            if bitpos + 8 + 6 * 8 > len {
                break;
            }
            ret = decode_at(ctx, bb, row, bitpos + 8);
            if ret > 0 {
                return ret;
            }
            bitpos += 15;
        }
    }
    ret
}

pub static DEVICES: &[Device] = &[Device {
    name: "Ambient Weather F007TH, TFA 30.3208.02, SwitchDocLabs F016TH temperature sensor",
    group: Group::Weather,
    modulation: Modulation::OokManchesterZeroBit,
    short_width: 500.0,
    long_width: 0.0,
    reset_limit: 2400.0,
    decode: ambient_weather_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: id 0x7e, channel 3, battery ok, 72.5 °F,
/// 45 % (preamble 0x0145, then 0x7e, 0x24, 0x65, 0x2d, digest 0xfc). The
/// row starts with the slicer's hard-coded 0.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Ambientweather-F007TH",
    code: "{56}01457e24652dfc",
    expect: &[
        ("id", "126"),
        ("channel", "3"),
        ("battery_ok", "1"),
        ("temperature_F", "72.5"),
        ("humidity", "45"),
        ("mic", "CRC"),
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
    fn bad_digest_fails() {
        assert!(decode_rows(&DEVICES[0], "{56}01457e24652dfd").is_empty());
    }

    #[test]
    fn three_repeats_in_one_row() {
        // Three packets back to back, as the sensor sends them.
        let m = decode_rows(
            &DEVICES[0],
            "{168}01457e24652dfc01457e24652dfc01457e24652dfc",
        );
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("temperature_F").unwrap().to_string(), "72.5");
    }
}
