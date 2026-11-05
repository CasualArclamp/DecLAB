// made by CasualArclamp
//! Nexus temperature/humidity sensors (also FreeTec NC-7345, infactory
//! NX-3980, Solight TE82S, TFA 30.3209) and the Nexus sauna sensor.
//!
//! Ported from rtl_433's `src/devices/nexus.c` (GPL-2.0-or-later).
//!
//! 36 bits sent 12 times, PPM: a ~500 µs pulse, then a ~1000 µs gap for 0
//! or ~2000 µs for 1; ~4000 µs between rows. Nine nibbles:
//! `[id0] [id1] [flags] [temp0] [temp1] [temp2] [const] [humi0] [humi1]`,
//! flags `B T C C` (battery ok, test, channel − 1), temperature 12-bit
//! signed in 0.1 °C, const 1111.

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn nexus_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let Some(r) = bb.find_repeated_row(3, 36) else {
        return DECODE_ABORT_EARLY;
    };
    let b = bb.row(r);
    // 36 bits, maybe a trailing 0.
    if bb.bits(r) > 37 {
        return DECODE_ABORT_LENGTH;
    }
    if b[3] & 0xf0 != 0xf0 {
        return DECODE_ABORT_EARLY;
    }
    if (b[0] == 0 && b[2] == 0 && b[3] == 0) || (b[0] == 0xff && b[2] == 0xff && b[3] == 0xff) {
        return DECODE_ABORT_EARLY;
    }
    if b[1] & 0x30 == 0x30 {
        return DECODE_ABORT_EARLY;
    }
    // Rubicson/Solight TE44 share the layout but end in a CRC-8 (0x31,
    // init 0x6c): a match means it is one of theirs.
    let crc_in = [
        b[0],
        b[1],
        b[2],
        b[3] & 0xf0,
        ((b[3] & 0x0f) << 4) | (b[4] >> 4),
    ];
    if crc8(&crc_in, 0x31, 0x6c) == 0 {
        ctx.log(|| "nexus: matches Solight-TE44/Rubicson CRC, rejecting".into());
        return DECODE_FAIL_SANITY;
    }
    let id = b[0];
    let battery = b[1] & 0x80 != 0;
    let test = b[1] & 0x40 != 0;
    let channel = ((b[1] & 0x30) >> 4) + 1;
    // 12-bit signed, sign-extended through an i16.
    let temp_raw = (((b[1] as u16) << 12) | ((b[2] as u16) << 4)) as i16;
    let temp_c = (temp_raw >> 4) as f64 * 0.1;
    let humidity = ((b[3] & 0x0f) << 4) | (b[4] >> 4);
    if humidity != 0 && humidity > 100 {
        return DECODE_FAIL_SANITY;
    }
    let m = Message::new(if humidity == 0 { "Nexus-T" } else { "Nexus-TH" })
        .int("id", id)
        .int("channel", channel)
        .int("battery_ok", battery as u8)
        .float("temperature_C", temp_c, 1)
        .cond(humidity != 0, |m| m.int("humidity", humidity))
        .cond(test, |m| m.int("test", 1));
    ctx.output(m);
    1
}

fn sauna_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let Some(r) = bb.find_repeated_row(3, 36) else {
        return DECODE_ABORT_EARLY;
    };
    let b = bb.row(r);
    if bb.bits(r) > 37 {
        return DECODE_ABORT_LENGTH;
    }
    if b[1] & 0xf != 0xf {
        return DECODE_ABORT_EARLY;
    }
    if b[0] == 0 || b[4] & 0x10 != 0x10 || (b[0] == 0xff && b[2] == 0xff && b[3] == 0xff) {
        return DECODE_ABORT_EARLY;
    }
    if b[1] & 0x30 != 0x30 {
        return DECODE_ABORT_EARLY;
    }
    let temp_raw = (((b[2] as u16) << 8) | b[3] as u16) as i16;
    let m = Message::new("Nexus-Sauna")
        .int("id", b[0])
        .int("channel", ((b[1] & 0x30) >> 4) + 1)
        .int("battery_ok", (b[1] & 0x80 != 0) as u8)
        .float("temperature_C", temp_raw as f64 * 0.1, 1)
        .cond(b[1] & 0x40 != 0, |m| m.int("test", 1));
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "Nexus, FreeTec NC-7345, NX-3980, Solight TE82S, TFA 30.3209 temperature/humidity sensor",
        group: Group::Weather,
        modulation: Modulation::OokPpm,
        short_width: 1000.0,
        long_width: 2000.0,
        gap_limit: 3000.0,
        reset_limit: 5000.0,
        decode: nexus_decode,
        priority: 10,
        ..Device::DEFAULT
    },
    Device {
        name: "Nexus, CRX, Prego sauna temperature sensor",
        group: Group::Weather,
        modulation: Modulation::OokPpm,
        short_width: 1000.0,
        long_width: 2000.0,
        gap_limit: 3000.0,
        reset_limit: 5000.0,
        decode: sauna_decode,
        priority: 10,
        ..Device::DEFAULT
    },
];

/// Made up for the simulator: id 0x5A, channel 2, battery ok, 18.4 °C,
/// 57 % (0x5a, flags 1001, 0x0b8, f, 0x39), sent 4 times.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Nexus-TH",
    code: "{36}5a90b8f390 {36}5a90b8f390 {36}5a90b8f390 {36}5a90b8f390",
    expect: &[
        ("id", "90"),
        ("channel", "2"),
        ("temperature_C", "18.4"),
        ("humidity", "57"),
        ("battery_ok", "1"),
    ],
    air: Air {
        pulse_us: 500.0,
        row_gap_us: 4000.0,
        ..Air::DEFAULT
    },
}];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_decode() {
        crate::devices::check_samples(DEVICES, SAMPLES);
    }

    #[test]
    fn rtl_433_example() {
        // A Nexus-TH row in rtl_433's tests style: id 0x5a? build one and
        // check the fields' arithmetic: -2.0 °C is 0xfec.
        let m = decode_rows(&DEVICES[0], "{36}5a0fecf460 {36}5a0fecf460 {36}5a0fecf460");
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "-2.0");
        assert_eq!(m[0].get("humidity").unwrap().to_string(), "70");
        assert_eq!(m[0].get("battery_ok").unwrap().to_string(), "0");
    }
}
