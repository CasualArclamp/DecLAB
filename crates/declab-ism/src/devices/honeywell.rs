// made by CasualArclamp
//! Honeywell (Ademco) 345 MHz door/window and security sensors (5811 and
//! kin), 2GIG DW10/DW11 door sensors and GB1 glass-break detectors,
//! Resolution Products RE208 wire-to-air repeater.
//!
//! Ported from rtl_433's `src/devices/honeywell.c` (GPL-2.0-or-later).
//!
//! Sliced as raw OOK PCM (half-bit ~136 µs); the decoder finds the
//! Manchester preamble itself and decodes. Decoded layout (64 bits):
//! `PP PP C IIIII EE SS SS`: preamble `ff fe` (raw `…55 55 56`), 4-bit
//! channel, 20-bit serial, 8-bit event (`CTRABHUU`: contact, tamper,
//! reed, alarm, battery low, heartbeat), CRC-16 (poly 0x8050 for 2GIG
//! channels 2, 4, 9, A, C; 0x8005 otherwise). DW11 sends 96 bits.

use crate::bits::{BitBuffer, crc16};
use crate::device::*;
use crate::sim::{Air, Sample};

fn honeywell_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // The decoded preamble 0xfffe is raw 0x555556: searched for at the raw
    // level, every match tried (the run of 01 can match one bit early)
    // and the right one picked by its CRC.
    const PREAMBLE: [u8; 3] = [0x55, 0x55, 0x56];

    let row = 0; // a single row only: fewer collisions
    if bb.num_rows != 1 || bb.bits(row) < 120 {
        return DECODE_ABORT_LENGTH;
    }
    let raw_len = bb.bits(row);
    let mut b = [0u8; 10];
    let mut channel = 0u8;
    let mut device_id = 0u32;
    let mut len = 0;
    let mut found = false;
    let mut raw_pos = 0;
    // One scratch buffer for the decoded bits, cleared each try (a
    // `BitBuffer` is several kB: keep it off the loop).
    let mut decoded = BitBuffer::new();
    loop {
        raw_pos = bb.search(row, raw_pos, &PREAMBLE, 24);
        if raw_pos + 24 >= raw_len {
            break;
        }
        decoded.clear();
        bb.manchester_decode(row, raw_pos + 24, &mut decoded, 96);
        raw_pos += 1; // try the next (maybe off-by-one) match too

        len = decoded.bits(0);
        if len < 48 {
            continue;
        }
        b.copy_from_slice(&decoded.row(0)[..10]);

        channel = b[0] >> 4;
        device_id = (((b[0] & 0x0f) as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
        let crc = ((b[4] as u16) << 8) | b[5] as u16;
        if device_id == 0 && crc == 0 {
            continue; // fewer collisions
        }
        let crc_calculated = if matches!(channel, 0x2 | 0x4 | 0x9 | 0xa | 0xc) {
            // 2GIG, and type 0xC tilt sensors
            crc16(&b[..4], 0x8050, 0)
        } else {
            crc16(&b[..4], 0x8005, 0)
        };
        if crc == crc_calculated {
            found = true;
            break;
        }
    }
    if !found {
        return DECODE_FAIL_MIC;
    }
    if len > 50 {
        // DW11
        ctx.log(|| {
            format!(
                "honeywell: {len} bits {:02x?}",
                &b[..len.min(80).div_ceil(8)]
            )
        });
    }

    let event = b[3];
    // NOTE (rtl_433): not sure these bits apply to every device type.
    let contact = (event & 0x80) >> 7;
    let tamper = (event & 0x40) >> 6;
    let reed = (event & 0x20) >> 5;
    let alarm = (event & 0x10) >> 4;
    let battery_low = (event & 0x08) >> 3;
    let heartbeat = (event & 0x04) >> 2;

    let m = Message::new("Honeywell-Security")
        .hex("id", device_id as u64, 5)
        .int("channel", channel)
        .hex("event", event as u64, 2)
        .str("state", if contact != 0 { "open" } else { "closed" })
        .int("contact_open", contact)
        .int("reed_open", reed)
        .int("alarm", alarm)
        .int("tamper", tamper)
        .int("battery_ok", (battery_low == 0) as u8)
        .int("heartbeat", heartbeat)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Honeywell Door/Window Sensor, 2Gig DW10/DW11, RE208 repeater",
    group: Group::Remote,
    modulation: Modulation::OokPcm,
    short_width: 136.0,
    long_width: 136.0,
    reset_limit: 408.0,
    decode: honeywell_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator, each the raw preamble `55555556` (its
/// leading 0 lost on the air) and 48 Manchester-coded bits.
pub static SAMPLES: &[Sample] = &[
    // Channel 8, serial 0x2c4a1, event 0x80 (open), CRC-16/0x8005 0x6265.
    Sample {
        dev: 0,
        model: "Honeywell-Security",
        code: "{130}aaaaaaacd54cb534cd52d5552d4d2d320",
        expect: &[
            ("id", "0x2c4a1"),
            ("channel", "8"),
            ("event", "0x80"),
            ("state", "open"),
            ("battery_ok", "1"),
            ("heartbeat", "0"),
        ],
        air: Air::DEFAULT,
    },
    // 2GIG channel 2, serial 0xa5b3c, event 0x04 (heartbeat), CRC/0x8050.
    Sample {
        dev: 0,
        model: "Honeywell-Security",
        code: "{129}aaaaaaad4ccd32cb4ab5553552b54b540",
        expect: &[
            ("id", "0xa5b3c"),
            ("channel", "2"),
            ("event", "0x04"),
            ("state", "closed"),
            ("heartbeat", "1"),
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
    fn bad_crc_fails() {
        // The first sample with its last CRC bit pair flipped.
        assert!(decode_rows(&DEVICES[0], "{130}aaaaaaacd54cb534cd52d5552d4d2d340").is_empty());
    }
}
