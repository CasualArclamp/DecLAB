// made by CasualArclamp
//! CurrentCost TX and CurrentCost EnviR current sensors (meters and pulse
//! counters).
//!
//! Ported from rtl_433's `src/devices/current_cost.c` (Copyright (C) 2015
//! Emmanuel Navarro, EnviR by Neil Cowburn; GPL-2.0-or-later).
//!
//! FSK NRZ at 250 µs a bit. The rows are inverted, then a preamble is
//! searched: the classic TX sends `cc cc cc ce 91 5d` (45 bits), the EnviR
//! `55 55 55 55 a4 57` (48 bits, its 2D D4 sync shifted by one). After it
//! come 64 Manchester-coded bits (01 = 1, 10 = 0): `0000 iiii iiii iiii`
//! (a meter, id 12 bits) then three power words `V www wwww wwww wwww`
//! (V: the reading is valid), or `0100 iiii iiii iiii` (a counter), an
//! unused byte, the sensor type and a 32-bit pulse count.

use crate::bits::BitBuffer;
use crate::device::*;
use crate::sim::{Air, Sample};

fn current_cost_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    bb.invert();

    const INIT_CLASSIC: [u8; 6] = [0xcc, 0xcc, 0xcc, 0xce, 0x91, 0x5d]; // 45 bits
    const INIT_ENVIR: [u8; 6] = [0x55, 0x55, 0x55, 0x55, 0xa4, 0x57]; // 48 bits

    let row_bits = bb.bits(0);
    let mut start = bb.search(0, 0, &INIT_ENVIR, 48);
    let is_envir;
    if start + 47 + 112 <= row_bits {
        is_envir = true;
        // The EnviR's data starts on the pattern's last bit (see the
        // module doc): back up one bit so the Manchester pairs line up.
        start += 47;
    } else {
        is_envir = false;
        start = bb.search(0, 0, &INIT_CLASSIC, 45);
        if start + 45 + 112 > row_bits {
            return DECODE_ABORT_EARLY;
        }
        start += 45;
    }

    let mut packet = BitBuffer::new();
    bb.manchester_decode(0, start, &mut packet, 0);
    if packet.bits(0) < 64 {
        return DECODE_ABORT_EARLY;
    }
    let b = packet.row(0);

    if b[0] & 0xf0 == 0 {
        // A meter: bits 5 and 4 of b[0] are unknown, always 0 so far.
        let id = ((b[0] as u16 & 0x0f) << 8) | b[1] as u16;
        // A power word counts only when its "data valid" bit is set.
        let watt = |hi: u8, lo: u8| -> u16 {
            if hi & 0x80 != 0 {
                ((hi as u16 & 0x7f) << 8) | lo as u16
            } else {
                0
            }
        };
        let m = Message::new(if is_envir {
            "CurrentCost-EnviR"
        } else {
            "CurrentCost-TX"
        })
        .int("id", id)
        .int("power0_W", watt(b[2], b[3]))
        .int("power1_W", watt(b[4], b[5]))
        .int("power2_W", watt(b[6], b[7]));
        ctx.output(m);
        return 1;
    } else if b[0] & 0xf0 == 0x40 {
        // A counter: b[2] is apparently unused; sensor type 2 electric,
        // 3 gas, 4 water.
        let id = ((b[0] as u16 & 0x0f) << 8) | b[1] as u16;
        let sensor_type = b[3];
        let impulses = u32::from_be_bytes([b[4], b[5], b[6], b[7]]);
        let m = Message::new(if is_envir {
            "CurrentCost-EnviRCounter"
        } else {
            "CurrentCost-Counter"
        })
        .int("subtype", sensor_type)
        .int("id", id)
        .int("power0", impulses);
        ctx.output(m);
        return 1;
    }
    0
}

pub static DEVICES: &[Device] = &[Device {
    name: "CurrentCost Current Sensor",
    group: Group::Meter,
    modulation: Modulation::FskPcm,
    short_width: 250.0,
    long_width: 250.0, // NRZ
    reset_limit: 8000.0,
    decode: current_cost_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator (rows as on the air, before the decoder's
/// invert): a classic TX meter, id 291, 300 W / 100 W / 0 W (data
/// `01 23 81 2c 80 64 00 00` after one byte of preamble); an EnviR
/// counter, id 342, type 2, 12345 pulses (`41 56 00 02 00 00 30 39`).
pub static SAMPLES: &[Sample] = &[
    Sample {
        dev: 0,
        model: "CurrentCost-TX",
        code: "{181}aa333333316ea2aab2cad4aab2cd2caaab4b2aaaaaaaa8",
        expect: &[
            ("id", "291"),
            ("power0_W", "300"),
            ("power1_W", "100"),
            ("power2_W", "0"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 0,
        model: "CurrentCost-EnviRCounter",
        code: "{175}aaaaaaaa5ba8caacccd2aaaaaab2aaaaaaaab4aab52c",
        expect: &[("subtype", "2"), ("id", "342"), ("power0", "12345")],
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
    fn noise_is_rejected() {
        assert!(decode_rows(&DEVICES[0], "{64}0123456789abcdef").is_empty());
    }
}
