// made by CasualArclamp
//! Interlogix / GE / UTC 319.5 MHz security devices (contacts, keyfobs,
//! motion, heat, glass-break, freeze, smoke, panic; also ELK-319DWM and
//! Alula RE101).
//!
//! Ported from rtl_433's `src/devices/interlogix.c` (GPL-2.0-or-later).
//!
//! OOK PPM (122/244 µs), 57–64 bits per US patent 5761206: front porch,
//! 12 sync zeros and a start 1, then a 20-bit ID (bit-reversed bytes), a
//! 4-bit device type, a 3-bit trigger count (the keyfob's buttons), low
//! battery, the latch bits of five inputs, and two parity bits (even/odd
//! bits of the 40 before them, both odd). The decoder finds the `0000 0001`
//! ending the preamble and reads the 46 bits after it.

use crate::bits::{BitBuffer, reverse8};
use crate::device::*;
use crate::sim::{Air, Sample};

const INTERLOGIX_MSG_BIT_LEN: usize = 46;

fn interlogix_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Only the bottom 8 bits of the 13-bit preamble: 0000 0001.
    const PREAMBLE: [u8; 1] = [0x01];
    let row = 0;
    if bb.num_rows != 1 {
        return DECODE_ABORT_EARLY;
    }
    // Between the 57 bits seen in test files and the patent's 64.
    let bits = bb.bits(row);
    if !(57..=64).contains(&bits) {
        return DECODE_ABORT_LENGTH;
    }
    let mut bit_offset = bb.search(row, 0, &PREAMBLE, 8);
    if bit_offset == bits {
        ctx.log(|| format!("interlogix: preamble not found, bit_offset: {bit_offset}"));
        return DECODE_FAIL_SANITY;
    }
    // Just past the preamble and start bit.
    bit_offset += 8;

    let mut message = [0u8; INTERLOGIX_MSG_BIT_LEN.div_ceil(8)];
    bb.extract_bytes(row, bit_offset, &mut message, INTERLOGIX_MSG_BIT_LEN);

    // Fewer false positives: ID or code all 0 or all 1.
    if message[..3] == [0x00; 3] || message[..3] == [0xff; 3] {
        return DECODE_FAIL_SANITY;
    }
    if message[3..6] == [0x00; 3] || message[3..6] == [0xff; 3] {
        return DECODE_FAIL_SANITY;
    }

    // Parity: even bits of message[0..40] and odd bits, i.e. 5 bytes
    // folded to 2 bits, then the two check bits on top of message[5].
    let mut parity = message[0] ^ message[1] ^ message[2] ^ message[3] ^ message[4];
    parity = (parity >> 4) ^ (parity & 0xf);
    parity = (parity >> 2) ^ (parity & 0x3);
    parity ^= message[5] >> 6;
    // Both parities odd: 1 each on success.
    if parity ^ 0x3 != 0 {
        ctx.log(|| {
            format!(
                "interlogix: parity check failed ({} {})",
                parity >> 1,
                parity & 1
            )
        });
        return DECODE_FAIL_MIC;
    }

    let type_code = reverse8(message[2]) >> 4;
    let device_type = match type_code {
        0xa => "contact",
        0xf => "keyfob",
        0x4 => "motion",
        0x6 => "heat",
        0x9 => "glass", // switch1 goes open → closed on trigger
        0xd => "glass", // newer Shatterpro
        0xe => "freeze",
        0x2 => "smoke",
        0x3 => "panic",
        _ => "unknown",
    };
    // The 2-bit parity lets ~1/4 of noise through; a device type outside
    // the patent's list is rejected as well.
    if device_type == "unknown" {
        ctx.log(|| format!("interlogix: implausible device type: {type_code:01x}"));
        return DECODE_FAIL_SANITY;
    }

    let device_serial = format!(
        "{:02x}{:02x}{:02x}",
        reverse8(message[2]),
        reverse8(message[1]),
        reverse8(message[0])
    );
    let raw_message = format!("{:02x}{:02x}{:02x}", message[3], message[4], message[5]);

    // Keyfobs send their buttons in the trigger-count bits, no battery.
    let open_closed = |closed: bool| if closed { "CLOSED" } else { "OPEN" };
    let (low_battery, f1, f2, f3, f4, f5);
    if type_code == 0xf {
        low_battery = false;
        f1 = open_closed(message[3] & 0xe == 0x4);
        f2 = open_closed(message[3] & 0xe == 0x8);
        f3 = open_closed(message[3] & 0xe == 0xc);
        f4 = open_closed(message[3] & 0xe == 0x2);
        f5 = open_closed(message[3] & 0xe == 0xa);
    } else {
        low_battery = message[3] & 0x10 != 0;
        f1 = open_closed(message[3] & 0x04 == 0);
        f2 = open_closed(message[3] & 0x01 == 0);
        f3 = open_closed(message[4] & 0x40 == 0);
        f4 = open_closed(message[4] & 0x10 == 0);
        f5 = open_closed(message[4] & 0x04 == 0);
    }

    let m = Message::new("Interlogix-Security")
        .str("subtype", device_type)
        .str("id", device_serial)
        .int("battery_ok", (!low_battery) as u8)
        .str("switch1", f1)
        .str("switch2", f2)
        .str("switch3", f3)
        .str("switch4", f4)
        .str("switch5", f5)
        .str("raw_message", raw_message);
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Interlogix GE UTC Security Devices",
    group: Group::Remote,
    modulation: Modulation::OokPpm,
    short_width: 122.0,
    long_width: 244.0,
    reset_limit: 500.0, // the longest gap before the end of message
    decode: interlogix_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: 12 sync zeros, the start 1 and 46 message
/// bits (parity computed).
pub static SAMPLES: &[Sample] = &[
    // A contact, ID ac74b1, battery low, switch3 and switch5 open.
    Sample {
        dev: 0,
        model: "Interlogix-Security",
        code: "{59}000c6971a882240",
        expect: &[
            ("subtype", "contact"),
            ("id", "ac74b1"),
            ("battery_ok", "0"),
            ("switch1", "CLOSED"),
            ("switch2", "CLOSED"),
            ("switch3", "OPEN"),
            ("switch4", "CLOSED"),
            ("switch5", "OPEN"),
            ("raw_message", "104480"),
        ],
        air: Air::DEFAULT,
    },
    // A keyfob, ID ff8956, button 1 (lock).
    Sample {
        dev: 0,
        model: "Interlogix-Security",
        code: "{59}000b548ff820060",
        expect: &[
            ("subtype", "keyfob"),
            ("id", "ff8956"),
            ("battery_ok", "1"),
            ("switch1", "CLOSED"),
            ("switch2", "OPEN"),
            ("raw_message", "0400c0"),
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
    fn bad_parity_fails() {
        // The contact with one ID bit flipped.
        assert!(decode_rows(&DEVICES[0], "{59}000c6871a882240").is_empty());
    }
}
