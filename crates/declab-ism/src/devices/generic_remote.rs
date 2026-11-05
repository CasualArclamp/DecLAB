// made by CasualArclamp
//! Generic remotes and sensors using the PT2260/PT2262, SC2260/SC2262 and
//! EV1527 protocol (cheap key fobs, door/window contacts, PIRs).
//!
//! Ported from rtl_433's `src/devices/generic_remote.c` (GPL-2.0-or-later).
//!
//! 25 bits, PWM: a short (~464 µs) pulse is 1 and a long (~1404 µs) pulse
//! 0 as the slicer sees them; the decoder inverts the first 24 so a short
//! pulse reads 0. 16-bit house code, 8-bit command, then a stop bit that
//! is always a short pulse (1). The 24 bits are also shown as a PT2262
//! tri-state code: each pair 00 → `0`, 11 → `1`, 01 → `Z` (floating),
//! 10 → `X` (invalid on SC226x but valid on EV1527).

use crate::bits::BitBuffer;
use crate::device::*;
use crate::sim::{Air, Sample};

fn generic_remote_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Invert the first three bytes in place: short pulse 0, long pulse 1.
    // Rust note: `bb.bb[0]` is the row's byte array, borrowed mutably
    // here and released at the end of the loop.
    for byte in &mut bb.bb[0][..3] {
        *byte = !*byte;
    }
    let bits = bb.bits(0);
    let b = bb.row(0);

    // Validate the packet: 25 bits, the stop bit 1, house code and
    // command not 0 (false positives).
    if bits != 25 || b[3] & 0x80 == 0 || (b[0] == 0 && b[1] == 0) || b[2] == 0 {
        return DECODE_ABORT_LENGTH;
    }

    let id_16b = ((b[0] as u32) << 8) | b[1] as u32;
    let cmd_8b = b[2];

    // The tri-state code, two bits a symbol, most significant first.
    let full = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
    let tristate: String = (0..12)
        .rev()
        .map(|i| match (full >> (i * 2)) & 0x03 {
            0x00 => '0',
            0x01 => 'Z',
            0x02 => 'X',
            _ => '1',
        })
        .collect();

    let m = Message::new("Generic-Remote")
        .int("id", id_16b)
        .int("cmd", cmd_8b)
        .str("tristate", tristate);
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "Generic Remote SC226x EV1527",
    group: Group::Remote,
    modulation: Modulation::OokPwm,
    short_width: 464.0,
    long_width: 1404.0,
    reset_limit: 1800.0,
    sync_width: 0.0,
    tolerance: 200.0,
    decode: generic_remote_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: house code 0x1234, command 0x5a, sent as the
/// inverted bits `ed cb a5` and the stop bit.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Generic-Remote",
    code: "{25}edcba58",
    expect: &[("id", "4660"), ("cmd", "90"), ("tristate", "0Z0X01Z0ZZXX")],
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
    fn rejects_wrong_length_and_zero_code() {
        assert!(decode_rows(&DEVICES[0], "{24}edcba5").is_empty());
        // House code 0 (raw ff ff) is rejected.
        assert!(decode_rows(&DEVICES[0], "{25}ffffa58").is_empty());
        // No stop bit.
        assert!(decode_rows(&DEVICES[0], "{25}edcba50").is_empty());
    }
}
