// made by CasualArclamp
//! Citroen tyre-pressure sensors (also Peugeot, and likely Fiat,
//! Mitsubishi and other VDO types): FSK, 10 Manchester-coded bytes with an
//! XOR checksum.
//!
//! Ported from rtl_433's `src/devices/tpms_citroen.c` (GPL-2.0-or-later).
//!
//! FSK PCM, 52 µs bits. Preamble `55 55 55 56` (inverted `aa aa aa a9`),
//! then (after inverting) Manchester bytes `UU IIIIIIII FR PP TT BB CC`:
//! state (not in the checksum), 32-bit ID, flags and repeat counter,
//! pressure (1.364 kPa/bit), temperature °C + 50, maybe battery, and a
//! checksum: XOR of bytes 1–9 is 0.

use crate::bits::BitBuffer;
use crate::device::*;
use crate::sim::{Air, Sample};

fn tpms_citroen_decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut packet_bits = BitBuffer::new();
    bb.manchester_decode(row, bitpos, &mut packet_bits, 88);
    if packet_bits.bits(0) < 80 {
        return DECODE_FAIL_SANITY;
    }
    let b = packet_bits.row(0);
    if b[6] == 0 || b[7] == 0 {
        return DECODE_ABORT_EARLY;
    }
    // Rust note: `fold` runs the XOR across the slice, starting from 0.
    if b[1..10].iter().fold(0u8, |a, &x| a ^ x) != 0 {
        return DECODE_FAIL_MIC;
    }
    let state = b[0]; // not covered by the checksum
    let id = u32::from_be_bytes([b[1], b[2], b[3], b[4]]);
    let flags = b[5] >> 4;
    let repeat = b[5] & 0x0f;
    let pressure = b[6];
    let temperature = b[7];
    let maybe_battery = b[8];

    let m = Message::new("Citroen")
        .str("type", "TPMS")
        .str("id", format!("{id:08x}"))
        .str("state", format!("{state:02x}"))
        .int("flags", flags)
        .int("repeat", repeat)
        .float("pressure_kPa", pressure as f64 * 1.364, 0)
        .float("temperature_C", temperature as f64 - 50.0, 0)
        .int("maybe_battery", maybe_battery)
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

fn tpms_citroen_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Full preamble 55 55 55 56, inverted aa aa aa a9; trailer 01111110.
    const PREAMBLE: [u8; 2] = [0xaa, 0xa9];
    let mut bitpos = 0;
    let mut ret = 0;
    let mut events = 0;
    bb.invert();
    // A preamble with enough bits after it for a whole packet.
    loop {
        bitpos = bb.search(0, bitpos, &PREAMBLE, 16);
        if bitpos + 178 > bb.bits(0) {
            break;
        }
        ret = tpms_citroen_decode_at(ctx, bb, 0, bitpos + 16);
        if ret > 0 {
            events += ret;
        }
        bitpos += 2;
    }
    if events > 0 { events } else { ret }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Citroen TPMS",
    group: Group::Tpms,
    modulation: Modulation::FskPcm,
    short_width: 52.0, // 12-13 samples @250k
    long_width: 52.0,  // FSK
    reset_limit: 150.0,
    decode: tpms_citroen_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the raw row as the slicer gives it (a
/// 40-bit `55…56` preamble less its leading 0, the inverted Manchester
/// payload, a `0101` trailer (inverted), the 2 zeros the reset gap adds;
/// rtl_433 wants 178 bits from the preamble on). State 0x20, ID 8a1b2c3d,
/// repeat 1, 0xa5 (225 kPa), 0x46 (20 °C), battery 1.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Citroen",
    code: "{204}aaaaaaaaacb2ab2b32ad34b34ab54caaad32cccad2aaacd2b54",
    expect: &[
        ("id", "8a1b2c3d"),
        ("state", "20"),
        ("flags", "0"),
        ("repeat", "1"),
        ("pressure_kPa", "225"),
        ("temperature_C", "20"),
        ("maybe_battery", "1"),
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
}
