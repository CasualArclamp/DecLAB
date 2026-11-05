// made by CasualArclamp
//! Renault tyre-pressure sensors (Clio, Captur, Zoe, maybe Dacia
//! Sandero): FSK, 9 Manchester-coded bytes with a CRC-8.
//!
//! Ported from rtl_433's `src/devices/tpms_renault.c` (GPL-2.0-or-later).
//!
//! FSK PCM, 52 µs bits. Preamble `55 55 55 56` (inverted `aa aa aa a9`),
//! then (after inverting) Manchester `F F/P PP TT II II II ?? ?? CC`:
//! 6 flag bits, 10-bit pressure (0.75 kPa/bit), temperature °C + 30,
//! 24-bit little-endian ID, 16 unknown bits, CRC-8 (poly 0x07, init 0).

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn tpms_renault_decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut packet_bits = BitBuffer::new();
    bb.manchester_decode(row, bitpos, &mut packet_bits, 160);
    // 72 data bits.
    if packet_bits.bits(0) < 72 {
        return 0; // DECODE_ABORT_LENGTH
    }
    let b = packet_bits.row(0);
    if crc8(&b[..8], 0x07, 0x00) != b[8] {
        return 0; // DECODE_FAIL_MIC
    }
    let flags = b[0] >> 2;
    let id = u32::from_le_bytes([b[3], b[4], b[5], 0]); // little-endian
    let pressure_raw = (((b[0] & 0x03) as u32) << 8) | b[1] as u32;
    let pressure_kpa = pressure_raw as f64 * 0.75;
    let temp_c = b[2] as i32 - 30;
    // Unknown, mostly 0xffff (rtl_433 formats it but does not output it).
    let _unknown = u16::from_le_bytes([b[6], b[7]]);

    let m = Message::new("Renault")
        .str("type", "TPMS")
        .str("id", format!("{id:06x}"))
        .str("flags", format!("{flags:02x}"))
        .float("pressure_kPa", pressure_kpa, 1)
        .float("temperature_C", temp_c as f64, 0)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn tpms_renault_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Full preamble 55 55 55 56, inverted aa aa aa a9.
    const PREAMBLE: [u8; 2] = [0xaa, 0xa9];
    let mut ret = 0;
    let mut events = 0;
    bb.invert();
    for row in 0..bb.num_rows {
        let mut bitpos = 0;
        // A preamble with enough bits after it for a whole packet.
        loop {
            bitpos = bb.search(row, bitpos, &PREAMBLE, 16);
            if bitpos + 160 > bb.bits(row) {
                break;
            }
            ret = tpms_renault_decode_at(ctx, bb, row, bitpos + 16);
            if ret > 0 {
                events += ret;
            }
            bitpos += 15;
        }
    }
    if events > 0 { events } else { ret }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Renault TPMS",
    group: Group::Tpms,
    modulation: Modulation::FskPcm,
    short_width: 52.0, // 12-13 samples @250k
    long_width: 52.0,  // FSK
    reset_limit: 150.0,
    decode: tpms_renault_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the raw row as the slicer gives it (a
/// 40-bit `55…56` preamble less its leading 0, the inverted Manchester
/// payload, the 2 zeros the reset gap adds). Flags 0x34, 320 × 0.75 =
/// 240 kPa, 0x46 (40 °C), ID 123456, unknown ffff.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Renault",
    code: "{185}aaaaaaaaad4caccaaacad2ccd2b4caacb3555555552cb20",
    expect: &[
        ("id", "123456"),
        ("flags", "34"),
        ("pressure_kPa", "240.0"),
        ("temperature_C", "40"),
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
