// made by CasualArclamp
//! Ford tyre-pressure sensors (Fiesta, Focus, Kuga, Escape, Transit…;
//! likely Continental/VDO S180084730Z), 315 and 433.92 MHz: FSK, 8
//! Manchester-coded bytes with a byte-sum checksum.
//!
//! Ported from rtl_433's `src/devices/tpms_ford.c` (GPL-2.0-or-later).
//!
//! FSK PCM, 52 µs bits. Preamble `55 55 55 56` (inverted `aa aa aa a9`),
//! then (after inverting) Manchester `II II II II PP TT FF CC`: 32-bit ID,
//! pressure PSI × 4 (9th bit in flags 0x20), temperature °C + 56 (not a
//! temperature when 0x80 is set), flags (0x40 moving, 0x08 learn, 0x04
//! moving or at rest, 0x03 unknown), checksum: sum of bytes 0–6.

use crate::bits::BitBuffer;
use crate::device::*;
use crate::sim::{Air, Sample};

fn tpms_ford_decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut packet_bits = BitBuffer::new();
    bb.manchester_decode(row, bitpos, &mut packet_bits, 160);
    // 64 data bits.
    if packet_bits.bits(0) < 64 {
        return 0;
    }
    let b = packet_bits.row(0);
    // Rust note: the sum in u32 so it cannot overflow, then its low byte.
    if (b[..7].iter().map(|&x| x as u32).sum::<u32>() & 0xff) as u8 != b[7] {
        return 0;
    }
    let id = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
    // Kept for debugging, as rtl_433 does.
    let code = ((b[4] as u32) << 16) | ((b[5] as u32) << 8) | b[6] as u32;

    // PSI × 4, the 9th bit from flags 0x20.
    let psibits = (((b[6] & 0x20) as u32) << 3) | b[4] as u32;
    let pressure_psi = psibits as f32 * 0.25;

    // Temperature + 56, unless 0x80 is set.
    let temperature_valid = b[5] & 0x80 == 0;
    let temperature_c = if temperature_valid {
        (b[5] & 0x7f) as i32 - 56
    } else {
        -1000
    };

    // Moving, learn and at-rest bits; anything else is the "unknown"
    // syndrome.
    let (mut learn, mut moving, mut unknown) = (0, 0, 0u8);
    match b[6] & 0x4c {
        0x08 => learn = 1, // the learn tool
        0x04 => {}         // at rest
        0x44 => moving = 1,
        other => unknown = other,
    }
    // 0x80 and 0x10 are expected to be 0.
    unknown |= b[6] & 0x90;
    // Every real capture has unknown == 0: reject what is not understood.
    if unknown != 0 {
        ctx.log(|| format!("tpms_ford: unexpected flag bits: {unknown:02x}"));
        return 0;
    }
    // Low 2 bits: variously 01, 10.
    let unknown_3 = b[6] & 0x3;

    let m = Message::new("Ford")
        .str("type", "TPMS")
        .str("id", format!("{id:08x}"))
        .float("pressure_PSI", pressure_psi as f64, 2)
        .cond(temperature_valid, |m| {
            m.float("temperature_C", temperature_c as f64, 1)
        })
        .int("moving", moving)
        .int("learn", learn)
        .str("code", format!("{code:06x}"))
        .str("unknown", format!("{unknown:02x}"))
        .str("unknown_3", format!("{unknown_3:01x}"))
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

fn tpms_ford_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
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
            if bitpos + 144 > bb.bits(row) {
                break;
            }
            ret = tpms_ford_decode_at(ctx, bb, row, bitpos + 16);
            if ret > 0 {
                events += ret;
            }
            bitpos += 15;
        }
    }
    if events > 0 { events } else { ret }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Ford TPMS",
    group: Group::Tpms,
    modulation: Modulation::FskPcm,
    short_width: 52.0, // 12-13 samples @250k
    long_width: 52.0,  // FSK
    reset_limit: 150.0,
    decode: tpms_ford_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the raw row as the slicer gives it (a
/// 40-bit `55…56` preamble less its leading 0, the inverted Manchester
/// payload, the 2 zeros the reset gap adds). ID 8a1b2c3d, 0x84 (33 PSI),
/// 0x65 (45 °C), flags 0x45 (moving).
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Ford",
    code: "{169}aaaaaaaaad2b32ad34b34ab54d2acad2cccaccb54a0",
    expect: &[
        ("id", "8a1b2c3d"),
        ("pressure_PSI", "33.00"),
        ("temperature_C", "45.0"),
        ("moving", "1"),
        ("learn", "0"),
        ("code", "846545"),
        ("unknown", "00"),
        ("unknown_3", "1"),
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
