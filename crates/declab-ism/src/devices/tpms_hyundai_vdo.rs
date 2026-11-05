// made by CasualArclamp
//! Hyundai tyre-pressure sensors by Continental/VDO (i30 PDE; VDO
//! A2C98607702, TG1C, FCC ID KR5TIS-01; similar ones in BMW, FCA, PSA,
//! Kia, Mitsubishi, Mazda…): FSK, 10 Manchester-coded bytes with a CRC-8.
//!
//! Ported from rtl_433's `src/devices/tpms_hyundai_vdo.c`
//! (GPL-2.0-or-later).
//!
//! FSK PCM, 52 µs bits. Preamble `55 55 55 56` (inverted `aa aa aa a9`),
//! then (after inverting) Manchester `UU IIIIIIII FR PP TT BB CC`: state,
//! 32-bit ID, flags and repeat counter, pressure (1.375 kPa/bit),
//! temperature °C + 50, maybe battery, CRC-8 (poly 0x07, init 0xaa) over
//! the first 9 bytes.

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn tpms_hyundai_vdo_decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut packet_bits = BitBuffer::new();
    bb.manchester_decode(row, bitpos, &mut packet_bits, 80);
    if packet_bits.bits(0) < 80 {
        return DECODE_FAIL_SANITY; // too short for a whole packet
    }
    let b = packet_bits.row(0);
    if crc8(&b[..9], 0x07, 0xaa) != b[9] {
        return 0;
    }
    let state = b[0];
    let id = u32::from_be_bytes([b[1], b[2], b[3], b[4]]);
    let flags = b[5] >> 4;
    let repeat = b[5] & 0x0f;
    let pressure = b[6];
    let temperature = b[7];
    let maybe_battery = b[8];

    let m = Message::new("Hyundai-VDO")
        .str("type", "TPMS")
        .str("id", format!("{id:08x}"))
        .int("state", state)
        .int("flags", flags)
        .int("repeat", repeat)
        .float("pressure_kPa", pressure as f64 * 1.375, 0)
        .float("temperature_C", temperature as f64 - 50.0, 0)
        .int("maybe_battery", maybe_battery)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn tpms_hyundai_vdo_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Full preamble 55 55 55 56, inverted aa aa aa a9: all 32 bits.
    const PREAMBLE: [u8; 4] = [0xaa, 0xaa, 0xaa, 0xa9];
    let mut bitpos = 0;
    let mut ret = 0;
    let mut events = 0;
    bb.invert();
    loop {
        bitpos = bb.search(0, bitpos, &PREAMBLE, 32);
        if bitpos + 80 > bb.bits(0) {
            break;
        }
        ret = tpms_hyundai_vdo_decode_at(ctx, bb, 0, bitpos + 32);
        if ret > 0 {
            events += ret;
        }
        bitpos += 2;
    }
    if events > 0 { events } else { ret }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Hyundai TPMS (VDO)",
    group: Group::Tpms,
    modulation: Modulation::FskPcm,
    short_width: 52.0, // 42 µs in the FCC test report, works with 52
    long_width: 52.0,  // FSK
    reset_limit: 150.0,
    decode: tpms_hyundai_vdo_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the raw row as the slicer gives it (a
/// 40-bit `55…56` preamble less its leading 0, so the inverted 32-bit
/// `aaaaaaa9` survives; the inverted Manchester payload; the 2 zeros the
/// reset gap adds). State 0x21, ID 8a1b2c3d, repeat 2, 0xa0 (220 kPa),
/// 0x44 (18 °C), battery 1.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Hyundai-VDO",
    code: "{201}aaaaaaaaacb2ad2b32ad34b34ab54caab332aacacaaaacb5520",
    expect: &[
        ("id", "8a1b2c3d"),
        ("state", "33"),
        ("flags", "0"),
        ("repeat", "2"),
        ("pressure_kPa", "220"),
        ("temperature_C", "18"),
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
