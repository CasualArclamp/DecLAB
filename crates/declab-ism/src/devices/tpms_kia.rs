// made by CasualArclamp
//! Kia Rio III (UB, 2011–2017) and some Hyundai tyre-pressure sensors:
//! FSK, a 16-bit preamble and 69 Manchester-coded bits with a 5-bit CRC.
//!
//! Ported from rtl_433's `src/devices/tpms_kia.c` (GPL-2.0-or-later).
//!
//! FSK PCM, 50 µs bits (rtl_433 suggests `-s 1000k`). 154 raw bits:
//! preamble `ed71` (not Manchester), then Manchester (01 = 1) of
//! `aaaa pppppppp tttttttt i×32 d×12 ccccc`: unknown (0xf), pressure
//! PSI × 5, temperature °C + 50, 32-bit ID, 12 unknown bits, and the top
//! 5 bits of a CRC-8 (poly 0x07, init 0x76) whose low 3 bits are 0.

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

const PREAMBLE_LENGTH: usize = 16;

fn tpms_kia_decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut packet_bits = BitBuffer::new();
    let start_pos = bb.manchester_decode(row, bitpos, &mut packet_bits, 154 - PREAMBLE_LENGTH);
    if start_pos - bitpos < 154 - PREAMBLE_LENGTH {
        return DECODE_ABORT_LENGTH;
    }
    let b = packet_bits.row(0);

    let unknown1 = b[0] >> 4;
    let pressure = (b[0] << 4) | (b[1] >> 4);
    let temperature = (b[1] << 4) | (b[2] >> 4);
    // rtl_433 shifts in 32 bits: the top nibble of b[2] falls off.
    let id = ((b[2] as u32) << 28)
        | ((b[3] as u32) << 20)
        | ((b[4] as u32) << 12)
        | ((b[5] as u32) << 4)
        | (b[6] >> 4) as u32;
    // rtl_433 stores `b[6] << 8 | b[7]` in a uint8_t: only b[7] is left.
    let unknown2 = b[7];

    // The last 3 bits of b[8] are past the 154 bits: taken as 000.
    let crc = b[8] & !0x7;
    if crc != crc8(&b[..8], 0x07, 0x76) {
        return DECODE_FAIL_MIC;
    }

    let raw: String = b[..9].iter().map(|x| format!("{x:02x}")).collect();
    let pressure_float = pressure as f32 / 5.0;
    let temperature_float = temperature as f32 - 50.0;

    let m = Message::new("Kia")
        .str("type", "TPMS")
        .str("id", format!("{id:08x}"))
        .str("unknown1", format!("{unknown1:02x}"))
        .str("unknown2", format!("{unknown2:03x}"))
        .float("pressure_PSI", pressure_float as f64, 1)
        .float("temperature_C", temperature_float as f64, 0)
        .str("raw", raw)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn tpms_kia_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    const PREAMBLE: [u8; 2] = [0xed, 0x71];
    let mut bitpos = 0;
    let mut ret = 0;
    let mut events = 0;
    // A preamble with enough bits after it for a whole packet.
    loop {
        bitpos = bb.search(0, bitpos, &PREAMBLE, PREAMBLE_LENGTH);
        if bitpos + 154 > bb.bits(0) {
            break;
        }
        ret = tpms_kia_decode_at(ctx, bb, 0, bitpos + PREAMBLE_LENGTH);
        if ret > 0 {
            events += ret;
        }
        bitpos += 2;
    }
    if events > 0 { events } else { ret }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Kia TPMS (-s 1000k)",
    group: Group::Tpms,
    modulation: Modulation::FskPcm,
    short_width: 50.0,
    long_width: 50.0,
    reset_limit: 200.0,
    decode: tpms_kia_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the preamble, 69 Manchester bits and the 4
/// zeros the reset gap adds. Unknown 0xf, 165 (33.0 PSI), 72 (22 °C), ID
/// 8a1b2c3d, unknown 0x004 (picked so the CRC's low 3 bits are 0).
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Kia",
    code: "{158}ed715566999a6a6a66a965a65aa559aaaa9a6940",
    expect: &[
        ("id", "8a1b2c3d"),
        ("unknown1", "0f"),
        ("unknown2", "004"),
        ("pressure_PSI", "33.0"),
        ("temperature_C", "22"),
        ("raw", "fa5488a1b2c3d00498"),
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
