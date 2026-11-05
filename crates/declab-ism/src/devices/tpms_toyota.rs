// made by CasualArclamp
//! Toyota tyre-pressure sensors (Pacific Industries PMV-C210, seen on an
//! Auris/Corolla; also TRW): FSK, 9 differential-Manchester bytes with a
//! CRC-8.
//!
//! Ported from rtl_433's `src/devices/tpms_toyota.c` (GPL-2.0-or-later).
//!
//! FSK PCM, 52 µs bits. 14 sync bits `01010101001111`, 72 differential
//! Manchester bits, a 3-bit trailer. 32-bit ID, a status bit, pressure
//! (¼ PSI, offset −7 PSI), temperature °C + 40, 7 status bits, the
//! pressure inverted, CRC-8 (poly 0x07, init 0x80).

use crate::bits::{BitBuffer, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn tpms_toyota_decode_at(ctx: &mut Ctx, bb: &BitBuffer, row: usize, bitpos: usize) -> i32 {
    let mut packet_bits = BitBuffer::new();
    // Starts on the preamble's last 1 (raw "01" skipped) to get 72 bits.
    let start_pos = bb.differential_manchester_decode(row, bitpos, &mut packet_bits, 80);
    if start_pos - bitpos < 144 {
        return 0;
    }
    let b = packet_bits.row(0);
    if crc8(&b[..8], 0x07, 0x80) != b[8] {
        return 0;
    }
    let id = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
    let status = (b[4] & 0x80) | (b[6] & 0x7f); // status bit and 0 filler
    let pressure1 = ((b[4] & 0x7f) << 1) | (b[5] >> 7);
    let temp = ((b[5] & 0x7f) << 1) | (b[6] >> 7);
    let pressure2 = b[7] ^ 0xff;
    if pressure1 != pressure2 {
        ctx.log(|| format!("Toyota TPMS pressure check error: {pressure1:02x} vs {pressure2:02x}"));
        return 0;
    }

    // rtl_433 gives pressure_PSI no format: 2 decimals show its ¼ PSI
    // steps exactly.
    let m = Message::new("Toyota")
        .str("type", "TPMS")
        .str("id", format!("{id:08x}"))
        .int("status", status)
        .float("pressure_PSI", pressure1 as f64 * 0.25 - 7.0, 2)
        .float("temperature_C", temp as f64 - 40.0, 0)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn tpms_toyota_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    // Full preamble 0101 0101 0011 11 = 55 3c, maybe shorter:
    // 11 0101 0011 11. 12 bits searched; the last is passed on to decode.
    const PREAMBLE: [u8; 2] = [0xa9, 0xe0];
    let mut bitpos = 0;
    let mut ret = 0;
    let mut events = 0;
    loop {
        bitpos = bb.search(0, bitpos, &PREAMBLE, 12);
        if bitpos + 156 > bb.bits(0) {
            break;
        }
        ret = tpms_toyota_decode_at(ctx, bb, 0, bitpos + 11);
        if ret > 0 {
            events += ret;
        }
        bitpos += 2;
    }
    if events > 0 { events } else { ret }
}

pub static DEVICES: &[Device] = &[Device {
    name: "Toyota TPMS",
    group: Group::Tpms,
    modulation: Modulation::FskPcm,
    short_width: 52.0, // 12-13 samples @250k
    long_width: 52.0,  // FSK
    reset_limit: 150.0,
    decode: tpms_toyota_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the raw row as the slicer gives it (the
/// sync less its leading 0, 72 differential Manchester bits, trailer
/// `01` and the 2 zeros the reset gap adds). ID 8a1b2c3d, status 0,
/// 160 (33 PSI), 62 (22 °C).
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "Toyota",
    code: "{161}aa795695596569aa99a5a555599aaaaa599aa5520",
    expect: &[
        ("id", "8a1b2c3d"),
        ("status", "0"),
        ("pressure_PSI", "33.00"),
        ("temperature_C", "22"),
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
