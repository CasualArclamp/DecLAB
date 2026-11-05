// made by CasualArclamp
//! ERT Standard Consumption Message (SCM): Itron ERT utility meters.
//!
//! Ported from rtl_433's `src/devices/ert_scm.c` (Copyright (C) 2020
//! Benjamin Larsson, GPL-2.0-or-later). See also bemasher's rtlamr.
//!
//! OOK Manchester, 30 µs half-bits, 96 bits:
//!
//! ```text
//! SAAA AAAA  AAAA AAAA  AAAA A
//!                             iiR PPTT TTEE  C×24  I×24  X×16
//! ```
//! S sync, A preamble, i ERT ID's top 2 bits, R reserved, P physical
//! tamper, T ERT type, E encoder tamper, C consumption, I ERT ID's low 24
//! bits, X CRC-16 (polynomial 0x6F63) from byte 2 on.

use crate::bits::{BitBuffer, crc16};
use crate::device::*;
use crate::sim::{Air, Sample};

fn ert_scm_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    if bb.bits(0) != 96 {
        return DECODE_ABORT_LENGTH;
    }
    let b = bb.row(0);
    if b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0 {
        ctx.log(|| "ert_scm: DECODE_FAIL_SANITY data all 0x00".into());
        return DECODE_FAIL_SANITY;
    }
    // The preamble is not checked: the CRC (whose remainder over the
    // message and itself is 0) does the work.
    if crc16(&b[2..12], 0x6F63, 0) != 0 {
        return DECODE_FAIL_MIC;
    }
    let physical_tamper = (b[3] & 0xc0) >> 6;
    let ert_type = (b[3] >> 2) & 0x0f;
    let encoder_tamper = b[3] & 0x03;
    let consumption = ((b[4] as u32) << 16) | ((b[5] as u32) << 8) | b[6] as u32;
    let ert_id =
        ((b[2] as u32 & 0x06) << 23) | ((b[7] as u32) << 16) | ((b[8] as u32) << 8) | b[9] as u32;

    let m = Message::new("ERT-SCM")
        .int("id", ert_id)
        .int("physical_tamper", physical_tamper)
        .int("ert_type", ert_type)
        .int("encoder_tamper", encoder_tamper)
        .int("consumption_data", consumption)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[Device {
    name: "ERT Standard Consumption Message (SCM)",
    group: Group::Meter,
    modulation: Modulation::OokManchesterZeroBit,
    short_width: 30.0,
    long_width: 0.0, // not used
    gap_limit: 0.0,
    reset_limit: 64.0,
    decode: ert_scm_decode,
    ..Device::DEFAULT
}];

/// Made up for the simulator: the slicer's 0 and a 20-bit preamble
/// (`0 1111 0010 1010 0110 0000`), ID top bits 01, type 7, 100000 units,
/// ID low bits 0x123456 (id 17970262), CRC 0x99bf.
pub static SAMPLES: &[Sample] = &[Sample {
    dev: 0,
    model: "ERT-SCM",
    code: "{96}7953021c0186a012345699bf",
    expect: &[
        ("id", "17970262"),
        ("physical_tamper", "0"),
        ("ert_type", "7"),
        ("encoder_tamper", "0"),
        ("consumption_data", "100000"),
        ("mic", "CRC"),
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

    #[test]
    fn bad_crc_and_length() {
        assert!(decode_rows(&DEVICES[0], "{96}7953021c0186a012345699be").is_empty());
        assert!(decode_rows(&DEVICES[0], "{95}7953021c0186a012345699be").is_empty());
    }
}
