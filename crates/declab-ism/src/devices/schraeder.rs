// made by CasualArclamp
//! Schrader tyre-pressure sensors: the original (FCC MRXGG4), EG53MA4
//! (Saab, Opel, Vauxhall, Chevrolet), SMD3MA4 (Subaru) and its twin
//! MRXNIS315G3 / 3039 (Infiniti, Nissan, Renault; Redi-Sensor SE10001HP),
//! and MRXBC5A4 (BMW).
//!
//! Ported from rtl_433's `src/devices/schraeder.c` (GPL-2.0-or-later).
//!
//! - Schrader: OOK Manchester, 68 bits: sync nibble, then
//!   `PF FI II II II PP TT CC`: flags, 28-bit ID, pressure 25 mbar/bit,
//!   temperature °C + 50, CRC-8 (poly 0x07, init 0xf0).
//! - EG53MA4: OOK Manchester, 120 bits: 40 sync bits, 4 flag bytes, 24-bit
//!   ID, pressure 25 mbar/bit, temperature °F, byte-sum checksum.
//! - SMD3MA4 / NIS315G3: raw OOK PCM, preamble `F5555555E` then 38
//!   Manchester bits (01 = 0, 10 = 1): a fixed 1, 3 flags, 24-bit ID,
//!   8-bit pressure (PSI × 5 or × 4), 2-bit sum checksum. No temperature.
//! - MRXBC5A4: OOK Manchester, 61 bits: fixed `0111 1111 1111 1111`, 3
//!   flags, 24-bit ID, 9-bit pressure (kPa), 2-bit check, 7-bit
//!   temperature °C + 50.

use crate::bits::{BitBuffer, add_bytes, crc8};
use crate::device::*;
use crate::sim::{Air, Sample};

fn schraeder_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    if bb.bits(0) != 68 {
        return DECODE_ABORT_LENGTH;
    }
    // Skip the 4 sync bits.
    let mut b = [0u8; 8];
    bb.extract_bytes(0, 4, &mut b, 64);
    if b[7] != crc8(&b[..7], 0x07, 0xf0) {
        return DECODE_FAIL_MIC;
    }
    let serial_id =
        (((b[1] & 0x0f) as u32) << 24) | ((b[2] as u32) << 16) | ((b[3] as u32) << 8) | b[4] as u32;
    let flags = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
    let pressure = b[5] as u32 * 25; // mbar
    let temperature = b[6] as i32 - 50;

    let m = Message::new("Schrader")
        .str("type", "TPMS")
        .str("flags", format!("{flags:02x}"))
        .str("id", format!("{serial_id:07X}"))
        .float("pressure_kPa", pressure as f64 * 0.1, 1)
        .float("temperature_C", temperature as f64, 0)
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn schrader_eg53ma4_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    if bb.bits(0) != 120 {
        return DECODE_ABORT_LENGTH;
    }
    // Skip the first 40 bits.
    let mut b = [0u8; 10];
    bb.extract_bytes(0, 40, &mut b, 80);
    // Serial, flags, pressure and temperature all zero: not a sensor.
    if b[1] == 0 && b[2] == 0 && b[4] == 0 && b[5] == 0 && b[7] == 0 && b[8] == 0 {
        ctx.log(|| "schrader_EG53MA4: DECODE_FAIL_SANITY data all 0x00".into());
        return DECODE_FAIL_SANITY;
    }
    if add_bytes(&b[..9]) & 0xff != b[9] as i32 {
        return DECODE_FAIL_MIC;
    }
    let serial_id = ((b[4] as u32) << 16) | ((b[5] as u32) << 8) | b[6] as u32;
    let flags = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
    let pressure = b[7] as u32 * 25; // mbar
    let temperature = b[8]; // °F

    let m = Message::new("Schrader-EG53MA4")
        .str("type", "TPMS")
        .str("flags", format!("{flags:08x}"))
        .str("id", format!("{serial_id:06X}"))
        .float("pressure_kPa", pressure as f64 * 0.1, 1)
        .float("temperature_F", temperature as f64, 1)
        .str("mic", "CHECKSUM");
    ctx.output(m);
    1
}

const NUM_BITS_PREAMBLE: usize = 36;
/// 1 fixed bit + 3 flags + 24 ID + 8 pressure + 2 checksum.
const NUM_BITS_DATA: usize = 38;
const NUM_BITS_TOTAL_MIN: usize = NUM_BITS_PREAMBLE / 2 + 2 * NUM_BITS_DATA;
const NUM_BITS_TOTAL_MAX: usize = NUM_BITS_PREAMBLE + 2 * NUM_BITS_DATA + 8;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Smd3Model {
    Smd3ma4,
    Nis315g3,
}

/// SMD3MA4 and NIS315G3 share the wire format; they differ only in the
/// pressure scale and model name (rtl_433 runs both, so one transmission
/// gives two records).
fn schrader_smd3ma4_family_decode(ctx: &mut Ctx, bb: &mut BitBuffer, model: Smd3Model) -> i32 {
    // The last 16 bits of the full preamble 0xF5555555E.
    const PREAMBLE: [u8; 2] = [0x55, 0x5e];
    let bits = bb.bits(0);
    if !(NUM_BITS_TOTAL_MIN..NUM_BITS_TOTAL_MAX).contains(&bits) {
        return DECODE_ABORT_LENGTH;
    }
    // Keep the preamble's last two (fixed) bits as the first data bit.
    let bitpos = bb.search(0, 0, &PREAMBLE, 16) + 14;
    if bitpos + NUM_BITS_DATA * 2 > bits {
        return DECODE_ABORT_EARLY;
    }
    let mut decoded = BitBuffer::new();
    let ret = bb.manchester_decode(0, bitpos, &mut decoded, NUM_BITS_DATA);
    if ret != bitpos + NUM_BITS_DATA * 2 {
        ctx.log(|| "schrader_SMD3MA4: invalid Manchester data".into());
        return DECODE_FAIL_MIC;
    }
    // 10 is 1 here, the reverse of manchester_decode's convention.
    decoded.invert();
    let b = decoded.row(0);

    if b[..4] == [0; 4] {
        ctx.log(|| "schrader_SMD3MA4: DECODE_FAIL_SANITY data all 0x00".into());
        return DECODE_FAIL_SANITY;
    }
    // Add every 2-bit group (with the fixed leading 1) modulo 4: always 1.
    let sum: u32 = b[..5]
        .iter()
        .map(|&x| {
            (x & 3) as u32 + ((x >> 2) & 3) as u32 + ((x >> 4) & 3) as u32 + ((x >> 6) & 3) as u32
        })
        .sum();
    if sum & 0x3 != 1 {
        ctx.log(|| "schrader_SMD3MA4: checksum failed".into());
        return DECODE_FAIL_MIC;
    }

    // 1FFFSSSS SSSSSSSS SSSSSSSS SSSSPPPP PPPPXX..
    let flags = (b[0] & 0x70) >> 4;
    let serial_id = (((b[0] & 0x0f) as u32) << 20)
        | ((b[1] as u32) << 12)
        | ((b[2] as u32) << 4)
        | (b[3] >> 4) as u32;
    let pressure = ((b[3] & 0x0f) << 4) | (b[4] >> 4);
    // Normal driving is flags 7.
    let flag_learn = flags == 0x0;
    let flag_alarm = flags == 0x3;
    let flag_wakeup = flags == 0x5;
    // rtl_433 scales in f32: 0.2f / 0.25f.
    let (name, scale) = match model {
        Smd3Model::Smd3ma4 => ("Schrader-SMD3MA4", 0.2f32),
        Smd3Model::Nis315g3 => ("Schrader-NIS315G3", 0.25f32),
    };

    let m = Message::new(name)
        .str("type", "TPMS")
        .str("id", format!("{serial_id:06X}"))
        .int("flags", flags)
        .cond(flag_learn, |m| m.int("learn", 1))
        .cond(flag_alarm, |m| m.int("alarm", 1))
        .cond(flag_wakeup, |m| m.int("wakeup", 1))
        .float("pressure_PSI", (pressure as f32 * scale) as f64, 1)
        .str("mic", "PARITY");
    ctx.output(m);
    1
}

fn schrader_smd3ma4_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    schrader_smd3ma4_family_decode(ctx, bb, Smd3Model::Smd3ma4)
}

fn schrader_nis315g3_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    schrader_smd3ma4_family_decode(ctx, bb, Smd3Model::Nis315g3)
}

fn schrader_mrxbc5a4_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    if bb.num_rows != 1 {
        return DECODE_ABORT_EARLY;
    }
    if bb.bits(0) != 61 {
        return DECODE_ABORT_LENGTH;
    }
    // The fixed 16-bit wake + sync + start prefix must be at the start.
    const SYNC: [u8; 2] = [0x7f, 0xff];
    if bb.search(0, 0, &SYNC, 16) != 0 {
        ctx.log(|| "schrader_MRXBC5A4: sync pattern not found".into());
        return DECODE_ABORT_EARLY;
    }
    let mut b = [0u8; 6];
    bb.extract_bytes(0, 16, &mut b, 46);

    // b[0] FFFIIIII, b[1] IIIIIIII, b[2] IIIIIIII, b[3] IIIPPPPP,
    // b[4] PPPPCCTT, b[5] TTTTTxxx
    let serial_id = (((b[0] & 0x1f) as u32) << 19)
        | ((b[1] as u32) << 11)
        | ((b[2] as u32) << 3)
        | (b[3] >> 5) as u32;
    if serial_id == 0 || serial_id == 0xff_ffff {
        ctx.log(|| "schrader_MRXBC5A4: DECODE_FAIL_SANITY data all 0x00".into());
        return DECODE_FAIL_SANITY;
    }

    // C1C2 = (even_ones + 2·n − 1) mod 4 over the 35-bit payload (ID,
    // pressure and the check bits): bits 3..38 of the 46.
    let (mut even_ones, mut n) = (0i32, 0i32);
    for i in 3..38 {
        if (b[i / 8] >> (7 - (i % 8))) & 1 != 0 {
            n += 1;
            if (i - 3) % 2 == 0 {
                even_ones += 1;
            }
        }
    }
    let c1c2 = (even_ones + 2 * n - 1) & 0x3;
    let c1 = ((b[4] >> 3) & 1) as i32;
    let c2 = ((b[4] >> 2) & 1) as i32;
    if c1c2 != ((c1 << 1) | c2) {
        return DECODE_FAIL_MIC;
    }

    let flags = (b[0] >> 5) & 0x7;
    let pressure = (((b[3] & 0x1f) as u32) << 4) | (b[4] >> 4) as u32; // kPa
    let temperature = (((b[4] & 0x03) as i32) << 5) | (b[5] >> 3) as i32;

    // A second filter for the noise the 2-bit check lets through: a real
    // tyre's pressure and temperature.
    if pressure > 450 || temperature - 50 < -40 || temperature - 50 > 85 {
        ctx.log(|| {
            format!(
                "schrader_MRXBC5A4: implausible pressure/temperature: {pressure} kPa, {} C",
                temperature - 50
            )
        });
        return DECODE_FAIL_SANITY;
    }

    let m = Message::new("Schrader-MRXBC5A4")
        .str("type", "TPMS")
        .str("flags", format!("{flags:01x}"))
        .str("id", format!("{serial_id:06X}"))
        .float("pressure_kPa", pressure as f64, 1)
        .float("temperature_C", (temperature - 50) as f64, 1)
        .str("sleep", if flags == 2 { "True" } else { "False" })
        .str("mic", "PARITY");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "Schrader TPMS",
        group: Group::Tpms,
        modulation: Modulation::OokManchesterZeroBit,
        short_width: 120.0,
        long_width: 0.0,
        reset_limit: 480.0,
        decode: schraeder_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Schrader TPMS EG53MA4, Saab, Opel, Vauxhall, Chevrolet",
        group: Group::Tpms,
        modulation: Modulation::OokManchesterZeroBit,
        short_width: 123.0,
        long_width: 0.0,
        reset_limit: 300.0,
        decode: schrader_eg53ma4_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Schrader TPMS SMD3MA4 (Subaru)",
        group: Group::Tpms,
        modulation: Modulation::OokPcm,
        short_width: 120.0,
        long_width: 120.0,
        reset_limit: 480.0,
        decode: schrader_smd3ma4_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Schrader TPMS MRXNIS315G3, 3039 (Infiniti, Nissan, Renault), aka Redi-Sensor SE10001HP/SE10001HPR",
        group: Group::Tpms,
        modulation: Modulation::OokPcm,
        short_width: 120.0,
        long_width: 120.0,
        reset_limit: 480.0,
        decode: schrader_nis315g3_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "Schrader TPMS MRXBC5A4 (BMW)",
        group: Group::Tpms,
        modulation: Modulation::OokManchesterZeroBit,
        short_width: 123.0,
        long_width: 0.0,
        reset_limit: 800.0,
        decode: schrader_mrxbc5a4_decode,
        ..Device::DEFAULT
    },
];

/// The Manchester devices' rows begin with the slicer's hard-coded 0.
pub static SAMPLES: &[Sample] = &[
    // Made up: rtl_433's comment example `7 f6 70 3a 38 b2 …` with
    // pressure 0x58 (220 kPa), 0x45 (19 °C) and its CRC.
    Sample {
        dev: 0,
        model: "Schrader",
        code: "{68}7f6703a38b25845c9",
        expect: &[
            ("flags", "67"),
            ("id", "03A38B2"),
            ("pressure_kPa", "220.0"),
            ("temperature_C", "19"),
        ],
        air: Air::DEFAULT,
    },
    // Made up: 40 zero-ish sync bits, flags 7f100100, ID 1A2B3C, 0x58
    // (220 kPa), 85 °F, byte sum.
    Sample {
        dev: 1,
        model: "Schrader-EG53MA4",
        code: "{120}00000000017f1001001a2b3c5855be",
        expect: &[
            ("flags", "7f100100"),
            ("id", "1A2B3C"),
            ("pressure_kPa", "220.0"),
            ("temperature_F", "85.0"),
        ],
        air: Air::DEFAULT,
    },
    // Made up: flags 7 (driving), ID 9A3B5C, pressure 160 (32.0 PSI as
    // SMD3MA4, 40.0 as NIS315G3).
    Sample {
        dev: 2,
        model: "Schrader-SMD3MA4",
        code: "{113}f5555555eaa5a656a699a96655680",
        expect: &[("id", "9A3B5C"), ("flags", "7"), ("pressure_PSI", "32.0")],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 3,
        model: "Schrader-NIS315G3",
        code: "{113}f5555555eaa5a656a699a96655680",
        expect: &[("id", "9A3B5C"), ("flags", "7"), ("pressure_PSI", "40.0")],
        air: Air::DEFAULT,
    },
    // rtl_433's example payload {37}698e08eb48 (with the fixed leading 1)
    // put back behind its preamble: flags 3 (alarm), ID 4C7047, 90.
    Sample {
        dev: 2,
        model: "Schrader-SMD3MA4",
        code: "{113}f5555555e699695a95595a99a6580",
        expect: &[
            ("id", "4C7047"),
            ("flags", "3"),
            ("alarm", "1"),
            ("pressure_PSI", "18.0"),
        ],
        air: Air::DEFAULT,
    },
    // Made up: flags 0, ID 8A4B2C, 230 kPa, 20 °C, its C1C2.
    Sample {
        dev: 4,
        model: "Schrader-MRXBC5A4",
        code: "{61}7fff1149658e6630",
        expect: &[
            ("flags", "0"),
            ("id", "8A4B2C"),
            ("pressure_kPa", "230.0"),
            ("temperature_C", "20.0"),
            ("sleep", "False"),
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
    fn rtl_433_example() {
        // The comment's packet `7 f6 70 3a 38 b2 00 49 49`: its CRC holds;
        // pressure 0, 0x49 − 50 = 23 °C.
        let m = decode_rows(&DEVICES[0], "{68}7f6703a38b2004949");
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].get("id").unwrap().to_string(), "03A38B2");
        assert_eq!(m[0].get("pressure_kPa").unwrap().to_string(), "0.0");
        assert_eq!(m[0].get("temperature_C").unwrap().to_string(), "23");
    }

    #[test]
    fn mrxbc5a4_bad_check_fails() {
        // The sample with C2 flipped (a C1 flip changes the check by 2 as well: undetectable).
        assert!(decode_rows(&DEVICES[4], "{61}7fff1149658e6230").is_empty());
    }
}
