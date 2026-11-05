// made by CasualArclamp
//! ERT Interval Data Message (IDM) and IDM for Net Meters (NetIDM): Itron
//! ERT utility meters, ~912.6 MHz.
//!
//! Ported from rtl_433's `src/devices/ert_idm.c` (Copyright (C) 2020 Peter
//! Shipley, GPL-2.0-or-later). Field names follow bemasher's rtlamr.
//!
//! OOK Manchester, 30 µs half-bits. After a preamble, 720 bits from the
//! sync word `16 a3` and packet type `1c`, protected by a CRC-16 (CCITT
//! polynomial 0x1021, init 0xD895) over bytes 2..88. IDM and NetIDM cannot
//! be told apart on the air (same sync, type, length, version and CRC), so
//! both decoders answer every packet, as in rtl_433.
//!
//! IDM layout (byte offsets from the sync word): type 2, length 3,
//! Hamming code 4, application version 5, endpoint type 6, endpoint ID
//! 7..11, consumption interval 11, programming state 12, tamper counters
//! 13..19, async counters 19..21, power outage flags 21..27, last
//! consumption 27..31, 47 9-bit differential intervals 31..84, transmit
//! time offset 84..86, meter ID CRC 86..88, packet CRC 88..90.
//!
//! NetIDM differs from byte 19: unknown 19..26, last generation count
//! 26..29, unknown 29..32, last consumption 32..36, 27 14-bit
//! differential intervals 36..84.
//!
//! rtl_433 outputs the intervals as a JSON array; here they are one
//! string, `[0,0,1,…]`, as the message has no array value.

use crate::bits::{BitBuffer, crc16};
use crate::device::*;
use crate::sim::{Air, Sample};

const IDM_PACKET_BYTES: usize = 92;
const IDM_PACKET_BITLEN: usize = 720; // 90 bytes from the sync word
const IDM_FRAME_SYNC: [u8; 3] = [0x16, 0xa3, 0x1c];

/// The endpoint type's low nibble is SCM's ERT type (rtlamr's compatible
/// meters list).
fn meter_type_name(ert_type: u8) -> &'static str {
    match ert_type & 0x0f {
        4 | 5 | 7 | 8 => "Electric",
        0 | 1 | 2 | 9 | 12 => "Gas",
        3 | 11 | 13 => "Water",
        _ => "unknown",
    }
}

/// `0x` and the bytes in upper-case hex, as rtl_433's `"%02X"` loops.
fn hex_str(b: &[u8]) -> String {
    let mut s = String::from("0x");
    for x in b {
        s.push_str(&format!("{x:02X}"));
    }
    s
}

/// `[a,b,c]`: stands in for rtl_433's `DATA_ARRAY` of ints.
fn array_str(v: &[u32]) -> String {
    let items: Vec<String> = v.iter().map(|x| x.to_string()).collect();
    format!("[{}]", items.join(","))
}

/// The checks both decoders share: length, sync word, CRC. Returns the
/// sync position and the 90 packet bytes (in a 92-byte array, as rtl_433).
///
/// Rust note: `Result<T, i32>` carries either the bytes or the decode
/// return code, so the caller can `match` and return the code as is.
fn packet(ctx: &mut Ctx, bb: &BitBuffer) -> Result<(usize, [u8; IDM_PACKET_BYTES]), i32> {
    let row_bits = bb.bits(0);
    if row_bits > 600 {
        ctx.log(|| format!("ert_idm: rows={}, row0 len={row_bits}", bb.num_rows));
    }
    if row_bits < IDM_PACKET_BITLEN {
        return Err(DECODE_ABORT_LENGTH);
    }
    let sync_index = bb.search(0, 0, &IDM_FRAME_SYNC, 24);
    ctx.log(|| format!("ert_idm: sync_index={sync_index}"));
    if sync_index >= row_bits {
        return Err(DECODE_ABORT_EARLY);
    }
    if row_bits - sync_index < IDM_PACKET_BITLEN {
        return Err(DECODE_ABORT_LENGTH);
    }
    let mut b = [0u8; IDM_PACKET_BYTES];
    bb.extract_bytes(0, sync_index, &mut b, IDM_PACKET_BITLEN);
    let packet_crc = ((b[88] as u16) << 8) | b[89] as u16;
    if crc16(&b[2..88], 0x1021, 0xD895) != packet_crc {
        return Err(DECODE_FAIL_MIC);
    }
    Ok((sync_index, b))
}

/// `count` intervals of `width` bits each from bit `pos` of row 0.
fn intervals(bb: &BitBuffer, mut pos: usize, count: usize, width: usize) -> Vec<u32> {
    let mut v = Vec::with_capacity(count);
    for _ in 0..count {
        let mut buf = [0u8; 4];
        bb.extract_bytes(0, pos, &mut buf, width);
        // The bits sit at the top of buf: shift them down to an integer.
        let word = u32::from_be_bytes(buf);
        v.push(word >> (32 - width));
        pos += width;
    }
    v
}

fn ert_idm_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let (sync_index, b) = match packet(ctx, bb) {
        Ok(p) => p,
        Err(code) => return code,
    };
    let serial = u32::from_be_bytes([b[7], b[8], b[9], b[10]]);
    let last_consumption = u32::from_be_bytes([b[27], b[28], b[29], b[30]]);
    let diff = intervals(bb, sync_index + 31 * 8, 47, 9);
    ctx.log(|| format!("ert_idm: DifferentialConsumptionIntervals {diff:?}"));

    let m = Message::new("IDM")
        .int("id", serial)
        .str("PacketTypeID", format!("0x{:02X}", b[2]))
        .int("PacketLength", b[3])
        .int("ApplicationVersion", b[5])
        .hex("ERTType", b[6] as u64, 2)
        .int("ERTSerialNumber", serial)
        .int("ConsumptionIntervalCount", b[11])
        .hex("ModuleProgrammingState", b[12] as u64, 2)
        .str("TamperCounters", hex_str(&b[13..19]))
        .hex(
            "AsynchronousCounters",
            (((b[19] as u16) << 8) | b[20] as u16) as u64,
            2,
        )
        .str("PowerOutageFlags", hex_str(&b[21..27]))
        .int("LastConsumptionCount", last_consumption)
        .str("DifferentialConsumptionIntervals", array_str(&diff))
        .int("TransmitTimeOffset", ((b[84] as u16) << 8) | b[85] as u16)
        .hex(
            "MeterIdCRC",
            (((b[86] as u16) << 8) | b[87] as u16) as u64,
            4,
        )
        .hex(
            "PacketCRC",
            (((b[88] as u16) << 8) | b[89] as u16) as u64,
            4,
        )
        .str("MeterType", meter_type_name(b[6]))
        .str("mic", "CRC");
    ctx.output(m);
    1
}

fn ert_netidm_decode(ctx: &mut Ctx, bb: &mut BitBuffer) -> i32 {
    let (sync_index, b) = match packet(ctx, bb) {
        Ok(p) => p,
        Err(code) => return code,
    };
    let serial = u32::from_be_bytes([b[7], b[8], b[9], b[10]]);
    let last_generation = ((b[26] as u32) << 16) | ((b[27] as u32) << 8) | b[28] as u32;
    let last_consumption = u32::from_be_bytes([b[32], b[33], b[34], b[35]]);
    let diff = intervals(bb, sync_index + 36 * 8, 27, 14);
    ctx.log(|| format!("ert_netidm: DifferentialConsumptionIntervals {diff:?}"));

    let m = Message::new("NETIDM")
        .int("id", serial)
        .str("PacketTypeID", format!("0x{:02X}", b[2]))
        .int("PacketLength", b[3])
        .int("ApplicationVersion", b[5])
        .hex("ERTType", b[6] as u64, 2)
        .int("ERTSerialNumber", serial)
        .int("ConsumptionIntervalCount", b[11])
        .hex("ModuleProgrammingState", b[12] as u64, 2)
        .str("TamperCounters", hex_str(&b[13..19]))
        .str("Unknown_field_1", hex_str(&b[19..26]))
        .int("LastGenerationCount", last_generation)
        .str("Unknown_field_2", hex_str(&b[29..32]))
        .int("LastConsumptionCount", last_consumption)
        .str("DifferentialConsumptionIntervals", array_str(&diff))
        .int("TransmitTimeOffset", ((b[84] as u16) << 8) | b[85] as u16)
        .hex(
            "MeterIdCRC",
            (((b[86] as u16) << 8) | b[87] as u16) as u64,
            4,
        )
        .hex(
            "PacketCRC",
            (((b[88] as u16) << 8) | b[89] as u16) as u64,
            4,
        )
        .str("MeterType", meter_type_name(b[6]))
        .str("mic", "CRC");
    ctx.output(m);
    1
}

pub static DEVICES: &[Device] = &[
    Device {
        name: "ERT Interval Data Message (IDM)",
        group: Group::Meter,
        modulation: Modulation::OokManchesterZeroBit,
        short_width: 30.0,
        long_width: 0.0, // not used
        gap_limit: 20000.0,
        reset_limit: 20000.0,
        decode: ert_idm_decode,
        ..Device::DEFAULT
    },
    Device {
        name: "ERT Interval Data Message (IDM) for Net Meters",
        group: Group::Meter,
        modulation: Modulation::OokManchesterZeroBit,
        short_width: 30.0,
        long_width: 0.0, // not used
        gap_limit: 20000.0,
        reset_limit: 20000.0,
        decode: ert_netidm_decode,
        ..Device::DEFAULT
    },
];

/// One packet built from the field values of rtlamr's IDM example in
/// rtl_433's comment (serial 11278109, type 7, interval count 246, last
/// consumption 339972, three intervals of 1, offset 476, meter ID CRC
/// 60090) after a `55 55` preamble, with its packet CRC computed (0x3e86).
/// The NetIDM decoder reads the same bytes its own way.
const IDM_ROW: &str = "{736}555516a31c5cc6040700ac171df6bc420516af2d07000041482608410500053004000000000000000000000000000000000000000000000000000000008000000000000000000000200000000000000000000008000001dceaba3e86";

pub static SAMPLES: &[Sample] = &[
    Sample {
        dev: 0,
        model: "IDM",
        code: IDM_ROW,
        expect: &[
            ("id", "11278109"),
            ("PacketTypeID", "0x1C"),
            ("PacketLength", "92"),
            ("ApplicationVersion", "4"),
            ("ERTType", "0x07"),
            ("ConsumptionIntervalCount", "246"),
            ("ModuleProgrammingState", "0xbc"),
            ("TamperCounters", "0x420516AF2D07"),
            ("AsynchronousCounters", "0x00"),
            ("PowerOutageFlags", "0x414826084105"),
            ("LastConsumptionCount", "339972"),
            (
                "DifferentialConsumptionIntervals",
                "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0]",
            ),
            ("TransmitTimeOffset", "476"),
            ("MeterIdCRC", "0xeaba"),
            ("PacketCRC", "0x3e86"),
            ("MeterType", "Electric"),
        ],
        air: Air::DEFAULT,
    },
    Sample {
        dev: 1,
        model: "NETIDM",
        code: IDM_ROW,
        expect: &[
            ("id", "11278109"),
            ("Unknown_field_1", "0x00004148260841"),
            ("LastGenerationCount", "327685"),
            ("Unknown_field_2", "0x300400"),
            ("LastConsumptionCount", "0"),
            (
                "DifferentialConsumptionIntervals",
                "[0,0,0,0,0,0,0,0,0,0,0,0,0,2048,0,0,0,0,0,32,0,0,0,0,0,0,8192]",
            ),
            ("MeterType", "Electric"),
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
    fn bad_crc_and_short() {
        let bad = IDM_ROW.replace("3e86", "3e87");
        assert!(decode_rows(&DEVICES[0], &bad).is_empty());
        assert!(decode_rows(&DEVICES[1], &bad).is_empty());
        assert!(decode_rows(&DEVICES[0], "{96}7953021c0186a012345699bf").is_empty());
    }
}
