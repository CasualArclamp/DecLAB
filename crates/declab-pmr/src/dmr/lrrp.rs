// made by CasualArclamp
//! What DMR data packets carry: IPv4/UDP (whole, or with the UDP/IPv4 header
//! compression of ETSI TS 102 361-3 clause 7.2), and on top of it
//! Motorola's Location Request/Response Protocol, LRRP (UDP port 4001).
//!
//! LRRP has no public specification. The token table, the field layouts and
//! the scale factors follow SDRTrunk (`module/decode/ip/mototrbo/lrrp`,
//! GPL-3.0-or-later, © 2014-2023 Dennis Sheirer), which works them out from
//! real traffic; where SDRTrunk is unsure (speed's unit, CIRCLE_3D's
//! length) this is too.

use std::fmt;

use super::data::{Packet, sap};

/// LRRP's UDP port.
pub const LRRP_PORT: u16 = 4001;

/// A UDP datagram out of a packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Udp {
    pub src: [u8; 4],
    pub dst: [u8; 4],
    pub sport: u16,
    pub dport: u16,
    pub payload: Vec<u8>,
}

/// The ports the compressed header's SPID/DPID index, table 7.17/7.18.
fn port_of_id(id: u8) -> Option<u16> {
    match id {
        1 => Some(5016),
        2 => Some(5017),
        _ => None,
    }
}

/// The datagram in a reassembled packet, if it is UDP over IPv4.
pub fn udp(p: &Packet) -> Option<Udp> {
    let d = &p.data;
    match p.header.sap {
        sap::IP => {
            if d.len() < 28 || d[0] >> 4 != 4 || d[9] != 17 {
                return None;
            }
            let ihl = usize::from(d[0] & 15) * 4;
            let total = usize::from(u16::from_be_bytes([d[2], d[3]])).min(d.len());
            if total < ihl + 8 {
                return None;
            }
            let u = &d[ihl..total];
            Some(Udp {
                src: d[12..16].try_into().ok()?,
                dst: d[16..20].try_into().ok()?,
                sport: u16::from_be_bytes([u[0], u[1]]),
                dport: u16::from_be_bytes([u[2], u[3]]),
                payload: u[8..].to_vec(),
            })
        }
        sap::UDP_HC => {
            // Table 7.14: IPv4 ID (16), SAID (4), DAID (4), opcode MSB +
            // SPID (8), opcode LSB + DPID (8), then the ports not indexed.
            if d.len() < 5 {
                return None;
            }
            let (said, daid) = (d[2] >> 4, d[2] & 15);
            let (spid, dpid) = (d[3] & 0x7f, d[4] & 0x7f);
            let mut at = 5;
            let mut ext = || {
                let v = d.get(at..at + 2).map(|b| u16::from_be_bytes([b[0], b[1]]));
                at += 2;
                v
            };
            let sport = match port_of_id(spid) {
                Some(p) => p,
                None => ext()?,
            };
            let dport = match port_of_id(dpid) {
                Some(p) => p,
                None => ext()?,
            };
            // DLL-derived addresses (TS 102 361-3 clause 5.1.1): the network
            // number before the LLID. 12 is the usual radio network; the
            // true one is configured in the radios, not sent.
            let addr = |id: u8, llid: u32| {
                let net = if id == 2 { 225 } else { 12 };
                let b = llid.to_be_bytes();
                [net, b[1], b[2], b[3]]
            };
            Some(Udp {
                src: addr(said, p.header.src),
                dst: addr(daid, p.header.dst),
                sport,
                dport,
                payload: d[at..].to_vec(),
            })
        }
        _ => None,
    }
}

/// The LRRP message types SDRTrunk knows.
pub fn message_name(t: u8) -> &'static str {
    match t {
        0x05 => "immediate location request",
        0x07 => "immediate location response",
        0x09 => "triggered location start request",
        0x0B => "triggered location start response",
        0x0D => "triggered location",
        0x0F => "triggered location stop request",
        0x11 => "triggered location stop response",
        0x14 => "protocol version request",
        0x15 => "protocol version response",
        _ => "unknown",
    }
}

/// One LRRP message, the tokens that matter pulled out.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Lrrp {
    pub message: u8,
    pub request_id: Option<u32>,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub altitude_m: Option<f64>,
    pub radius_m: Option<f64>,
    /// SDRTrunk is unsure whether this is mph or km/h (hundredths).
    pub speed: Option<f64>,
    pub heading_deg: Option<f64>,
    /// UTC (year, month, day, hour, minute, second).
    pub time: Option<(u16, u8, u8, u8, u8, u8)>,
    pub response: Option<u16>,
    /// Token IDs seen that are not parsed.
    pub other: Vec<u8>,
}

fn bits(b: &[u8], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0, |v, i| {
        (v << 1) | u64::from((b[i / 8] >> (7 - i % 8)) & 1)
    })
}

/// A token's payload length (after its ID byte), from TokenType.java; `None`
/// for variable (length in the next byte).
fn token_len(id: u8) -> Option<usize> {
    Some(match id {
        0x23 | 0x31 | 0x36 | 0x4A | 0x56 | 0x61 | 0x73 | 0x78 => 1,
        0x34 => 5,
        0x38 | 0x3A | 0x42 | 0x50 | 0x52 | 0x54 | 0x57 | 0x62 | 0x64 => 0,
        0x51 => 10,
        0x55 => 15,
        0x66 => 8,
        0x69 => 11,
        0x6C => 2,
        _ => return None,
    })
}

impl Lrrp {
    pub fn parse(p: &[u8]) -> Option<Lrrp> {
        if p.len() < 2 {
            return None;
        }
        let mut l = Lrrp {
            message: p[0],
            ..Lrrp::default()
        };
        let end = (2 + usize::from(p[1])).min(p.len());
        let response = matches!(p[0], 0x07 | 0x0B | 0x0D | 0x11 | 0x15);
        let mut i = 2;
        while i < end {
            let id = p[i];
            // In a request, most bytes are bare "send me this" flags.
            if !response && !matches!(id, 0x22 | 0x4A | 0x42 | 0x78 | 0x31 | 0x61 | 0x73) {
                i += 1;
                continue;
            }
            let t = &p[i..end];
            let len = match id {
                0x22 => usize::from(*t.get(1)?) + 1,
                // Response: bit 8 says whether its code is one byte or two.
                0x37 => 1 + usize::from(t.get(1)? & 0x80 != 0),
                _ => match token_len(id) {
                    Some(n) => n,
                    // Unknown: nothing after it can be trusted.
                    None => {
                        l.other.push(id);
                        break;
                    }
                },
            };
            if t.len() < len + 1 {
                break;
            }
            let point = |l: &mut Lrrp| {
                let lat = bits(t, 9, 31) as f64 * 180.0 / 4_294_967_295.0;
                l.lat = Some(if t[1] & 0x80 != 0 { -lat } else { lat });
                l.lon = Some(bits(t, 40, 32) as u32 as i32 as f64 * 360.0 / 4_294_967_295.0);
            };
            match id {
                0x22 => {
                    // Little-endian, as SDRTrunk reads it.
                    let n = usize::from(t[1]).min(4);
                    l.request_id = Some(
                        t[2..2 + n]
                            .iter()
                            .rev()
                            .fold(0, |v, &b| (v << 8) | u32::from(b)),
                    );
                }
                0x34 => {
                    l.time = Some((
                        bits(t, 8, 14) as u16,
                        bits(t, 22, 4) as u8,
                        bits(t, 26, 5) as u8,
                        bits(t, 31, 5) as u8,
                        bits(t, 36, 6) as u8,
                        bits(t, 42, 6) as u8,
                    ));
                }
                0x37 => {
                    l.response = Some(if t[1] & 0x80 != 0 {
                        bits(t, 9, 15)
                    } else {
                        bits(t, 9, 7)
                    } as u16);
                }
                0x38 => l.response = Some(0),
                0x51 | 0x55 => {
                    point(&mut l);
                    l.radius_m = Some(bits(t, 72, 16) as f64 * 0.01);
                    if id == 0x55 {
                        l.altitude_m = Some(bits(t, 88, 16) as f64 * 0.01);
                    }
                }
                0x66 | 0x69 => {
                    point(&mut l);
                    if id == 0x69 {
                        l.altitude_m = Some(bits(t, 72, 24) as f64 * 0.01);
                    }
                }
                0x56 => l.heading_deg = Some(f64::from(t[1]) * 2.0),
                0x6C => l.speed = Some(bits(t, 8, 16) as f64 * 0.01),
                _ => l.other.push(id),
            }
            i += 1 + len;
        }
        Some(l)
    }

    /// Encode a location response (for the simulator and the tests).
    pub fn location_response(
        request_id: u32,
        lat: f64,
        lon: f64,
        time: (u16, u8, u8, u8, u8, u8),
    ) -> Vec<u8> {
        let mut t = vec![0x22, 4];
        t.extend_from_slice(&request_id.to_le_bytes());
        let mut ts = [0u8; 6];
        ts[0] = 0x34;
        let v = (u64::from(time.0) << 26)
            | (u64::from(time.1) << 22)
            | (u64::from(time.2) << 17)
            | (u64::from(time.3) << 12)
            | (u64::from(time.4) << 6)
            | u64::from(time.5);
        ts[1..].copy_from_slice(&v.to_be_bytes()[3..]);
        t.extend_from_slice(&ts);
        let mag = ((lat.abs() * 4_294_967_295.0 / 180.0).round() as u64) & 0x7fff_ffff;
        let lat_field = (u32::from(lat < 0.0) << 31) | mag as u32;
        let lon_field = (lon * 4_294_967_295.0 / 360.0).round() as i64 as i32 as u32;
        t.push(0x66);
        t.extend_from_slice(&lat_field.to_be_bytes());
        t.extend_from_slice(&lon_field.to_be_bytes());
        let mut m = vec![0x0D, t.len() as u8];
        m.extend(t);
        m
    }
}

impl fmt::Display for Lrrp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "LRRP {}", message_name(self.message))?;
        if let (Some(lat), Some(lon)) = (self.lat, self.lon) {
            write!(f, " {lat:.5}, {lon:.5}")?;
        }
        if let Some(a) = self.altitude_m {
            write!(f, " alt {a:.0} m")?;
        }
        if let Some(r) = self.radius_m {
            write!(f, " ±{r:.0} m")?;
        }
        if let Some(s) = self.speed {
            write!(f, " speed {s:.1}")?;
        }
        if let Some(h) = self.heading_deg {
            write!(f, " heading {h:.0}°")?;
        }
        if let Some((y, mo, d, h, mi, s)) = self.time {
            write!(f, " at {y:04}-{mo:02}-{d:02} {h:02}:{mi:02}:{s:02} UTC")?;
        }
        if let Some(r) = self.response.filter(|&r| r != 0) {
            let name = match r {
                0x0A => "invalid command",
                0x10 => "no GPS",
                0x16 => "duplicate request",
                0x200 => "GPS initialising",
                _ => "error",
            };
            write!(f, " ({name} {r:#x})")?;
        }
        if let Some(id) = self.request_id {
            write!(f, " [request {id}]")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn location_round_trip() {
        let m = Lrrp::location_response(77, 37.7749, -122.4194, (2026, 10, 29, 14, 5, 9));
        let l = Lrrp::parse(&m).unwrap();
        assert_eq!(l.message, 0x0D);
        assert_eq!(l.request_id, Some(77));
        assert!((l.lat.unwrap() - 37.7749).abs() < 1e-6);
        assert!((l.lon.unwrap() + 122.4194).abs() < 1e-6);
        assert_eq!(l.time, Some((2026, 10, 29, 14, 5, 9)));
        assert!(
            l.to_string()
                .starts_with("LRRP triggered location 37.77490, -122.41940")
        );
    }
}
