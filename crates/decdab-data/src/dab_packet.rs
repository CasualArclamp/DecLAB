// made by CasualArclamp
//! DAB packet mode (EN 300 401 §5.3.2): the packets of a packet-mode
//! sub-channel, and the data groups reassembled from them, one address at a
//! time.
//!
//! This is DAB's packet layer, not DRM's ([`crate::packet`] is DRM's, with a
//! one-byte header and a fixed length per stream). A DAB packet is 24, 48,
//! 72 or 96 bytes:
//!
//! ```text
//! byte 0   packet length (2) | continuity index (2) | first/last (2) | address 9..8
//! byte 1   address 7..0
//! byte 2   command (1) | useful data length (7)
//! ...      packet data field (length − 5 bytes, the useful data first, then padding)
//! last 2   packet CRC (CRC-16 over everything before it)
//! ```
//!
//! Address 0 is padding. Packet-mode FEC (FIG 0/14, §5.3.5) puts its RS
//! parity in nine 24-byte packets on address 1022 after every 2256 bytes of
//! data packets. Those carry no packet CRC; they are skipped here, and the
//! packet CRC simply rejects packets the RS code could have mended.

use crate::crc::crc16_check;

/// Packet lengths by the two-bit length field (§5.3.2.1).
pub const PACKET_LENGTHS: [usize; 4] = [24, 48, 72, 96];
/// Header (3 bytes) plus CRC (2 bytes).
pub const OVERHEAD: usize = 5;
/// The padding address.
pub const PADDING_ADDRESS: u16 = 0;
/// The address of packet-mode FEC's parity packets (§5.3.5).
pub const FEC_ADDRESS: u16 = 1022;
/// A data group longer than this is dropped (§5.3.3: a data group's
/// data field is at most 8191 bytes).
const MAX_GROUP: usize = 8191 + 16;

/// One packet's header fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketHeader {
    /// Total length, bytes.
    pub len: usize,
    /// Continuity index, counting the address's packets mod 4.
    pub continuity: u8,
    /// The first packet of a data group.
    pub first: bool,
    /// The last packet of a data group.
    pub last: bool,
    /// Packet address (FIG 0/3).
    pub address: u16,
    /// Command flag (0: data).
    pub command: bool,
    /// Bytes of useful data in the data field.
    pub useful: usize,
}

impl PacketHeader {
    /// From the packet's first three bytes.
    pub fn parse(b: &[u8; 3]) -> Self {
        PacketHeader {
            len: PACKET_LENGTHS[usize::from(b[0] >> 6)],
            continuity: (b[0] >> 4) & 3,
            first: b[0] & 0x08 != 0,
            last: b[0] & 0x04 != 0,
            address: (u16::from(b[0] & 3) << 8) | u16::from(b[1]),
            command: b[2] & 0x80 != 0,
            useful: usize::from(b[2] & 0x7F),
        }
    }
}

/// Counters for the whole sub-channel.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DabPacketStats {
    /// Packets with a good CRC, any address.
    pub packets_ok: u64,
    /// Packets with a bad CRC.
    pub packets_bad: u64,
    /// Padding packets.
    pub padding: u64,
    /// FEC parity packets skipped.
    pub fec: u64,
    /// Packets on the wanted address.
    pub packets_ours: u64,
    /// Data groups completed on our address.
    pub groups: u64,
    /// Data groups lost to a missing or bad packet (continuity break or a
    /// packet with no start).
    pub groups_dropped: u64,
}

/// Packets out of a packet-mode sub-channel's logical frames, data groups out
/// of the packets on one address.
#[derive(Debug, Clone)]
pub struct DabPacketDemux {
    address: u16,
    /// Bytes of a packet that runs past the end of a logical frame.
    carry: Vec<u8>,
    group: Vec<u8>,
    in_group: bool,
    next_ci: Option<u8>,
    stats: DabPacketStats,
}

impl DabPacketDemux {
    /// For the service component on `address` (FIG 0/3).
    pub fn new(address: u16) -> Self {
        DabPacketDemux {
            address,
            carry: Vec::new(),
            group: Vec::new(),
            in_group: false,
            next_ci: None,
            stats: DabPacketStats::default(),
        }
    }

    /// The counters.
    pub fn stats(&self) -> &DabPacketStats {
        &self.stats
    }

    /// Drop anything half-received (after a loss of lock).
    pub fn reset(&mut self) {
        self.carry.clear();
        self.group.clear();
        self.in_group = false;
        self.next_ci = None;
    }

    /// Take one logical frame's bytes; complete data groups on our address
    /// go to `out` (each one as sent, its own CRC still on the end).
    pub fn push_frame(&mut self, frame: &[u8], out: &mut Vec<Vec<u8>>) {
        // The packets sit on a 24-byte grid from the start of the stream, and
        // a logical frame is a whole number of 24-byte units (n × 8 kbit/s),
        // so a bad packet costs 24 bytes before the next try.
        self.carry.extend_from_slice(frame);
        let buf = std::mem::take(&mut self.carry);
        let mut at = 0;
        while buf.len() - at >= PACKET_LENGTHS[0] {
            let h = PacketHeader::parse(&[buf[at], buf[at + 1], buf[at + 2]]);
            if buf.len() - at < h.len {
                break; // the rest arrives with the next frame
            }
            let p = &buf[at..at + h.len];
            if h.address == FEC_ADDRESS && h.len == PACKET_LENGTHS[0] {
                self.stats.fec += 1;
                at += h.len;
                continue;
            }
            if !crc16_check(p) || h.useful > h.len - OVERHEAD {
                self.stats.packets_bad += 1;
                // Whatever group was in flight on our address has a hole now
                // (the bad packet may have been one of its own).
                self.break_group();
                at += PACKET_LENGTHS[0];
                continue;
            }
            at += h.len;
            self.stats.packets_ok += 1;
            if h.address == PADDING_ADDRESS {
                self.stats.padding += 1;
            } else if h.address == self.address {
                self.stats.packets_ours += 1;
                self.packet(&h, &p[3..3 + h.useful], out);
            }
        }
        self.carry = buf[at..].to_vec();
    }

    fn break_group(&mut self) {
        if self.in_group {
            self.stats.groups_dropped += 1;
        }
        self.in_group = false;
        self.group.clear();
        self.next_ci = None;
    }

    fn packet(&mut self, h: &PacketHeader, data: &[u8], out: &mut Vec<Vec<u8>>) {
        // §5.3.2.1: the continuity index counts the packets on each address
        // modulo 4.
        if self.next_ci.is_some_and(|ci| ci != h.continuity) {
            self.break_group();
        }
        self.next_ci = Some((h.continuity + 1) & 3);
        if h.first {
            if self.in_group {
                self.stats.groups_dropped += 1;
            }
            self.group.clear();
            self.in_group = true;
        } else if !self.in_group {
            return; // the middle of a group whose start we missed
        }
        self.group.extend_from_slice(data);
        if self.group.len() > MAX_GROUP {
            self.break_group();
            return;
        }
        if h.last {
            self.stats.groups += 1;
            out.push(std::mem::take(&mut self.group));
            self.in_group = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::append_crc16;

    fn packet(len_code: u8, ci: u8, first: bool, last: bool, addr: u16, data: &[u8]) -> Vec<u8> {
        let len = PACKET_LENGTHS[usize::from(len_code)];
        let mut p = vec![
            (len_code << 6)
                | (ci << 4)
                | (u8::from(first) << 3)
                | (u8::from(last) << 2)
                | (addr >> 8) as u8,
            addr as u8,
            data.len() as u8,
        ];
        p.extend_from_slice(data);
        p.resize(len - 2, 0);
        append_crc16(&mut p);
        p
    }

    #[test]
    fn groups_across_packets_and_frames() {
        let a: Vec<u8> = (0..60).collect();
        let mut stream = packet(1, 0, true, false, 5, &a[..43]);
        stream.extend(packet(0, 0, false, false, 9, b"other")); // another address
        stream.extend(packet(0, 1, false, true, 5, &a[43..]));
        stream.extend(packet(0, 0, false, false, 0, b"")); // padding
        let mut d = DabPacketDemux::new(5);
        let mut out = Vec::new();
        // Cut the stream mid-packet: the second half comes a frame later.
        d.push_frame(&stream[..30], &mut out);
        assert!(out.is_empty());
        d.push_frame(&stream[30..], &mut out);
        assert_eq!(out, vec![a]);
        assert_eq!(d.stats().packets_ok, 4);
        assert_eq!(d.stats().padding, 1);
    }

    #[test]
    fn a_bad_packet_drops_its_group() {
        let mut stream = packet(0, 0, true, false, 5, b"abc");
        let mut bad = packet(0, 1, false, false, 5, b"def");
        bad[10] ^= 1;
        stream.extend(bad);
        stream.extend(packet(0, 2, false, true, 5, b"ghi"));
        stream.extend(packet(0, 3, true, true, 5, b"whole"));
        let mut d = DabPacketDemux::new(5);
        let mut out = Vec::new();
        d.push_frame(&stream, &mut out);
        assert_eq!(out, vec![b"whole".to_vec()]);
        assert_eq!(d.stats().packets_bad, 1);
        assert_eq!(d.stats().groups_dropped, 1);
    }
}
