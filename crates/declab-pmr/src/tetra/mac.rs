// made by CasualArclamp
//! TETRA downlink PDUs: the MAC (clause 21.4), the basic-link LLC (21.2.2)
//! and, inside, the MLE protocol discriminator (18.4.1) with CMCE call
//! control and short data (14.7.1). Each has a parser and, for the
//! simulator, a writer.
//!
//! Bits are 0/1 bytes, first transmitted first.

/// A cursor over 0/1 bits. Rust note: the lifetime `'a` ties the reader to
/// the slice it borrows, so it cannot outlive the block.
pub struct Bits<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Bits<'a> {
    pub fn new(b: &'a [u8]) -> Self {
        Bits { b, pos: 0 }
    }
    pub fn left(&self) -> usize {
        self.b.len() - self.pos
    }
    pub fn pos(&self) -> usize {
        self.pos
    }
    /// The next `n` bits (n ≤ 32) as a number, or `None` past the end.
    pub fn take(&mut self, n: usize) -> Option<u32> {
        if self.left() < n {
            return None;
        }
        let v = self.b[self.pos..self.pos + n]
            .iter()
            .fold(0u32, |v, &x| (v << 1) | u32::from(x & 1));
        self.pos += n;
        Some(v)
    }
    pub fn flag(&mut self) -> Option<bool> {
        self.take(1).map(|v| v == 1)
    }
    pub fn rest(&self) -> &'a [u8] {
        &self.b[self.pos..]
    }
}

/// Appends numbers as bits, for the simulator.
#[derive(Default)]
pub struct Writer(pub Vec<u8>);

impl Writer {
    pub fn put(&mut self, v: u32, n: usize) -> &mut Self {
        self.0.extend((0..n).rev().map(|i| ((v >> i) & 1) as u8));
        self
    }
    pub fn bits(&mut self, b: &[u8]) -> &mut Self {
        self.0.extend_from_slice(b);
        self
    }
}

/// The SYNC PDU with its D-MLE-SYNC, clause 21.4.4.2 table 21.76 and
/// 18.4.2.1 table 18.17: the 60 bits of the BSCH.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sync {
    pub system_code: u8,
    pub colour: u8,
    /// Timeslot 0–3 (TN 1–4).
    pub tn: u8,
    /// Frame number 1–18.
    pub frame: u8,
    /// Multiframe number 1–60.
    pub multiframe: u8,
    pub sharing: u8,
    pub ts_reserved: u8,
    pub u_plane_dtx: bool,
    pub frame18_ext: bool,
    pub mcc: u16,
    pub mnc: u16,
    pub neighbour_bcast: u8,
    pub cell_load: u8,
    pub late_entry: bool,
}

impl Sync {
    pub fn parse(b: &[u8]) -> Option<Sync> {
        let mut r = Bits::new(b);
        let s = Sync {
            system_code: r.take(4)? as u8,
            colour: r.take(6)? as u8,
            tn: r.take(2)? as u8,
            frame: r.take(5)? as u8,
            multiframe: r.take(6)? as u8,
            sharing: r.take(2)? as u8,
            ts_reserved: r.take(3)? as u8,
            u_plane_dtx: r.flag()?,
            frame18_ext: r.flag()?,
            mcc: {
                r.take(1)?; // reserved
                r.take(10)? as u16
            },
            mnc: r.take(14)? as u16,
            neighbour_bcast: r.take(2)? as u8,
            cell_load: r.take(2)? as u8,
            late_entry: r.flag()?,
        };
        (1..=18).contains(&s.frame).then_some(s)
    }

    pub fn write(&self) -> Vec<u8> {
        let mut w = Writer::default();
        w.put(self.system_code.into(), 4)
            .put(self.colour.into(), 6)
            .put(self.tn.into(), 2)
            .put(self.frame.into(), 5)
            .put(self.multiframe.into(), 6)
            .put(self.sharing.into(), 2)
            .put(self.ts_reserved.into(), 3)
            .put(self.u_plane_dtx.into(), 1)
            .put(self.frame18_ext.into(), 1)
            .put(0, 1)
            .put(self.mcc.into(), 10)
            .put(self.mnc.into(), 14)
            .put(self.neighbour_bcast.into(), 2)
            .put(self.cell_load.into(), 2)
            .put(self.late_entry.into(), 1);
        w.0
    }
}

/// SYSINFO on π/4-DQPSK with its D-MLE-SYSINFO, clause 21.4.4.1 table 21.65
/// and 18.4.2.2 table 18.19: exactly the 124 bits of an SCH/HD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SysInfo {
    pub main_carrier: u16,
    pub band: u8,
    pub offset: u8,
    pub duplex: u8,
    pub reverse: bool,
    pub common_scch: u8,
    pub tx_max: u8,
    pub rx_min: u8,
    pub access_param: u8,
    pub timeout: u8,
    /// The hyperframe number, or (flag set) the cipher key identifier.
    pub hyperframe_or_cck: (bool, u16),
    pub option_flag: u8,
    pub option: u32,
    pub la: u16,
    pub subscriber_class: u16,
    /// BS service details, table 18.26 (12 bits).
    pub services: u16,
}

impl SysInfo {
    /// Parsed from after the MAC PDU type and broadcast type.
    fn parse(r: &mut Bits) -> Option<SysInfo> {
        Some(SysInfo {
            main_carrier: r.take(12)? as u16,
            band: r.take(4)? as u8,
            offset: r.take(2)? as u8,
            duplex: r.take(3)? as u8,
            reverse: r.flag()?,
            common_scch: r.take(2)? as u8,
            tx_max: r.take(3)? as u8,
            rx_min: r.take(4)? as u8,
            access_param: r.take(4)? as u8,
            timeout: r.take(4)? as u8,
            hyperframe_or_cck: (r.flag()?, r.take(16)? as u16),
            option_flag: r.take(2)? as u8,
            option: r.take(20)?,
            la: r.take(14)? as u16,
            subscriber_class: r.take(16)? as u16,
            services: r.take(12)? as u16,
        })
    }

    pub fn write(&self) -> Vec<u8> {
        let mut w = Writer::default();
        w.put(0b10, 2)
            .put(0b00, 2)
            .put(self.main_carrier.into(), 12)
            .put(self.band.into(), 4)
            .put(self.offset.into(), 2)
            .put(self.duplex.into(), 3)
            .put(self.reverse.into(), 1)
            .put(self.common_scch.into(), 2)
            .put(self.tx_max.into(), 3)
            .put(self.rx_min.into(), 4)
            .put(self.access_param.into(), 4)
            .put(self.timeout.into(), 4)
            .put(self.hyperframe_or_cck.0.into(), 1)
            .put(self.hyperframe_or_cck.1.into(), 16)
            .put(self.option_flag.into(), 2)
            .put(self.option, 20)
            .put(self.la.into(), 14)
            .put(self.subscriber_class.into(), 16)
            .put(self.services.into(), 12);
        w.0
    }

    /// The main carrier's downlink frequency.
    pub fn main_carrier_hz(&self) -> f64 {
        super::carrier_hz(self.band, self.main_carrier, self.offset)
    }

    /// Air interface encryption available on the cell (table 18.26).
    pub fn encryption(&self) -> bool {
        self.services >> 1 & 1 == 1
    }
}

/// ACCESS-ASSIGN, the AACH's 14 bits, clause 21.4.7.2 tables 21.82/21.83.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AccessAssign {
    pub header: u8,
    pub field1: u8,
    pub field2: u8,
}

impl AccessAssign {
    pub fn from_bits(d: u16) -> Self {
        AccessAssign {
            header: (d >> 12) as u8 & 3,
            field1: (d >> 6) as u8 & 0x3f,
            field2: d as u8 & 0x3f,
        }
    }
    pub fn to_bits(self) -> u16 {
        (u16::from(self.header) << 12) | (u16::from(self.field1) << 6) | u16::from(self.field2)
    }
    /// The downlink traffic usage marker, in frames 1–17: field 1 when the
    /// header is not 00 and it is not one of the four preset markers.
    pub fn traffic(self, frame: u8) -> Option<u8> {
        (frame != 18 && self.header != 0 && self.field1 >= 4).then_some(self.field1)
    }
}

/// A MAC address, table 21.55's address types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Address {
    Ssi(u32),
    EventLabel(u16),
    Ussi(u32),
    Smi(u32),
    SsiEventLabel(u32, u16),
    SsiUsageMarker(u32, u8),
    SmiEventLabel(u32, u16),
}

impl Address {
    pub fn ssi(self) -> Option<u32> {
        match self {
            Address::Ssi(s)
            | Address::Ussi(s)
            | Address::SsiEventLabel(s, _)
            | Address::SsiUsageMarker(s, _) => Some(s),
            _ => None,
        }
    }
}

/// The parts of a channel allocation the receiver uses, clause 21.5.2
/// table 21.87.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChanAlloc {
    pub alloc_type: u8,
    /// Bit 3 = timeslot 1 … bit 0 = timeslot 4; 0 = the control channel.
    pub timeslots: u8,
    pub up_down: u8,
    pub carrier: u16,
    pub ext: Option<(u8, u8, u8, bool)>,
}

impl ChanAlloc {
    fn parse(r: &mut Bits) -> Option<ChanAlloc> {
        let alloc_type = r.take(2)? as u8;
        let timeslots = r.take(4)? as u8;
        let up_down = r.take(2)? as u8;
        r.take(1)?; // CLCH permission
        r.take(1)?; // cell change
        let carrier = r.take(12)? as u16;
        let ext = if r.flag()? {
            Some((
                r.take(4)? as u8,
                r.take(2)? as u8,
                r.take(3)? as u8,
                r.flag()?,
            ))
        } else {
            None
        };
        let monitoring = r.take(2)?;
        if monitoring == 0 {
            r.take(2)?; // frame 18 monitoring pattern
        }
        // Augmented allocations (up/downlink 00) carry much more; not
        // followed here (the caller drops the TM-SDU).
        (up_down != 0).then_some(ChanAlloc {
            alloc_type,
            timeslots,
            up_down,
            carrier,
            ext,
        })
    }

    fn write(&self, w: &mut Writer) {
        w.put(self.alloc_type.into(), 2)
            .put(self.timeslots.into(), 4)
            .put(self.up_down.into(), 2)
            .put(0, 2)
            .put(self.carrier.into(), 12)
            .put(0, 1)
            .put(1, 2); // one monitoring pattern
    }
}

/// A MAC PDU found in a downlink signalling block.
#[derive(Debug, Clone, PartialEq)]
pub enum Pdu {
    /// Address type 000: nothing follows in the block.
    Null,
    /// MAC-RESOURCE, table 21.55. `sdu` is the TM-SDU (empty if none);
    /// `frag` marks the start of a fragmented one.
    Resource {
        address: Address,
        encryption: u8,
        chan: Option<ChanAlloc>,
        sdu: Vec<u8>,
        frag: bool,
        /// On STCH: the second half slot is stolen too.
        second_half_stolen: bool,
    },
    /// MAC-FRAG's TM-SDU piece.
    Frag(Vec<u8>),
    /// MAC-END's last piece, table 21.59 (downlink).
    End {
        chan: Option<ChanAlloc>,
        sdu: Vec<u8>,
    },
    SysInfo(SysInfo),
    AccessDefine,
    /// MAC-D-BLCK, U-plane signalling or anything not followed.
    Other(u8),
}

/// Strip fill bits (a 1 then 0s, clause 23.4.3.2) from the end.
fn unfill(b: &[u8]) -> &[u8] {
    match b.iter().rposition(|&x| x == 1) {
        Some(i) => &b[..i],
        None => b,
    }
}

/// All MAC PDUs in a signalling block's type-1 bits (SCH/F 268, SCH/HD,
/// BNCH or STCH 124), in order, π/4-DQPSK rules (Y2 = Z2 = 1 octet).
pub fn parse_block(b: &[u8]) -> Vec<Pdu> {
    let mut out = Vec::new();
    let mut at = 0;
    while b.len() - at >= 16 {
        let blk = &b[at..];
        let mut r = Bits::new(blk);
        let Some(kind) = r.take(2) else { break };
        match kind {
            0b00 => {
                let Some((pdu, len)) = resource(&mut r, blk) else {
                    break;
                };
                let null = pdu == Pdu::Null;
                out.push(pdu);
                if null || len == 0 {
                    break;
                }
                at += len;
            }
            0b01 => {
                // MAC-FRAG (subtype 0) runs to the end of the block.
                let sub = r.take(1).unwrap_or(0);
                let fill = r.flag().unwrap_or(false);
                if sub == 0 {
                    let rest = r.rest();
                    out.push(Pdu::Frag(if fill { unfill(rest) } else { rest }.to_vec()));
                    break;
                }
                match end(&mut r, blk, fill) {
                    Some((pdu, len)) => {
                        out.push(pdu);
                        at += len.max(blk.len());
                    }
                    None => break,
                }
            }
            0b10 => {
                match r.take(2) {
                    Some(0b00) => {
                        if let Some(s) = SysInfo::parse(&mut r) {
                            out.push(Pdu::SysInfo(s));
                        }
                    }
                    Some(0b01) => out.push(Pdu::AccessDefine),
                    Some(t) => out.push(Pdu::Other(0b1000 | t as u8)),
                    None => {}
                }
                break;
            }
            _ => {
                out.push(Pdu::Other(0b11));
                break;
            }
        }
    }
    out
}

/// The MAC PDU length in bits for a length indication, table 21.55 with
/// Y2 = Z2 = 1: octets up to 0b111010.
fn li_bits(li: u32) -> Option<usize> {
    (1..=0b111010).contains(&li).then_some(8 * li as usize)
}

fn address(r: &mut Bits, kind: u32) -> Option<Address> {
    Some(match kind {
        0b001 => Address::Ssi(r.take(24)?),
        0b010 => Address::EventLabel(r.take(10)? as u16),
        0b011 => Address::Ussi(r.take(24)?),
        0b100 => Address::Smi(r.take(24)?),
        0b101 => Address::SsiEventLabel(r.take(24)?, r.take(10)? as u16),
        0b110 => Address::SsiUsageMarker(r.take(24)?, r.take(6)? as u8),
        0b111 => Address::SmiEventLabel(r.take(24)?, r.take(10)? as u16),
        _ => return None,
    })
}

/// MAC-RESOURCE after its type bits: the PDU and its length in bits (0 when
/// it runs to the end of the block).
fn resource(r: &mut Bits, blk: &[u8]) -> Option<(Pdu, usize)> {
    let fill = r.flag()?;
    r.take(1)?; // position of grant
    let encryption = r.take(2)? as u8;
    r.take(1)?; // random access flag
    let li = r.take(6)?;
    let at = r.take(3)?;
    if at == 0 {
        return Some((Pdu::Null, 16));
    }
    let address = address(r, at)?;
    if r.flag()? {
        r.take(4)?; // power control
    }
    if r.flag()? {
        r.take(8)?; // basic slot granting
    }
    let chan = if r.flag()? {
        Some(ChanAlloc::parse(r)?)
    } else {
        None
    };
    let (len, frag, second) = match li {
        0b111111 => (blk.len(), true, false),
        0b111110 => (blk.len(), false, true),
        _ => (li_bits(li)?.min(blk.len()), false, false),
    };
    if len < r.pos() {
        return None;
    }
    let body = &blk[r.pos()..len];
    let sdu = if fill { unfill(body) } else { body };
    Some((
        Pdu::Resource {
            address,
            encryption,
            chan,
            sdu: sdu.to_vec(),
            frag,
            second_half_stolen: second,
        },
        if frag || second { 0 } else { len },
    ))
}

fn end(r: &mut Bits, blk: &[u8], fill: bool) -> Option<(Pdu, usize)> {
    r.take(1)?; // position of grant
    let li = r.take(6)?;
    if r.flag()? {
        r.take(8)?;
    }
    let chan = if r.flag()? {
        Some(ChanAlloc::parse(r)?)
    } else {
        None
    };
    let len = li_bits(li).map_or(blk.len(), |l| l.min(blk.len()));
    if len < r.pos() {
        return None;
    }
    let body = &blk[r.pos()..len];
    let sdu = if fill { unfill(body) } else { body };
    Some((
        Pdu::End {
            chan,
            sdu: sdu.to_vec(),
        },
        len,
    ))
}

/// Write a MAC-RESOURCE with an SSI-type address and the TM-SDU, padded
/// with fill bits to whole octets; for the simulator.
pub fn write_resource(address: Address, chan: Option<&ChanAlloc>, sdu: &[u8]) -> Vec<u8> {
    let mut h = Writer::default();
    let (kind, a, an) = match address {
        Address::Ssi(s) => (0b001, u64::from(s), 24),
        Address::EventLabel(e) => (0b010, u64::from(e), 10),
        Address::Ussi(s) => (0b011, u64::from(s), 24),
        Address::Smi(s) => (0b100, u64::from(s), 24),
        Address::SsiEventLabel(s, e) => (0b101, (u64::from(s) << 10) | u64::from(e), 34),
        Address::SsiUsageMarker(s, u) => (0b110, (u64::from(s) << 6) | u64::from(u), 30),
        Address::SmiEventLabel(s, e) => (0b111, (u64::from(s) << 10) | u64::from(e), 34),
    };
    let mut body = Writer::default();
    body.put(kind, 3);
    for i in (0..an).rev() {
        body.put(((a >> i) & 1) as u32, 1);
    }
    body.put(0, 1).put(0, 1); // no power control, no slot grant
    match chan {
        Some(c) => {
            body.put(1, 1);
            c.write(&mut body);
        }
        None => {
            body.put(0, 1);
        }
    }
    body.bits(sdu);
    let total = 13 + body.0.len();
    let octets = total.div_ceil(8);
    let fill = octets * 8 > total;
    h.put(0b00, 2)
        .put(fill.into(), 1)
        .put(0, 1)
        .put(0, 2)
        .put(0, 1)
        .put(octets as u32, 6)
        .bits(&body.0);
    if fill {
        h.put(1, 1);
        while h.0.len() < octets * 8 {
            h.put(0, 1);
        }
    }
    h.0
}

/// The null PDU (16 bits) then fill to `n` bits.
pub fn write_null(n: usize) -> Vec<u8> {
    let mut w = Writer::default();
    w.put(0b00, 2).put(1, 1).put(0, 4).put(2, 6).put(0, 3);
    w.put(1, 1);
    while w.0.len() < n {
        w.put(0, 1);
    }
    w.0
}

/// Pad a block's PDUs with fill bits to its size.
pub fn pad(mut b: Vec<u8>, n: usize) -> Vec<u8> {
    if b.len() < n {
        b.push(1);
        b.resize(n, 0);
    }
    b.truncate(n);
    b
}

/// Upper-layer content of a TM-SDU: what the receiver reports.
#[derive(Debug, Clone, PartialEq)]
pub enum Signal {
    /// D-SETUP, table 14.15 (`service` is the basic service information).
    Setup {
        call: u16,
        service: u8,
        priority: u8,
        calling: Option<u32>,
    },
    Connect {
        call: u16,
    },
    /// D-TX GRANTED, table 14.18.
    TxGranted {
        call: u16,
        grant: u8,
        encrypted: bool,
        party: Option<u32>,
    },
    TxCeased {
        call: u16,
    },
    Release {
        call: u16,
        cause: u8,
    },
    /// D-STATUS, table 14.14.
    Status {
        from: Option<u32>,
        status: u16,
    },
    /// D-SDS-DATA, table 14.13: the user data, and its length in bits.
    Sds {
        from: Option<u32>,
        kind: u8,
        data: Vec<u8>,
        bits: usize,
    },
    /// Any other CMCE PDU, by type (table 14.66).
    Cmce(u8),
    /// MM (protocol discriminator 001), by PDU type.
    Mm(u8),
    /// MLE (101), by PDU type.
    Mle(u8),
    /// Another protocol discriminator.
    Other(u8),
}

/// Bits to bytes, the last byte padded with zeros.
fn bytes(b: &[u8]) -> Vec<u8> {
    b.chunks(8)
        .map(|c| {
            c.iter()
                .enumerate()
                .fold(0u8, |v, (i, &x)| v | (x << (7 - i)))
        })
        .collect()
}

/// The calling/transmitting party after a party type identifier (CPTI 1:
/// SSI; 2: SSI and extension).
fn party(r: &mut Bits, cpti: u32) -> Option<Option<u32>> {
    Some(match cpti {
        1 => Some(r.take(24)?),
        2 => {
            let s = r.take(24)?;
            r.take(24)?;
            Some(s)
        }
        _ => None,
    })
}

/// Parse a TM-SDU: the LLC basic-link header (table 21.1), then the MLE
/// protocol discriminator (18.4.1.1) and the PDU.
pub fn parse_sdu(sdu: &[u8]) -> Option<Signal> {
    let mut r = Bits::new(sdu);
    let llc = r.take(4)?;
    match llc {
        0b0000 | 0b0100 => {
            r.take(2)?;
        }
        0b0001 | 0b0101 | 0b0011 | 0b0111 => {
            r.take(1)?;
        }
        0b0010 | 0b0110 => {}
        // Advanced link and the rest: not followed.
        _ => return None,
    }
    // A basic-link FCS (32 bits) at the end is ignored: the MAC's CRC has
    // already checked the block.
    let pd = r.take(3)?;
    match pd {
        0b010 => cmce(&mut r),
        0b001 => Some(Signal::Mm(r.take(4)? as u8)),
        0b101 => Some(Signal::Mle(r.take(3)? as u8)),
        p => Some(Signal::Other(p as u8)),
    }
}

/// CMCE downlink PDUs, table 14.66 types.
fn cmce(r: &mut Bits) -> Option<Signal> {
    let t = r.take(5)?;
    Some(match t {
        0b00111 => {
            let call = r.take(14)? as u16;
            r.take(4 + 1 + 1)?; // time-out, hook, simplex
            let service = r.take(8)? as u8;
            r.take(2 + 1)?; // transmission grant, request permission
            let priority = r.take(4)? as u8;
            let mut calling = None;
            if r.flag() == Some(true) {
                if r.flag()? {
                    r.take(6)?;
                }
                if r.flag()? {
                    r.take(24)?;
                }
                if r.flag()? {
                    let cpti = r.take(2)?;
                    calling = party(r, cpti)?;
                }
            }
            Signal::Setup {
                call,
                service,
                priority,
                calling,
            }
        }
        0b00010 => Signal::Connect {
            call: r.take(14)? as u16,
        },
        0b01011 => {
            let call = r.take(14)? as u16;
            let grant = r.take(2)? as u8;
            r.take(1)?;
            let encrypted = r.flag()?;
            r.take(1)?;
            let mut p = None;
            if r.flag() == Some(true) {
                if r.flag()? {
                    r.take(6)?;
                }
                if r.flag()? {
                    let tpti = r.take(2)?;
                    p = party(r, tpti)?;
                }
            }
            Signal::TxGranted {
                call,
                grant,
                encrypted,
                party: p,
            }
        }
        0b01001 => Signal::TxCeased {
            call: r.take(14)? as u16,
        },
        0b00110 => Signal::Release {
            call: r.take(14)? as u16,
            cause: r.take(5)? as u8,
        },
        0b01000 => {
            let cpti = r.take(2)?;
            let from = party(r, cpti)?;
            Signal::Status {
                from,
                status: r.take(16)? as u16,
            }
        }
        0b01111 => {
            let cpti = r.take(2)?;
            let from = party(r, cpti)?;
            let kind = r.take(2)? as u8;
            let n = match kind {
                0 => 16,
                1 => 32,
                2 => 64,
                _ => r.take(11)? as usize,
            };
            if r.left() < n {
                return None;
            }
            let d = &r.rest()[..n];
            Signal::Sds {
                from,
                kind,
                data: bytes(d),
                bits: n,
            }
        }
        t => Signal::Cmce(t as u8),
    })
}

/// A TM-SDU for a CMCE PDU in BL-UDATA, for the simulator.
pub fn write_signal(s: &Signal) -> Vec<u8> {
    let mut w = Writer::default();
    w.put(0b0010, 4).put(0b010, 3);
    let party = |w: &mut Writer, p: Option<u32>| match p {
        Some(s) => {
            w.put(1, 2).put(s, 24);
        }
        None => {
            w.put(0, 2);
        }
    };
    match *s {
        Signal::Setup {
            call,
            service,
            priority,
            calling,
        } => {
            w.put(0b00111, 5)
                .put(call.into(), 14)
                .put(0, 6)
                .put(service.into(), 8)
                .put(0, 3)
                .put(priority.into(), 4);
            match calling {
                Some(c) => {
                    w.put(1, 1)
                        .put(0, 1)
                        .put(0, 1)
                        .put(1, 1)
                        .put(1, 2)
                        .put(c, 24);
                    w.put(0, 1); // no type 3 elements
                }
                None => {
                    w.put(0, 1);
                }
            }
        }
        Signal::Connect { call } => {
            w.put(0b00010, 5).put(call.into(), 14).put(0, 11).put(0, 1);
        }
        Signal::TxGranted {
            call,
            grant,
            encrypted,
            party: p,
        } => {
            w.put(0b01011, 5)
                .put(call.into(), 14)
                .put(grant.into(), 2)
                .put(0, 1)
                .put(encrypted.into(), 1)
                .put(0, 1);
            match p {
                Some(s) => {
                    w.put(1, 1)
                        .put(0, 1)
                        .put(1, 1)
                        .put(1, 2)
                        .put(s, 24)
                        .put(0, 1);
                }
                None => {
                    w.put(0, 1);
                }
            }
        }
        Signal::TxCeased { call } => {
            w.put(0b01001, 5).put(call.into(), 14).put(0, 1).put(0, 1);
        }
        Signal::Release { call, cause } => {
            w.put(0b00110, 5)
                .put(call.into(), 14)
                .put(cause.into(), 5)
                .put(0, 1);
        }
        Signal::Status { from, status } => {
            w.put(0b01000, 5);
            party(&mut w, from);
            w.put(status.into(), 16).put(0, 1);
        }
        Signal::Sds {
            from,
            kind,
            ref data,
            bits,
        } => {
            w.put(0b01111, 5);
            party(&mut w, from);
            w.put(kind.into(), 2);
            if kind == 3 {
                w.put(bits as u32, 11);
            }
            for i in 0..bits {
                w.put(u32::from(data[i / 8] >> (7 - i % 8) & 1), 1);
            }
            w.put(0, 1);
        }
        _ => {}
    }
    w.0
}

/// A short name for a CMCE PDU type (table 14.66), MM or MLE type.
pub fn signal_name(s: &Signal) -> String {
    const CMCE: [&str; 17] = [
        "D-ALERT",
        "D-CALL-PROCEEDING",
        "D-CONNECT",
        "D-CONNECT-ACK",
        "D-DISCONNECT",
        "D-INFO",
        "D-RELEASE",
        "D-SETUP",
        "D-STATUS",
        "D-TX-CEASED",
        "D-TX-CONTINUE",
        "D-TX-GRANTED",
        "D-TX-WAIT",
        "D-TX-INTERRUPT",
        "D-CALL-RESTORE",
        "D-SDS-DATA",
        "D-FACILITY",
    ];
    match s {
        Signal::Cmce(t) => CMCE
            .get(*t as usize)
            .map_or_else(|| format!("CMCE {t}"), |n| (*n).to_string()),
        Signal::Mm(t) => format!("MM type {t}"),
        Signal::Mle(t) => format!("MLE type {t}"),
        Signal::Other(p) => format!("protocol {p}"),
        Signal::Setup { .. } => "D-SETUP".into(),
        Signal::Connect { .. } => "D-CONNECT".into(),
        Signal::TxGranted { .. } => "D-TX-GRANTED".into(),
        Signal::TxCeased { .. } => "D-TX-CEASED".into(),
        Signal::Release { .. } => "D-RELEASE".into(),
        Signal::Status { .. } => "D-STATUS".into(),
        Signal::Sds { .. } => "D-SDS-DATA".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_round_trip() {
        let s = Sync {
            system_code: 3,
            colour: 17,
            tn: 2,
            frame: 18,
            multiframe: 33,
            sharing: 0,
            ts_reserved: 0,
            u_plane_dtx: false,
            frame18_ext: true,
            mcc: 234,
            mnc: 1234,
            neighbour_bcast: 2,
            cell_load: 1,
            late_entry: true,
        };
        let b = s.write();
        assert_eq!(b.len(), 60);
        assert_eq!(Sync::parse(&b), Some(s));
    }

    #[test]
    fn sysinfo_fills_a_half_slot() {
        let s = SysInfo {
            main_carrier: 1234,
            band: 4,
            offset: 1,
            duplex: 2,
            reverse: false,
            common_scch: 0,
            tx_max: 5,
            rx_min: 3,
            access_param: 7,
            timeout: 1,
            hyperframe_or_cck: (false, 4321),
            option_flag: 2,
            option: 0xabcde,
            la: 99,
            subscriber_class: 0xffff,
            services: 0b1111_1110_1011,
        };
        let b = s.write();
        assert_eq!(b.len(), 124);
        assert_eq!(parse_block(&b), vec![Pdu::SysInfo(s)]);
        assert!((s.main_carrier_hz() - 430_856_250.0).abs() < 1.0);
    }

    #[test]
    fn resource_with_setup() {
        let sig = Signal::Setup {
            call: 1234,
            service: 0,
            priority: 3,
            calling: Some(4_000_123),
        };
        let chan = ChanAlloc {
            alloc_type: 0,
            timeslots: 0b0100,
            up_down: 3,
            carrier: 1234,
            ext: None,
        };
        let mut blk = write_resource(
            Address::SsiUsageMarker(9001, 7),
            Some(&chan),
            &write_signal(&sig),
        );
        blk.extend(write_resource(
            Address::Ssi(77),
            None,
            &write_signal(&Signal::Status {
                from: Some(5),
                status: 0x8001,
            }),
        ));
        let blk = pad(blk, 268);
        let p = parse_block(&blk);
        assert_eq!(p.len(), 2, "{p:?}");
        match &p[0] {
            Pdu::Resource {
                address,
                chan: c,
                sdu,
                ..
            } => {
                assert_eq!(*address, Address::SsiUsageMarker(9001, 7));
                assert_eq!(*c, Some(chan));
                assert_eq!(parse_sdu(sdu), Some(sig));
            }
            x => panic!("{x:?}"),
        }
        match &p[1] {
            Pdu::Resource { sdu, .. } => assert_eq!(
                parse_sdu(sdu),
                Some(Signal::Status {
                    from: Some(5),
                    status: 0x8001
                })
            ),
            x => panic!("{x:?}"),
        }
    }

    #[test]
    fn sds_and_grant_round_trip() {
        for s in [
            Signal::Sds {
                from: Some(123),
                kind: 3,
                data: [0x82, 4, 104, 101, 108, 108, 111].to_vec(),
                bits: 56,
            },
            Signal::TxGranted {
                call: 5,
                grant: 0,
                encrypted: false,
                party: Some(42),
            },
            Signal::Release { call: 5, cause: 1 },
        ] {
            let blk = pad(
                write_resource(Address::Ssi(1), None, &write_signal(&s)),
                268,
            );
            match &parse_block(&blk)[0] {
                Pdu::Resource { sdu, .. } => assert_eq!(parse_sdu(sdu), Some(s.clone())),
                x => panic!("{x:?}"),
            }
        }
    }

    #[test]
    fn null_pdu() {
        assert_eq!(parse_block(&write_null(124)), vec![Pdu::Null]);
    }
}
