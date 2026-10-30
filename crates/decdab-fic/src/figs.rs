// made by CasualArclamp
//! FIG parsing (EN 300 401 §5.2.2, §6, §8): FIB data fields into the
//! [`Ensemble`] model.
//!
//! Only the current configuration of this ensemble is taken: FIG 0s with
//! the C/N flag set (the next configuration) or the OE flag set (other
//! ensembles) are counted and skipped.

use decdab_core::charset;
use decdab_fec::EepSet;

use crate::ensemble::{
    Component, DabTime, Ensemble, Label, PacketComponent, Protection, Service, SubChannel,
    Transport, UserApp,
};

/// FIGs seen, by type and extension, for the diagnostics.
#[derive(Debug, Clone, Default)]
pub struct FigCounts {
    pub fig0: [u64; 32],
    pub fig1: [u64; 8],
    pub fig2: [u64; 8],
    pub other: u64,
    pub skipped: u64,
    pub malformed: u64,
}

/// A big-endian bit reader over a FIG's data field.
struct Bits<'a> {
    d: &'a [u8],
    pos: usize,
}

impl<'a> Bits<'a> {
    fn new(d: &'a [u8]) -> Self {
        Bits { d, pos: 0 }
    }

    fn left(&self) -> usize {
        self.d.len() * 8 - self.pos
    }

    /// The next `n` (≤ 32) bits, or `None` past the end.
    fn get(&mut self, n: usize) -> Option<u32> {
        if n > self.left() {
            return None;
        }
        let mut v = 0u32;
        for _ in 0..n {
            let b = (self.d[self.pos / 8] >> (7 - self.pos % 8)) & 1;
            v = (v << 1) | b as u32;
            self.pos += 1;
        }
        Some(v)
    }

    fn bytes(&mut self, n: usize) -> Option<&'a [u8]> {
        debug_assert!(self.pos.is_multiple_of(8));
        let s = self.pos / 8;
        let out = self.d.get(s..s + n)?;
        self.pos += 8 * n;
        Some(out)
    }
}

/// Parse one FIB's 30 data bytes.
pub fn parse_fib(data: &[u8], ens: &mut Ensemble, counts: &mut FigCounts) {
    let mut i = 0;
    while i < data.len() {
        let h = data[i];
        if h == 0xFF {
            break; // end marker, padding after
        }
        let ty = h >> 5;
        let len = (h & 31) as usize;
        let Some(body) = data.get(i + 1..i + 1 + len) else {
            counts.malformed += 1;
            break;
        };
        let ok = match ty {
            0 => fig0(body, ens, counts),
            1 => fig1(body, ens, counts),
            2 => {
                if let Some(&b) = body.first() {
                    counts.fig2[(b & 7) as usize] += 1;
                }
                Some(())
            }
            _ => {
                counts.other += 1;
                Some(())
            }
        };
        if ok.is_none() {
            counts.malformed += 1;
        }
        i += 1 + len;
    }
}

fn fig0(body: &[u8], ens: &mut Ensemble, counts: &mut FigCounts) -> Option<()> {
    let mut b = Bits::new(body);
    let cn = b.get(1)? == 1;
    let oe = b.get(1)? == 1;
    let pd = b.get(1)? == 1;
    let ext = b.get(5)? as usize;
    counts.fig0[ext] += 1;
    if oe || (cn && matches!(ext, 1 | 2 | 3 | 8 | 13 | 14)) {
        counts.skipped += 1;
        return Some(());
    }
    match ext {
        0 => {
            // §6.4: EId, change flags, Al flag, CIF count.
            let eid = b.get(16)? as u16;
            let _change = b.get(2)?;
            let _al = b.get(1)?;
            let hi = b.get(5)? as u16;
            let lo = b.get(8)? as u16;
            ens.eid = Some(eid);
            ens.cif_count = Some(hi * 250 + lo);
        }
        1 => {
            // §6.2.1: sub-channels, short (UEP) or long (EEP) form.
            while b.left() >= 24 {
                let id = b.get(6)? as u8;
                let start = b.get(10)? as u16;
                let long = b.get(1)? == 1;
                let (size, protection) = if !long {
                    let _switch = b.get(1)?;
                    let index = b.get(6)? as u8;
                    let size = decdab_fec::conv::UEP_TABLE.get(index as usize)?.0;
                    (size, Protection::Uep(index))
                } else {
                    let option = b.get(3)?;
                    let level = b.get(2)? as u8 + 1;
                    let size = b.get(10)? as u16;
                    let set = match option {
                        0 => EepSet::A,
                        1 => EepSet::B,
                        _ => return None,
                    };
                    (size, Protection::Eep(set, level))
                };
                let fec_scheme = ens.subchannels.get(&id).map_or(0, |s| s.fec_scheme);
                ens.subchannels.insert(
                    id,
                    SubChannel {
                        id,
                        start,
                        size,
                        protection,
                        fec_scheme,
                    },
                );
            }
        }
        2 => {
            // §6.3.1: services and their components.
            while b.left() >= if pd { 40 } else { 24 } {
                let sid = b.get(if pd { 32 } else { 16 })?;
                let _local = b.get(1)?;
                let _caid = b.get(3)?;
                let n = b.get(4)? as usize;
                let mut comps = Vec::with_capacity(n);
                for _ in 0..n {
                    let tmid = b.get(2)?;
                    let a = b.get(6)? as u8;
                    let transport = match tmid {
                        0 => Transport::Audio {
                            ascty: a,
                            subch: b.get(6)? as u8,
                        },
                        1 => Transport::StreamData {
                            dscty: a,
                            subch: b.get(6)? as u8,
                        },
                        2 => {
                            let _rfa = b.get(6)?;
                            Transport::Fidc { fidcid: a }
                        }
                        _ => Transport::Packet {
                            scid: ((a as u16) << 6) | b.get(6)? as u16,
                        },
                    };
                    let primary = b.get(1)? == 1;
                    let ca = b.get(1)? == 1;
                    comps.push((transport, primary, ca));
                }
                let svc = ens.services.entry(sid).or_insert_with(|| Service {
                    sid,
                    ..Default::default()
                });
                for (transport, primary, ca) in comps {
                    match svc.components.iter_mut().find(|c| c.transport == transport) {
                        Some(c) => {
                            c.primary = primary;
                            c.ca = ca;
                        }
                        None => svc.components.push(Component {
                            transport,
                            primary,
                            ca,
                            scids: if primary { Some(0) } else { None },
                            label: None,
                            apps: Vec::new(),
                        }),
                    }
                }
            }
        }
        3 => {
            // §6.3.2: packet-mode components.
            while b.left() >= 40 {
                let scid = b.get(12)? as u16;
                let _rfa = b.get(3)?;
                let caorg = b.get(1)? == 1;
                let dg = b.get(1)? == 1;
                let _rfu = b.get(1)?;
                let dscty = b.get(6)? as u8;
                let subch = b.get(6)? as u8;
                let address = b.get(10)? as u16;
                if caorg {
                    b.get(16)?;
                }
                ens.packet.insert(
                    scid,
                    PacketComponent {
                        scid,
                        subch,
                        address,
                        dscty,
                        data_groups: !dg,
                    },
                );
            }
        }
        8 => {
            // §6.3.5: global definition: which SCIdS a component has.
            while b.left() >= if pd { 40 } else { 24 } {
                let sid = b.get(if pd { 32 } else { 16 })?;
                let ext_flag = b.get(1)? == 1;
                let _rfa = b.get(3)?;
                let scids = b.get(4)? as u8;
                let long = b.get(1)? == 1;
                let target = if !long {
                    let fic = b.get(1)? == 1;
                    let id = b.get(6)? as u8;
                    (!fic).then_some(Err(id))
                } else {
                    let _rfa = b.get(3)?;
                    Some(Ok(b.get(12)? as u16))
                };
                if ext_flag {
                    b.get(8)?;
                }
                let Some(svc) = ens.services.get_mut(&sid) else {
                    continue;
                };
                for c in &mut svc.components {
                    let hit = match (target, c.transport) {
                        (Some(Err(id)), Transport::Audio { subch, .. })
                        | (Some(Err(id)), Transport::StreamData { subch, .. }) => id == subch,
                        (Some(Ok(s)), Transport::Packet { scid }) => s == scid,
                        _ => false,
                    };
                    if hit {
                        c.scids = Some(scids);
                    }
                }
            }
        }
        9 => {
            // §8.1.3.2: ensemble LTO and ECC.
            let _ext = b.get(1)?;
            let _rfa = b.get(1)?;
            let sign = b.get(1)?;
            let half = b.get(5)? as i8;
            let ecc = b.get(8)? as u8;
            ens.lto_half_hours = Some(if sign == 1 { -half } else { half });
            ens.ecc = Some(ecc);
        }
        10 => {
            // §8.1.3.1: date and time.
            let _rfu = b.get(1)?;
            let mjd = b.get(17)?;
            let _lsi = b.get(1)?;
            let _rfa = b.get(1)?;
            let long = b.get(1)? == 1;
            let hours = b.get(5)? as u8;
            let minutes = b.get(6)? as u8;
            let seconds = if long { Some(b.get(6)? as u8) } else { None };
            ens.time = Some(DabTime {
                mjd,
                hours,
                minutes,
                seconds,
            });
        }
        13 => {
            // §6.3.6: user applications of a component.
            while b.left() >= if pd { 40 } else { 24 } {
                let sid = b.get(if pd { 32 } else { 16 })?;
                let scids = b.get(4)? as u8;
                let n = b.get(4)? as usize;
                let mut apps = Vec::with_capacity(n);
                for _ in 0..n {
                    let kind = b.get(11)? as u16;
                    let len = b.get(5)? as usize;
                    apps.push(UserApp {
                        kind,
                        data: b.bytes(len)?.to_vec(),
                    });
                }
                if let Some(c) = ens
                    .services
                    .get_mut(&sid)
                    .and_then(|s| s.components.iter_mut().find(|c| c.scids == Some(scids)))
                {
                    c.apps = apps;
                }
            }
        }
        14 => {
            // §6.2.2: packet-mode FEC.
            while b.left() >= 8 {
                let id = b.get(6)? as u8;
                let scheme = b.get(2)? as u8;
                if let Some(s) = ens.subchannels.get_mut(&id) {
                    s.fec_scheme = scheme;
                }
            }
        }
        17 => {
            // §8.1.5: programme type (the v1 layout; the flags v2 dropped
            // are sent as zero).
            while b.left() >= 32 {
                let sid = b.get(16)?;
                let _sd = b.get(1)?;
                let _ps = b.get(1)?;
                let l = b.get(1)? == 1;
                let cc = b.get(1)? == 1;
                let _rfa = b.get(4)?;
                if l {
                    b.get(8)?;
                }
                let _rfa2 = b.get(3)?;
                let pty = b.get(5)? as u8;
                if cc {
                    b.get(8)?;
                }
                if let Some(s) = ens.services.get_mut(&sid) {
                    s.pty = Some(pty);
                }
            }
        }
        _ => {}
    }
    Some(())
}

/// A FIG 1 label field: 16 bytes and the character flags.
fn label(charset_id: u8, b: &mut Bits) -> Option<Label> {
    let raw = b.bytes(16)?;
    let flags = b.get(16)? as u16;
    let text = charset::decode(charset_id, raw);
    // The short label: the characters whose flag is set (bit 15 is the
    // first character). Taken from the raw bytes so a multi-byte charset
    // keeps its alignment.
    let short_raw: Vec<u8> = raw
        .iter()
        .enumerate()
        .filter(|(i, _)| flags & (0x8000 >> i) != 0)
        .map(|(_, &c)| c)
        .collect();
    Some(Label {
        text: text.trim_end().to_string(),
        short: charset::decode(charset_id, &short_raw).trim().to_string(),
    })
}

fn fig1(body: &[u8], ens: &mut Ensemble, counts: &mut FigCounts) -> Option<()> {
    let mut b = Bits::new(body);
    let cs = b.get(4)? as u8;
    let oe = b.get(1)? == 1;
    let ext = b.get(3)? as usize;
    counts.fig1[ext] += 1;
    if oe {
        counts.skipped += 1;
        return Some(());
    }
    match ext {
        0 => {
            let _eid = b.get(16)?;
            ens.label = Some(label(cs, &mut b)?);
        }
        1 => {
            let sid = b.get(16)?;
            let l = label(cs, &mut b)?;
            ens.services
                .entry(sid)
                .or_insert_with(|| Service {
                    sid,
                    ..Default::default()
                })
                .label = Some(l);
        }
        4 => {
            let pd = b.get(1)? == 1;
            let _rfa = b.get(3)?;
            let scids = b.get(4)? as u8;
            let sid = b.get(if pd { 32 } else { 16 })?;
            let l = label(cs, &mut b)?;
            if let Some(c) = ens
                .services
                .get_mut(&sid)
                .and_then(|s| s.components.iter_mut().find(|c| c.scids == Some(scids)))
            {
                c.label = Some(l);
            }
        }
        5 => {
            let sid = b.get(32)?;
            let l = label(cs, &mut b)?;
            ens.services
                .entry(sid)
                .or_insert_with(|| Service {
                    sid,
                    ..Default::default()
                })
                .label = Some(l);
        }
        _ => {}
    }
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subchannels_services_and_labels() {
        let mut ens = Ensemble::default();
        let mut counts = FigCounts::default();
        let mut fib = Vec::new();
        // FIG 0/1: SubChId 5 at CU 100, long form, EEP 3-A, 48 CUs.
        let v: u32 = (5 << 26) | (100 << 16) | (1 << 15) | (2 << 10) | 48;
        fib.extend_from_slice(&[0x05, 0x01]);
        fib.extend_from_slice(&v.to_be_bytes());
        // FIG 0/2: SId C221, one component, DAB+ audio in sub-channel 5,
        // primary.
        fib.extend_from_slice(&[0x06, 0x02, 0xC2, 0x21, 0x01, 63, (5 << 2) | 2]);
        fib.resize(30, 0xFF);
        parse_fib(&fib, &mut ens, &mut counts);
        // FIG 1/1 in the next FIB: the service's label, short "Radio".
        let mut fib = vec![0x35, 0x01, 0xC2, 0x21];
        fib.extend_from_slice(b"Radio Test      ");
        fib.extend_from_slice(&[0xF8, 0x00]);
        fib.resize(30, 0xFF);
        parse_fib(&fib, &mut ens, &mut counts);
        let s = &ens.subchannels[&5];
        assert_eq!((s.start, s.size), (100, 48));
        assert_eq!(s.bitrate(), 64);
        assert_eq!(s.protection_name(), "EEP 3-A");
        let svc = &ens.services[&0xC221];
        assert_eq!(svc.name(), "Radio Test");
        assert_eq!(svc.label.as_ref().unwrap().short, "Radio");
        let c = svc.primary().unwrap();
        assert!(c.is_dab_plus());
        assert_eq!(ens.subchannel_of(c).unwrap().id, 5);
        assert_eq!(counts.malformed, 0);
    }

    #[test]
    fn mjd_to_date() {
        // MJD 61337 is 2026-10-24.
        let t = DabTime {
            mjd: 61337,
            hours: 0,
            minutes: 0,
            seconds: None,
        };
        assert_eq!(t.date(), (2026, 10, 24));
    }
}
