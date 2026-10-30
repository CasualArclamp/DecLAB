// made by CasualArclamp
//! A simulated TETRA base station (continuous π/4-DQPSK downlink) for the
//! tests, sweeps and GUI checks.
//!
//! Timeslot 1 is the MCCH: a synchronization burst (BSCH + BNCH SYSINFO)
//! in frame 18 and whenever no signalling is queued, else an SCH/F with the
//! queued MAC-RESOURCE PDUs. One timeslot may carry a call's traffic
//! (TCH/S, frames 1–17; its type-4 bits are a stand-in pattern since the
//! speech codec lives outside this crate). The rest are unallocated
//! half-slot bursts with null PDUs.

use std::collections::VecDeque;

use super::coding::{self, Chan};
use super::mac::{self, AccessAssign, Address, ChanAlloc, Signal, Sync, SysInfo};
use super::{ALPHA, SLOT_BITS, SYMBOL_RATE, normal_burst, sync_burst, transition};
use crate::fsk4::{Iq, rrc};

/// Deterministic stand-in TCH/S type-4 bits for traffic slot `n`.
pub fn test_traffic(n: u32) -> Vec<u8> {
    let mut x = n.wrapping_mul(2_654_435_761).wrapping_add(99);
    (0..432)
        .map(|_| {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            (x & 1) as u8
        })
        .collect()
}

pub struct Cell {
    pub sync: Sync,
    pub sysinfo: SysInfo,
    /// Signalling waiting for the MCCH: (address, channel allocation, PDU).
    pub queue: VecDeque<(Address, Option<ChanAlloc>, Signal)>,
    /// The traffic timeslot (0–3) and its usage marker, while a call runs.
    pub traffic: Option<(u8, u8)>,
    /// Traffic slots sent so far (the `test_traffic` index).
    pub traffic_sent: u32,
    tn: u8,
    frame: u8,
    multiframe: u8,
    scramble: Vec<u8>,
    bsch_scramble: Vec<u8>,
}

impl Cell {
    pub fn new(mcc: u16, mnc: u16, colour: u8) -> Cell {
        let sync = Sync {
            system_code: 3,
            colour,
            tn: 0,
            frame: 1,
            multiframe: 1,
            sharing: 0,
            ts_reserved: 0,
            u_plane_dtx: false,
            frame18_ext: false,
            mcc,
            mnc,
            neighbour_bcast: 1,
            cell_load: 0,
            late_entry: true,
        };
        let sysinfo = SysInfo {
            main_carrier: 1234,
            band: 4,
            offset: 0,
            duplex: 0,
            reverse: false,
            common_scch: 0,
            tx_max: 5,
            rx_min: 3,
            access_param: 7,
            timeout: 1,
            hyperframe_or_cck: (false, 0),
            option_flag: 2,
            option: 0,
            la: 1001,
            subscriber_class: 0xffff,
            // Registration, …, voice service; no air interface encryption.
            services: 0b1000_0111_0000,
        };
        let e = coding::extended_colour_code(mcc, mnc, colour);
        Cell {
            sync,
            sysinfo,
            queue: VecDeque::new(),
            traffic: None,
            traffic_sent: 0,
            tn: 0,
            frame: 1,
            multiframe: 1,
            scramble: coding::scrambling(e, 432),
            bsch_scramble: coding::scrambling(0, 120),
        }
    }

    fn aach(&self, a: AccessAssign) -> Vec<u8> {
        coding::encode_aach(a.to_bits(), &self.scramble)
    }

    fn null_half(&self) -> Vec<u8> {
        coding::encode(Chan::Half, &mac::write_null(124), &self.scramble)
    }

    /// The next slot's 510 bits.
    pub fn next_slot(&mut self) -> [u8; SLOT_BITS] {
        let (tn, frame) = (self.tn, self.frame);
        let common = AccessAssign {
            header: 0,
            field1: 0,
            field2: 0,
        };
        let burst = if tn == 0 {
            if frame == 18 || self.queue.is_empty() {
                let mut s = self.sync;
                s.tn = tn;
                s.frame = frame;
                s.multiframe = self.multiframe;
                let sb = coding::encode(Chan::Bsch, &s.write(), &self.bsch_scramble);
                let bnch = coding::encode(Chan::Half, &self.sysinfo.write(), &self.scramble);
                sync_burst(&sb, &self.aach(common), &bnch)
            } else {
                let mut blk = Vec::new();
                while let Some((a, c, s)) = self.queue.front() {
                    let pdu = mac::write_resource(*a, c.as_ref(), &mac::write_signal(s));
                    if !blk.is_empty() && blk.len() + pdu.len() > 268 {
                        break;
                    }
                    blk.extend(pdu);
                    self.queue.pop_front();
                }
                let b5 = coding::encode(Chan::Full, &mac::pad(blk, 268), &self.scramble);
                normal_burst(true, &b5[..216], &self.aach(common), &b5[216..])
            }
        } else if let Some((ttn, um)) = self.traffic.filter(|t| t.0 == tn && frame != 18) {
            let _ = ttn;
            let a = AccessAssign {
                header: 3,
                field1: um,
                field2: um,
            };
            let b4 = test_traffic(self.traffic_sent);
            self.traffic_sent += 1;
            let b5: Vec<u8> = b4
                .iter()
                .zip(&self.scramble)
                .map(|(&b, &p)| b ^ p)
                .collect();
            normal_burst(true, &b5[..216], &self.aach(a), &b5[216..])
        } else {
            let half = self.null_half();
            normal_burst(false, &half, &self.aach(common), &half)
        };
        self.tn = (self.tn + 1) % 4;
        if self.tn == 0 {
            self.frame = self.frame % 18 + 1;
            if self.frame == 1 {
                self.multiframe = self.multiframe % 60 + 1;
            }
        }
        burst
    }

    /// `n` slots' bits, concatenated.
    pub fn slots(&mut self, n: usize) -> Vec<u8> {
        (0..n).flat_map(|_| self.next_slot()).collect()
    }
}

/// A group call on a cell for `frames` TDMA frames (about 57 ms each):
/// D-SETUP with a channel allocation to timeslot 2 and a D-TX-GRANTED,
/// then traffic, then D-TX-CEASED and D-RELEASE, then an SDS and a status.
/// Returns the bits and the traffic slots sent.
pub fn call(cell: &mut Cell, gssi: u32, from: u32, frames: usize) -> (Vec<u8>, u32) {
    let mut bits = cell.slots(4 * 20);
    let chan = ChanAlloc {
        alloc_type: 0,
        timeslots: 0b0100,
        up_down: 3,
        carrier: cell.sysinfo.main_carrier,
        ext: None,
    };
    cell.queue.push_back((
        Address::SsiUsageMarker(gssi, 9),
        Some(chan),
        Signal::Setup {
            call: 77,
            // Basic service (clause 14.8.2): 000 speech TCH/S, 0 clear,
            // 01 point-to-multipoint (a group call), 00.
            service: 0b0000_0100,
            priority: 0,
            calling: Some(from),
        },
    ));
    cell.queue.push_back((
        Address::Ssi(gssi),
        None,
        Signal::TxGranted {
            call: 77,
            grant: 0,
            encrypted: false,
            party: Some(from),
        },
    ));
    cell.traffic = Some((1, 9));
    let start = cell.traffic_sent;
    bits.extend(cell.slots(4 * frames));
    cell.traffic = None;
    cell.queue
        .push_back((Address::Ssi(gssi), None, Signal::TxCeased { call: 77 }));
    cell.queue.push_back((
        Address::Ssi(gssi),
        None,
        Signal::Release { call: 77, cause: 1 },
    ));
    cell.queue.push_back((
        Address::Ssi(from),
        None,
        Signal::Sds {
            from: Some(gssi),
            kind: 3,
            data: b"\x82\x04ok".to_vec(),
            bits: 32,
        },
    ));
    cell.queue.push_back((
        Address::Ssi(from),
        None,
        Signal::Status {
            from: Some(2_000),
            status: 0x8002,
        },
    ));
    bits.extend(cell.slots(4 * 20));
    (bits, cell.traffic_sent - start)
}

/// Continuous π/4-DQPSK at `rate` (a whole number of samples a symbol),
/// unit power, with `pad_s` of silence either side. S(0) = 1 (clause 5.4).
pub fn modulate(bits: &[u8], rate: f64, pad_s: f64) -> Vec<Iq> {
    let sps = (rate / SYMBOL_RATE).round() as usize;
    let h = rrc(ALPHA, sps, 8);
    let pad = (pad_s * rate) as usize;
    let n = bits.len() / 2 + 1;
    let mut x = vec![Iq::new(0.0, 0.0); pad + n * sps + h.len() + pad];
    let mut phase = 0i32;
    x[pad] = Iq::new(1.0, 0.0);
    for k in 1..n {
        phase += transition(bits[2 * k - 2], bits[2 * k - 1]);
        x[pad + k * sps] = Iq::from_polar(1.0, phase as f32 * std::f32::consts::FRAC_PI_4);
    }
    let mut y = vec![Iq::new(0.0, 0.0); x.len()];
    let half = h.len() / 2;
    for (i, &v) in x.iter().enumerate() {
        if v.norm_sqr() == 0.0 {
            continue;
        }
        for (j, &t) in h.iter().enumerate() {
            let o = i + j;
            if o >= half && o - half < y.len() {
                y[o - half] += v * t;
            }
        }
    }
    // Unit power over the modulated part.
    let p: f32 = y[pad..pad + n * sps]
        .iter()
        .map(|v| v.norm_sqr())
        .sum::<f32>()
        / (n * sps) as f32;
    let k = 1.0 / p.sqrt();
    y.iter_mut().for_each(|v| *v *= k);
    y
}

/// Noise for a C/N measured in the 25 kHz channel.
pub fn add_noise(iq: &mut [Iq], rate: f64, cn_db: f64, offset_hz: f64, seed: u64) {
    crate::fsk4::add_noise(iq, rate, cn_db, 25_000.0, offset_hz, seed);
}
