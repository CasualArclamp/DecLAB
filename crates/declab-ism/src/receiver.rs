// made by CasualArclamp
//! The receiver: channel IQ in, decoded device messages out.
//!
//! [`BurstFinder`] cuts bursts out of the channel, [`Detector`] turns each
//! into packages of pulses, and every enabled device whose coding fits
//! (OOK devices for OOK packages, FSK for FSK) slices and decodes them,
//! lowest priority number first: a device with a higher number runs only
//! when nothing before it decoded the package (rtl_433's `priority`). A
//! sensor repeats itself, so the same message heard again within two
//! seconds counts as a repeat, not a new message.

use std::collections::VecDeque;

use num_complex::Complex32;

use crate::bits::BitBuffer;
use crate::burst::{Burst, BurstFinder};
use crate::device::{Ctx, Device, Group, Message};
use crate::flex::Flex;
use crate::pulse::{Detector, Package};
use crate::slicer::{self, PulseData};

/// What the receiver decodes.
#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    /// Device groups on.
    pub groups: Vec<Group>,
    /// Devices rtl_433 keeps off by default (false positives) run too.
    pub include_disabled: bool,
    /// Flex specs (rtl_433's `-X`).
    pub flex: Vec<String>,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        ReceiverConfig {
            groups: vec![
                Group::Weather,
                Group::Remote,
                Group::Tpms,
                Group::Meter,
                Group::Flex,
            ],
            include_disabled: false,
            flex: Vec::new(),
        }
    }
}

/// A decoded message and where it came from.
#[derive(Debug, Clone)]
pub struct Decoded {
    /// Stream time, seconds.
    pub at_s: f64,
    /// Hz from the channel's centre.
    pub freq_hz: f64,
    pub rssi_db: f32,
    pub snr_db: f32,
    pub fsk: bool,
    /// The decoder's name (rtl_433's long name, or the flex spec's).
    pub device: String,
    pub group: Group,
    pub message: Message,
}

/// A package nothing decoded, described the way rtl_433's analyzer does.
#[derive(Debug, Clone)]
pub struct Unknown {
    pub at_s: f64,
    pub freq_hz: f64,
    pub rssi_db: f32,
    pub snr_db: f32,
    pub fsk: bool,
    pub pulses: usize,
    /// "OOK 36 pulses: pulse 244/488 µs, gap 488/244 µs".
    pub summary: String,
    /// A flex spec to start from ("n=name,m=OOK_PWM,s=244,l=488,r=6000").
    pub flex_hint: String,
}

#[derive(Debug, Clone)]
pub enum Event {
    Decoded(Decoded),
    /// A message heard again within two seconds (index into nothing:
    /// just the model and id, for the counts).
    Repeat {
        at_s: f64,
        model: String,
        id: String,
    },
    Unknown(Unknown),
}

/// Counters for the card.
#[derive(Debug, Clone, Copy, Default)]
pub struct Stats {
    pub bursts: u64,
    pub packages: u64,
    pub decoded: u64,
    pub repeats: u64,
    pub unknown: u64,
}

enum Entry {
    Builtin(&'static Device),
    Flex(Box<Flex>),
}

impl Entry {
    fn device(&self) -> &Device {
        match self {
            Entry::Builtin(d) => d,
            Entry::Flex(f) => &f.device,
        }
    }
}

/// The last package seen, for the GUI's "Pulses" scope.
#[derive(Debug, Clone, Default)]
pub struct LastPackage {
    pub at_s: f64,
    pub freq_hz: f64,
    pub fsk: bool,
    pub pulses: PulseData,
    /// What decoded it ("" if nothing).
    pub model: String,
    pub rssi_db: f32,
    pub snr_db: f32,
}

pub struct Receiver {
    finder: BurstFinder,
    detector: Detector,
    entries: Vec<Entry>,
    bits: BitBuffer,
    ctx: Ctx,
    packages: Vec<Package>,
    recent: VecDeque<(f64, Message)>,
    pub events: Vec<Event>,
    pub stats: Stats,
    pub last: Option<LastPackage>,
    /// Flex specs that did not parse, with why.
    pub flex_errors: Vec<String>,
}

impl Receiver {
    pub fn new(rate: f64, cfg: &ReceiverConfig) -> Self {
        let mut r = Receiver {
            finder: BurstFinder::new(rate),
            detector: Detector::default(),
            entries: Vec::new(),
            bits: BitBuffer::new(),
            ctx: Ctx::default(),
            packages: Vec::new(),
            recent: VecDeque::new(),
            events: Vec::new(),
            stats: Stats::default(),
            last: None,
            flex_errors: Vec::new(),
        };
        r.configure(cfg);
        r
    }

    pub fn rate(&self) -> f64 {
        self.finder.rate()
    }

    /// Change what is decoded (keeps the burst finder's state).
    pub fn configure(&mut self, cfg: &ReceiverConfig) {
        self.entries.clear();
        self.flex_errors.clear();
        for d in crate::devices::all() {
            if cfg.groups.contains(&d.group) && (!d.disabled || cfg.include_disabled) {
                self.entries.push(Entry::Builtin(d));
            }
        }
        if cfg.groups.contains(&Group::Flex) {
            for spec in &cfg.flex {
                match Flex::parse(spec) {
                    Ok(f) => self.entries.push(Entry::Flex(Box::new(f))),
                    Err(e) => self.flex_errors.push(e),
                }
            }
        }
        // Stable: rtl_433's order within a priority.
        self.entries.sort_by_key(|e| e.device().priority);
    }

    /// Devices that will be tried.
    pub fn device_count(&self) -> usize {
        self.entries.len()
    }

    pub fn reset(&mut self) {
        self.finder.reset();
        self.recent.clear();
        self.last = None;
    }

    /// The band map (`every` FFT frames a row; 0 off).
    pub fn set_map(&mut self, every: u32) {
        self.finder.map_every = every;
    }

    pub fn take_map_rows(&mut self) -> Vec<crate::burst::MapRow> {
        std::mem::take(&mut self.finder.map_rows)
    }

    pub fn bin_hz(&self) -> f64 {
        self.finder.bin_hz()
    }

    pub fn process(&mut self, iq: &[Complex32]) {
        self.finder.process(iq);
        self.decode_done();
    }

    /// End of the stream: decode what is still open.
    pub fn flush(&mut self) {
        self.finder.flush();
        self.decode_done();
    }

    fn decode_done(&mut self) {
        let done = std::mem::take(&mut self.finder.done);
        for b in &done {
            self.burst(b);
        }
        // Give the buffer back to keep its capacity.
        self.finder.done = done;
        self.finder.done.clear();
    }

    fn burst(&mut self, b: &Burst) {
        self.stats.bursts += 1;
        self.packages.clear();
        self.detector
            .detect(&b.iq, b.rate, b.half_hz as f32, &mut self.packages);
        let in_rate = self.finder.rate();
        let decim = (in_rate / b.rate as f64).round();
        let packages = std::mem::take(&mut self.packages);
        for p in &packages {
            self.stats.packages += 1;
            let at_s = (b.start_sample as f64 + p.start as f64 * decim) / in_rate;
            let freq_hz = b.mix_hz + p.freq_hz as f64;
            let model = self.package(p, at_s, freq_hz);
            self.last = Some(LastPackage {
                at_s,
                freq_hz,
                fsk: p.fsk,
                pulses: p.pulses.clone(),
                model,
                rssi_db: p.rssi_db,
                snr_db: p.snr_db,
            });
        }
        self.packages = packages;
    }

    /// Decode one package; returns the first model decoded.
    fn package(&mut self, p: &Package, at_s: f64, freq_hz: f64) -> String {
        let mut found = 0;
        let mut first_model = String::new();
        let mut prio = 0;
        for e in &self.entries {
            let dev = e.device();
            if dev.modulation.is_fsk() != p.fsk {
                continue;
            }
            if dev.priority > prio {
                if found > 0 {
                    break;
                }
                prio = dev.priority;
            }
            self.ctx.out.clear();
            match e {
                Entry::Builtin(d) => {
                    slicer::slice(&p.pulses, d, &mut self.bits, &mut self.ctx);
                }
                Entry::Flex(f) => {
                    slicer::slice_with(
                        &p.pulses,
                        &f.device,
                        &|c, b| f.decode(c, b),
                        &mut self.bits,
                        &mut self.ctx,
                    );
                }
            }
            for m in self.ctx.out.drain(..) {
                found += 1;
                if first_model.is_empty() {
                    first_model = m.model.clone();
                }
                // Forget what is older than two seconds.
                while self.recent.front().is_some_and(|r| at_s - r.0 > 2.0) {
                    self.recent.pop_front();
                }
                if self.recent.iter().any(|r| r.1 == m) {
                    self.stats.repeats += 1;
                    self.events.push(Event::Repeat {
                        at_s,
                        id: m.id_text(),
                        model: m.model,
                    });
                    continue;
                }
                self.recent.push_back((at_s, m.clone()));
                self.stats.decoded += 1;
                self.events.push(Event::Decoded(Decoded {
                    at_s,
                    freq_hz,
                    rssi_db: p.rssi_db,
                    snr_db: p.snr_db,
                    fsk: p.fsk,
                    device: match e {
                        Entry::Builtin(d) => d.name.to_string(),
                        Entry::Flex(f) => format!("flex {}", f.name),
                    },
                    group: dev.group,
                    message: m,
                }));
            }
        }
        if found == 0 && p.pulses.len() >= 8 {
            self.stats.unknown += 1;
            let (summary, flex_hint) = analyse(&p.pulses, p.fsk);
            self.events.push(Event::Unknown(Unknown {
                at_s,
                freq_hz,
                rssi_db: p.rssi_db,
                snr_db: p.snr_db,
                fsk: p.fsk,
                pulses: p.pulses.len(),
                summary,
                flex_hint,
            }));
        }
        first_model
    }
}

/// Width clusters (±20 %), most common first: (mean µs, count).
fn clusters(widths: impl Iterator<Item = f32>) -> Vec<(f32, usize)> {
    let mut c: Vec<(f32, usize)> = Vec::new();
    for w in widths {
        if let Some(k) = c.iter_mut().find(|k| (w - k.0).abs() <= 0.2 * k.0.max(w)) {
            k.0 = (k.0 * k.1 as f32 + w) / (k.1 + 1) as f32;
            k.1 += 1;
        } else {
            c.push((w, 1));
        }
    }
    c.sort_by_key(|k| std::cmp::Reverse(k.1));
    c
}

/// rtl_433's analyzer, cut down: the pulse and gap widths, a guess at the
/// coding, and a flex spec to start from.
pub fn analyse(p: &PulseData, fsk: bool) -> (String, String) {
    let us = 1e6 / p.sample_rate;
    let n = p.len();
    let pc = clusters(p.pulse.iter().map(|&v| v as f32 * us));
    // The last gap ends the package: leave it out.
    let gc = clusters(
        p.gap
            .iter()
            .take(n.saturating_sub(1))
            .map(|&v| v as f32 * us),
    );
    let list = |c: &[(f32, usize)]| {
        c.iter()
            .filter(|k| k.1 * 20 >= n)
            .take(3)
            .map(|k| format!("{:.0}", k.0))
            .collect::<Vec<_>>()
            .join("/")
    };
    let kind = if fsk { "FSK" } else { "OOK" };
    let summary = format!(
        "{kind} {n} pulses: pulse {} µs, gap {} µs",
        list(&pc),
        list(&gc)
    );
    let main = |c: &[(f32, usize)]| {
        c.iter()
            .filter(|k| k.1 * 10 >= n)
            .take(2)
            .map(|k| k.0)
            .collect::<Vec<_>>()
    };
    let (pm, gm) = (main(&pc), main(&gc));
    let longest_gap = p
        .gap
        .iter()
        .take(n.saturating_sub(1))
        .copied()
        .max()
        .unwrap_or(0) as f32
        * us;
    let reset = (longest_gap * 1.2)
        .max(gm.iter().fold(0.0f32, |a, &b| a.max(b)) * 3.0)
        .max(1000.0);
    let hint = if pm.len() >= 2 {
        let (s, l) = (pm[0].min(pm[1]), pm[0].max(pm[1]));
        format!("n=name,m={}_PWM,s={s:.0},l={l:.0},r={reset:.0}", kind)
    } else if pm.len() == 1 && gm.len() >= 2 {
        let (s, l) = (gm[0].min(gm[1]), gm[0].max(gm[1]));
        format!("n=name,m=OOK_PPM,s={s:.0},l={l:.0},r={reset:.0}")
    } else if let Some(&w) = pm.first() {
        format!("n=name,m={kind}_PCM,s={w:.0},l={w:.0},r={reset:.0}")
    } else {
        String::new()
    };
    (summary, hint)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{self, Noise, Rf};

    /// Every sample, put on the air at its own frequency in noise, comes
    /// out of the whole receiver.
    #[test]
    fn every_sample_over_the_air() {
        let rate = 2.5e6;
        let mut fails = Vec::new();
        for (i, (dev, s)) in crate::devices::samples().enumerate() {
            if std::env::var("ISM_LIST").is_ok() {
                println!(
                    "{} {} off {}",
                    i,
                    s.model,
                    -600e3 + (i as f64 * 137e3) % 1.2e6
                );
            }
            let lv = sim::levels(dev, s);
            let dur = sim::duration_us(&lv) as f64 * 1e-6;
            let mut iq = vec![Complex32::default(); ((dur + 0.15) * rate) as usize];
            let mut noise = Noise::new(11 + i as u64);
            for x in iq.iter_mut() {
                *x = noise.gauss(1e-4);
            }
            let rf = Rf {
                offset_hz: -600e3 + (i as f64 * 137e3) % 1.2e6,
                // ~23 dB in a 75 kHz channel: the chain, not sensitivity
                // (a 736-bit meter packet at 20 dB loses a bit now and then).
                amplitude: 0.015,
                deviation_hz: 30e3,
            };
            sim::add_burst(
                &mut iq,
                rate,
                (0.05 * rate) as usize,
                &lv,
                dev.modulation.is_fsk(),
                rf,
            );
            let cfg = ReceiverConfig {
                include_disabled: true,
                ..Default::default()
            };
            let mut rx = Receiver::new(rate, &cfg);
            for chunk in iq.chunks(16384) {
                rx.process(chunk);
            }
            rx.flush();
            let got = rx
                .events
                .iter()
                .any(|e| matches!(e, Event::Decoded(d) if d.message.model == s.model));
            if !got {
                let what: Vec<String> = rx
                    .events
                    .iter()
                    .map(|e| match e {
                        Event::Decoded(d) => d.message.model.clone(),
                        Event::Repeat { model, .. } => format!("repeat {model}"),
                        Event::Unknown(u) => u.summary.clone(),
                    })
                    .collect();
                fails.push(format!(
                    "{} ({}): {:?}",
                    s.model,
                    dev.modulation.name(),
                    what
                ));
            }
        }
        assert!(
            fails.is_empty(),
            "not decoded over the air:\n{}",
            fails.join("\n")
        );
    }
}

#[cfg(test)]
mod debug {
    use super::*;
    use crate::sim::{self, Noise, Rf};

    /// `ISM_MODEL=Oregon-THGR228N cargo test -p declab-ism debug_sample -- --ignored --nocapture`:
    /// one sample's pulses on the air against the ideal ones.
    #[test]
    #[ignore]
    fn debug_sample() {
        let want = std::env::var("ISM_MODEL").unwrap_or_default();
        let amp: f32 = std::env::var("ISM_AMP")
            .ok()
            .and_then(|a| a.parse().ok())
            .unwrap_or(0.01);
        let rate = 2.5e6;
        let (dev, s) = crate::devices::samples()
            .find(|(_, s)| s.model == want)
            .expect("model");
        let lv = sim::levels(dev, s);
        let mut ideal = PulseData::default();
        sim::levels_to_pulses(&lv, 500_000.0, &mut ideal);
        println!(
            "ideal {} pulses: {:?}",
            ideal.len(),
            ideal
                .pulse
                .iter()
                .zip(&ideal.gap)
                .take(40)
                .collect::<Vec<_>>()
        );
        let dur = sim::duration_us(&lv) as f64 * 1e-6;
        let mut iq = vec![Complex32::default(); ((dur + 0.15) * rate) as usize];
        let mut noise = Noise::new(
            std::env::var("ISM_SEED")
                .ok()
                .and_then(|a| a.parse().ok())
                .unwrap_or(5),
        );
        for x in iq.iter_mut() {
            *x = noise.gauss(1e-4);
        }
        let off: f64 = std::env::var("ISM_OFF")
            .ok()
            .and_then(|a| a.parse().ok())
            .unwrap_or(123e3);

        let rf = Rf {
            offset_hz: off,
            amplitude: amp,
            deviation_hz: 30e3,
        };
        sim::add_burst(
            &mut iq,
            rate,
            (0.05 * rate) as usize,
            &lv,
            dev.modulation.is_fsk(),
            rf,
        );
        let mut bf = BurstFinder::new(rate);
        bf.process(&iq);
        bf.flush();
        for b in &bf.done {
            println!(
                "burst {:.0}..{:.0} centre {:.0} half {:.0} frames {} len {}",
                b.lo_hz,
                b.hi_hz,
                b.centre_hz,
                b.half_hz,
                b.active_frames,
                b.iq.len()
            );
            let mut pk = Vec::new();
            Detector::default().detect(&b.iq, b.rate, b.half_hz as f32, &mut pk);
            for p in &pk {
                println!(
                    "  {} {} pulses snr {:.1} f {:.0} {:.0}: {:?}",
                    if p.fsk { "FSK" } else { "OOK" },
                    p.pulses.len(),
                    p.snr_db,
                    p.f1_hz,
                    p.f2_hz,
                    p.pulses
                        .pulse
                        .iter()
                        .zip(&p.pulses.gap)
                        .take(40)
                        .collect::<Vec<_>>()
                );
                let mut bits = BitBuffer::new();
                let mut ctx = Ctx::default();
                slicer::slice(&p.pulses, dev, &mut bits, &mut ctx);
                println!("  decoded {:?}", ctx.out);
                let rows = std::cell::RefCell::new(Vec::new());
                slicer::slice_with(
                    &p.pulses,
                    dev,
                    &|_, b| {
                        rows.borrow_mut().push(format!("{b:?}"));
                        0
                    },
                    &mut bits,
                    &mut ctx,
                );
                println!("  rows {:?}", rows.borrow());
                let mut i2 = PulseData::default();
                sim::levels_to_pulses(&lv, b.rate, &mut i2);
                let rows = std::cell::RefCell::new(Vec::new());
                slicer::slice_with(
                    &i2,
                    dev,
                    &|_, b| {
                        rows.borrow_mut().push(format!("{b:?}"));
                        0
                    },
                    &mut bits,
                    &mut ctx,
                );
                println!("  ideal rows {:?}", rows.borrow());
            }
        }
    }
}
