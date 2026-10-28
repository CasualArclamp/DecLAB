// made by CasualArclamp
//! One pager channel: FM discriminator, then a slicer and decoder for each
//! POCSAG rate and for FLEX, all fed the same samples.
//!
//! The channel arrives as complex baseband centred on the carrier (the
//! caller's DDC does that), at any rate from about 25 kS/s up; the slicers
//! cope with fractional samples per symbol.

use declab_core::Iq;

use crate::Page;
use crate::flex::FlexDecoder;
use crate::fsk::{Slicer, Symbol};
use crate::modulate::{FLEX_DEV, POCSAG_DEV};
use crate::pocsag::{self, Charset, PocsagDecoder};
use crate::tones::ToneBank;

#[derive(Debug, Clone, PartialEq)]
pub struct ReceiverConfig {
    /// POCSAG bit rates to listen for.
    pub pocsag: Vec<u32>,
    pub flex: bool,
    /// Report FLEX BIW date/time/system words.
    pub system_words: bool,
    pub charset: Charset,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        ReceiverConfig {
            pocsag: pocsag::RATES.to_vec(),
            flex: true,
            system_words: false,
            charset: Charset::Auto,
        }
    }
}

pub struct Receiver {
    rate: f64,
    prev: Iq,
    /// Discriminator scale: radians a sample → Hz.
    hz_per_rad: f32,
    pocsag: Vec<Chain<PocsagDecoder>>,
    flex: Option<Chain<FlexDecoder>>,
    samples: u64,
    fm: Vec<f32>,
    tap: Option<Tap>,
}

/// Points for a symbol plot and an eye diagram, from the decoder with sync
/// (FLEX first), else from FLEX's (or the first POCSAG) slicer.
#[derive(Debug, Default)]
pub struct Tap {
    /// "FLEX 3200/4 · 00.012", "POCSAG 1200".
    pub source: String,
    pub synced: bool,
    /// Soft symbols in order (±1 outer, ±⅓ inner); the newest last.
    pub symbols: Vec<f32>,
    /// Eye-diagram points: (0..2 across two symbols, level).
    pub eye: Vec<(f32, f32)>,
}

/// One decoder's demodulator.
///
/// With IQ: the tone bank makes the decisions, the energy slicer (on the
/// bank's metric) finds the symbol instants, and the discriminator slicer
/// only estimates the tuning offset for the bank while unlocked. From FM
/// audio there is no IQ: the discriminator slicer does everything.
struct Chain<D> {
    disc: Slicer,
    timing: Slicer,
    bank: ToneBank,
    dec: D,
}

/// What a [`Chain`] needs of its decoder.
trait Feed {
    fn feed(&mut self, s: &Symbol);
    fn synced(&self) -> bool;
    /// The speed the decoder wants next, if it changes speed (FLEX).
    fn mode(&self) -> Option<(u32, u8)> {
        None
    }
}

impl Feed for PocsagDecoder {
    fn feed(&mut self, s: &Symbol) {
        self.push(s.sym > 1, s.rel[0]);
    }
    fn synced(&self) -> bool {
        self.in_sync()
    }
}

impl Feed for FlexDecoder {
    fn feed(&mut self, s: &Symbol) {
        self.push(s.sym, s.rel);
    }
    fn synced(&self) -> bool {
        self.in_sync()
    }
    fn mode(&self) -> Option<(u32, u8)> {
        Some(self.wanted())
    }
}

impl<D: Feed> Chain<D> {
    /// Rust note: generic over the decoder, so this compiles to one loop
    /// per protocol with the calls inlined, not a virtual call a sample.
    fn run(&mut self, fm: &[f32], iq: Option<&[Iq]>, mut tap: Option<&mut Tap>) {
        for (n, &x) in fm.iter().enumerate() {
            let sym = match iq {
                Some(iq) => {
                    self.bank.push(iq[n]);
                    self.disc.push(x);
                    let m = self.bank.metric();
                    let bank = &mut self.bank;
                    self.timing.push_with(m, || Some(bank.decide()))
                }
                None => self.disc.push(x),
            };
            if let Some(t) = &mut tap {
                let shown = if iq.is_some() {
                    &self.timing
                } else {
                    &self.disc
                };
                t.eye.push(shown.eye_point());
            }
            if let Some(s) = sym {
                self.dec.feed(&s);
                let lock = self.dec.synced();
                if let Some((baud, levels)) = self.dec.mode() {
                    self.disc.set_mode(baud, levels);
                    self.timing.set_mode(baud, levels);
                    self.bank.set_mode(baud, levels);
                }
                self.disc.set_locked(lock);
                self.timing.set_locked(lock);
                self.bank.set_locked(lock, self.disc.dc());
                if let Some(t) = &mut tap {
                    t.symbols.push(s.soft);
                }
            }
        }
    }
}

/// The most of each kept between reads: older points are dropped.
const TAP_SYMBOLS: usize = 2048;
const TAP_EYE: usize = 8192;

/// Which decoder the tap follows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    Flex,
    Pocsag(usize),
    None,
}

impl Receiver {
    pub fn new(rate: f64, cfg: &ReceiverConfig) -> Self {
        let pocsag = cfg
            .pocsag
            .iter()
            .map(|&b| {
                let mut d = PocsagDecoder::new(b);
                d.set_charset(cfg.charset);
                Chain {
                    disc: Slicer::new(rate, b, 2, b),
                    timing: Slicer::for_energy(rate, b, 2),
                    bank: ToneBank::new(rate, b, 2, f64::from(POCSAG_DEV), b),
                    dec: d,
                }
            })
            .collect();
        let flex = cfg.flex.then(|| {
            let mut d = FlexDecoder::new();
            d.system_words = cfg.system_words;
            Chain {
                disc: Slicer::new(rate, 1600, 2, 1600),
                // The energy slicer's ring is unused (no box filter).
                timing: Slicer::for_energy(rate, 1600, 2),
                bank: ToneBank::new(rate, 1600, 2, f64::from(FLEX_DEV), 1600),
                dec: d,
            }
        });
        Receiver {
            rate,
            prev: Iq::new(1.0, 0.0),
            hz_per_rad: (rate / std::f64::consts::TAU) as f32,
            pocsag,
            flex,
            samples: 0,
            fm: Vec::new(),
            tap: None,
        }
    }

    pub fn sample_rate(&self) -> f64 {
        self.rate
    }

    /// Complex baseband in, pages out.
    pub fn process_iq(&mut self, iq: &[Iq], out: &mut Vec<Page>) {
        let mut fm = std::mem::take(&mut self.fm);
        fm.clear();
        fm.reserve(iq.len());
        let mut prev = self.prev;
        for &z in iq {
            let d = z * prev.conj();
            fm.push(d.im.atan2(d.re) * self.hz_per_rad);
            prev = z;
        }
        self.prev = prev;
        self.process(&fm, Some(iq), out);
        self.fm = fm;
    }

    /// Already-demodulated FM (a discriminator tap or an audio file), any
    /// scale and polarity. Decisions come from the slicer alone: there is no
    /// IQ for the tone bank.
    pub fn process_fm(&mut self, fm: &[f32], out: &mut Vec<Page>) {
        self.process(fm, None, out);
    }

    /// The discriminator output `fm`, and when there is one the IQ it came
    /// from (sample for sample) for the tone banks.
    fn process(&mut self, fm: &[f32], iq: Option<&[Iq]>, out: &mut Vec<Page>) {
        let start = out.len();
        let (source, synced) = self.source();
        let mut tap = self.tap.take();
        if let Some(t) = &mut tap {
            t.source = self.describe(source);
            t.synced = synced;
        }
        for (i, c) in self.pocsag.iter_mut().enumerate() {
            // Rust note: `filter` on an `Option` keeps it only when the test
            // holds, so the tap goes to the followed decoder and `None` to
            // the others.
            c.run(fm, iq, tap.as_mut().filter(|_| source == Source::Pocsag(i)));
            c.dec.drain(out);
        }
        if let Some(c) = &mut self.flex {
            c.run(fm, iq, tap.as_mut().filter(|_| source == Source::Flex));
            c.dec.drain(out);
        }
        if let Some(t) = &mut tap {
            if t.symbols.len() > TAP_SYMBOLS {
                t.symbols.drain(..t.symbols.len() - TAP_SYMBOLS);
            }
            if t.eye.len() > TAP_EYE {
                t.eye.drain(..t.eye.len() - TAP_EYE);
            }
        }
        self.tap = tap;
        self.samples += fm.len() as u64;
        self.stamp(&mut out[start..]);
    }

    /// Collect points for a symbol plot and eye diagram (off by default:
    /// it costs a push a sample).
    pub fn enable_tap(&mut self, on: bool) {
        self.tap = on.then(Tap::default);
    }

    /// The tap, to read and clear.
    pub fn tap_mut(&mut self) -> Option<&mut Tap> {
        self.tap.as_mut()
    }

    /// The decoder with sync, FLEX first; else the one the tap shows idle.
    fn source(&self) -> (Source, bool) {
        if self.flex.as_ref().is_some_and(|f| f.dec.in_sync()) {
            return (Source::Flex, true);
        }
        if let Some(i) = self.pocsag.iter().position(|p| p.dec.in_sync()) {
            return (Source::Pocsag(i), true);
        }
        let idle = if self.flex.is_some() {
            Source::Flex
        } else if self.pocsag.is_empty() {
            Source::None
        } else {
            Source::Pocsag(0)
        };
        (idle, false)
    }

    fn describe(&self, s: Source) -> String {
        match s {
            Source::Flex => match &self.flex {
                Some(Chain { dec, .. }) if dec.in_sync() => {
                    let (baud, levels, cycle, frame) = dec.mode();
                    format!(
                        "FLEX {}/{levels} · {cycle:02}.{frame:03}",
                        baud * u32::from(levels / 2)
                    )
                }
                _ => "FLEX".into(),
            },
            Source::Pocsag(i) => format!("POCSAG {}", self.pocsag[i].dec.baud()),
            Source::None => String::new(),
        }
    }

    /// What has sync now ("FLEX 6400/4 · 01.023"), if anything.
    pub fn synced(&self) -> Option<String> {
        let (s, synced) = self.source();
        synced.then(|| self.describe(s))
    }

    /// End of input: report messages still in progress.
    pub fn finish(&mut self, out: &mut Vec<Page>) {
        let start = out.len();
        for c in &mut self.pocsag {
            c.dec.finish();
            c.dec.drain(out);
        }
        if let Some(c) = &mut self.flex {
            c.dec.finish();
            c.dec.drain(out);
        }
        self.stamp(&mut out[start..]);
    }

    /// Pages are stamped with the end of the block they finished in.
    fn stamp(&self, pages: &mut [Page]) {
        let t = self.samples as f64 / self.rate;
        for p in pages {
            p.time_s = t;
        }
    }

    /// FLEX frames decoded and POCSAG codewords seen, for a status line.
    pub fn stats(&self) -> (u64, u64, u64) {
        let frames = self.flex.as_ref().map_or(0, |f| f.dec.frames);
        let words = self.pocsag.iter().map(|p| p.dec.codewords).sum();
        let bad = self.pocsag.iter().map(|p| p.dec.bad_words).sum();
        (frames, words, bad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flex::{self, FlexBody, FlexPage};
    use crate::modulate::SignalBuilder;
    use crate::pocsag::{Body, PocsagMessage};
    use crate::{Kind, Protocol};

    fn decode(iq: &[Iq], rate: f64) -> Vec<Page> {
        let mut rx = Receiver::new(rate, &ReceiverConfig::default());
        let mut out = Vec::new();
        for block in iq.chunks(4096) {
            rx.process_iq(block, &mut out);
        }
        rx.finish(&mut out);
        out
    }

    fn pocsag_msgs(tag: &str) -> Vec<PocsagMessage> {
        vec![
            PocsagMessage {
                capcode: 1_234_567,
                function: 3,
                body: Body::Alpha(format!("POCSAG {tag} alpha")),
            },
            PocsagMessage {
                capcode: 7_654,
                function: 0,
                body: Body::Numeric("0800-123 456".into()),
            },
            PocsagMessage {
                capcode: 42,
                function: 2,
                body: Body::Tone,
            },
        ]
    }

    #[test]
    fn tap_shows_four_levels_on_4fsk() {
        let rate = 50_000.0;
        let mut sig = SignalBuilder::new(rate);
        let pages = flex::tests::sample_pages();
        for frame in 0..2 {
            sig.flex(&flex::encode_frame(1600, 4, 0, frame, &pages).unwrap());
        }
        let iq = sig.to_iq(0.0, None, 1.0 / 9600.0, 1);
        let mut rx = Receiver::new(rate, &ReceiverConfig::default());
        rx.enable_tap(true);
        let mut out = Vec::new();
        // Up to the middle of the second frame's data.
        for block in iq[..iq.len() * 3 / 4].chunks(4096) {
            rx.process_iq(block, &mut out);
        }
        let t = rx.tap_mut().unwrap();
        assert!(t.synced && t.source.starts_with("FLEX 3200/4"), "{t:?}");
        assert_eq!(t.symbols.len(), TAP_SYMBOLS);
        assert_eq!(t.eye.len(), TAP_EYE);
        // Every soft symbol near one of ±1, ±1/3.
        let near = t
            .symbols
            .iter()
            .filter(|&&v| {
                [-1.0f32, -1.0 / 3.0, 1.0 / 3.0, 1.0]
                    .iter()
                    .any(|l| (v - l).abs() < 0.12)
            })
            .count();
        assert!(near > TAP_SYMBOLS * 95 / 100, "{near}");
    }

    /// Every POCSAG rate and FLEX speed through the modulator, a tuning
    /// error and noise, at a rate that divides none of the symbol rates.
    #[test]
    fn every_mode_over_the_air() {
        let rate = 50_000.0;
        let mut sig = SignalBuilder::new(rate);
        sig.silence(0.2);
        for baud in crate::pocsag::RATES {
            sig.pocsag(baud, &pocsag_msgs(&baud.to_string()));
            sig.silence(0.3);
        }
        for (n, (baud, levels)) in flex::SPEEDS.into_iter().enumerate() {
            for frame in 0..2u8 {
                let mut pages = flex::tests::sample_pages();
                if frame == 1 {
                    pages = vec![FlexPage {
                        capcode: 10 + n as u64,
                        body: FlexBody::Alpha(format!("{baud}/{levels}")),
                    }];
                }
                sig.flex(
                    &flex::encode_frame(baud, levels, 1, frame + 2 * n as u8, &pages).unwrap(),
                );
            }
            sig.silence(0.3);
        }
        let iq = sig.to_iq(1500.0, Some(25.0), 1.0 / 9600.0, 7);
        let pages = decode(&iq, rate);
        for baud in crate::pocsag::RATES {
            let got: Vec<&Page> = pages
                .iter()
                .filter(|p| p.protocol == Protocol::Pocsag && p.baud == baud)
                .collect();
            assert_eq!(got.len(), 3, "POCSAG {baud}: {got:#?}");
            assert_eq!(got[0].text, format!("POCSAG {baud} alpha"));
            assert_eq!(got[1].text, "0800-123 456");
            assert_eq!(got[2].kind, Kind::Tone);
        }
        for (n, (baud, levels)) in flex::SPEEDS.into_iter().enumerate() {
            let got: Vec<Page> = pages
                .iter()
                .filter(|p| p.protocol == Protocol::Flex && p.baud == baud && p.levels == levels)
                .cloned()
                .collect();
            flex::tests::check(&got);
            assert!(
                got.iter()
                    .any(|p| p.capcode == 10 + n as u64 && p.text == format!("{baud}/{levels}")),
                "FLEX {baud}/{levels}: {got:#?}"
            );
        }
    }
}
