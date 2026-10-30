// made by CasualArclamp
//! Transmitter identification (TII, EN 300 401 §14.8): in an SFN every
//! transmitter sends a few carrier pairs in the null symbol, chosen by its
//! main identifier (the pattern: which 4 of 8 carrier groups) and its sub
//! identifier (the comb: where in each group). Their power says how strong
//! each transmitter is here, and the phase slope across its carriers says
//! how late it arrives against the others.
//!
//! Mode I layout, after welle.io's `tii-decoder.cpp` (GPL-2.0+): for comb
//! c (0–23) and group b (0–7) whose pattern bit is set, kk = 1 + 2c + 48b,
//! the pairs (s, s+1) for s = kk − 769, kk − 385, kk, kk + 384. Both
//! carriers of a pair take the phase reference's phase at s.
//!
//! The carriers of one comb sit on a 48-carrier grid, so the phase slope
//! gives a delay only modulo N/48 samples (20.8 µs). The impulse response
//! from the phase reference settles it: the strongest transmitter is taken
//! to be its strongest path, and each other transmitter goes where one of
//! its candidates (every 20.8 µs) meets the most energy in the response.
//!
//! The main identifiers number the 70 eight-bit words with four bits set,
//! in ascending order, b = 0 being the word's top bit.

use std::f32::consts::TAU;

use num_complex::Complex32 as Iq;

use crate::params::{CARRIERS, FFT};

pub const COMBS: usize = 24;
pub const GROUPS: usize = 8;
/// Exponential averaging of the null symbols: about two seconds.
const ALPHA: f32 = 1.0 / 20.0;
/// A group counts as sent when it is this far over the noise (power).
const DETECT: f32 = 2.0;
/// ... and this far over the strongest of the four groups not sent.
const CONTRAST: f32 = 2.5;
/// The averaged impulse response: samples before the first path, and in
/// all.
pub const CIR_PRE: usize = 64;
pub const CIR_LEN: usize = 1280;
/// The delay ambiguity of the comb layout, samples.
const GRID: f32 = FFT as f32 / 48.0;

/// One transmitter heard.
#[derive(Debug, Clone, PartialEq)]
pub struct Transmitter {
    /// Main identifier (0–69) and sub identifier (the comb, 0–23).
    pub main: u8,
    pub sub: u8,
    /// Its TII power against the strongest transmitter's, dB.
    pub rel_db: f32,
    /// Over the noise between the TII carriers, dB.
    pub snr_db: f32,
    /// Arrival, µs: after the first path of the impulse response (or,
    /// before there is one, after the strongest transmitter).
    pub delay_us: f32,
    /// The impulse response's level at that delay against its peak, dB.
    pub path_db: Option<f32>,
}

/// What the detector found over the last seconds.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TiiReport {
    pub transmitters: Vec<Transmitter>,
    /// Power of every comb's groups over the noise, dB, for a picture of
    /// the null symbol.
    pub groups_db: Vec<[f32; GROUPS]>,
    /// Null symbols averaged so far.
    pub frames: u64,
    /// The averaged impulse response against its peak, dB, a sample apart
    /// from [`CIR_PRE`] samples before the first path (empty before one).
    pub cir_db: Vec<f32>,
}

/// Carrier k (−768…768, no 0) to its index among the active carriers.
fn slot(k: i32) -> usize {
    if k < 0 { (k + 768) as usize } else { (k + 767) as usize }
}

/// The pair starts of comb c, group b.
pub fn pair_starts(c: usize, b: usize) -> [i32; 4] {
    let kk = (1 + 2 * c + 48 * b) as i32;
    [kk - 769, kk - 385, kk, kk + 384]
}

/// The pattern of main identifier `main`: bit b set means group b is sent.
pub fn pattern(main: u8) -> Option<[bool; GROUPS]> {
    let w = (0u16..256).filter(|w| w.count_ones() == 4).nth(usize::from(main))?;
    Some(std::array::from_fn(|b| w & (0x80 >> b) != 0))
}

fn main_of(bits: &[bool; GROUPS]) -> Option<u8> {
    let w: u16 = (0..GROUPS).filter(|&b| bits[b]).map(|b| 0x80 >> b).sum();
    (0u16..256)
        .filter(|w| w.count_ones() == 4)
        .position(|x| x == w)
        .map(|p| p as u8)
}

pub struct Tii {
    /// Per active carrier: averaged power, and the averaged value with the
    /// frame's common phase taken off (so it can be averaged coherently).
    power: Vec<f32>,
    value: Vec<Iq>,
    frames: u64,
    /// Impulse response power by delay from the first path (−CIR_PRE…).
    cir: Vec<f32>,
    cir_frames: u64,
}

impl Default for Tii {
    fn default() -> Self {
        Tii {
            power: vec![0.0; CARRIERS],
            value: vec![Iq::default(); CARRIERS],
            frames: 0,
            cir: vec![0.0; CIR_LEN],
            cir_frames: 0,
        }
    }
}

impl Tii {
    /// Take one null symbol's carriers, and the phase reference's common
    /// phase (unit) from the same frame.
    pub fn push(&mut self, null: &[Iq], common: Iq) {
        let a = if self.frames == 0 { 1.0 } else { ALPHA };
        let turn = common.conj();
        for ((p, v), z) in self.power.iter_mut().zip(&mut self.value).zip(null) {
            *p += a * (z.norm_sqr() - *p);
            *v += (z * turn - *v) * a;
        }
        self.frames += 1;
    }

    /// Take a frame's impulse response power (`pow`, by FFT delay bin) and
    /// where its first path is (a signed bin).
    pub fn push_cir(&mut self, pow: &[f32], first: i32) {
        let a = if self.cir_frames == 0 { 1.0 } else { ALPHA };
        for (i, c) in self.cir.iter_mut().enumerate() {
            let d = first + i as i32 - CIR_PRE as i32;
            *c += a * (pow[d.rem_euclid(FFT as i32) as usize] - *c);
        }
        self.cir_frames += 1;
    }

    pub fn reset(&mut self) {
        *self = Tii::default();
    }

    /// The transmitters heard. `prs` is the phase reference by FFT bin.
    pub fn report(&self, prs: &[Iq; FFT]) -> TiiReport {
        let bin = |k: i32| k.rem_euclid(FFT as i32) as usize;
        let energy = |c: usize, b: usize| -> f32 {
            pair_starts(c, b)
                .iter()
                .map(|&s| self.power[slot(s)] + self.power[slot(s + 1)])
                .sum()
        };
        let e: Vec<[f32; GROUPS]> = (0..COMBS)
            .map(|c| std::array::from_fn(|b| energy(c, b)))
            .collect();
        // The noise: the median group (most combs are empty in any SFN).
        let mut all: Vec<f32> = e.iter().flatten().copied().collect();
        all.sort_by(f32::total_cmp);
        let noise = all[all.len() / 2].max(1e-30);
        let groups_db = e
            .iter()
            .map(|g| g.map(|x| 10.0 * (x / noise).max(1e-3).log10()))
            .collect();
        let mut found = Vec::new();
        for (c, g) in e.iter().enumerate() {
            let mut order: Vec<usize> = (0..GROUPS).collect();
            order.sort_by(|&x, &y| g[y].total_cmp(&g[x]));
            // Four groups clearly over the noise, and over the other four.
            if g[order[3]] < DETECT * noise || g[order[3]] < CONTRAST * g[order[4]] {
                continue;
            }
            let mut bits = [false; GROUPS];
            for &b in &order[..4] {
                bits[b] = true;
            }
            let Some(main) = main_of(&bits) else { continue };
            let p: f32 = order[..4].iter().map(|&b| g[b]).sum::<f32>() / 4.0;
            found.push((c, main, bits, p));
        }
        let strongest = found.iter().map(|f| f.3).fold(0f32, f32::max);
        // Raw delays (samples, modulo the grid), strongest first.
        let mut raw: Vec<(Transmitter, f32)> = found
            .iter()
            .map(|&(c, main, bits, p)| {
                (
                    Transmitter {
                        main,
                        sub: c as u8,
                        rel_db: 10.0 * (p / strongest).log10(),
                        snr_db: 10.0 * (p / noise).log10(),
                        delay_us: 0.0,
                        path_db: None,
                    },
                    self.delay_samples(c, &bits, prs, &bin),
                )
            })
            .collect();
        raw.sort_by(|a, b| b.0.rel_db.total_cmp(&a.0.rel_db));
        let (peak_i, peak) = self
            .cir
            .iter()
            .enumerate()
            .fold((0, 0f32), |b, (i, &v)| if v > b.1 { (i, v) } else { b });
        // The response's energy near delay d (samples from the first path).
        let at = |d: f32| -> Option<f32> {
            let i = d.round() as i32 + CIR_PRE as i32;
            (i >= 2 && i + 2 < CIR_LEN as i32)
                .then(|| (i - 2..=i + 2).map(|j| self.cir[j as usize]).fold(0f32, f32::max))
        };
        let d0 = raw.first().map_or(0.0, |r| r.1);
        let transmitters = raw
            .into_iter()
            .map(|(mut t, d)| {
                if peak > 0.0 {
                    // The strongest transmitter on the strongest path.
                    let base = d - d0 + (peak_i as f32 - CIR_PRE as f32);
                    let best = (-60..=60)
                        .map(|n| base + n as f32 * GRID)
                        .filter_map(|c| at(c).map(|e| (c, e)))
                        .fold((base, 0f32), |b, x| if x.1 > b.1 { x } else { b });
                    t.delay_us = best.0 / 2.048;
                    t.path_db = Some(10.0 * (best.1 / peak).max(1e-6).log10());
                } else {
                    t.delay_us = (d - d0) / 2.048;
                }
                t
            })
            .collect();
        TiiReport {
            transmitters,
            groups_db,
            frames: self.frames,
            cir_db: if peak > 0.0 {
                self.cir.iter().map(|v| 10.0 * (v / peak).max(1e-6).log10()).collect()
            } else {
                Vec::new()
            },
        }
    }

    /// Delay, samples, of comb c: the slope of its carriers' phases (each
    /// against the phase reference), found by trying delays across twice
    /// the guard interval each way, a quarter sample apart.
    fn delay_samples(
        &self,
        c: usize,
        bits: &[bool; GROUPS],
        prs: &[Iq; FFT],
        bin: &impl Fn(i32) -> usize,
    ) -> f32 {
        let mut h: Vec<(f32, Iq)> = Vec::with_capacity(32);
        for b in (0..GROUPS).filter(|&b| bits[b]) {
            for s in pair_starts(c, b) {
                let r = prs[bin(s)].conj();
                h.push((s as f32, self.value[slot(s)] * r));
                h.push(((s + 1) as f32, self.value[slot(s + 1)] * r));
            }
        }
        let mut best = (0f32, 0f32);
        let mut d = -1008.0f32;
        while d <= 1008.0 {
            // A path d samples late turns carrier k by −2πkd/N: turn back.
            let w = TAU * d / FFT as f32;
            let s: Iq = h.iter().map(|(k, z)| z * Iq::from_polar(1.0, w * k)).sum();
            if s.norm_sqr() > best.1 {
                best = (d, s.norm_sqr());
            }
            d += 0.25;
        }
        best.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tables::prs_bins;

    #[test]
    fn patterns_number_the_four_bit_words() {
        assert_eq!(pattern(0).unwrap(), [false, false, false, false, true, true, true, true]);
        assert_eq!(pattern(69).unwrap(), [true, true, true, true, false, false, false, false]);
        assert!(pattern(70).is_none());
        for m in 0..70 {
            assert_eq!(main_of(&pattern(m).unwrap()), Some(m));
        }
    }

    #[test]
    fn finds_two_transmitters_and_their_delay() {
        let prs = prs_bins();
        let mut null = vec![Iq::default(); CARRIERS];
        // Main 12 sub 3, and main 12 sub 9 at −6 dB arriving 20 µs later.
        for (sub, amp, delay) in [(3usize, 1.0f32, 0.0f32), (9, 0.5, 20.0 * 2.048)] {
            let bits = pattern(12).unwrap();
            for b in (0..GROUPS).filter(|&b| bits[b]) {
                for s in pair_starts(sub, b) {
                    let ph = prs[(s.rem_euclid(FFT as i32)) as usize];
                    for k in [s, s + 1] {
                        let turn = Iq::from_polar(1.0, -TAU * k as f32 * delay / FFT as f32);
                        null[slot(k)] += ph * turn * amp;
                    }
                }
            }
        }
        // A little noise everywhere.
        for (i, z) in null.iter_mut().enumerate() {
            *z += Iq::new(((i * 7919) % 13) as f32 * 0.002, ((i * 104729) % 11) as f32 * 0.002);
        }
        let mut t = Tii::default();
        t.push(&null, Iq::new(1.0, 0.0));
        let r = t.report(prs);
        assert_eq!(r.transmitters.len(), 2, "{r:?}");
        let (a, b) = (&r.transmitters[0], &r.transmitters[1]);
        assert_eq!((a.main, a.sub, b.main, b.sub), (12, 3, 12, 9));
        assert!((b.rel_db + 6.0).abs() < 0.5);
        assert!((b.delay_us - 20.0).abs() < 0.3, "{}", b.delay_us);
    }
}
