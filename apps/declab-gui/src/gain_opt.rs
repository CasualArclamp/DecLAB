// made by CasualArclamp
//! The Airspy's gain optimiser, after DecDVB-T's and DecDAB's for the HackRF
//! (`sdr.rs` there): try gain settings on the signal the radio is tuned to,
//! read how good each is, and keep the best.
//!
//! The Airspy R2 has three gain stages with the AGCs off (LNA 0-14, mixer
//! 0-15, IF/VGA 0-15), so a plain sweep of one number cannot reach them all.
//! The sweep therefore climbs libairspy's "sensitivity" ladder (one knob that
//! moves the three together, tuned by Airspy for noise figure), then a few
//! rounds of hill climbing step each stage on its own around the best rung.
//!
//! The figure of merit is the decoder's MER while it is locked (DVB-T), else
//! the channel's C/N read off the spectrum ([`channel_cn`]), which every
//! decoder has, FM included. A setting is out if it clips or runs hotter than
//! [`TOO_HOT_DBFS`]: too much gain lifts the intermodulation floor, and the
//! C/N then falls by itself, but a fade would clip a setting chosen at the
//! edge.

use std::time::{Duration, Instant};

/// Time given to a setting to take hold (the radio's buffers, the level
/// meter's second, the spectrum's averaging) and then to measure it.
const SETTLE: Duration = Duration::from_millis(800);
const MEASURE: Duration = Duration::from_millis(1300);
/// A rung this far under the best so far, twice running, is past the peak.
const PAST_PEAK_DB: f32 = 3.0;
/// Clipping over this share of samples, or a level over this, spoils a point.
/// An OFDM signal's peaks are about 10 dB over its rms: at -12 dBFS they stay
/// off the converter's limit, with a little over for the signal to fade up.
const CLIP_PERCENT: f32 = 0.05;
const TOO_HOT_DBFS: f32 = -12.0;
/// Points this close to the best figure are as good as it (the reading
/// wanders by about this much); of them the coolest is chosen.
const NEAR_BEST_DB: f32 = 0.3;
/// How far one stage moves in a hill-climbing step, and the rounds made.
const REFINE_STEP: u8 = 2;
const REFINE_ROUNDS: usize = 3;
const MAX_POINTS: usize = 40;
/// Under this C/N at the best rung of the sweep there is no signal to tune to.
const MIN_CN_DB: f32 = 3.0;

/// The Airspy's three gain indices (`declab_io::AirspyGains` without the
/// dependency, so this module builds and tests without the driver).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gains {
    pub lna: u8,
    pub mixer: u8,
    pub vga: u8,
}

/// The top of each stage's range (`airspy/proto.rs`: `LNA_MAX` etc.).
const TOP: Gains = Gains {
    lna: 14,
    mixer: 15,
    vga: 15,
};

impl Gains {
    /// The settings one step from this on a single stage, in range.
    fn neighbours(self) -> Vec<Gains> {
        let mut out = Vec::with_capacity(6);
        for stage in 0..3 {
            let (value, top) = match stage {
                0 => (self.lna, TOP.lna),
                1 => (self.mixer, TOP.mixer),
                _ => (self.vga, TOP.vga),
            };
            for v in [
                value.checked_sub(REFINE_STEP),
                Some(value + REFINE_STEP).filter(|v| *v <= top),
            ]
            .into_iter()
            .flatten()
            {
                let mut g = self;
                match stage {
                    0 => g.lna = v,
                    1 => g.mixer = v,
                    _ => g.vga = v,
                }
                out.push(g);
            }
        }
        out
    }
}

/// What the optimiser looks at, each frame.
#[derive(Debug, Clone, Copy)]
pub struct Reading {
    pub now: Instant,
    /// The decoder's MER, while it is decoding.
    pub mer: Option<f32>,
    /// The spectrum's C/N on the channel, when it can be read.
    pub cn: Option<f32>,
    pub rms_dbfs: f32,
    pub clip_percent: f32,
}

/// One gain setting tried.
#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub gains: Gains,
    pub mer: Option<f32>,
    pub cn: Option<f32>,
    pub rms_dbfs: f32,
    /// Clipped or too hot: not a candidate.
    pub spoiled: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// Up the ladder of presets.
    Sweep,
    /// Single stages stepped about the best setting.
    Refine,
}

pub enum Action {
    Wait,
    /// Set these gains (the optimiser is waiting on them).
    Apply(Gains),
    /// Done: the best gains found, or `None` if nothing was measurable (the
    /// original ones are to be put back).
    Finish(Option<Gains>),
}

pub struct Optimiser {
    pub original: Gains,
    pub points: Vec<Point>,
    pub current: Gains,
    ladder: Vec<Gains>,
    next_rung: usize,
    stage: Stage,
    /// The setting being stepped about, and the round.
    centre: Option<Gains>,
    round: usize,
    queue: Vec<Gains>,
    changed_at: Option<Instant>,
    mer: (f32, u32),
    cn: (f32, u32),
    samples: u32,
    settle: Duration,
    measure: Duration,
}

impl Optimiser {
    /// `ladder`: the settings to sweep, least gain first.
    pub fn new(original: Gains, ladder: Vec<Gains>) -> Optimiser {
        Optimiser::with_times(original, ladder, SETTLE, MEASURE)
    }

    fn with_times(
        original: Gains,
        ladder: Vec<Gains>,
        settle: Duration,
        measure: Duration,
    ) -> Optimiser {
        Optimiser {
            original,
            points: Vec::new(),
            current: ladder.first().copied().unwrap_or(original),
            ladder,
            next_rung: 1,
            stage: Stage::Sweep,
            centre: None,
            round: 0,
            queue: Vec::new(),
            changed_at: None,
            mer: (0.0, 0),
            cn: (0.0, 0),
            samples: 0,
            settle,
            measure,
        }
    }

    /// Points measured so far.
    pub fn done(&self) -> usize {
        self.points.len()
    }

    /// The point with the best figure by the one measure that most points
    /// have (the MER if the decoder read one anywhere, else the C/N); of
    /// those within [`NEAR_BEST_DB`] of it, the one with the most headroom.
    pub fn best(&self) -> Option<Point> {
        let scored: Vec<(f32, Point)> = self
            .points
            .iter()
            .filter_map(|p| Some((self.figure(p)?, *p)))
            .collect();
        let top = scored.iter().map(|(v, _)| *v).max_by(f32::total_cmp)?;
        scored
            .into_iter()
            .filter(|(v, _)| *v >= top - NEAR_BEST_DB)
            .min_by(|a, b| a.1.rms_dbfs.total_cmp(&b.1.rms_dbfs))
            .map(|(_, p)| p)
    }

    /// The figure a point is judged by, and its name for display.
    pub fn figure(&self, p: &Point) -> Option<f32> {
        if p.spoiled {
            None
        } else if self.uses_mer() {
            p.mer
        } else {
            p.cn
        }
    }

    pub fn uses_mer(&self) -> bool {
        self.points.iter().any(|p| !p.spoiled && p.mer.is_some())
    }

    pub fn tick(&mut self, r: &Reading) -> Action {
        let Some(t0) = self.changed_at else {
            self.changed_at = Some(r.now);
            return Action::Apply(self.current);
        };
        let elapsed = r.now.saturating_duration_since(t0);
        if elapsed < self.settle {
            return Action::Wait;
        }
        if elapsed < self.settle + self.measure {
            if let Some(m) = r.mer {
                self.mer = (self.mer.0 + m, self.mer.1 + 1);
            }
            if let Some(c) = r.cn {
                self.cn = (self.cn.0 + c, self.cn.1 + 1);
            }
            self.samples += 1;
            return Action::Wait;
        }
        // The point is measured. The level and the clipping are the last
        // second's, all of it after the change.
        let mean = |(s, n): (f32, u32)| (n > 0).then(|| s / n as f32);
        let locked = self.mer.1 * 2 >= self.samples.max(1);
        let point = Point {
            gains: self.current,
            mer: mean(self.mer).filter(|_| locked),
            cn: mean(self.cn),
            rms_dbfs: r.rms_dbfs,
            spoiled: r.clip_percent > CLIP_PERCENT || r.rms_dbfs > TOO_HOT_DBFS,
        };
        self.points.push(point);
        self.mer = (0.0, 0);
        self.cn = (0.0, 0);
        self.samples = 0;
        self.changed_at = None;
        self.next(point)
    }

    fn next(&mut self, last: Point) -> Action {
        match self.stage {
            Stage::Sweep => {
                let past_peak = {
                    let v: Vec<f32> = self.points.iter().filter_map(|p| self.figure(p)).collect();
                    let top = self.best().and_then(|b| self.figure(&b));
                    match (top, v.len()) {
                        (Some(top), n) if n >= 3 => {
                            v[n - 2..].iter().all(|x| *x < top - PAST_PEAK_DB)
                        }
                        _ => false,
                    }
                };
                let done = last.spoiled
                    || past_peak
                    || self.next_rung >= self.ladder.len()
                    || self.points.len() >= MAX_POINTS;
                if !done {
                    self.current = self.ladder[self.next_rung];
                    self.next_rung += 1;
                    return Action::Wait;
                }
                let Some(best) = self.best() else {
                    return Action::Finish(None);
                };
                // A C/N this small is noise reading noise: the channel has no
                // signal, and the "best" would be chance and the tie-break.
                if !self.uses_mer() && self.figure(&best).is_some_and(|c| c < MIN_CN_DB) {
                    return Action::Finish(None);
                }
                self.stage = Stage::Refine;
                self.round = 1;
                self.step_about(best.gains);
                self.next_refine()
            }
            Stage::Refine => self.next_refine(),
        }
    }

    /// Queue the untried settings one step from `g`.
    fn step_about(&mut self, g: Gains) {
        self.centre = Some(g);
        self.queue = g
            .neighbours()
            .into_iter()
            .filter(|n| !self.points.iter().any(|p| p.gains == *n))
            .collect();
    }

    fn next_refine(&mut self) -> Action {
        if let Some(g) = self.queue.pop() {
            self.current = g;
            return Action::Wait;
        }
        // A round is over: go round again if the best has moved.
        let Some(best) = self.best() else {
            return Action::Finish(None);
        };
        if self.round < REFINE_ROUNDS
            && Some(best.gains) != self.centre
            && self.points.len() < MAX_POINTS
        {
            self.round += 1;
            self.step_about(best.gains);
            if !self.queue.is_empty() {
                return self.next_refine();
            }
        }
        Action::Finish(Some(best.gains))
    }
}

/// The C/N of the channel from a spectrum row (dB, lowest frequency first,
/// made at `row_center_hz` over `row_rate`): the channel's mean power over
/// the 10th percentile of the bins outside it, which is the noise floor when
/// the band is mostly empty. The outer 5 % of the row (the Airspy's half-band
/// roll-off) and a guard beside the channel are left out of the floor.
/// `None` when the row has too little outside the channel to read a floor.
pub fn channel_cn(
    db: &[f32],
    row_center_hz: f64,
    row_rate: f64,
    chan_hz: f64,
    bandwidth_hz: f64,
) -> Option<f32> {
    let n = db.len();
    if n < 64 || row_rate <= 0.0 || bandwidth_hz <= 0.0 {
        return None;
    }
    let bin_hz = row_rate / n as f64;
    let lo = row_center_hz - row_rate / 2.0;
    let bin = |hz: f64| ((hz - lo) / bin_hz).clamp(0.0, n as f64 - 1.0);
    let (a, b) = (
        bin(chan_hz - bandwidth_hz / 2.0).ceil() as usize,
        bin(chan_hz + bandwidth_hz / 2.0).floor() as usize,
    );
    if b < a + 2 || chan_hz < lo || chan_hz > lo + row_rate {
        return None;
    }
    let power = db[a..=b].iter().map(|d| 10f32.powf(d / 10.0)).sum::<f32>() / (b - a + 1) as f32;
    let guard = ((bandwidth_hz * 0.15 / bin_hz) as usize).max(2);
    let edge = n / 20;
    let mut floor: Vec<f32> = (edge..n - edge)
        .filter(|&i| i + guard < a || i > b + guard)
        .map(|i| db[i])
        .collect();
    if floor.len() < 32 {
        return None;
    }
    let k = floor.len() / 10;
    let (_, p10, _) = floor.select_nth_unstable_by(k, f32::total_cmp);
    Some(10.0 * power.log10() - *p10)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// libairspy's sensitivity ladder for the tests: (lna, mixer, vga) at
    /// rungs 0, 2 .. 20, 21 (the first and last few of `AirspyGains`).
    fn ladder() -> Vec<Gains> {
        let v = |lna, mixer, vga| Gains { lna, mixer, vga };
        vec![
            v(0, 0, 4),
            v(1, 0, 4),
            v(3, 0, 4),
            v(6, 2, 4),
            v(8, 3, 4),
            v(9, 4, 5),
            v(12, 7, 5),
            v(13, 9, 5),
            v(14, 10, 6),
            v(14, 12, 9),
            v(14, 12, 11),
            v(14, 12, 13),
        ]
    }

    /// A made-up front end: the gain is the sum of the stages' indices, the
    /// C/N rises with it until the intermodulation takes over, the level
    /// follows it and clips at the top.
    fn radio(g: Gains) -> (f32, f32, f32) {
        let total = (g.lna + g.mixer + g.vga) as f32;
        let cn = 30.0
            - 10.0 * (1.0 + 10f32.powf((15.0 - total) / 10.0)).log10()
            - (total - 30.0).max(0.0) * 2.0;
        let rms = -60.0 + total;
        (cn, rms, if rms > -12.0 { 1.0 } else { 0.0 })
    }

    /// Run the optimiser against `radio`, 100 ms a tick.
    fn run(mut o: Optimiser) -> (Option<Gains>, Optimiser) {
        let t0 = Instant::now();
        let mut applied = o.original;
        for i in 0..100_000u32 {
            let (cn, rms_dbfs, clip_percent) = radio(applied);
            let r = Reading {
                now: t0 + Duration::from_millis(100 * i as u64),
                mer: None,
                cn: Some(cn),
                rms_dbfs,
                clip_percent,
            };
            match o.tick(&r) {
                Action::Wait => {}
                Action::Apply(g) => applied = g,
                Action::Finish(g) => return (g, o),
            }
        }
        panic!("never finished");
    }

    #[test]
    fn it_finds_the_top_of_the_cn_curve() {
        let (best, o) = run(Optimiser::new(
            Gains {
                lna: 8,
                mixer: 8,
                vga: 8,
            },
            ladder(),
        ));
        let g = best.expect("a result");
        let (cn, _, _) = radio(g);
        // The curve peaks at a total of about 30 with 30 - 0.4 dB.
        let peak = (0..=14u8)
            .flat_map(|l| {
                (0..=15u8).flat_map(move |m| {
                    (0..=15u8).map(move |v| Gains {
                        lna: l,
                        mixer: m,
                        vga: v,
                    })
                })
            })
            .map(|g| radio(g).0)
            .fold(f32::MIN, f32::max);
        assert!(
            cn > peak - 0.5,
            "{g:?} gives {cn} dB against the best {peak}"
        );
        assert!(radio(g).1 <= TOO_HOT_DBFS);
        assert!(o.done() < MAX_POINTS);
    }

    #[test]
    fn a_setting_that_clips_is_never_chosen() {
        // Same radio, but the clip limit is reached early: -45 dBFS is hot.
        let mut o = Optimiser::new(
            Gains {
                lna: 8,
                mixer: 8,
                vga: 8,
            },
            ladder(),
        );
        let t0 = Instant::now();
        let mut applied = o.original;
        let mut result = None;
        for i in 0..100_000u32 {
            let (cn, rms, _) = radio(applied);
            let r = Reading {
                now: t0 + Duration::from_millis(100 * i as u64),
                mer: None,
                cn: Some(cn),
                rms_dbfs: rms,
                clip_percent: if rms > -40.0 { 1.0 } else { 0.0 },
            };
            match o.tick(&r) {
                Action::Wait => {}
                Action::Apply(g) => applied = g,
                Action::Finish(g) => {
                    result = g;
                    break;
                }
            }
        }
        let g = result.expect("a result");
        assert!(radio(g).1 <= -40.0, "{g:?} at {} dBFS", radio(g).1);
    }

    #[test]
    fn nothing_measurable_gives_back_none() {
        let mut o = Optimiser::new(
            Gains {
                lna: 8,
                mixer: 8,
                vga: 8,
            },
            ladder(),
        );
        let t0 = Instant::now();
        let mut out = None;
        for i in 0..1000u32 {
            let r = Reading {
                now: t0 + Duration::from_millis(100 * i as u64),
                mer: None,
                cn: None,
                rms_dbfs: -40.0,
                clip_percent: 0.0,
            };
            if let Action::Finish(g) = o.tick(&r) {
                out = Some(g);
                break;
            }
        }
        assert_eq!(out, Some(None));
    }

    #[test]
    fn mer_wins_over_cn_and_ties_go_to_the_cooler_setting() {
        let mut o = Optimiser::new(
            Gains {
                lna: 0,
                mixer: 0,
                vga: 0,
            },
            ladder(),
        );
        let p = |lna, mer, rms| Point {
            gains: Gains {
                lna,
                mixer: 0,
                vga: 0,
            },
            mer: Some(mer),
            cn: Some(50.0 - mer),
            rms_dbfs: rms,
            spoiled: false,
        };
        o.points = vec![
            p(1, 20.0, -30.0),
            p(2, 24.0, -20.0),
            p(3, 23.9, -26.0),
            p(4, 25.0, -10.0),
        ];
        // The -10 dBFS point is the best figure but is hot (spoiled below).
        o.points[3].spoiled = true;
        // 24.0 and 23.9 are within 0.3 dB: the cooler, 3, is chosen.
        assert_eq!(o.best().unwrap().gains.lna, 3);
    }

    #[test]
    fn a_channel_with_no_signal_keeps_the_old_gains() {
        // C/N wanders around 1 dB at every setting: noise reading noise.
        let mut o = Optimiser::new(
            Gains {
                lna: 8,
                mixer: 8,
                vga: 8,
            },
            ladder(),
        );
        let t0 = Instant::now();
        let mut out = None;
        for i in 0..10_000u32 {
            let cn = 1.0 + (i % 7) as f32 * 0.05;
            let r = Reading {
                now: t0 + Duration::from_millis(100 * i as u64),
                mer: None,
                cn: Some(cn),
                rms_dbfs: -40.0,
                clip_percent: 0.0,
            };
            if let Action::Finish(g) = o.tick(&r) {
                out = Some(g);
                break;
            }
        }
        assert_eq!(out, Some(None));
    }

    #[test]
    fn neighbours_stay_in_range() {
        let n = Gains {
            lna: 0,
            mixer: 15,
            vga: 1,
        }
        .neighbours();
        assert!(
            n.iter()
                .all(|g| g.lna <= 14 && g.mixer <= 15 && g.vga <= 15)
        );
        // lna can only go up, mixer only down, vga only up (1 - 2 underflows).
        assert_eq!(n.len(), 3);
    }

    #[test]
    fn cn_is_the_channel_over_the_floor() {
        // A 10 MS/s row of 4096 bins at -90 dB with a 200 kHz channel at -60.
        let mut db = vec![-90.0f32; 4096];
        let bin_hz = 10e6 / 4096.0;
        let (centre_bin, half) = (3000usize, (100e3 / bin_hz) as usize);
        for d in &mut db[centre_bin - half..=centre_bin + half] {
            *d = -60.0;
        }
        let chan_hz = 98e6 - 5e6 + (centre_bin as f64 + 0.5) * bin_hz;
        let cn = channel_cn(&db, 98e6, 10e6, chan_hz, 180e3).unwrap();
        assert!((cn - 30.0).abs() < 0.5, "{cn}");
        // A channel off the row has no reading.
        assert!(channel_cn(&db, 98e6, 10e6, 120e6, 180e3).is_none());
    }
}
