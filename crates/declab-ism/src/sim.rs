// made by CasualArclamp
//! Test transmissions: a device's rows of bits back into pulses (the
//! slicers run backwards) and then into IQ, OOK or FSK, anywhere in the
//! band. The simulator example and the tests use it; nothing here is
//! rtl_433's.

use num_complex::Complex32;

use crate::device::{Device, Modulation};

/// A known transmission of one device: the rows its slicer hands over and
/// what the decoder must make of them. Each device module lists a few.
#[derive(Debug, Clone, Copy)]
pub struct Sample {
    /// Which of its module's `DEVICES` sends it (index).
    pub dev: u8,
    /// The model the decoder outputs ("Nexus-TH").
    pub model: &'static str,
    /// The rows as the slicer outputs them, rtl_433's text form:
    /// `"{36}5a2f81f40 {36}5a2f81f40"`. For Manchester zero-bit devices
    /// each row starts with the slicer's hard-coded 0.
    pub code: &'static str,
    /// Fields the message must hold, as shown (`("temperature_C", "18.4")`).
    pub expect: &'static [(&'static str, &'static str)],
    /// How the rows go on the air (pulse width for PPM, gaps, syncs).
    pub air: Air,
}

/// How a sample's rows are put back on the air. Zero widths take
/// defaults from the device.
#[derive(Debug, Clone, Copy)]
pub struct Air {
    /// PPM: the pulse before each gap. PWM: the gap after each pulse
    /// (0: the bit period short + long less the pulse).
    pub pulse_us: f32,
    /// The gap between rows (0: between gap_limit and reset_limit, or the
    /// reset when the device has no gap limit).
    pub row_gap_us: f32,
    /// Pulses and gaps sent before each row (a sync, a wake-up).
    pub prefix: &'static [(f32, f32)],
    /// Repeat the whole set of rows this many times (each a separate
    /// transmission, reset gap between).
    pub repeats: u8,
}

impl Air {
    pub const DEFAULT: Air = Air {
        pulse_us: 0.0,
        row_gap_us: 0.0,
        prefix: &[],
        repeats: 1,
    };
}

/// Levels on the air, microseconds each: `(level, duration)`: [`ON`] is
/// the carrier (OOK) or the mark tone (FSK), [`OFF`] no carrier (OOK) or
/// the space tone (FSK), [`SILENT`] no carrier in either (between rows and
/// transmissions).
pub type Levels = Vec<(i8, f32)>;
pub const ON: i8 = 1;
pub const OFF: i8 = 0;
pub const SILENT: i8 = -1;

fn push(levels: &mut Levels, on: i8, us: f32) {
    if us <= 0.0 {
        return;
    }
    if let Some(last) = levels.last_mut()
        && last.0 == on
    {
        last.1 += us;
        return;
    }
    levels.push((on, us));
}

fn lvl(on: bool) -> i8 {
    if on { ON } else { OFF }
}

fn row_bits(code: &str) -> Vec<Vec<u8>> {
    let b = crate::bits::BitBuffer::parse(code);
    (0..b.num_rows)
        .map(|r| {
            (0..b.bits(r))
                .map(|i| crate::bits::get_bit(b.row(r), i))
                .collect()
        })
        .collect()
}

/// The sample's transmission as levels, ending in the device's reset gap.
pub fn levels(dev: &Device, s: &Sample) -> Levels {
    let rows = row_bits(s.code);
    let a = &s.air;
    let short = dev.short_width;
    let long = dev.long_width;
    let reset = dev.reset_limit.max(long * 2.0) * 1.5;
    let row_gap = if a.row_gap_us > 0.0 {
        a.row_gap_us
    } else if dev.gap_limit > 0.0 {
        (dev.gap_limit + dev.reset_limit) / 2.0
    } else {
        reset
    };
    let mut lv = Levels::new();
    push(&mut lv, SILENT, 1000.0);
    for rep in 0..a.repeats.max(1) {
        if rep > 0 {
            push(&mut lv, SILENT, reset);
        }
        for (ri, row) in rows.iter().enumerate() {
            let last_row = ri + 1 == rows.len();
            let sep = if last_row { reset } else { row_gap };
            for &(p, g) in a.prefix {
                push(&mut lv, ON, p);
                push(&mut lv, OFF, g);
            }
            match dev.modulation {
                Modulation::OokPpm => {
                    let pulse = if a.pulse_us > 0.0 {
                        a.pulse_us
                    } else {
                        short / 2.0
                    };
                    for &b in row {
                        push(&mut lv, ON, pulse);
                        push(&mut lv, OFF, if b == 1 { long } else { short });
                    }
                    push(&mut lv, ON, pulse);
                    push(&mut lv, SILENT, sep);
                }
                Modulation::OokPwm | Modulation::FskPwm => {
                    for (i, &b) in row.iter().enumerate() {
                        let p = if b == 1 { short } else { long };
                        if i + 1 == row.len() {
                            push(&mut lv, ON, p);
                            push(&mut lv, SILENT, sep);
                            continue;
                        }
                        let g = if a.pulse_us > 0.0 {
                            a.pulse_us
                        } else {
                            short + long - p
                        };
                        push(&mut lv, ON, p);
                        push(&mut lv, OFF, g);
                    }
                }
                Modulation::OokPcm | Modulation::FskPcm if short == long => {
                    for &b in row {
                        push(&mut lv, lvl(b == 1), short);
                    }
                    push(&mut lv, SILENT, sep);
                }
                Modulation::OokPcm | Modulation::FskPcm => {
                    // RZ: a 1 is a short pulse in a long bit period.
                    for &b in row {
                        if b == 1 {
                            push(&mut lv, ON, short);
                            push(&mut lv, OFF, long - short);
                        } else {
                            push(&mut lv, OFF, long);
                        }
                    }
                    push(&mut lv, SILENT, sep);
                }
                Modulation::OokManchesterZeroBit | Modulation::FskManchesterZeroBit => {
                    // 1: high then low; 0: low then high. The first (hard-
                    // coded) 0's low half merges into the silence before.
                    for &b in row {
                        push(&mut lv, lvl(b == 1), short);
                        push(&mut lv, lvl(b == 0), short);
                    }
                    push(&mut lv, SILENT, sep);
                }
                Modulation::OokPiwmDc | Modulation::OokPiwmRaw => {
                    let mut on = true;
                    for &b in row {
                        push(&mut lv, lvl(on), if b == 1 { short } else { long });
                        on = !on;
                    }
                    push(&mut lv, SILENT, sep);
                }
                Modulation::OokDmc => {
                    // 1: two short half-bits, 0: one long; the level flips
                    // each half or whole bit.
                    let mut on = true;
                    for &b in row {
                        if b == 1 {
                            push(&mut lv, lvl(on), short);
                            on = !on;
                            push(&mut lv, lvl(on), short);
                        } else {
                            push(&mut lv, lvl(on), long);
                        }
                        on = !on;
                    }
                    push(&mut lv, SILENT, sep);
                }
                Modulation::OokNrzs => {
                    for &b in row {
                        push(&mut lv, lvl(b == 1), short);
                    }
                    push(&mut lv, SILENT, sep);
                }
            }
        }
    }
    lv
}

/// A simple xorshift for repeatable noise without a dependency.
pub struct Noise(u64);

impl Noise {
    pub fn new(seed: u64) -> Self {
        Noise(seed.max(1))
    }

    fn next_f32(&mut self) -> f32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        (x >> 40) as f32 / (1u64 << 24) as f32
    }

    /// One complex Gaussian sample of power `p` (Box-Muller).
    pub fn gauss(&mut self, p: f32) -> Complex32 {
        let u1 = self.next_f32().max(1e-9);
        let u2 = self.next_f32();
        let r = (-u1.ln() * p).sqrt();
        let a = std::f32::consts::TAU * u2;
        Complex32::new(r * a.cos(), r * a.sin())
    }
}

/// How a burst goes on the air.
#[derive(Debug, Clone, Copy)]
pub struct Rf {
    /// Offset from the band's centre, Hz.
    pub offset_hz: f64,
    /// Carrier amplitude (full scale 1).
    pub amplitude: f32,
    /// FSK deviation, Hz (ignored for OOK).
    pub deviation_hz: f64,
}

/// Add the levels to `iq` (at `rate`) from sample `start`, as OOK or FSK.
/// Edges are shaped over a few microseconds, as a real transmitter's are.
pub fn add_burst(iq: &mut [Complex32], rate: f64, start: usize, lv: &Levels, fsk: bool, rf: Rf) {
    let mut t = start;
    let mut phase = 0.0f64;
    let ramp = (rate * 4e-6).max(1.0) as usize;
    let mut env = 0.0f32;
    for &(level, us) in lv {
        let n = (us as f64 * 1e-6 * rate).round() as usize;
        let f = if fsk {
            rf.offset_hz
                + if level == ON {
                    rf.deviation_hz
                } else {
                    -rf.deviation_hz
                }
        } else {
            rf.offset_hz
        };
        let dphi = std::f64::consts::TAU * f / rate;
        let target = if level == ON || (fsk && level == OFF) {
            1.0
        } else {
            0.0
        };
        for _ in 0..n {
            if t >= iq.len() {
                return;
            }
            // First-order ramp towards the level.
            env += (target - env) / ramp as f32;
            if env > 1e-4 {
                let a = rf.amplitude * env;
                iq[t] += Complex32::new(a * phase.cos() as f32, a * phase.sin() as f32);
            }
            phase = (phase + dphi) % std::f64::consts::TAU;
            t += 1;
        }
    }
}

/// The levels' total length, microseconds.
pub fn duration_us(lv: &Levels) -> f32 {
    lv.iter().map(|l| l.1).sum()
}

/// The levels as the pulse detector would see them, in samples at `rate`:
/// a pulse for each run of [`ON`], the gap after it for the rest.
pub fn levels_to_pulses(lv: &Levels, rate: f32, out: &mut crate::slicer::PulseData) {
    out.clear();
    out.sample_rate = rate;
    let mut pulse = 0.0f32;
    let mut gap = 0.0f32;
    for &(level, us) in lv {
        let n = us * 1e-6 * rate;
        if level == ON {
            if pulse > 0.0 && gap > 0.0 {
                out.push(pulse.round() as i32, gap.round() as i32);
                pulse = 0.0;
                gap = 0.0;
            }
            pulse += n;
        } else if pulse > 0.0 {
            gap += n;
        }
    }
    if pulse > 0.0 {
        out.push(pulse.round() as i32, gap.round().max(1.0) as i32);
    }
}
