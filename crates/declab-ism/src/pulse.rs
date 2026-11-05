// made by CasualArclamp
//! A burst's samples into packages of pulses, OOK or FSK, for the
//! slicers.
//!
//! The rules follow rtl_433's `pulse_detect.c` and `pulse_detect_fsk.c`
//! (Tommy Vestermark, Benjamin Larsson, Christian W. Zuckschwerdt;
//! GPL-2.0-or-later): a pulse must last `PD_MIN_PULSE_SAMPLES`, a short
//! dip inside a pulse is not a gap, a package ends after a gap of 10× its
//! longest pulse and 10 ms, or 100 ms; FSK is looked for inside a long
//! carrier and, when found, the package is FSK. What differs: the burst is
//! already cut out, so the levels are measured over the whole burst
//! (noise from its quiet edges, signal from its peaks) instead of tracked
//! sample by sample, and FSK's two tones are found by 2-means over the
//! whole carrier.

use num_complex::Complex32;

use crate::slicer::PulseData;

/// rtl_433's `PD_MIN_PULSE_SAMPLES` is 10 at its 250 kS/s (40 µs); here
/// 20 µs, so 30 µs chips (ERT meters) pass.
const MIN_PULSE_S: f32 = 20e-6;
const MIN_GAP_S: f32 = 0.010;
const MAX_GAP_S: f32 = 0.100;
const MAX_GAP_RATIO: i32 = 10;
/// rtl_433's `PD_MIN_PULSES`: an FSK package needs more pulses than this.
const MIN_FSK_PULSES: usize = 16;
/// rtl_433's `PD_MAX_PULSES`.
const MAX_PULSES: usize = 1200;
/// Dips in a carrier up to this long (noise) do not split it for FSK.
const DIP_S: f32 = 100e-6;
/// A carrier this long is looked at for FSK.
const MIN_FSK_RUN_S: f32 = 0.0008;
/// The two tones must be this far apart to be FSK, Hz.
const MIN_FSK_SHIFT_HZ: f32 = 4000.0;

/// One package of pulses and what was measured of it.
#[derive(Debug, Clone, Default)]
pub struct Package {
    pub fsk: bool,
    pub pulses: PulseData,
    /// Sample index (burst rate) of its first pulse.
    pub start: usize,
    /// Signal and noise, dB full scale (power), and their difference.
    pub rssi_db: f32,
    pub noise_db: f32,
    pub snr_db: f32,
    /// The carrier's mean frequency in the burst's samples, Hz (FSK: the
    /// two tones' mid-point; `f1_hz`/`f2_hz` the tones).
    pub freq_hz: f32,
    pub f1_hz: f32,
    pub f2_hz: f32,
}

/// Reusable buffers for [`detect`].
#[derive(Default)]
pub struct Detector {
    env: Vec<f32>,
    fm: Vec<f32>,
    tmp: Vec<f32>,
}

fn percentile(v: &mut [f32], q: f32) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    let k = ((v.len() - 1) as f32 * q) as usize;
    *v.select_nth_unstable_by(k, f32::total_cmp).1
}

impl Detector {
    /// Find the packages in `iq` (one burst at `rate`), appending them to
    /// `out`.
    /// `half_hz` is the filter the burst was cut out with (±): the
    /// envelope is smoothed over about half its correlation time.
    pub fn detect(&mut self, iq: &[Complex32], rate: f32, half_hz: f32, out: &mut Vec<Package>) {
        let n = iq.len();
        if n < 64 {
            return;
        }
        let w = ((rate / (4.0 * half_hz.max(1.0))) as usize).clamp(1, (rate * 16e-6) as usize + 1);
        self.env.clear();
        self.env.reserve(n);
        let mut acc = 0.0f32;
        for i in 0..n {
            acc += iq[i].norm_sqr();
            if i >= w {
                acc -= iq[i - w].norm_sqr();
            }
            self.env.push(acc.max(0.0) / w as f32);
        }
        // Levels: noise from the quietest fifth, signal from the top.
        self.tmp.clear();
        self.tmp.extend_from_slice(&self.env);
        let lo = percentile(&mut self.tmp, 0.2).max(1e-20);
        let hi = percentile(&mut self.tmp, 0.995).max(lo);
        let snr_db = 10.0 * (hi / lo).log10();
        if snr_db < 6.0 {
            return;
        }
        // The carrier's level: the median of what is clearly carrier (the
        // top percentile alone sits high on the noise riding on it, and a
        // threshold set from it trims every pulse).
        let first = (0.5 * (lo.sqrt() + hi.sqrt())).powi(2);
        self.tmp.clear();
        self.tmp
            .extend(self.env.iter().copied().filter(|&e| e > first));
        let on = percentile(&mut self.tmp, 0.5).max(lo);
        // rtl_433's threshold is half-way in amplitude, ±12 % hysteresis.
        let thr_amp = 0.5 * (lo.sqrt() + on.sqrt());
        let up = (thr_amp * 1.125).powi(2);
        let down = (thr_amp * 0.875).powi(2);
        let min_pulse = ((MIN_PULSE_S * rate) as i32).max(2);
        let min_gap_eop = (MIN_GAP_S * rate) as i32;
        let max_gap_eop = (MAX_GAP_S * rate) as i32;

        // Pulse/gap runs: (start, pulse, gap).
        let mut runs: Vec<(usize, i32, i32)> = Vec::new();
        let mut i = 0;
        let mut high = false;
        let mut run_start = 0usize;
        let mut pulse_len = 0i32;
        // The state machine of rtl_433's pulse_detect_package, without
        // its level estimators.
        while i < n {
            let e = self.env[i];
            if !high {
                if e > up {
                    high = true;
                    if pulse_len > 0 {
                        let gap = (i - run_start) as i32;
                        if gap < min_pulse {
                            // Spurious short gap: back into the pulse.
                            pulse_len += gap;
                            run_start = i;
                            i += 1;
                            continue;
                        }
                        let start = run_start - pulse_len as usize;
                        runs.push((start, pulse_len, gap));
                        pulse_len = 0;
                    }
                    run_start = i;
                }
            } else if e < down {
                high = false;
                let p = (i - run_start) as i32;
                if p < min_pulse && pulse_len == 0 {
                    // Spurious short pulse: drop it.
                    run_start = i;
                    i += 1;
                    continue;
                }
                pulse_len += p;
                run_start = i;
            }
            i += 1;
        }
        if high {
            pulse_len += (n - run_start) as i32;
            run_start = n;
        }
        if pulse_len > 0 {
            let start = run_start - pulse_len as usize;
            runs.push((start, pulse_len, (n - run_start) as i32 + 1));
        }
        if runs.is_empty() {
            return;
        }
        let rssi_db = 10.0 * hi.log10();
        let noise_db = 10.0 * lo.log10();

        // Packages by rtl_433's end-of-package rules.
        let mut k = 0;
        while k < runs.len() {
            let first = k;
            let mut max_pulse = 0;
            while k < runs.len() {
                max_pulse = max_pulse.max(runs[k].1);
                let gap = runs[k].2;
                k += 1;
                if (gap > MAX_GAP_RATIO * max_pulse && gap > min_gap_eop)
                    || gap > max_gap_eop
                    || k - first >= MAX_PULSES
                {
                    break;
                }
            }
            let pkg = &runs[first..k];
            // FSK inside a long carrier. Pulses with only short dips between
            // them (noise at a low SNR) are one stretch of carrier.
            let mut fsk_found = false;
            let max_dip = (DIP_S * rate) as i32;
            let mut a = 0;
            while a < pkg.len() {
                let mut b = a;
                let mut on = pkg[a].1;
                while b + 1 < pkg.len() && pkg[b].2 <= max_dip {
                    b += 1;
                    on += pkg[b].1;
                }
                let start = pkg[a].0;
                let end = pkg[b].0 + pkg[b].1 as usize;
                let span = (end - start) as f32;
                if span >= MIN_FSK_RUN_S * rate
                    && on as f32 >= 0.85 * span
                    && let Some(f) = self.fsk(&iq[start..end], rate, min_pulse)
                {
                    out.push(Package {
                        start,
                        rssi_db,
                        noise_db,
                        snr_db,
                        ..f
                    });
                    fsk_found = true;
                }
                a = b + 1;
            }
            if fsk_found {
                continue;
            }
            let mut pulses = PulseData {
                sample_rate: rate,
                ..Default::default()
            };
            let mut fsum = 0.0;
            let mut fcount = 0;
            for &(start, p, g) in pkg {
                pulses.push(p, g);
                // The carrier's frequency over the middle of each pulse.
                let a = start + p as usize / 4;
                let b = start + 3 * p as usize / 4;
                for j in a.max(1)..b {
                    let d = iq[j] * iq[j - 1].conj();
                    fsum += d.arg();
                    fcount += 1;
                }
            }
            let freq_hz = if fcount > 0 {
                fsum / fcount as f32 * rate / std::f32::consts::TAU
            } else {
                0.0
            };
            out.push(Package {
                fsk: false,
                pulses,
                start: pkg[0].0,
                rssi_db,
                noise_db,
                snr_db,
                freq_hz,
                f1_hz: freq_hz,
                f2_hz: freq_hz,
            });
        }
    }

    /// FSK pulses (high tone = pulse, low tone = gap, as rtl_433's) from
    /// one carrier, if it has two tones.
    fn fsk(&mut self, iq: &[Complex32], rate: f32, min_pulse: i32) -> Option<Package> {
        let n = iq.len();
        // FM discriminator, smoothed over ~4 µs.
        let w = ((rate * 4e-6) as usize).max(1);
        let k_hz = rate / std::f32::consts::TAU;
        self.fm.clear();
        let mut acc = 0.0f32;
        let mut raw = Vec::with_capacity(n);
        for j in 1..n {
            raw.push((iq[j] * iq[j - 1].conj()).arg() * k_hz);
        }
        for j in 0..raw.len() {
            acc += raw[j];
            if j >= w {
                acc -= raw[j - w];
            }
            self.fm.push(acc / w.min(j + 1) as f32);
        }
        // Two tones by 2-means, from the 10th and 90th percentiles.
        self.tmp.clear();
        self.tmp.extend_from_slice(&self.fm);
        let mut f2 = percentile(&mut self.tmp, 0.1);
        let mut f1 = percentile(&mut self.tmp, 0.9);
        for _ in 0..4 {
            let mid = 0.5 * (f1 + f2);
            let (mut s1, mut n1, mut s2, mut n2) = (0.0f64, 0usize, 0.0f64, 0usize);
            for &v in &self.fm {
                if v > mid {
                    s1 += v as f64;
                    n1 += 1;
                } else {
                    s2 += v as f64;
                    n2 += 1;
                }
            }
            if n1 == 0 || n2 == 0 {
                return None;
            }
            f1 = (s1 / n1 as f64) as f32;
            f2 = (s2 / n2 as f64) as f32;
        }
        if f1 - f2 < MIN_FSK_SHIFT_HZ {
            return None;
        }
        // The tones must stand clear of the noise: the spread about each
        // tone well under the shift.
        let mid = 0.5 * (f1 + f2);
        let mut var = 0.0f64;
        for &v in &self.fm {
            let c = if v > mid { f1 } else { f2 };
            var += ((v - c) as f64).powi(2);
        }
        let sd = (var / self.fm.len() as f64).sqrt() as f32;
        if sd > 0.35 * (f1 - f2) {
            return None;
        }
        // Runs of each tone, with hysteresis.
        let hyst = 0.1 * (f1 - f2);
        let mut runs: Vec<(bool, i32)> = Vec::new();
        let mut high = self.fm[0] > mid;
        let mut len = 0i32;
        for &v in &self.fm {
            let flip = if high { v < mid - hyst } else { v > mid + hyst };
            if flip {
                runs.push((high, len));
                high = !high;
                len = 0;
            }
            len += 1;
        }
        runs.push((high, len));
        // A run shorter than a pulse is a glitch: it and the run after it
        // join the run before (rtl_433 rewinds the same way).
        let mut merged: Vec<(bool, i32)> = Vec::with_capacity(runs.len());
        let mut j = 0;
        while j < runs.len() {
            let (lvl, l) = runs[j];
            if l < min_pulse && !merged.is_empty() {
                let last = merged.last_mut().unwrap();
                last.1 += l;
                if j + 1 < runs.len() {
                    last.1 += runs[j + 1].1;
                    j += 1;
                }
            } else {
                merged.push((lvl, l));
            }
            j += 1;
        }
        // Pulses (high tone) and the gaps after them; a carrier opening on
        // the low tone starts with a zero-width pulse, as rtl_433's.
        let mut pulses = PulseData {
            sample_rate: rate,
            ..Default::default()
        };
        let mut pending: Option<i32> = None;
        for (lvl, l) in merged {
            if lvl {
                if let Some(p) = pending.take() {
                    pulses.push(p, 0);
                }
                pending = Some(l);
            } else {
                pulses.push(pending.take().unwrap_or(0), l);
            }
        }
        if let Some(p) = pending {
            pulses.push(p, 0);
        }
        if pulses.len() <= MIN_FSK_PULSES {
            return None;
        }
        Some(Package {
            fsk: true,
            pulses,
            freq_hz: mid,
            f1_hz: f1,
            f2_hz: f2,
            ..Default::default()
        })
    }
}
