// made by CasualArclamp
//! Pulse slicers: a package of pulse and gap widths into rows of bits, by
//! the device's line coding and timings, then the device's decoder.
//!
//! Ported from rtl_433's `pulse_slicer.c` (Copyright (C) 2015 Tommy
//! Vestermark, GPL-2.0-or-later), one function each, same rules. The OSv1
//! slicer is not ported (no Oregon v1 device here).

use crate::bits::BitBuffer;
use crate::device::{Ctx, Device, Modulation};

/// rtl_433's `pulse_data_t`: one package of pulses (mark, or carrier on)
/// and the gaps after them, in samples at `sample_rate`.
#[derive(Debug, Clone, Default)]
pub struct PulseData {
    pub sample_rate: f32,
    pub pulse: Vec<i32>,
    pub gap: Vec<i32>,
}

impl PulseData {
    pub fn clear(&mut self) {
        self.pulse.clear();
        self.gap.clear();
    }

    pub fn len(&self) -> usize {
        self.pulse.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pulse.is_empty()
    }

    pub fn push(&mut self, pulse: i32, gap: i32) {
        self.pulse.push(pulse);
        self.gap.push(gap);
    }
}

/// The device's widths in samples.
struct Widths {
    short: i32,
    long: i32,
    reset: i32,
    gap: i32,
    sync: i32,
    tol: i32,
}

fn widths(p: &PulseData, d: &Device) -> Option<Widths> {
    let spu = p.sample_rate / 1e6;
    let w = Widths {
        short: (d.short_width * spu) as i32,
        long: (d.long_width * spu) as i32,
        // 10 % over rtl_433's: some limits sit within a few microseconds of
        // the longest symbol (ERT meters: 64 µs against 60), closer than
        // the jitter of edges found in noise.
        reset: (d.reset_limit * 1.1 * spu) as i32,
        gap: (d.gap_limit * spu) as i32,
        sync: (d.sync_width * spu) as i32,
        tol: (d.tolerance * spu) as i32,
    };
    // A width that rounds to zero samples: the rate is too low for it.
    let zero = |us: f32, s: i32| us > 0.0 && s <= 0;
    if zero(d.short_width, w.short)
        || zero(d.long_width, w.long)
        || zero(d.reset_limit, w.reset)
        || zero(d.gap_limit, w.gap)
        || zero(d.sync_width, w.sync)
        || zero(d.tolerance, w.tol)
    {
        return None;
    }
    Some(w)
}

/// A decode function or closure (a flex decoder carries its spec).
pub type Decode<'a> = &'a dyn Fn(&mut Ctx, &mut BitBuffer) -> i32;

/// rtl_433's `account_event`: run the decoder, then clear the rows.
fn account(dec: Decode, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let ret = dec(ctx, bits);
    bits.clear();
    ret.max(0)
}

/// Slice `p` by `dev`'s coding and run its decoder on each transmission;
/// returns the messages output.
pub fn slice(p: &PulseData, dev: &Device, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    slice_with(p, dev, &dev.decode, bits, ctx)
}

/// As [`slice`], with `dec` in place of the device's own decode function
/// (`dev` gives only the coding and timings).
pub fn slice_with(
    p: &PulseData,
    dev: &Device,
    dec: Decode,
    bits: &mut BitBuffer,
    ctx: &mut Ctx,
) -> i32 {
    bits.clear();
    if p.is_empty() {
        return 0;
    }
    let Some(w) = widths(p, dev) else { return 0 };
    match dev.modulation {
        Modulation::OokPcm | Modulation::FskPcm => pcm(p, dev, dec, &w, bits, ctx),
        Modulation::OokPpm => ppm(p, dec, &w, bits, ctx),
        Modulation::OokPwm | Modulation::FskPwm => pwm(p, dec, &w, bits, ctx),
        Modulation::OokManchesterZeroBit | Modulation::FskManchesterZeroBit => {
            mzb(p, dec, &w, bits, ctx)
        }
        Modulation::OokDmc => dmc(p, dec, &w, bits, ctx),
        Modulation::OokPiwmRaw => piwm_raw(p, dev, dec, &w, bits, ctx),
        Modulation::OokPiwmDc => piwm_dc(p, dec, &w, bits, ctx),
        Modulation::OokNrzs => nrzs(p, dec, &w, bits, ctx),
    }
}

/// rtl_433's `pulse_slicer_pcm`: NRZ when short == long, else RZ; the bit
/// period is measured from a preamble (or every in-tolerance bit).
fn pcm(
    p: &PulseData,
    dev: &Device,
    dec: Decode,
    w: &Widths,
    bits: &mut BitBuffer,
    ctx: &mut Ctx,
) -> i32 {
    let spu = p.sample_rate / 1e6;
    let mut f_short = if dev.short_width > 0.0 {
        1.0 / (dev.short_width * spu)
    } else {
        0.0
    };
    let mut f_long = if dev.long_width > 0.0 {
        1.0 / (dev.long_width * spu)
    } else {
        0.0
    };
    let n = p.len();
    let gap_limit = if w.gap != 0 { w.gap } else { w.reset };
    let max_zeros = if w.long > 0 { gap_limit / w.long } else { 0 };
    let tol = if w.tol <= 0 { w.long / 4 } else { w.tol };
    let (s_short, s_long) = (w.short, w.long);
    let mut min_count = if s_short == s_long { 12 } else { 4 };
    let mut preamble_len = 0;

    if s_short != s_long {
        // RZ: a run of bit-wide pulses (a preamble) tunes the period.
        let mut i = 0;
        while i < n {
            let (mut sw, mut lw, mut count) = (0, 0, 0);
            while i < n
                && (p.pulse[i] - s_short).abs() <= tol
                && (p.pulse[i] + p.gap[i] - s_long).abs() <= tol
            {
                sw += p.pulse[i];
                lw += p.pulse[i] + p.gap[i];
                count += 1;
                i += 1;
            }
            if count >= min_count {
                f_long = count as f32 / lw as f32;
                f_short = count as f32 / sw as f32;
                min_count = count;
                preamble_len = count;
            }
            i += 1;
        }
        if preamble_len == 0 {
            let (mut sw, mut lw, mut count) = (0, 0, 0);
            for i in 0..n {
                if (p.pulse[i] - s_short).abs() <= tol
                    && (p.pulse[i] + p.gap[i] - s_long).abs() <= tol
                {
                    sw += p.pulse[i];
                    lw += p.pulse[i] + p.gap[i];
                    count += 1;
                }
            }
            if count > 8 {
                f_long = count as f32 / lw as f32;
                f_short = count as f32 / sw as f32;
            }
        }
    } else {
        // NRZ: a run of alternating single bits tunes the period.
        let mut i = 0;
        while i < n {
            let (mut width, mut count) = (0, 0);
            while i < n
                && (p.pulse[i] as f32 * f_short + 0.5) as i32 == 1
                && (p.gap[i] as f32 * f_long + 0.5) as i32 == 1
            {
                width += p.pulse[i] + p.gap[i];
                count += 2;
                i += 1;
            }
            if count >= min_count {
                f_short = count as f32 / width as f32;
                f_long = f_short;
                min_count = count;
                preamble_len = count;
            }
            i += 1;
        }
        if preamble_len == 0 {
            let (mut width, mut count) = (0, 0);
            for i in 0..n {
                let (pl, gp) = (p.pulse[i], p.gap[i]);
                if (pl - s_short).abs() <= tol {
                    width += pl;
                    count += 1;
                }
                if (pl - 2 * s_short).abs() <= tol {
                    width += pl;
                    count += 2;
                }
                if (gp - s_long).abs() <= tol {
                    width += gp;
                    count += 1;
                }
                if (gp - 2 * s_long).abs() <= tol {
                    width += gp;
                    count += 2;
                }
            }
            if count > 20 {
                f_short = count as f32 / width as f32;
                f_long = f_short;
            }
        }
    }

    let mut events = 0;
    for i in 0..n {
        let highs = (p.pulse[i] as f32 * f_short + 0.5) as i32;
        let lows = ((p.gap[i] + s_short - s_long) as f32 * f_long + 0.5) as i32;
        for _ in 0..highs {
            bits.add_bit(1);
        }
        for _ in 0..lows.min(max_zeros) {
            bits.add_bit(0);
        }
        if s_short != s_long && (p.pulse[i] - s_short).abs() > tol {
            bits.clear();
        } else if p.gap[i] > gap_limit && p.gap[i] <= w.reset {
            bits.add_row();
        }
        if (i == n - 1 || p.gap[i] > w.reset) && (bits.bits_per_row[0] > 0 || bits.num_rows > 1) {
            events += account(dec, bits, ctx);
        }
    }
    events
}

/// rtl_433's `pulse_slicer_ppm`: the gap after each pulse codes the bit.
fn ppm(p: &PulseData, dec: Decode, w: &Widths, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let (zero_l, zero_u, one_l, one_u, mut sync_l, mut sync_u);
    if w.tol > 0 {
        zero_l = w.short - w.tol;
        zero_u = w.short + w.tol;
        one_l = w.long - w.tol;
        one_u = w.long + w.tol;
        sync_l = 0;
        sync_u = 0;
        if w.sync > 0 {
            sync_l = w.sync - w.tol;
            sync_u = w.sync + w.tol;
        }
    } else {
        zero_l = 0;
        zero_u = (w.short + w.long) / 2 + 1;
        one_l = zero_u - 1;
        one_u = if w.gap != 0 { w.gap } else { w.reset };
        sync_l = 0;
        sync_u = 0;
    }
    let n = p.len();
    let mut events = 0;
    for i in 0..n {
        let g = p.gap[i];
        if g > zero_l && g < zero_u {
            bits.add_bit(0);
        } else if g > one_l && g < one_u {
            bits.add_bit(1);
        } else if g > sync_l && g < sync_u {
            bits.add_sync();
        } else if g < w.reset {
            bits.add_row();
        }
        if (i == n - 1 || g >= w.reset) && (bits.bits_per_row[0] > 0 || bits.num_rows > 1) {
            events += account(dec, bits, ctx);
        }
    }
    events
}

/// rtl_433's `pulse_slicer_pwm`: each pulse's width codes the bit.
fn pwm(p: &PulseData, dec: Decode, w: &Widths, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let (s, l, y) = (w.short, w.long, w.sync);
    let (one_l, one_u, zero_l, zero_u, sync_l, sync_u);
    if w.tol > 0 {
        one_l = s - w.tol;
        one_u = s + w.tol;
        zero_l = l - w.tol;
        zero_u = l + w.tol;
        if y > 0 {
            sync_l = y - w.tol;
            sync_u = y + w.tol;
        } else {
            sync_l = 0;
            sync_u = 0;
        }
    } else if y <= 0 {
        one_l = 0;
        one_u = (s + l) / 2 + 1;
        zero_l = one_u - 1;
        zero_u = i32::MAX;
        sync_l = 0;
        sync_u = 0;
    } else if y < s {
        sync_l = 0;
        sync_u = (y + s) / 2 + 1;
        one_l = sync_u - 1;
        one_u = (s + l) / 2 + 1;
        zero_l = one_u - 1;
        zero_u = i32::MAX;
    } else if y < l {
        one_l = 0;
        one_u = (s + y) / 2 + 1;
        sync_l = one_u - 1;
        sync_u = (y + l) / 2 + 1;
        zero_l = sync_u - 1;
        zero_u = i32::MAX;
    } else {
        one_l = 0;
        one_u = (s + l) / 2 + 1;
        zero_l = one_u - 1;
        zero_u = (l + y) / 2 + 1;
        sync_l = zero_u - 1;
        sync_u = i32::MAX;
    }
    let n = p.len();
    let mut events = 0;
    for i in 0..n {
        let pl = p.pulse[i];
        if pl > one_l && pl < one_u {
            bits.add_bit(1);
        } else if pl > zero_l && pl < zero_u {
            bits.add_bit(0);
        } else if pl > sync_l && pl < sync_u {
            bits.add_sync();
        } else if pl <= one_l {
            // a spurious short pulse: ignored
        } else {
            bits.add_row();
        }
        if (i == n - 1 || p.gap[i] > w.reset) && bits.num_rows > 0 {
            events += account(dec, bits, ctx);
        } else if w.gap > 0
            && p.gap[i] > w.gap
            && bits.num_rows > 0
            && bits.bits_per_row[bits.num_rows - 1] > 0
        {
            bits.add_row();
        }
    }
    events
}

/// rtl_433's `pulse_slicer_manchester_zerobit`: the first rising edge is
/// a 0, then each mid-bit edge is a bit (falling 1, rising 0).
fn mzb(p: &PulseData, dec: Decode, w: &Widths, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let s = w.short;
    let s15 = s as f32 * 1.5;
    let mut events = 0;
    let mut since = 0;
    bits.add_bit(0);
    let n = p.len();
    for i in 0..n {
        let (pl, gp) = (p.pulse[i], p.gap[i]);
        if w.tol > 0
            && (pl < s - w.tol || pl > s * 2 + w.tol || gp < s - w.tol || gp > s * 2 + w.tol)
        {
            if pl as f32 > s15 && pl <= s * 2 + w.tol {
                bits.add_bit(1);
            }
            bits.add_row();
            bits.add_bit(0);
            since = 0;
        } else if (pl + since) as f32 > s15 {
            bits.add_bit(1);
            since = 0;
        } else {
            since += pl;
        }
        if (i == n - 1 || gp > w.reset) && bits.num_rows > 0 {
            events += account(dec, bits, ctx);
            bits.add_bit(0);
            since = 0;
        } else if (gp + since) as f32 > s15 {
            bits.add_bit(0);
            since = 0;
        } else {
            since += gp;
        }
    }
    events
}

#[inline]
fn symbol(p: &PulseData, n: usize) -> i32 {
    if n.is_multiple_of(2) {
        p.pulse[n / 2]
    } else {
        p.gap[n / 2]
    }
}

/// rtl_433's `pulse_slicer_dmc`: differential Manchester.
fn dmc(p: &PulseData, dec: Decode, w: &Widths, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let mut events = 0;
    let total = p.len() * 2;
    let mut n = 0;
    while n < total {
        let mut sym = symbol(p, n);
        if (sym - w.short).abs() < w.tol {
            bits.add_bit(1);
            sym = if n + 1 < total {
                n += 1;
                symbol(p, n)
            } else {
                0
            };
            if (sym - w.short).abs() > w.tol {
                if sym >= w.reset - w.tol {
                    n -= 1;
                } else if bits.num_rows > 0 && bits.bits_per_row[bits.num_rows - 1] > 0 {
                    bits.add_row();
                }
            }
        } else if (sym - w.long).abs() < w.tol {
            bits.add_bit(0);
        } else if sym >= w.reset - w.tol && bits.num_rows > 0 {
            events += account(dec, bits, ctx);
        }
        n += 1;
    }
    events
}

/// rtl_433's `pulse_slicer_piwm_raw`.
fn piwm_raw(
    p: &PulseData,
    dev: &Device,
    dec: Decode,
    w: &Widths,
    bits: &mut BitBuffer,
    ctx: &mut Ctx,
) -> i32 {
    let f_short = if dev.short_width > 0.0 {
        1.0 / (dev.short_width * p.sample_rate / 1e6)
    } else {
        0.0
    };
    let total = p.len() * 2;
    let mut events = 0;
    for n in 0..total {
        let sym = symbol(p, n);
        let mut k = (sym as f32 * f_short + 0.5) as i32;
        if sym > w.long {
            bits.add_row();
        } else if (sym - k * w.short).abs() < w.tol {
            while k > 0 {
                bits.add_bit(1 - (n % 2) as u8);
                k -= 1;
            }
        } else if sym < w.reset && bits.num_rows > 0 && bits.bits_per_row[bits.num_rows - 1] > 0 {
            bits.add_row();
        }
        if (n == total - 1 || sym > w.reset) && bits.num_rows > 0 {
            events += account(dec, bits, ctx);
        }
    }
    events
}

/// rtl_433's `pulse_slicer_piwm_dc`.
fn piwm_dc(p: &PulseData, dec: Decode, w: &Widths, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let total = p.len() * 2;
    let mut events = 0;
    for n in 0..total {
        let sym = symbol(p, n);
        if (sym - w.short).abs() < w.tol {
            bits.add_bit(1);
        } else if (sym - w.long).abs() < w.tol {
            bits.add_bit(0);
        } else if sym < w.reset && bits.num_rows > 0 && bits.bits_per_row[bits.num_rows - 1] > 0 {
            bits.add_row();
        }
        if (n == total - 1 || sym > w.reset) && bits.num_rows > 0 {
            events += account(dec, bits, ctx);
        }
    }
    events
}

/// rtl_433's `pulse_slicer_nrzs`.
fn nrzs(p: &PulseData, dec: Decode, w: &Widths, bits: &mut BitBuffer, ctx: &mut Ctx) -> i32 {
    let limit = w.short.max(1);
    let n = p.len();
    let mut events = 0;
    for i in 0..n {
        if p.pulse[i] > limit {
            for _ in 0..p.pulse[i] / limit {
                bits.add_bit(1);
            }
            bits.add_bit(0);
        } else if p.pulse[i] < limit {
            bits.add_bit(0);
        }
        if i == n - 1 || p.gap[i] >= w.reset {
            events += account(dec, bits, ctx);
        }
    }
    events
}
