// made by CasualArclamp
//! Waterfall history, colour map and GPU ring texture (from DecDVB-T's GUI,
//! same author, plus [`History::push_at`], which keeps the rows in one frame
//! of reference while the radio is retuned, so a dragged band glides).
//!
//! **Anchoring.** The rows are held as if made at one centre, the anchor. A
//! row made elsewhere (the radio retuned) is shifted by whole bins as it
//! goes in, the edge it leaves filled with the floor, so it costs one row's
//! work and the ring texture takes it like any other row. Only when the
//! radio has moved more than a quarter of the span from the anchor is
//! everything shifted and redrawn once, about the new centre. Dragging used
//! to shift and redraw every row at every retune (eleven times a second).
//!
//! Adapted from DecSAT's `waterfall.rs` (same author), whose ring-texture
//! technique is from DecDRM's `ring_image.rs` (same
//! author, GPL-2.0-or-later, compatible with this project's GPL-3): rows arrive
//! 25 times a second and a full history is megabytes, so each new row is
//! uploaded on its own just above the newest one (wrapping round) and the image
//! is drawn with its texture coordinates shifted so the newest row is at the
//! top — the texture repeats vertically. Each row keeps the colours of the
//! levels it arrived with, as in most SDR waterfalls.

use std::collections::VecDeque;
use std::ops::Range;

use eframe::egui::{
    self, Color32, ColorImage, Rect, TextureHandle, TextureId, TextureOptions, TextureWrapMode,
    pos2,
};

/// Rows kept by default: ~25 s at 20 rows/s. A history can hold another
/// number (see [`History::set_capacity`]): the faster the rows come, the more
/// are needed to hold the same time.
pub const ROWS: usize = 512;
/// Smallest level range shown, dB.
const MIN_RANGE_DB: f32 = 25.0;

/// The colour map: dark violet through orange to pale yellow (the "inferno"
/// family DecDRM uses), so the two apps read the same.
pub fn palette() -> Vec<Color32> {
    const STOPS: [(f32, [u8; 3]); 6] = [
        (0.0, [0, 0, 4]),
        (0.2, [40, 11, 84]),
        (0.4, [101, 21, 110]),
        (0.6, [159, 42, 99]),
        (0.8, [237, 105, 37]),
        (1.0, [252, 255, 164]),
    ];
    (0..256)
        .map(|i| {
            let t = i as f32 / 255.0;
            let k = STOPS
                .windows(2)
                .position(|w| t <= w[1].0)
                .unwrap_or(STOPS.len() - 2);
            let ((t0, c0), (t1, c1)) = (STOPS[k], STOPS[k + 1]);
            let f = (t - t0) / (t1 - t0);
            let mix =
                |a: u8, b: u8| (f32::from(a) + f * (f32::from(b) - f32::from(a))).round() as u8;
            Color32::from_rgb(mix(c0[0], c1[0]), mix(c0[1], c1[1]), mix(c0[2], c1[2]))
        })
        .collect()
}

/// Rows of the wideband spectrum, newest last, plus display levels.
#[derive(Debug, Clone)]
pub struct History {
    rows: VecDeque<Vec<f32>>,
    width: usize,
    pushed: u64,
    epoch: u64,
    /// Automatic levels following the floor and the peaks.
    pub auto: bool,
    /// Manual levels, dB.
    pub manual: (f32, f32),
    /// Rows kept.
    capacity: usize,
    /// Whether the floor and the peak are followed (a sort of every row):
    /// not needed with fixed levels.
    pub track: bool,
    floor: Option<f32>,
    peak: Option<f32>,
    /// The centre and rate the rows are held at (see the module notes).
    anchor: Option<(f64, f64)>,
}

impl Default for History {
    fn default() -> Self {
        History {
            rows: VecDeque::with_capacity(ROWS),
            width: 0,
            pushed: 0,
            epoch: 0,
            auto: true,
            manual: (-110.0, -40.0),
            capacity: ROWS,
            track: true,
            floor: None,
            peak: None,
            anchor: None,
        }
    }
}

impl History {
    /// Forget everything (a new source or FFT size).
    pub fn reset(&mut self) {
        let (auto, manual, epoch, track, capacity) = (
            self.auto,
            self.manual,
            self.epoch + 1,
            self.track,
            self.capacity,
        );
        *self = History {
            auto,
            manual,
            epoch,
            track,
            capacity,
            ..History::default()
        };
    }

    /// Keep this many rows (starting afresh if it is a change).
    pub fn set_capacity(&mut self, rows: usize) {
        let rows = rows.clamp(64, 4096);
        if rows != self.capacity {
            self.capacity = rows;
            self.reset();
        }
    }

    pub fn push(&mut self, row: Vec<f32>) {
        if row.is_empty() {
            return;
        }
        if row.len() != self.width {
            self.reset();
            self.width = row.len();
        }
        if self.track {
            // Track floor (20th percentile) and peak (99.5th) with some
            // inertia, so the colours do not pump with every row.
            let mut sorted = row.clone();
            sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let q = |p: f32| sorted[((sorted.len() - 1) as f32 * p) as usize];
            let (f, p) = (q(0.2), q(0.995));
            self.floor = Some(self.floor.map_or(f, |x| x + 0.05 * (f - x)));
            self.peak = Some(self.peak.map_or(p, |x| x + 0.05 * (p - x)));
        }

        if self.rows.len() >= self.capacity {
            self.rows.pop_front();
        }
        self.rows.push_back(row);
        self.pushed += 1;
    }

    /// Push a row made at `center_hz`, `rate` wide, into the anchor's frame
    /// (see the module notes). Returns true when the rows were re-anchored
    /// (everything redrawn).
    pub fn push_at(&mut self, mut row: Vec<f32>, center_hz: f64, rate: f64) -> bool {
        let fill = self.floor.unwrap_or(-120.0) - 6.0;
        let fresh = match self.anchor {
            Some((_, r)) => r != rate || row.len() != self.width,
            None => true,
        };
        if fresh {
            self.push(row);
            self.anchor = Some((center_hz, rate));
            return false;
        }
        let (anchor, _) = self.anchor.expect("checked above");
        let bin = rate / self.width as f64;
        let mut moved = false;
        if (center_hz - anchor).abs() > rate / 4.0 {
            // Re-anchor on the radio: shift what is held once.
            let bins = ((center_hz - anchor) / bin).round() as i64;
            for old in self.rows.iter_mut() {
                shift(old, bins, fill);
            }
            self.anchor = Some((center_hz, rate));
            self.epoch += 1;
            moved = true;
        } else {
            // This row into the anchor's frame: what is at bin k of the row
            // is at bin k + bins about the anchor.
            let bins = ((center_hz - anchor) / bin).round() as i64;
            shift(&mut row, -bins, fill);
        }
        self.push(row);
        moved
    }

    /// The centre and rate the rows are drawn about.
    pub fn anchor(&self) -> Option<(f64, f64)> {
        self.anchor
    }

    /// Colour range, dB (low, high).
    pub fn levels(&self) -> (f32, f32) {
        if !self.auto {
            return self.manual;
        }
        match (self.floor, self.peak) {
            (Some(f), Some(p)) => (f - 3.0, (p + 3.0).max(f + MIN_RANGE_DB)),
            _ => self.manual,
        }
    }
}

/// Move `row` by `bins` (positive: towards lower bins, as when the centre
/// it is drawn about moves up), filling what comes into view with `fill`.
fn shift(row: &mut [f32], bins: i64, fill: f32) {
    let n = row.len() as i64;
    if bins.abs() >= n {
        row.fill(fill);
    } else if bins > 0 {
        row.rotate_left(bins as usize);
        row[(n - bins) as usize..].fill(fill);
    } else if bins < 0 {
        row.rotate_right((-bins) as usize);
        row[..(-bins) as usize].fill(fill);
    }
}

/// Linear filtering, repeating vertically (the ring's wrap-round).
const OPTIONS: TextureOptions = TextureOptions {
    wrap_mode: TextureWrapMode::Repeat,
    ..TextureOptions::LINEAR
};

/// The waterfall's texture.
#[derive(Default)]
pub struct RingImage {
    handle: Option<TextureHandle>,
    size: [usize; 2],
    head: usize,
    uploaded: u64,
    epoch: Option<u64>,
    lut: Vec<Color32>,
}

impl RingImage {
    /// Bring the texture up to date and return it with the texture coordinates
    /// that put the newest row at the top; `None` while there are no rows.
    pub fn update(&mut self, ctx: &egui::Context, h: &History) -> Option<(TextureId, Rect)> {
        let width = h.rows.back()?.len().max(1);
        let cap = h.capacity;
        if self.lut.is_empty() {
            self.lut = palette();
        }
        let levels = h.levels();
        let new = h.pushed.saturating_sub(self.uploaded);
        if self.handle.is_none()
            || self.epoch != Some(h.epoch)
            || self.size != [width, cap]
            || new >= cap as u64
        {
            let image = ColorImage::new(
                [width, cap],
                render(h.rows.iter().rev().take(cap), width, cap, levels, &self.lut),
            );
            match &mut self.handle {
                Some(t) => t.set(image, OPTIONS),
                None => self.handle = Some(ctx.load_texture("waterfall", image, OPTIONS)),
            }
            self.size = [width, cap];
            self.head = 0;
            self.epoch = Some(h.epoch);
        } else if new > 0 {
            let k = (new as usize).min(h.rows.len());
            let block = render(h.rows.iter().rev().take(k), width, k, levels, &self.lut);
            let (head, parts) = plan(self.head, k, cap);
            let handle = self.handle.as_mut()?;
            for (row, range) in parts {
                let pixels = block[range.start * width..range.end * width].to_vec();
                handle.set_partial(
                    [0, row],
                    ColorImage::new([width, range.len()], pixels),
                    OPTIONS,
                );
            }
            self.head = head;
        }
        self.uploaded = h.pushed;
        let v0 = self.head as f32 / cap as f32;
        Some((
            self.handle.as_ref()?.id(),
            Rect::from_min_max(pos2(0.0, v0), pos2(1.0, v0 + 1.0)),
        ))
    }
}

/// Where `k` new rows go in a ring of `cap` rows whose newest row is at texture
/// row `head`: the new head, and the uploads as (first texture row, range of
/// the new rows counted newest first).
fn plan(head: usize, k: usize, cap: usize) -> (usize, Vec<(usize, Range<usize>)>) {
    let new_head = (head + cap - k % cap) % cap;
    if k <= head {
        return (new_head, vec![(new_head, 0..k)]);
    }
    let bottom = k - head;
    let mut parts = vec![(cap - bottom, 0..bottom)];
    if head > 0 {
        parts.push((0, bottom..k));
    }
    (new_head, parts)
}

/// Colour `rows` (newest first) into `width` × `height` pixels.
fn render<'a>(
    rows: impl Iterator<Item = &'a Vec<f32>>,
    width: usize,
    height: usize,
    (lo, hi): (f32, f32),
    lut: &[Color32],
) -> Vec<Color32> {
    let mut pixels = vec![lut[0]; width * height];
    let top = (lut.len() - 1) as f32;
    let scale = top / (hi - lo).max(1e-3);
    for (line, row) in pixels.chunks_exact_mut(width).zip(rows) {
        for (px, &db) in line.iter_mut().zip(row) {
            *px = lut[((db - lo) * scale).clamp(0.0, top) as usize];
        }
    }
    pixels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_plan_wraps() {
        assert_eq!(plan(5, 3, 10), (2, vec![(2, 0..3)]));
        assert_eq!(plan(1, 3, 10), (8, vec![(8, 0..2), (0, 2..3)]));
        assert_eq!(plan(0, 2, 10), (8, vec![(8, 0..2)]));
    }

    #[test]
    fn auto_levels_bracket_floor_and_peak() {
        let mut h = History::default();
        let mut row = vec![-100.0f32; 1000];
        row[500] = -40.0;
        row[501] = -40.0;
        row[502] = -40.0;
        row[503] = -40.0;
        row[504] = -40.0;
        row[505] = -40.0;
        for _ in 0..200 {
            h.push(row.clone());
        }
        let (lo, hi) = h.levels();
        assert!(lo < -100.0 && lo > -110.0, "lo {lo}");
        assert!(hi > -45.0, "hi {hi}");
    }

    #[test]
    fn a_retuned_row_goes_in_shifted_and_the_rest_stay() {
        // 100 bins over 1 MHz: a carrier at bin 60. A row made 100 kHz
        // higher has it at bin 50; about the anchor it is still at 60.
        let mut h = History::default();
        let mut row = vec![-100.0f32; 100];
        row[60] = -40.0;
        h.push_at(row, 100e6, 1e6);
        let mut moved = vec![-100.0f32; 100];
        moved[50] = -40.0;
        assert!(!h.push_at(moved, 100.1e6, 1e6), "no redraw for a small move");
        assert_eq!(h.anchor(), Some((100e6, 1e6)));
        assert_eq!(h.rows[0][60], -40.0);
        assert_eq!(h.rows[1][60], -40.0);
        // The edge that came into view holds no made-up signal.
        assert!(h.rows[1][95] < -90.0);
    }

    #[test]
    fn a_long_move_re_anchors_once() {
        let mut h = History::default();
        let mut row = vec![-100.0f32; 100];
        row[60] = -40.0;
        h.push_at(row, 100e6, 1e6);
        let e = h.epoch;
        // 300 kHz up: past a quarter of the span.
        let mut moved = vec![-100.0f32; 100];
        moved[30] = -40.0;
        assert!(h.push_at(moved, 100.3e6, 1e6));
        assert_eq!(h.anchor(), Some((100.3e6, 1e6)));
        assert_ne!(h.epoch, e);
        assert_eq!(h.rows[0][30], -40.0, "the old row moved with the anchor");
        assert_eq!(h.rows[1][30], -40.0);
    }

    #[test]
    fn width_change_starts_afresh() {
        let mut h = History::default();
        h.push(vec![0.0; 100]);
        h.push(vec![0.0; 100]);
        let e = h.epoch;
        h.push(vec![0.0; 200]);
        assert_eq!(h.rows.len(), 1);
        assert_ne!(h.epoch, e);
    }
}
