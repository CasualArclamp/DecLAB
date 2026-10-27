// made by CasualArclamp
//! The band: the spectrum over the waterfall, one frequency axis for both,
//! and the decoder's channel bracketed. Drawing follows DecDVB-T's band view
//! (same author); the view here can be zoomed and moved:
//!
//! - **left click** tunes the decoder's channel to the frequency clicked;
//! - **middle-drag** moves the band (a radio is retuned to follow);
//! - **middle click** (no drag) opens the decoder wheel where the pointer is;
//! - **scroll** zooms about the pointer.
//!
//! The rows were made at the radio's centre and rate, which need not be the
//! view's: each is drawn where its own frequencies fall, so a drag shows at
//! once, before the retuned radio's rows arrive.

use eframe::egui::{
    self, Align2, Color32, CornerRadius, CursorIcon, FontId, Mesh, PointerButton, Pos2, Rect,
    Sense, Shape, Stroke, TextureId, pos2,
};

/// Width reserved left of the plot for the dB scale.
const LEFT: f32 = 48.0;
/// Height of the frequency scale under the spectrum.
const AXIS_H: f32 = 20.0;
const TRACE: Color32 = Color32::from_rgb(150, 205, 255);

/// What to draw.
pub struct BandInput<'a> {
    /// Smoothed spectrum, dBFS, the most negative frequency first, made at
    /// `row_center_hz` over `row_rate` (empty before the first row).
    pub spectrum_db: &'a [f32],
    pub row_center_hz: f64,
    pub row_rate: f64,
    /// The view: its centre and width, Hz.
    pub view_center_hz: f64,
    pub view_span_hz: f64,
    /// Colour range of the waterfall, which the spectrum's scale follows.
    pub levels: (f32, f32),
    /// The waterfall texture and its texture coordinates (newest row on top).
    pub waterfall: Option<(TextureId, Rect)>,
    /// The decoder's channel: centre, width, colour and label.
    pub channel: Option<(f64, f64, Color32, String)>,
    /// A drag is moving the band: say so over it.
    pub moving: Option<String>,
    /// Take no input (the wheel is open).
    pub inert: bool,
    /// A line under the spectrum when there is nothing to show.
    pub idle_text: &'a str,
}

/// What the pointer did.
pub enum Action {
    Tune(f64),
    /// Move the view by this many Hz (positive: to higher frequencies).
    Pan(f64),
    /// The middle button was released without a drag.
    DragEnd,
    OpenWheel(Pos2),
    /// Zoom by this factor (> 1: in) keeping `anchor_hz` under the pointer.
    Zoom { factor: f64, anchor_hz: f64, frac: f64 },
}

/// The spectrum's height in a band view `height` points tall.
fn spectrum_height(height: f32) -> f32 {
    (height * 0.36).clamp(110.0, 340.0)
}

/// The waterfall's height in whole physical pixels: the app keeps exactly
/// this many rows so a row is a pixel (from DecDAB, same author).
pub fn waterfall_rows(height: f32, ppp: f32) -> usize {
    ((height - spectrum_height(height) - AXIS_H - 4.0).max(0.0) * ppp).floor() as usize
}

/// A tick step giving at most about ten labels across `span` Hz.
fn tick_step(span: f64) -> f64 {
    const STEPS: [f64; 13] = [1e3, 2e3, 5e3, 10e3, 20e3, 50e3, 100e3, 200e3, 500e3, 1e6, 2e6, 5e6, 10e6];
    STEPS.iter().copied().find(|s| span / s <= 10.0).unwrap_or(20e6)
}

pub fn show(ui: &mut egui::Ui, inp: &BandInput) -> Vec<Action> {
    let mut actions = Vec::new();
    let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
    let rect = resp.rect;
    painter.rect_filled(rect, CornerRadius::ZERO, Color32::from_rgb(12, 12, 14));
    if rect.width() < 200.0 || rect.height() < 160.0 {
        return actions;
    }

    let spec_h = spectrum_height(rect.height());
    let plot_l = rect.left() + LEFT;
    let plot_r = rect.right() - 6.0;
    let spec = Rect::from_min_max(pos2(plot_l, rect.top() + 4.0), pos2(plot_r, rect.top() + spec_h));
    let ppp = ui.ctx().pixels_per_point();
    let wf_top = ((spec.bottom() + AXIS_H) * ppp).round() / ppp;
    let wf_px = waterfall_rows(rect.height(), ppp) as f32;
    let wf = Rect::from_min_max(pos2(plot_l, wf_top), pos2(plot_r, wf_top + wf_px / ppp));
    let span = inp.view_span_hz.max(1.0);
    let lo_hz = inp.view_center_hz - span / 2.0;
    let x_of = |hz: f64| plot_l + ((hz - lo_hz) / span) as f32 * (plot_r - plot_l);
    let hz_of = |x: f32| lo_hz + ((x - plot_l) / (plot_r - plot_l)) as f64 * span;
    let (lo_db, hi_db) = (inp.levels.0 - 10.0, inp.levels.1 + 5.0);
    let y_of = |db: f32| {
        let f = ((db - lo_db) / (hi_db - lo_db).max(1.0)).clamp(0.0, 1.0);
        spec.bottom() - f * spec.height()
    };
    let font = FontId::monospace(11.0);
    let grid = Color32::from_rgba_unmultiplied(255, 255, 255, 22);
    let dim = Color32::from_gray(150);
    painter.rect_filled(spec, CornerRadius::ZERO, Color32::from_rgb(22, 32, 42));

    // dB grid and scale.
    let mut db = (lo_db / 10.0).ceil() * 10.0;
    while db < hi_db {
        let y = y_of(db);
        painter.line_segment([pos2(plot_l, y), pos2(plot_r, y)], Stroke::new(1.0, grid));
        painter.text(pos2(plot_l - 4.0, y), Align2::RIGHT_CENTER, format!("{db:.0}"), font.clone(), dim);
        db += 10.0;
    }
    painter.text(pos2(rect.left() + 2.0, spec.top()), Align2::LEFT_TOP, "dBFS", font.clone(), dim);

    // Frequency scale.
    let step = tick_step(span);
    let decimals = if step >= 1e6 { 0 } else if step >= 100e3 { 1 } else if step >= 10e3 { 2 } else { 3 };
    let mut f = (lo_hz / step).ceil() * step;
    while f < lo_hz + span {
        let x = x_of(f);
        painter.line_segment([pos2(x, spec.top()), pos2(x, spec.bottom())], Stroke::new(1.0, grid));
        painter.text(
            pos2(x, spec.bottom() + 3.0),
            Align2::CENTER_TOP,
            format!("{:.*}", decimals, f / 1e6),
            font.clone(),
            dim,
        );
        f += step;
    }
    painter.text(pos2(rect.left() + 2.0, spec.bottom() + 3.0), Align2::LEFT_TOP, "MHz", font.clone(), dim);

    // The rows' span, clipped to the view.
    let row_lo = inp.row_center_hz - inp.row_rate / 2.0;
    let row_hi = inp.row_center_hz + inp.row_rate / 2.0;
    let seen_lo = row_lo.max(lo_hz);
    let seen_hi = row_hi.min(lo_hz + span);

    // The trace: the strongest bin under each pixel column, filled below.
    let n = inp.spectrum_db.len();
    if n > 1 && inp.row_rate > 0.0 && seen_hi > seen_lo {
        let (x0, x1) = (x_of(seen_lo).floor() as i32, x_of(seen_hi).ceil() as i32);
        let bin_of = |hz: f64| (((hz - row_lo) / inp.row_rate) * n as f64).clamp(0.0, n as f64 - 1.0) as usize;
        let mut pts: Vec<Pos2> = Vec::with_capacity((x1 - x0).max(0) as usize + 1);
        for xi in x0..x1 {
            let x = xi as f32;
            let (k0, k1) = (bin_of(hz_of(x)), bin_of(hz_of(x + 1.0)));
            let m = inp.spectrum_db[k0..=k1.max(k0)].iter().copied().fold(f32::MIN, f32::max);
            pts.push(pos2(x + 0.5, y_of(m)));
        }
        let fill = Color32::from_rgba_unmultiplied(70, 150, 255, 38);
        let mut mesh = Mesh::default();
        for p in &pts {
            let i = mesh.vertices.len() as u32;
            mesh.colored_vertex(*p, fill);
            mesh.colored_vertex(pos2(p.x, spec.bottom()), fill);
            if i >= 2 {
                mesh.add_triangle(i - 2, i - 1, i);
                mesh.add_triangle(i - 1, i + 1, i);
            }
        }
        painter.add(Shape::mesh(mesh));
        painter.add(Shape::line(pts, Stroke::new(1.2, TRACE)));
    }

    // The waterfall: the part of the rows in view, newest on top.
    if let Some((tex, uv)) = inp.waterfall
        && inp.row_rate > 0.0
        && seen_hi > seen_lo
    {
        let u0 = ((seen_lo - row_lo) / inp.row_rate) as f32;
        let u1 = ((seen_hi - row_lo) / inp.row_rate) as f32;
        let r = Rect::from_min_max(pos2(x_of(seen_lo), wf.top()), pos2(x_of(seen_hi), wf.bottom()));
        let uv = Rect::from_min_max(pos2(u0, uv.min.y), pos2(u1, uv.max.y));
        painter.image(tex, r, uv, Color32::WHITE);
    }
    if inp.row_rate <= 0.0 {
        painter.text(wf.center(), Align2::CENTER_CENTER, inp.idle_text, FontId::monospace(13.0), Color32::GRAY);
    }

    // The decoder's channel: its band shaded, its edges in its colour, a label.
    if let Some((hz, bw, colour, label)) = &inp.channel {
        let (xa, xb) = (x_of(hz - bw / 2.0), x_of(hz + bw / 2.0));
        let (xa, xb) = if xb - xa < 3.0 { (x_of(*hz) - 1.5, x_of(*hz) + 1.5) } else { (xa, xb) };
        if xb >= plot_l && xa <= plot_r {
            let (ca, cb) = (xa.max(plot_l), xb.min(plot_r));
            painter.rect_filled(
                Rect::from_min_max(pos2(ca, spec.top()), pos2(cb, wf.bottom())),
                CornerRadius::ZERO,
                colour.gamma_multiply(0.12),
            );
            let edge = Stroke::new(1.6, *colour);
            for x in [xa, xb] {
                if (plot_l..=plot_r).contains(&x) {
                    painter.line_segment([pos2(x, spec.top()), pos2(x, wf.bottom())], edge);
                }
            }
            let galley = painter.layout_no_wrap(label.clone(), font.clone(), *colour);
            let c = ((ca + cb) / 2.0).clamp(plot_l + galley.size().x / 2.0 + 4.0, plot_r - galley.size().x / 2.0 - 4.0);
            let r = Rect::from_center_size(pos2(c, spec.top() + 12.0), galley.size() + egui::vec2(10.0, 4.0));
            painter.rect(r, CornerRadius::same(3), Color32::from_rgb(20, 20, 20), Stroke::new(1.0, *colour), egui::StrokeKind::Inside);
            painter.galley(r.min + egui::vec2(5.0, 2.0), galley, *colour);
        }
    }

    // Hints, and the drag's own note.
    painter.text(
        pos2(plot_r - 6.0, wf.bottom() - 6.0),
        Align2::RIGHT_BOTTOM,
        "left click: tune · middle-drag: move band · middle click: decoders · scroll: zoom",
        FontId::monospace(11.0),
        Color32::from_gray(160),
    );
    if let Some(m) = &inp.moving {
        let g = painter.layout_no_wrap(m.clone(), FontId::monospace(13.0), Color32::from_rgb(255, 176, 40));
        let r = Rect::from_center_size(pos2((plot_l + plot_r) / 2.0, spec.top() + spec.height() * 0.45), g.size() + egui::vec2(24.0, 12.0));
        painter.rect(r, CornerRadius::same(4), Color32::from_rgba_unmultiplied(20, 20, 20, 235), Stroke::new(1.0, Color32::from_rgb(255, 176, 40)), egui::StrokeKind::Inside);
        painter.galley(r.min + egui::vec2(12.0, 6.0), g, Color32::WHITE);
    }

    if inp.inert {
        return actions;
    }

    // Pointer: a crosshair and a readout.
    let in_plot = |p: Pos2| (plot_l..=plot_r).contains(&p.x) && (spec.top()..=wf.bottom()).contains(&p.y);
    if let Some(p) = resp.hover_pos()
        && in_plot(p)
    {
        let hz = hz_of(p.x);
        painter.line_segment(
            [pos2(p.x, spec.top()), pos2(p.x, wf.bottom())],
            Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 90)),
        );
        let level = (n > 1 && hz >= row_lo && hz < row_hi)
            .then(|| inp.spectrum_db[(((hz - row_lo) / inp.row_rate) * n as f64) as usize % n])
            .map(|d| format!("  {d:.1} dBFS"))
            .unwrap_or_default();
        painter.text(
            pos2(plot_r - 4.0, spec.top() + 26.0),
            Align2::RIGHT_TOP,
            format!("{:.4} MHz{level}", hz / 1e6),
            FontId::monospace(12.0),
            Color32::WHITE,
        );
        ui.ctx().set_cursor_icon(CursorIcon::Crosshair);

        // Scroll: zoom about the pointer, a notch at a time.
        let notches: i32 = ui.input(|i| {
            i.raw
                .events
                .iter()
                .map(|e| match e {
                    egui::Event::MouseWheel { delta, .. } if delta.y > 0.0 => 1,
                    egui::Event::MouseWheel { delta, .. } if delta.y < 0.0 => -1,
                    _ => 0,
                })
                .sum()
        });
        if notches != 0 {
            let frac = ((p.x - plot_l) / (plot_r - plot_l)) as f64;
            actions.push(Action::Zoom { factor: 1.25f64.powi(notches), anchor_hz: hz, frac });
        }
    }

    if resp.clicked_by(PointerButton::Primary)
        && let Some(p) = resp.interact_pointer_pos()
        && in_plot(p)
    {
        actions.push(Action::Tune(hz_of(p.x)));
    }
    if resp.dragged_by(PointerButton::Middle) {
        ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
        let dx = resp.drag_delta().x as f64;
        if dx != 0.0 {
            actions.push(Action::Pan(-dx / (plot_r - plot_l) as f64 * span));
        }
    }
    if resp.drag_stopped_by(PointerButton::Middle) {
        actions.push(Action::DragEnd);
    }
    if resp.clicked_by(PointerButton::Middle)
        && let Some(p) = resp.interact_pointer_pos()
    {
        actions.push(Action::OpenWheel(p));
    }
    actions
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{Sim, button, moved, scroll};

    const W: f32 = 1000.0;
    /// The plot's width in a `W`-wide band (the dB scale and margin off).
    const PLOT_W: f64 = (W - LEFT - 6.0) as f64;

    /// Run frames of the band (98 MHz, 10 MHz wide) with these events, one
    /// list per frame, and gather what it asks for.
    fn run(frames: Vec<Vec<egui::Event>>, inert: bool) -> Vec<Action> {
        let spectrum = vec![-80.0f32; 1024];
        let mut sim = Sim::new(W, 600.0);
        let mut out = Vec::new();
        let mut all = vec![vec![]];
        all.extend(frames);
        all.push(vec![]);
        for events in all {
            sim.frame(events, |ui| {
                let inp = BandInput {
                    spectrum_db: &spectrum,
                    row_center_hz: 98e6,
                    row_rate: 10e6,
                    view_center_hz: 98e6,
                    view_span_hz: 10e6,
                    levels: (-90.0, -40.0),
                    waterfall: None,
                    channel: None,
                    moving: None,
                    inert,
                    idle_text: "",
                };
                out.extend(show(ui, &inp));
            });
        }
        out
    }

    fn pans(a: &[Action]) -> f64 {
        a.iter().map(|a| if let Action::Pan(d) = a { *d } else { 0.0 }).sum()
    }

    #[test]
    fn middle_click_opens_the_wheel_where_clicked() {
        let a = run(
            vec![
                vec![moved(500.0, 300.0)],
                vec![button(500.0, 300.0, PointerButton::Middle, true)],
                vec![button(500.0, 300.0, PointerButton::Middle, false)],
            ],
            false,
        );
        let opened: Vec<Pos2> =
            a.iter().filter_map(|a| if let Action::OpenWheel(p) = a { Some(*p) } else { None }).collect();
        assert_eq!(opened.len(), 1, "one wheel");
        assert!((opened[0] - pos2(500.0, 300.0)).length() < 1.0);
        assert_eq!(pans(&a), 0.0);
    }

    #[test]
    fn middle_drag_moves_the_band_and_does_not_open_the_wheel() {
        let a = run(
            vec![
                vec![moved(400.0, 300.0)],
                vec![button(400.0, 300.0, PointerButton::Middle, true)],
                vec![moved(420.0, 300.0)],
                vec![moved(460.0, 300.0)],
                vec![moved(500.0, 300.0)],
                vec![button(500.0, 300.0, PointerButton::Middle, false)],
            ],
            false,
        );
        // 100 px to the right drags lower frequencies into view.
        let want = -100.0 / PLOT_W * 10e6;
        let got = pans(&a);
        assert!((got - want).abs() < 0.15 * want.abs(), "panned {got:.0} Hz, wanted about {want:.0}");
        assert!(a.iter().any(|a| matches!(a, Action::DragEnd)));
        assert!(!a.iter().any(|a| matches!(a, Action::OpenWheel(_))));
    }

    #[test]
    fn left_click_tunes_to_the_frequency_under_the_pointer() {
        let x = LEFT + 0.25 * PLOT_W as f32;
        let a = run(
            vec![
                vec![moved(x, 300.0)],
                vec![button(x, 300.0, PointerButton::Primary, true)],
                vec![button(x, 300.0, PointerButton::Primary, false)],
            ],
            false,
        );
        let tuned: Vec<f64> = a.iter().filter_map(|a| if let Action::Tune(h) = a { Some(*h) } else { None }).collect();
        assert_eq!(tuned.len(), 1);
        assert!((tuned[0] - 95.5e6).abs() < 20e3, "{}", tuned[0]);
    }

    #[test]
    fn scroll_zooms_about_the_pointer() {
        let a = run(vec![vec![moved(600.0, 300.0)], vec![scroll(true)]], false);
        let z: Vec<(f64, f64)> = a
            .iter()
            .filter_map(|a| if let Action::Zoom { factor, anchor_hz, .. } = a { Some((*factor, *anchor_hz)) } else { None })
            .collect();
        assert_eq!(z.len(), 1);
        assert!((z[0].0 - 1.25).abs() < 1e-9);
        let want = 93e6 + (600.0 - LEFT as f64) / PLOT_W * 10e6;
        assert!((z[0].1 - want).abs() < 20e3);
    }

    #[test]
    fn an_open_wheel_leaves_the_band_alone() {
        let a = run(
            vec![
                vec![moved(500.0, 300.0)],
                vec![button(500.0, 300.0, PointerButton::Middle, true)],
                vec![button(500.0, 300.0, PointerButton::Middle, false)],
            ],
            true,
        );
        assert!(a.is_empty());
    }

    #[test]
    fn ticks_stay_readable() {
        assert_eq!(tick_step(10e6), 1e6);
        assert_eq!(tick_step(200e3), 20e3);
        assert_eq!(tick_step(2e6), 200e3);
    }
}
