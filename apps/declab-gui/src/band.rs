// made by CasualArclamp
//! The band: the spectrum over the waterfall, one frequency axis for both,
//! and the decoder's channel bracketed. Drawing follows DecDVB-T's band view
//! (same author); the view here can be zoomed and moved:
//!
//! - **left click** tunes the decoder's channel to the frequency clicked;
//! - **left-drag an edge** of the channel sets its bandwidth (both edges
//!   move, the channel stays centred); **left-drag inside** it moves it;
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
    /// The waterfall's frame: the centre and rate its rows are held about
    /// (`History::anchor`), which need not be the newest row's.
    pub wf_center_hz: f64,
    pub wf_rate: f64,
    /// The view: its centre and width, Hz.
    pub view_center_hz: f64,
    pub view_span_hz: f64,
    /// Colour range of the waterfall, which the spectrum's scale follows.
    pub levels: (f32, f32),
    /// The waterfall texture and its texture coordinates (newest row on top).
    pub waterfall: Option<(TextureId, Rect)>,
    /// The selected VFO's channel: centre, width, colour and label.
    pub channel: Option<(f64, f64, Color32, String)>,
    /// The other VFOs' channels, drawn lighter; a click inside one selects it.
    pub others: Vec<Other>,
    /// A drag is moving the band: say so over it.
    pub moving: Option<String>,
    /// Take no input (the wheel is open).
    pub inert: bool,
    /// A line under the spectrum when there is nothing to show.
    pub idle_text: &'a str,
}

/// Another VFO's channel on the band.
pub struct Other {
    /// Its place in the app's VFO list, handed back by [`Action::Select`].
    pub slot: usize,
    pub hz: f64,
    pub bandwidth_hz: f64,
    pub colour: Color32,
    pub label: String,
}

/// What the pointer did.
pub enum Action {
    Tune(f64),
    /// A click inside another VFO's channel: select it (its `Other::slot`).
    Select(usize),
    /// Ctrl+click on empty band: a new VFO here.
    Add(f64),
    /// An edge of the channel was dragged: this bandwidth, Hz.
    Bandwidth(f64),
    /// The channel was dragged here (its centre, Hz).
    MoveChannel(f64),
    /// A drag of the channel or its edges ended.
    ChannelDragEnd,
    /// Move the view by this many Hz (positive: to higher frequencies).
    Pan(f64),
    /// The middle button was released without a drag.
    DragEnd,
    OpenWheel(Pos2),
    /// Zoom by this factor (> 1: in) keeping `anchor_hz` under the pointer.
    Zoom { factor: f64, anchor_hz: f64, frac: f64 },
}

/// How near an edge of the channel, points, a drag takes the edge.
const EDGE_GRAB: f32 = 5.0;

/// What a left-drag started on.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Grab {
    /// An edge of the channel: the bandwidth.
    Edge,
    /// Inside the channel, this far from its centre (Hz): the channel.
    Body(f64),
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

    // The waterfall: the part of its rows in view, newest on top. Drawn at
    // fractional pixels, so a drag moves it smoothly.
    let wf_lo = inp.wf_center_hz - inp.wf_rate / 2.0;
    let (w_lo, w_hi) = (wf_lo.max(lo_hz), (wf_lo + inp.wf_rate).min(lo_hz + span));
    if let Some((tex, uv)) = inp.waterfall
        && inp.wf_rate > 0.0
        && w_hi > w_lo
    {
        let u0 = ((w_lo - wf_lo) / inp.wf_rate) as f32;
        let u1 = ((w_hi - wf_lo) / inp.wf_rate) as f32;
        let r = Rect::from_min_max(pos2(x_of(w_lo), wf.top()), pos2(x_of(w_hi), wf.bottom()));
        let uv = Rect::from_min_max(pos2(u0, uv.min.y), pos2(u1, uv.max.y));
        painter.image(tex, r, uv, Color32::WHITE);
    }
    if inp.row_rate <= 0.0 {
        painter.text(wf.center(), Align2::CENTER_CENTER, inp.idle_text, FontId::monospace(13.0), Color32::GRAY);
    }

    // The other VFOs' channels: lighter than the selected one's.
    for o in &inp.others {
        let (xa, xb) = (x_of(o.hz - o.bandwidth_hz / 2.0), x_of(o.hz + o.bandwidth_hz / 2.0));
        let (xa, xb) = if xb - xa < 3.0 { (x_of(o.hz) - 1.5, x_of(o.hz) + 1.5) } else { (xa, xb) };
        if xb < plot_l || xa > plot_r {
            continue;
        }
        let (ca, cb) = (xa.max(plot_l), xb.min(plot_r));
        painter.rect_filled(
            Rect::from_min_max(pos2(ca, spec.top()), pos2(cb, wf.bottom())),
            CornerRadius::ZERO,
            o.colour.gamma_multiply(0.07),
        );
        let edge = Stroke::new(1.0, o.colour.gamma_multiply(0.8));
        for x in [xa, xb] {
            if (plot_l..=plot_r).contains(&x) {
                painter.line_segment([pos2(x, spec.top()), pos2(x, wf.bottom())], edge);
            }
        }
        let galley = painter.layout_no_wrap(o.label.clone(), font.clone(), o.colour);
        let c = ((ca + cb) / 2.0).clamp(plot_l + galley.size().x / 2.0 + 4.0, plot_r - galley.size().x / 2.0 - 4.0);
        let r = Rect::from_center_size(pos2(c, spec.top() + 12.0), galley.size() + egui::vec2(10.0, 4.0));
        painter.rect(r, CornerRadius::same(3), Color32::from_rgb(20, 20, 20), Stroke::new(1.0, o.colour.gamma_multiply(0.8)), egui::StrokeKind::Inside);
        painter.galley(r.min + egui::vec2(5.0, 2.0), galley, o.colour);
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
        "click: tune · click a channel: select · ctrl+click: add a VFO · drag edges: bandwidth · middle-drag: move band · middle click: decoders · scroll: zoom",
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

    // The channel's edges and body: hovering shows what a drag would do.
    let edges = inp.channel.as_ref().map(|(hz, bw, _, _)| (*hz, *bw, x_of(hz - bw / 2.0), x_of(hz + bw / 2.0)));
    let grab_of = |p: Pos2| -> Option<Grab> {
        let (hz, _, xa, xb) = edges?;
        if !in_plot(p) {
            return None;
        }
        if (p.x - xa).abs() <= EDGE_GRAB || (p.x - xb).abs() <= EDGE_GRAB {
            Some(Grab::Edge)
        } else if p.x > xa && p.x < xb {
            Some(Grab::Body(hz_of(p.x) - hz))
        } else {
            None
        }
    };
    if let Some(p) = resp.hover_pos() {
        match grab_of(p) {
            Some(Grab::Edge) => ui.ctx().set_cursor_icon(CursorIcon::ResizeHorizontal),
            Some(Grab::Body(_)) => ui.ctx().set_cursor_icon(CursorIcon::Grab),
            None => {}
        }
    }
    let grab_id = resp.id.with("grab");
    // Where the button went down, not where the drag was recognised (a few
    // points on): an edge is only a few points wide.
    if resp.drag_started_by(PointerButton::Primary)
        && let Some(p) = ui.input(|i| i.pointer.press_origin()).or(resp.interact_pointer_pos())
    {
        let g = grab_of(p);
        ui.data_mut(|d| d.insert_temp(grab_id, g));
    }
    if resp.dragged_by(PointerButton::Primary)
        && let Some(p) = resp.interact_pointer_pos()
        && let Some((hz, _, _, _)) = edges
    {
        match ui.data(|d| d.get_temp::<Option<Grab>>(grab_id)).flatten() {
            Some(Grab::Edge) => {
                ui.ctx().set_cursor_icon(CursorIcon::ResizeHorizontal);
                actions.push(Action::Bandwidth(2.0 * (hz_of(p.x) - hz).abs()));
            }
            Some(Grab::Body(off)) => {
                ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
                actions.push(Action::MoveChannel(hz_of(p.x) - off));
            }
            None => {}
        }
    }
    if resp.drag_stopped_by(PointerButton::Primary) {
        if ui.data(|d| d.get_temp::<Option<Grab>>(grab_id)).flatten().is_some() {
            actions.push(Action::ChannelDragEnd);
        }
        ui.data_mut(|d| d.remove::<Option<Grab>>(grab_id));
    }

    if resp.clicked_by(PointerButton::Primary)
        && let Some(p) = resp.interact_pointer_pos()
        && in_plot(p)
    {
        let hz = hz_of(p.x);
        // Inside another VFO's channel (and not the selected one's): select it.
        let selected = edges.is_some_and(|(_, _, xa, xb)| p.x >= xa && p.x <= xb);
        let hit = inp.others.iter().find(|o| (hz - o.hz).abs() <= (o.bandwidth_hz / 2.0).max(60e3));
        match hit {
            Some(o) if !selected => actions.push(Action::Select(o.slot)),
            _ if ui.input(|i| i.modifiers.command) => actions.push(Action::Add(hz)),
            _ => actions.push(Action::Tune(hz)),
        }
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
        run_with(frames, inert, None)
    }

    fn run_with(frames: Vec<Vec<egui::Event>>, inert: bool, channel: Option<(f64, f64)>) -> Vec<Action> {
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
                    wf_center_hz: 98e6,
                    wf_rate: 10e6,
                    view_center_hz: 98e6,
                    view_span_hz: 10e6,
                    levels: (-90.0, -40.0),
                    waterfall: None,
                    channel: channel.map(|(hz, bw)| (hz, bw, Color32::WHITE, String::new())),
                    others: Vec::new(),
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

    /// The x of `hz` in the test band.
    fn x(hz: f64) -> f32 {
        LEFT + ((hz - 93e6) / 10e6 * PLOT_W) as f32
    }

    fn drag(x0: f32, x1: f32) -> Vec<Vec<egui::Event>> {
        let mut f = vec![vec![moved(x0, 300.0)], vec![button(x0, 300.0, PointerButton::Primary, true)]];
        let steps = 5;
        for k in 1..=steps {
            f.push(vec![moved(x0 + (x1 - x0) * k as f32 / steps as f32, 300.0)]);
        }
        f.push(vec![button(x1, 300.0, PointerButton::Primary, false)]);
        f
    }

    #[test]
    fn dragging_an_edge_sets_the_bandwidth() {
        // A 1 MHz channel at 98 MHz: drag its upper edge out to 98.8 MHz.
        let a = run_with(drag(x(98.5e6), x(98.8e6)), false, Some((98e6, 1e6)));
        let bw: Vec<f64> = a.iter().filter_map(|a| if let Action::Bandwidth(b) = a { Some(*b) } else { None }).collect();
        let last = *bw.last().expect("bandwidth actions");
        assert!((last - 1.6e6).abs() < 30e3, "{last}");
        assert!(a.iter().any(|a| matches!(a, Action::ChannelDragEnd)));
        assert!(!a.iter().any(|a| matches!(a, Action::Tune(_) | Action::MoveChannel(_))));
    }

    #[test]
    fn dragging_inside_the_channel_moves_it() {
        // Grab it 200 kHz above its centre and move 1 MHz down.
        let a = run_with(drag(x(98.2e6), x(97.2e6)), false, Some((98e6, 1e6)));
        let to: Vec<f64> = a.iter().filter_map(|a| if let Action::MoveChannel(h) = a { Some(*h) } else { None }).collect();
        let last = *to.last().expect("move actions");
        assert!((last - 97e6).abs() < 30e3, "{last}");
        assert!(!a.iter().any(|a| matches!(a, Action::Bandwidth(_))));
    }

    #[test]
    fn a_left_drag_on_the_empty_band_does_nothing() {
        let a = run_with(drag(x(95e6), x(96e6)), false, Some((98e6, 1e6)));
        assert!(a.iter().all(|a| matches!(a, Action::Zoom { .. })), "{} actions", a.len());
    }

    /// A click at `hz` with other VFOs on the band, ctrl held or not.
    fn click_among(hz: f64, others: &[(usize, f64, f64)], ctrl: bool, selected: Option<(f64, f64)>) -> Vec<Action> {
        let spectrum = vec![-80.0f32; 1024];
        let mut sim = Sim::new(W, 600.0);
        // egui learns of held keys from an event.
        let held = egui::Modifiers { ctrl, command: ctrl, ..egui::Modifiers::default() };
        let px = x(hz);
        let mut out = Vec::new();
        for events in [vec![egui::Event::ModifiersChanged(held)], vec![moved(px, 300.0)], vec![button(px, 300.0, PointerButton::Primary, true)], vec![button(px, 300.0, PointerButton::Primary, false)], vec![]] {
            sim.frame(events, |ui| {
                let inp = BandInput {
                    spectrum_db: &spectrum,
                    row_center_hz: 98e6,
                    row_rate: 10e6,
                    wf_center_hz: 98e6,
                    wf_rate: 10e6,
                    view_center_hz: 98e6,
                    view_span_hz: 10e6,
                    levels: (-90.0, -40.0),
                    waterfall: None,
                    channel: selected.map(|(h, b)| (h, b, Color32::WHITE, String::new())),
                    others: others
                        .iter()
                        .map(|&(slot, hz, bw)| Other { slot, hz, bandwidth_hz: bw, colour: Color32::WHITE, label: String::new() })
                        .collect(),
                    moving: None,
                    inert: false,
                    idle_text: "",
                };
                out.extend(show(ui, &inp));
            });
        }
        out
    }

    #[test]
    fn a_click_inside_another_vfos_channel_selects_it() {
        let a = click_among(94.5e6, &[(1, 94.5e6, 200e3), (2, 100.3e6, 200e3)], false, Some((98.8e6, 200e3)));
        let sel: Vec<usize> = a.iter().filter_map(|a| if let Action::Select(i) = a { Some(*i) } else { None }).collect();
        assert_eq!(sel, vec![1]);
        assert!(!a.iter().any(|a| matches!(a, Action::Tune(_) | Action::Add(_))));
    }

    #[test]
    fn a_click_on_empty_band_tunes_and_ctrl_click_adds_a_vfo() {
        let others = [(1, 94.5e6, 200e3)];
        let a = click_among(96.0e6, &others, false, Some((98.8e6, 200e3)));
        assert!(a.iter().any(|a| matches!(a, Action::Tune(h) if (h - 96.0e6).abs() < 20e3)));
        assert!(!a.iter().any(|a| matches!(a, Action::Add(_) | Action::Select(_))));
        let a = click_among(96.0e6, &others, true, Some((98.8e6, 200e3)));
        assert!(a.iter().any(|a| matches!(a, Action::Add(h) if (h - 96.0e6).abs() < 20e3)));
        assert!(!a.iter().any(|a| matches!(a, Action::Tune(_))));
    }

    #[test]
    fn a_click_inside_the_selected_channel_still_tunes() {
        // Where the selected VFO's channel overlaps another's, the selected one wins.
        let a = click_among(98.8e6, &[(1, 98.85e6, 400e3)], false, Some((98.8e6, 400e3)));
        assert!(a.iter().any(|a| matches!(a, Action::Tune(_))));
        assert!(!a.iter().any(|a| matches!(a, Action::Select(_))));
    }

    #[test]
    fn ticks_stay_readable() {
        assert_eq!(tick_step(10e6), 1e6);
        assert_eq!(tick_step(200e3), 20e3);
        assert_eq!(tick_step(2e6), 200e3);
    }
}
