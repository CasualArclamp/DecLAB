// made by CasualArclamp
//! VFOs in the GUI: the list the app keeps, and the rail down the left that
//! shows it (mockup B on the design canvas, with a collapsible rail).
//!
//! The app's own fields (`dec`, `chan_hz`, `view`…) always hold the
//! *selected* VFO, so the card, the settings window and the TV windows keep
//! working on one decoder as before; the other VFOs wait in [`Slot`]s and are
//! swapped in when selected.

use declab_api::Lock;
use declab_engine::{DecoderView, VfoId};
use eframe::egui::{
    self, Align2, Color32, CornerRadius, FontId, Pos2, Rect, RichText, Sense, Shape, Stroke,
    StrokeKind, pos2, vec2,
};

/// A VFO's colour on the band and in the rail, by its number. They differ in
/// lightness as well as hue, so they tell apart without relying on colour.
pub const COLOURS: [Color32; 8] = [
    Color32::from_rgb(255, 176, 40),
    Color32::from_rgb(77, 208, 225),
    Color32::from_rgb(255, 126, 182),
    Color32::from_rgb(155, 229, 100),
    Color32::from_rgb(183, 156, 255),
    Color32::from_rgb(255, 138, 92),
    Color32::from_rgb(111, 168, 255),
    Color32::from_rgb(230, 224, 106),
];

/// Width of the rail open, and collapsed, points.
const OPEN_W: f32 = 300.0;
const SHUT_W: f32 = 46.0;
const ROW_H: f32 = 66.0;

/// One VFO the app knows of. The selected VFO's live values are in the app's
/// own fields; its slot is brought up to date whenever it is swapped out.
pub struct Slot {
    /// The number shown (1–8), the lowest one free when it was added.
    pub n: usize,
    pub colour: Color32,
    /// Its id in the running host (none while stopped).
    pub vid: Option<VfoId>,
    /// Its sound is off (the host's per-VFO gain is 0).
    pub muted: bool,
    /// Index into `declab_decoders::ALL`.
    pub dec: usize,
    pub chan_hz: f64,
    pub bandwidth_hz: f64,
    pub scope: String,
    pub selected_service: Option<u32>,
    /// What its decoder last said (the selected one's is the app's `view`).
    pub view: DecoderView,
}

/// What the rail shows of one VFO.
pub struct Row {
    pub n: usize,
    pub colour: Color32,
    pub decoder: &'static str,
    pub freq_hz: f64,
    pub status: String,
    pub lock: Lock,
    pub cpu: f32,
    /// A level reading, dBFS, if the decoder reports one.
    pub level_db: Option<f32>,
    pub muted: bool,
    /// The first VFO, the one that may retune the radio.
    pub primary: bool,
    pub running: bool,
}

/// What was clicked in the rail.
pub enum Act {
    Select(usize),
    ToggleMute(usize),
    Remove(usize),
    Add,
}

/// The first level the decoder reports, for the rail's meter.
pub fn level_of(view: &DecoderView) -> Option<f32> {
    view.measurements
        .iter()
        .find(|m| m.unit == "dBFS")
        .map(|m| m.value as f32)
}

/// A row's status line: the decoder's lock and what its card says.
pub fn status_of(view: &DecoderView, running: bool) -> String {
    if !running {
        return "stopped".into();
    }
    let lock = match view.lock {
        Lock::Searching => "searching",
        Lock::Synced => "synced",
        Lock::Locked => "locked",
    };
    match view
        .card
        .as_ref()
        .map(|c| c.headline.as_str())
        .filter(|h| !h.is_empty() && *h != "-")
    {
        Some(h) => format!("{lock} · {h}"),
        None => lock.to_string(),
    }
}

/// The rail, as a panel on the left edge. Add it before the other panels so
/// it runs the full height of the window.
pub fn rail(
    ctx_ui: &mut egui::Ui,
    rows: &[Row],
    selected: usize,
    open: &mut bool,
    max: usize,
) -> Vec<Act> {
    let mut acts = Vec::new();
    let width = if *open { OPEN_W } else { SHUT_W };
    egui::Panel::left("vfo_rail")
        .resizable(false)
        .exact_size(width)
        .frame(
            egui::Frame::NONE
                .fill(Color32::from_rgb(23, 23, 23))
                .stroke(Stroke::new(1.0, Color32::from_gray(42))),
        )
        .show(ctx_ui, |ui| {
            if *open {
                open_rail(ui, rows, selected, open, max, &mut acts);
            } else {
                shut_rail(ui, rows, selected, open, max, &mut acts);
            }
        });
    acts
}

fn open_rail(
    ui: &mut egui::Ui,
    rows: &[Row],
    selected: usize,
    open: &mut bool,
    max: usize,
    acts: &mut Vec<Act>,
) {
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        ui.add_space(10.0);
        ui.label(RichText::new("VFOs").strong());
        ui.label(RichText::new(format!("{} / {}", rows.len(), max)).color(Color32::GRAY));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.add_space(6.0);
            if ui
                .button("«")
                .on_hover_text("Collapse the VFO list")
                .clicked()
            {
                *open = false;
            }
            if ui
                .add_enabled(rows.len() < max, egui::Button::new("+ add"))
                .clicked()
            {
                acts.push(Act::Add);
            }
        });
    });
    ui.add_space(6.0);
    ui.separator();
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.add_space(4.0);
            for (i, r) in rows.iter().enumerate() {
                row(ui, i, r, i == selected, rows.len() > 1, acts);
                ui.add_space(6.0);
            }
        });
}

/// One VFO's card. The whole card selects it; the speaker and × sit on top.
fn row(ui: &mut egui::Ui, i: usize, r: &Row, on: bool, can_remove: bool, acts: &mut Vec<Act>) {
    let (rect, _) =
        ui.allocate_exact_size(vec2(ui.available_width() - 10.0, ROW_H), Sense::hover());
    let rect = rect.translate(vec2(5.0, 0.0));
    // Interact first: the buttons added after it are above it and win clicks.
    let card = ui.interact(rect, ui.id().with(("vfo", i)), Sense::click());
    let p = ui.painter();
    let fill = if on {
        Color32::from_rgb(35, 35, 35)
    } else if card.hovered() {
        Color32::from_rgb(30, 30, 30)
    } else {
        Color32::from_rgb(27, 27, 27)
    };
    p.rect(
        rect,
        CornerRadius::same(4),
        fill,
        Stroke::new(1.0, if on { r.colour } else { Color32::from_gray(44) }),
        StrokeKind::Inside,
    );
    p.rect_filled(
        Rect::from_min_max(rect.min, pos2(rect.min.x + 5.0, rect.max.y)),
        CornerRadius {
            nw: 4,
            sw: 4,
            ne: 0,
            se: 0,
        },
        r.colour,
    );
    let (l, rt) = (rect.min.x + 14.0, rect.max.x - 52.0);
    let font = FontId::monospace(13.0);
    let small = FontId::monospace(11.0);
    p.text(
        pos2(l, rect.min.y + 9.0),
        Align2::LEFT_TOP,
        r.n.to_string(),
        FontId::monospace(13.0),
        r.colour,
    );
    let name = p.text(
        pos2(l + 14.0, rect.min.y + 9.0),
        Align2::LEFT_TOP,
        r.decoder,
        font.clone(),
        Color32::WHITE,
    );
    if r.primary {
        // The one that may retune the radio.
        p.text(
            pos2(name.max.x + 8.0, rect.min.y + 11.0),
            Align2::LEFT_TOP,
            "primary",
            FontId::monospace(10.0),
            Color32::from_gray(130),
        );
    }
    p.text(
        pos2(rt, rect.min.y + 7.0),
        Align2::RIGHT_TOP,
        format!("{:.3} MHz", r.freq_hz / 1e6),
        FontId::monospace(15.0),
        Color32::from_gray(235),
    );
    let status_col = match (r.running, r.lock) {
        (false, _) => Color32::GRAY,
        (_, Lock::Locked) => Color32::from_rgb(110, 230, 130),
        _ => Color32::from_rgb(255, 176, 40),
    };
    // The status gets what room the CPU note leaves, cut with an ellipsis.
    let cpu = format!("{:.0} % cpu", r.cpu);
    let cpu_w = p
        .layout_no_wrap(cpu.clone(), small.clone(), Color32::GRAY)
        .size()
        .x;
    let max_w = (rt - l - cpu_w - 8.0).max(30.0);
    let mut status = r.status.clone();
    while status.chars().count() > 3
        && p.layout_no_wrap(status.clone(), small.clone(), status_col)
            .size()
            .x
            > max_w
    {
        status.pop();
        if status.chars().count() <= 3 {
            break;
        }
    }
    if status != r.status {
        status.push('…');
    }
    p.text(
        pos2(l, rect.min.y + 31.0),
        Align2::LEFT_TOP,
        status,
        small.clone(),
        status_col,
    );
    p.text(
        pos2(rt, rect.min.y + 31.0),
        Align2::RIGHT_TOP,
        cpu,
        small,
        Color32::GRAY,
    );
    // The level meter.
    let bar = Rect::from_min_max(pos2(l, rect.max.y - 14.0), pos2(rt, rect.max.y - 8.0));
    p.rect_filled(bar, CornerRadius::same(3), Color32::from_gray(38));
    if let Some(db) = r.level_db {
        let f = ((db + 80.0) / 60.0).clamp(0.02, 1.0);
        let fill = Rect::from_min_max(bar.min, pos2(bar.min.x + bar.width() * f, bar.max.y));
        p.rect_filled(
            fill,
            CornerRadius::same(3),
            r.colour.gamma_multiply(if r.muted { 0.45 } else { 1.0 }),
        );
    }

    if card.clicked() {
        acts.push(Act::Select(i));
    }
    // The speaker and the ×, down the right edge.
    let col = r.colour;
    let (top, bot) = (
        Rect::from_min_size(pos2(rect.max.x - 44.0, rect.min.y), vec2(44.0, ROW_H / 2.0)),
        Rect::from_min_size(
            pos2(rect.max.x - 44.0, rect.min.y + ROW_H / 2.0),
            vec2(44.0, ROW_H / 2.0),
        ),
    );
    let mute = ui.interact(top, ui.id().with(("mute", i)), Sense::click());
    speaker(
        ui.painter(),
        top.center(),
        if r.muted {
            Color32::from_gray(120)
        } else {
            col
        },
        r.muted,
        mute.hovered(),
    );
    mute.clone().on_hover_text(if r.muted {
        "Unmute this VFO"
    } else {
        "Mute this VFO"
    });
    if mute.clicked() {
        acts.push(Act::ToggleMute(i));
    }
    let close = ui.interact(bot, ui.id().with(("close", i)), Sense::click());
    let cc = bot.center();
    let c = if !can_remove {
        Color32::from_gray(70)
    } else if close.hovered() {
        Color32::WHITE
    } else {
        Color32::from_gray(160)
    };
    ui.painter().line_segment(
        [cc + vec2(-4.0, -4.0), cc + vec2(4.0, 4.0)],
        Stroke::new(1.5, c),
    );
    ui.painter().line_segment(
        [cc + vec2(-4.0, 4.0), cc + vec2(4.0, -4.0)],
        Stroke::new(1.5, c),
    );
    if can_remove {
        close.clone().on_hover_text("Remove this VFO");
        if close.clicked() {
            acts.push(Act::Remove(i));
        }
    }
    ui.painter().line_segment(
        [
            pos2(rect.max.x - 44.0, rect.min.y + 4.0),
            pos2(rect.max.x - 44.0, rect.max.y - 4.0),
        ],
        Stroke::new(1.0, Color32::from_gray(44)),
    );
}

/// A speaker, crossed out when muted.
fn speaker(p: &egui::Painter, c: Pos2, colour: Color32, muted: bool, hot: bool) {
    let colour = if hot {
        colour
    } else {
        colour.gamma_multiply(0.85)
    };
    p.add(Shape::convex_polygon(
        vec![
            c + vec2(-8.0, -3.0),
            c + vec2(-4.0, -3.0),
            c + vec2(0.0, -7.0),
            c + vec2(0.0, 7.0),
            c + vec2(-4.0, 3.0),
            c + vec2(-8.0, 3.0),
        ],
        colour,
        Stroke::NONE,
    ));
    let s = Stroke::new(1.4, colour);
    if muted {
        p.line_segment([c + vec2(3.0, -4.0), c + vec2(9.0, 4.0)], s);
        p.line_segment([c + vec2(3.0, 4.0), c + vec2(9.0, -4.0)], s);
    } else {
        p.add(Shape::line(
            vec![
                c + vec2(4.0, -4.0),
                c + vec2(6.0, -2.0),
                c + vec2(6.0, 2.0),
                c + vec2(4.0, 4.0),
            ],
            s,
        ));
        p.add(Shape::line(
            vec![
                c + vec2(6.0, -7.0),
                c + vec2(9.0, -3.0),
                c + vec2(9.0, 3.0),
                c + vec2(6.0, 7.0),
            ],
            s,
        ));
    }
}

/// The rail collapsed: a column of numbered discs.
fn shut_rail(
    ui: &mut egui::Ui,
    rows: &[Row],
    selected: usize,
    open: &mut bool,
    max: usize,
    acts: &mut Vec<Act>,
) {
    ui.add_space(8.0);
    ui.vertical_centered(|ui| {
        if ui.button("»").on_hover_text("Open the VFO list").clicked() {
            *open = true;
        }
    });
    ui.add_space(8.0);
    for (i, r) in rows.iter().enumerate() {
        let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 38.0), Sense::click());
        let c = rect.center();
        let p = ui.painter();
        if i == selected {
            p.circle_stroke(c, 14.0, Stroke::new(2.0, r.colour));
        } else if resp.hovered() {
            p.circle_stroke(c, 14.0, Stroke::new(1.0, Color32::from_gray(110)));
        }
        let fill = if r.muted {
            r.colour.gamma_multiply(0.45)
        } else {
            r.colour
        };
        p.circle_filled(c, 10.0, fill);
        p.text(
            c,
            Align2::CENTER_CENTER,
            r.n.to_string(),
            FontId::monospace(12.0),
            Color32::from_rgb(20, 20, 20),
        );
        resp.clone().on_hover_text(format!(
            "VFO {} · {} · {:.3} MHz{}",
            r.n,
            r.decoder,
            r.freq_hz / 1e6,
            if r.muted { " · muted" } else { "" }
        ));
        if resp.clicked() {
            acts.push(Act::Select(i));
        }
    }
    ui.add_space(4.0);
    ui.vertical_centered(|ui| {
        if ui
            .add_enabled(rows.len() < max, egui::Button::new("+"))
            .on_hover_text("Add a VFO")
            .clicked()
        {
            acts.push(Act::Add);
        }
    });
}
