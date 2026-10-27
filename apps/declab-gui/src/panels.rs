// made by CasualArclamp
//! The parts of the window below the band: the decoder's card, its scope
//! and readings, its own settings panel, its services and text, and the
//! status bar.

use declab_api::{Command, Iq, Lock, Panel};
use declab_engine::{DecoderView, Stats};
use eframe::egui::{
    self, Align2, Color32, CornerRadius, FontId, RichText, Sense, Stroke, Ui, pos2, vec2,
};

pub const GREEN: Color32 = Color32::from_rgb(110, 230, 130);
pub const AMBER: Color32 = Color32::from_rgb(255, 176, 40);
pub const RED: Color32 = Color32::from_rgb(255, 138, 128);
const GREY: Color32 = Color32::from_gray(154);
const BOX: Color32 = Color32::from_rgb(24, 24, 24);
/// The scope's side, points.
const SCOPE: f32 = 170.0;

/// Format a reading: few digits for big numbers, more for small ones.
pub fn value(v: f64) -> String {
    let a = v.abs();
    if a >= 1000.0 || (a > 0.0 && a < 0.01) {
        if a >= 1e6 || a < 0.01 { format!("{v:.2e}") } else { format!("{v:.0}") }
    } else if a >= 100.0 || v.fract() == 0.0 {
        format!("{v:.0}")
    } else {
        format!("{v:.1}")
    }
}

/// The card: the decoder's name, its lock and a one-line summary.
pub fn card(ui: &mut Ui, name: &str, colour: Color32, view: &DecoderView, running: bool) {
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new(name).strong().color(colour));
        let (state, c) = match (running, view.lock) {
            (false, _) => ("stopped", GREY),
            (true, Lock::Locked) => ("locked", GREEN),
            (true, Lock::Synced) => ("synced", AMBER),
            (true, Lock::Searching) => ("searching", RED),
        };
        ui.label(RichText::new(state).color(c));
        let line: Vec<String> = view
            .measurements
            .iter()
            .take(4)
            .map(|m| format!("{} {}{}{}", m.name.to_lowercase(), value(m.value), if m.unit.is_empty() { "" } else { " " }, m.unit))
            .collect();
        if running && !line.is_empty() {
            ui.label(RichText::new(line.join(" · ")).color(GREEN));
        }
        if let Some(s) = view.services.first().filter(|s| !s.name.is_empty()) {
            ui.label(RichText::new(format!("· {}", s.name)).color(Color32::WHITE));
        }
    });
}

/// A titled box filling its share of the row.
pub fn boxed(ui: &mut Ui, add: impl FnOnce(&mut Ui)) {
    egui::Frame::NONE
        .fill(BOX)
        .stroke(Stroke::new(1.0, Color32::from_gray(42)))
        .corner_radius(CornerRadius::same(4))
        .inner_margin(10.0)
        .show(ui, |ui| {
            ui.set_min_height(ui.available_height() - 22.0);
            ui.set_min_width(ui.available_width());
            add(ui)
        });
}

/// The constellation scope (with its choice of scopes above it) and the
/// decoder's card beside it, in the style of DecDVB-T's and DecSAT's cards.
///
/// `which` names the scope shown: "" for the channel IQ, else one of the
/// decoder's own (`DecoderView::scopes`).
pub fn scope_and_card(
    ui: &mut Ui,
    view: &DecoderView,
    colour: Color32,
    which: &mut String,
    name: &str,
    freq_hz: f64,
) {
    ui.horizontal_top(|ui| {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                let mut pick = |ui: &mut Ui, key: &str, label: &str| {
                    let on = which.as_str() == key;
                    let text = RichText::new(label).size(11.0).color(if on { Color32::WHITE } else { GREY });
                    if ui.selectable_label(on, text).clicked() {
                        *which = key.to_string();
                    }
                };
                pick(ui, "", "channel IQ");
                for (n, _) in &view.scopes {
                    pick(ui, n, n);
                }
            });
            let pts = view.scopes.iter().find(|(n, _)| *n == which.as_str()).map_or(&view.constellation, |(_, p)| p);
            scope(ui, pts, colour, SCOPE);
        });
        ui.add_space(10.0);
        ui.vertical(|ui| {
            let fallback;
            let card = match &view.card {
                Some(c) => c,
                None => {
                    fallback = card_from_measurements(name, view);
                    &fallback
                }
            };
            egui::ScrollArea::vertical().id_salt("card").max_height(ui.available_height()).show(ui, |ui| {
                draw_card(ui, card, colour, freq_hz, view.cpu_percent);
            });
        });
    });
}

/// A card for a decoder that sends none: its measurements as details.
fn card_from_measurements(name: &str, view: &DecoderView) -> declab_api::Card {
    declab_api::Card {
        title: name.to_string(),
        headline: match view.lock {
            Lock::Locked => "locked",
            Lock::Synced => "synced",
            Lock::Searching => "searching",
        }
        .to_string(),
        gauge: None,
        details: view
            .measurements
            .iter()
            .map(|m| (m.name.to_string(), format!("{} {}", value(m.value), m.unit)))
            .collect(),
        text: Vec::new(),
    }
}

/// A margin's colour: red below 0 dB, amber within 1 dB, else green (as in
/// DecDVB-T's card).
fn health(margin: f32) -> Color32 {
    if margin < 0.0 {
        Color32::from_rgb(230, 110, 110)
    } else if margin < 1.0 {
        Color32::from_rgb(240, 200, 80)
    } else {
        Color32::from_rgb(110, 220, 110)
    }
}

fn draw_card(ui: &mut Ui, c: &declab_api::Card, colour: Color32, freq_hz: f64, cpu: f32) {
    let strong = Color32::WHITE;
    // The title bar: what is received, and where.
    let bar = egui::Frame::new()
        .fill(colour.gamma_multiply(0.15))
        .corner_radius(CornerRadius { nw: 4, ne: 4, sw: 0, se: 0 })
        .inner_margin(egui::Margin { left: 8, right: 8, top: 3, bottom: 3 })
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(RichText::new(&c.title).size(14.0).strong().color(colour));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(RichText::new("MHz").size(11.0).color(GREY));
                    ui.label(RichText::new(format!("{:.3}", freq_hz / 1e6)).size(16.0).color(strong));
                });
            });
        })
        .response;
    ui.painter().hline(bar.rect.x_range(), bar.rect.bottom(), Stroke::new(2.0, colour));
    ui.add_space(4.0);

    // The headline and the margin beside it.
    let margin = c.gauge.as_ref().and_then(|g| g.need_db.map(|n| ((g.value_db - n) * 10.0).round() / 10.0 + 0.0));
    ui.horizontal(|ui| {
        let empty = c.headline == "–" || c.headline.is_empty();
        ui.label(RichText::new(if empty { "–" } else { &c.headline }).size(24.0).color(if empty { GREY } else { colour }));
        if let Some(m) = margin {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(format!("{m:+5.1} dB")).size(17.0).color(health(m)));
            });
        }
    });

    // The gauge, ticked where the signal needs to be.
    if let Some(g) = &c.gauge {
        let (lo, hi) = g.range;
        let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), 10.0), Sense::hover());
        let x_of = |db: f32| rect.left() + rect.width() * ((db - lo) / (hi - lo)).clamp(0.0, 1.0);
        let p = ui.painter();
        p.rect_filled(rect, CornerRadius::same(2), Color32::from_rgb(10, 10, 10));
        let fill = margin.map_or(Color32::from_rgb(100, 170, 255).gamma_multiply(0.8), health);
        p.rect_filled(egui::Rect::from_min_max(rect.min, pos2(x_of(g.value_db), rect.max.y)), CornerRadius::same(2), fill);
        if let Some(n) = g.need_db {
            let x = x_of(n);
            p.rect_filled(
                egui::Rect::from_min_max(pos2(x - 1.0, rect.top() - 3.0), pos2(x + 1.0, rect.bottom() + 3.0)),
                CornerRadius::ZERO,
                strong,
            );
        }
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("{} {:.1} dB", g.label, g.value_db)).size(12.0).color(GREY));
            if let Some(n) = g.need_db {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(RichText::new(format!("{} {n:.1} dB", g.need_label)).size(12.0).color(GREY));
                });
            }
        });
    }

    // The details, two pairs a row, the decoder's CPU last.
    ui.add_space(4.0);
    let mut details: Vec<(String, String)> = c.details.clone();
    details.push(("CPU".into(), format!("{cpu:.0} %")));
    egui::Grid::new("card_details").num_columns(4).spacing([10.0, 3.0]).show(ui, |ui| {
        for pair in details.chunks(2) {
            for (k, v) in pair {
                ui.label(RichText::new(k).size(12.0).color(GREY));
                ui.label(RichText::new(v).size(12.0).color(strong));
            }
            ui.end_row();
        }
    });

    // The text, each line under its label.
    for (k, v) in &c.text {
        ui.add_space(4.0);
        ui.label(RichText::new(k).size(11.0).color(GREY));
        ui.add(egui::Label::new(RichText::new(v).size(13.0).color(strong)).wrap());
    }
}

/// IQ points, scaled so their RMS sits at 45 % of the half-width.
fn scope(ui: &mut Ui, pts: &[Iq], colour: Color32, size: f32) {
    let (rect, _) = ui.allocate_exact_size(vec2(size, size), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, CornerRadius::ZERO, Color32::from_rgb(13, 13, 13));
    let grid = Stroke::new(1.0, Color32::from_gray(42));
    p.line_segment([rect.center_top(), rect.center_bottom()], grid);
    p.line_segment([rect.left_center(), rect.right_center()], grid);
    if pts.is_empty() {
        return;
    }
    let rms = (pts.iter().map(|z| z.norm_sqr()).sum::<f32>() / pts.len() as f32).sqrt().max(1e-9);
    let k = 0.45 * size / 2.0 / rms;
    let c = rect.center();
    let dot = colour.gamma_multiply(0.85);
    for z in pts {
        let q = pos2(c.x + z.re * k, c.y - z.im * k);
        if rect.contains(q) {
            p.circle_filled(q, 1.4, dot);
        }
    }
}

/// The decoder's own panel, then the host's channel width and the volume.
/// Returns commands for the decoder, a new width and a new volume.
pub fn decoder_settings(
    ui: &mut Ui,
    name: &str,
    panel: Option<&mut Box<dyn Panel>>,
    bandwidth_hz: &mut f64,
    volume: &mut f32,
) -> (Vec<Command>, bool, bool) {
    let mut cmds = Vec::new();
    ui.horizontal(|ui| {
        ui.label(RichText::new(format!("{name} settings")).strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(RichText::new("drawn by the decoder").size(11.0).color(GREY));
        });
    });
    ui.add_space(4.0);
    match panel {
        Some(p) => p.ui(ui, &mut cmds),
        None => {
            ui.label(RichText::new("This decoder has no settings of its own.").color(GREY));
        }
    }
    ui.add_space(8.0);
    ui.separator();
    let mut bw_khz = *bandwidth_hz / 1e3;
    let speed = bw_khz * 0.01;
    let bw_changed = ui
        .horizontal(|ui| {
            ui.label("Bandwidth");
            ui.add(
                egui::DragValue::new(&mut bw_khz)
                    .range(0.5..=10_000.0)
                    .speed(speed)
                    .suffix(" kHz")
                    .max_decimals(1),
            )
            .changed()
        })
        .inner;
    if bw_changed {
        *bandwidth_hz = bw_khz * 1e3;
    }
    let vol_changed = ui
        .horizontal(|ui| {
            ui.label("Volume   ");
            ui.add(egui::Slider::new(volume, 0.0..=1.0).show_value(false)).changed()
        })
        .inner;
    (cmds, bw_changed, vol_changed)
}

/// Services (click one to decode it) and the decoder's text.
pub fn services(ui: &mut Ui, view: &DecoderView, selected: Option<u32>) -> Option<u32> {
    let mut pick = None;
    ui.label(RichText::new("Services").strong());
    if view.services.is_empty() {
        ui.label(RichText::new("none announced").color(GREY));
    }
    egui::ScrollArea::vertical().id_salt("services").max_height(110.0).show(ui, |ui| {
        for s in &view.services {
            let name = if s.name.is_empty() { format!("[{}]", s.id) } else { s.name.clone() };
            let on = selected == Some(s.id) || (selected.is_none() && view.services.len() == 1);
            let text = format!("{name:<24} {}", s.kind);
            if ui.selectable_label(on, RichText::new(text)).clicked() {
                pick = Some(s.id);
            }
        }
    });
    if !view.text.is_empty() {
        ui.add_space(6.0);
        ui.label(RichText::new("text").size(11.0).color(GREY));
        for t in view.text.values() {
            ui.label(RichText::new(t).color(Color32::WHITE));
        }
    }
    if let Some(last) = view.log.back() {
        ui.add_space(6.0);
        ui.label(RichText::new(format!("log: {last}")).size(11.0).color(GREY));
    }
    pick
}

/// The status bar.
pub fn status(ui: &mut Ui, stats: &Stats, view: &DecoderView, running: bool, app_cpu: f32, error: Option<&str>) {
    let kv = |ui: &mut Ui, k: &str, v: String| {
        ui.label(RichText::new(k).color(GREY).size(12.0));
        ui.label(RichText::new(v).size(12.0));
        ui.add_space(14.0);
    };
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing.x = 5.0;
        let state = if let Some(e) = error {
            format!("error: {e}")
        } else if running {
            "running".into()
        } else if stats.ended {
            "ended".into()
        } else {
            "stopped".into()
        };
        kv(ui, "state", state);
        if !stats.describe.is_empty() {
            kv(ui, "source", stats.describe.clone());
        }
        let ch = view.channel;
        if ch.out_rate > 0.0 {
            let rs = ch.resample.map(|(u, d)| format!(" ×{u}/{d}")).unwrap_or_default();
            kv(ui, "channel", format!("{:.0} kS/s (÷{}{rs})", ch.out_rate / 1e3, ch.decimation));
        }
        if stats.handoff_dropped > 0 {
            kv(ui, "dropped", format!("{} blocks", stats.handoff_dropped));
        }
        if stats.clip_percent > 0.1 {
            ui.label(RichText::new(format!("CLIPPING {:.1} %", stats.clip_percent)).color(RED).size(12.0));
            ui.add_space(14.0);
        }
        if let Some(a) = &view.audio {
            kv(ui, "sound", a.clone());
        }
        let p = stats.stages.percent;
        kv(
            ui,
            "CPU",
            format!(
                "app {app_cpu:.0} % · source {:.0} · front end {:.0} · spectrum {:.0} · decoder {:.0}",
                p[0] + p[1],
                p[2],
                p[3],
                view.cpu_percent
            ),
        );
    });
}

/// A level meter bar like the other Dec apps'.
pub fn level_bar(ui: &mut Ui, dbfs: f32, live: bool) {
    let (rect, _) = ui.allocate_exact_size(vec2(180.0, 20.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, CornerRadius::same(10), Color32::from_rgb(38, 38, 38));
    if live {
        let f = ((dbfs + 60.0) / 60.0).clamp(0.0, 1.0);
        let fill = egui::Rect::from_min_size(rect.min, vec2(rect.width() * f, rect.height()));
        p.rect_filled(fill, CornerRadius::same(10), Color32::from_rgb(11, 93, 122));
        p.text(
            rect.left_center() + vec2(10.0, 0.0),
            Align2::LEFT_CENTER,
            format!("{dbfs:.1} dBFS"),
            FontId::monospace(12.0),
            Color32::WHITE,
        );
    }
}
