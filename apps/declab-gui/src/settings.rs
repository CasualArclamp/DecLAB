// made by CasualArclamp
//! The settings window, from the top bar's Settings button: every mode's
//! settings in one place. A list on the left (General, then each decoder by
//! family), the chosen one's settings on the right. A decoder's settings are
//! its own panel (`declab_api::Panel`); the window keeps one for every
//! decoder, so a mode can be set up before it runs, and a decoder starts as
//! its panel shows.

use declab_api::{Command, DecoderFactory, Family, Panel};
use eframe::egui::{self, Color32, RichText};

use crate::wheel::family_colour;

/// What the window changed this frame.
#[derive(Default)]
pub struct Changes {
    /// Commands from decoder `i`'s panel.
    pub commands: Vec<(usize, Command)>,
    pub bandwidth: bool,
    pub volume: bool,
}

pub struct View<'a> {
    pub decoders: &'static [&'static dyn DecoderFactory],
    pub panels: &'a mut [Option<Box<dyn Panel>>],
    /// The decoder running (or chosen to run).
    pub current: usize,
    pub bandwidth_hz: &'a mut f64,
    pub volume: &'a mut f32,
}

/// `page`: `None` for General, else a decoder's index.
pub fn show(ctx: &egui::Context, open: &mut bool, page: &mut Option<usize>, v: View) -> Changes {
    let mut ch = Changes::default();
    egui::Window::new("Settings")
        .open(open)
        .default_size([620.0, 360.0])
        .resizable(true)
        .collapsible(false)
        .show(ctx, |ui| {
            ui.horizontal_top(|ui| {
                // The list.
                ui.vertical(|ui| {
                    ui.set_width(170.0);
                    if ui.selectable_label(page.is_none(), RichText::new("General").strong()).clicked() {
                        *page = None;
                    }
                    for fam in Family::ALL {
                        let members: Vec<usize> =
                            (0..v.decoders.len()).filter(|&i| v.decoders[i].info().family == fam).collect();
                        if members.is_empty() {
                            continue;
                        }
                        ui.add_space(6.0);
                        ui.label(RichText::new(fam.label()).size(11.0).color(family_colour(fam)));
                        for i in members {
                            let info = v.decoders[i].info();
                            // The running one is marked with a dot.
                            let label = if i == v.current { format!("{}  ●", info.name) } else { info.name.to_string() };
                            if ui.selectable_label(*page == Some(i), label).clicked() {
                                *page = Some(i);
                            }
                        }
                    }
                });
                ui.separator();
                // The page.
                ui.vertical(|ui| match *page {
                    None => general(ui, v.decoders[v.current].info().name, v.bandwidth_hz, v.volume, &mut ch),
                    Some(i) => {
                        let info = v.decoders[i].info();
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(info.name).size(16.0).strong().color(family_colour(info.family)));
                            if i == v.current {
                                ui.label(RichText::new("running").size(11.0).color(Color32::from_rgb(110, 230, 130)));
                            }
                        });
                        ui.label(RichText::new(info.summary).size(11.0).color(Color32::GRAY));
                        ui.add_space(8.0);
                        match v.panels.get_mut(i).and_then(|p| p.as_mut()) {
                            Some(p) => {
                                let mut cmds = Vec::new();
                                p.ui(ui, &mut cmds);
                                ch.commands.extend(cmds.into_iter().map(|c| (i, c)));
                            }
                            None => {
                                ui.label(RichText::new("This decoder has no settings of its own.").color(Color32::GRAY));
                            }
                        }
                    }
                });
            });
        });
    ch
}

fn general(ui: &mut egui::Ui, current: &str, bandwidth_hz: &mut f64, volume: &mut f32, ch: &mut Changes) {
    ui.label(RichText::new("General").size(16.0).strong());
    ui.add_space(8.0);
    egui::Grid::new("general").num_columns(2).spacing([16.0, 8.0]).show(ui, |ui| {
        ui.label("Volume");
        ch.volume = ui.add(egui::Slider::new(volume, 0.0..=1.0).show_value(false)).changed();
        ui.end_row();
        ui.label(format!("Bandwidth ({current})"));
        let mut khz = *bandwidth_hz / 1e3;
        let speed = khz * 0.01;
        if ui
            .add(egui::DragValue::new(&mut khz).range(0.5..=10_000.0).speed(speed).suffix(" kHz").max_decimals(1))
            .changed()
        {
            *bandwidth_hz = khz * 1e3;
            ch.bandwidth = true;
        }
        ui.end_row();
    });
    ui.add_space(6.0);
    ui.label(RichText::new("The bandwidth is also set by dragging an edge of the channel on the band.").size(11.0).color(Color32::GRAY));
}
