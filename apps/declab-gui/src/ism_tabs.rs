// made by CasualArclamp
//! The ISM devices decoder's tabs over the right-hand box (the agreed
//! mockup): Devices (one row per device heard, sortable; click one for all
//! its fields), Log (every message, rtl_433 style) and Flex (the decoder's
//! panel: device groups, flex specs, what was not decoded).

use declab_api::{Command, Devices, HeardDevice, Panel};
use declab_engine::DecoderView;
use eframe::egui::{self, Color32, RichText, Ui, vec2};

use crate::panels::{AMBER, GREEN, clock, natural, sort_header};

const GREY: Color32 = Color32::from_gray(154);
const TEXT: Color32 = Color32::from_gray(230);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Devices,
    Log,
    Flex,
}

#[derive(Default)]
pub struct IsmTabs {
    pub tab: Tab,
    /// The device picked (model, id, channel).
    picked: Option<(String, String, String)>,
}

impl IsmTabs {
    pub fn opening_on(tab: Tab) -> Self {
        IsmTabs { tab, picked: None }
    }

    /// The tab bar; returns the tab to draw.
    pub fn bar(&mut self, ui: &mut Ui, view: &DecoderView) -> Tab {
        let n = view.devices.as_ref().map_or(0, |d| d.devices.len());
        let since = view.devices.as_ref().map_or(0.0, |d| d.now_s - d.since_s);
        ui.horizontal(|ui| {
            for (t, name) in [
                (Tab::Devices, "Devices"),
                (Tab::Log, "Log"),
                (Tab::Flex, "Flex"),
            ] {
                if ui.selectable_label(self.tab == t, name).clicked() {
                    self.tab = t;
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    RichText::new(format!("{n} heard in {}", clock(since)))
                        .color(GREY)
                        .size(12.0),
                );
            });
        });
        ui.add_space(4.0);
        self.tab
    }

    pub fn devices(&mut self, ui: &mut Ui, view: &DecoderView) {
        let Some(d) = &view.devices else {
            ui.label(RichText::new("listening: no device heard yet").color(GREY));
            return;
        };
        if d.devices.is_empty() {
            ui.label(RichText::new("listening: no device heard yet").color(GREY));
            return;
        }
        // The picked device's details take a few lines at the bottom.
        let detail_h = if self.picked.is_some() { 64.0 } else { 0.0 };
        egui::ScrollArea::vertical()
            .id_salt("ism_devices")
            .max_height((ui.available_height() - detail_h).max(40.0))
            .show(ui, |ui| self.table(ui, d));
        if let Some(key) = &self.picked
            && let Some(dev) = d
                .devices
                .iter()
                .find(|x| (&x.model, &x.id, &x.channel) == (&key.0, &key.1, &key.2))
        {
            ui.separator();
            details(ui, d, dev);
        }
    }

    fn table(&mut self, ui: &mut Ui, d: &Devices) {
        // Readings get what the other columns leave (~7.5 px a character).
        // (The visible edge: in a scroll area the available width can run
        // past it.)
        let w = ui
            .available_width()
            .min(ui.clip_rect().right() - ui.cursor().left());
        let room = ((w - 330.0) / 7.5).clamp(10.0, 80.0) as usize;
        egui::Grid::new("ism_devices_grid")
            .spacing(vec2(12.0, 2.0))
            .show(ui, |ui| {
                let cols = [
                    ("model", false),
                    ("id", false),
                    ("ch", false),
                    ("n", true),
                    ("last", true),
                    ("readings", false),
                ];
                let by = sort_header(ui, "ism", &cols, 4);
                let mut rows: Vec<&HeardDevice> = d.devices.iter().collect();
                rows.sort_by(|a, b| {
                    let o = match by.col {
                        0 => a.model.cmp(&b.model),
                        1 => natural(&a.id).cmp(&natural(&b.id)),
                        2 => natural(&a.channel).cmp(&natural(&b.channel)),
                        3 => b.count.cmp(&a.count),
                        4 => b.last_s.total_cmp(&a.last_s),
                        _ => a.readings.cmp(&b.readings),
                    };
                    if by.reversed { o.reverse() } else { o }
                });
                for dev in rows {
                    let key = (dev.model.clone(), dev.id.clone(), dev.channel.clone());
                    let picked = self.picked.as_ref() == Some(&key);
                    let c = if dev.live { GREEN } else { TEXT };
                    let name = RichText::new(format!(
                        "{}{}",
                        if dev.live { "● " } else { "" },
                        cut(&dev.model, 18)
                    ))
                    .color(c);
                    if ui
                        .selectable_label(picked, name)
                        .on_hover_text(format!("{}: click for every field", dev.model))
                        .clicked()
                    {
                        self.picked = if picked { None } else { Some(key) };
                    }
                    ui.label(RichText::new(cut(&dev.id, 10)).color(c));
                    ui.label(RichText::new(cut(&dev.channel, 3)).color(c));
                    right(ui, dev.count.to_string(), c);
                    right(
                        ui,
                        if dev.live && d.now_s - dev.last_s < 2.0 {
                            "now".into()
                        } else {
                            clock(d.now_s - dev.last_s)
                        },
                        c,
                    );
                    // Readings cut to what fits beside the counts; all of it on
                    // hover and under the table when picked.
                    let mut r = RichText::new(cut(&dev.readings, room)).color(TEXT);
                    if dev.battery_low {
                        r = r.color(AMBER);
                    }
                    ui.label(r).on_hover_text(&dev.readings);
                    ui.end_row();
                }
            });
    }

    pub fn log(&mut self, ui: &mut Ui, view: &DecoderView) {
        if view.log.is_empty() {
            ui.label(RichText::new("nothing yet").color(GREY));
            return;
        }
        let h = ui.text_style_height(&egui::TextStyle::Monospace);
        let n = view.log.len();
        egui::ScrollArea::both()
            .id_salt("ism_log")
            .stick_to_bottom(true)
            .auto_shrink([false, false])
            .show_rows(ui, h, n, |ui, range| {
                for i in range {
                    let line = &view.log[i];
                    let c = if line.contains(" unknown ") {
                        GREY
                    } else {
                        TEXT
                    };
                    // One line a message (the box scrolls sideways), so
                    // `show_rows` can count rows.
                    ui.add(
                        egui::Label::new(RichText::new(line).monospace().size(11.5).color(c))
                            .extend(),
                    );
                }
            });
    }

    /// The decoder's own panel; its commands go to `commands`.
    pub fn flex(
        &mut self,
        ui: &mut Ui,
        panel: Option<&mut Box<dyn Panel>>,
        commands: &mut Vec<Command>,
    ) {
        egui::ScrollArea::vertical()
            .id_salt("ism_flex")
            .show(ui, |ui| match panel {
                Some(p) => p.ui(ui, commands),
                None => {
                    ui.label(RichText::new("no panel").color(GREY));
                }
            });
    }
}

/// At most `n` characters, with an ellipsis when cut.
fn cut(s: &str, n: usize) -> String {
    if s.chars().count() <= n {
        s.to_string()
    } else {
        let mut t: String = s.chars().take(n.saturating_sub(1)).collect();
        t.push('…');
        t
    }
}

fn right(ui: &mut Ui, text: String, c: Color32) {
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        ui.label(RichText::new(text).color(c))
    });
}

/// One device: where and how well it was heard, and every field.
fn details(ui: &mut Ui, d: &Devices, dev: &HeardDevice) {
    let mut head = dev.model.clone();
    if !dev.id.is_empty() {
        head += &format!(" · id {}", dev.id);
    }
    if !dev.channel.is_empty() {
        head += &format!(" · ch {}", dev.channel);
    }
    ui.label(RichText::new(head).color(AMBER));
    ui.label(
        RichText::new(format!(
            "{}  ·  first {} ago  ·  {} messages  ·  {:.3} MHz  ·  {:.0} dB SNR",
            dev.kind,
            clock(d.now_s - dev.first_s),
            dev.count,
            dev.freq_hz / 1e6,
            dev.snr_db
        ))
        .size(11.5)
        .color(GREY),
    );
    let fields: Vec<String> = dev.fields.iter().map(|(k, v)| format!("{k} {v}")).collect();
    ui.add(egui::Label::new(RichText::new(fields.join("  ·  ")).size(11.5).color(TEXT)).wrap());
}
