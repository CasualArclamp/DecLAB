// made by CasualArclamp
//! Everything that follows a decoder's transport stream, from DecDVB-T's
//! GUI (same author): the services list with its buttons and outputs, the
//! player window (video, sound, subtitles, Red Button), the guide and the TS
//! viewer. Any decoder that sends `Sink::ts` gets them; the host's TS hub
//! does the work, this keeps the windows and the choices.

use std::sync::Arc;
use std::time::{Duration, Instant};

use declab_engine::{Host, Outputs, TsView};
use decdvbt_ts::{Epg, TsTap};
use eframe::egui::{self, RichText};

use crate::{epg_view, external, player, services, ts_viewer};

pub struct Tv {
    pub ts: TsView,
    epg: Arc<Epg>,
    ts_at: Instant,
    epg_at: Instant,
    pub player: player::TvWindow,
    pub epg_window: epg_view::EpgView,
    pub ts_window: ts_viewer::TsViewer,
    outputs_ui: services::OutputsUi,
    outputs_sent: Option<Outputs>,
    tap: Option<TsTap>,
    pending_vlc: Option<u16>,
    pub selected: Option<u16>,
    /// A service to play as soon as its streams are known (`--play`).
    pub play_when_ready: Option<u16>,
    /// Why the last request came to nothing.
    pub note: String,
}

impl Tv {
    pub fn new(window: Option<&str>) -> Tv {
        let now = Instant::now();
        Tv {
            ts: TsView::default(),
            epg: Arc::new(Epg::new()),
            ts_at: now - Duration::from_secs(1),
            epg_at: now - Duration::from_secs(2),
            player: player::TvWindow::default(),
            epg_window: epg_view::EpgView::new(window == Some("epg")),
            ts_window: ts_viewer::TsViewer::new(window == Some("ts")),
            outputs_ui: services::OutputsUi::default(),
            outputs_sent: None,
            tap: None,
            pending_vlc: None,
            selected: None,
            play_when_ready: None,
            note: String::new(),
        }
    }

    /// Silence the player (test and screenshot runs).
    pub fn mute(&mut self) {
        self.player.mute();
    }

    /// The decoder makes a transport stream (so the services list, player
    /// and guide apply).
    pub fn active(&self) -> bool {
        self.ts.packets > 0
    }

    /// A new source or decoder: the stream is a different one.
    pub fn reset(&mut self) {
        self.player.close();
        self.tap = None;
        self.selected = None;
        self.ts = TsView::default();
        self.outputs_sent = None;
        self.note.clear();
    }

    /// The hub's view and guide, now and then; a pending `--play` or VLC.
    pub fn poll(&mut self, host: &Host, name: &str) {
        if self.ts_at.elapsed() >= Duration::from_millis(250) {
            self.ts_at = Instant::now();
            self.ts = host.ts_view();
        }
        if self.epg_window.open && self.epg_at.elapsed() >= Duration::from_secs(1) {
            self.epg_at = Instant::now();
            self.epg = host.epg();
        }
        if let Some(sid) = self.play_when_ready
            // Its streams (PMT) and its type (SDT: whether it is HD, which
            // picks the player's cheaper one-field path) are known.
            && self.ts.report.programmes.iter().any(|p| p.number == sid && !p.streams.is_empty() && p.service_type.is_some())
        {
            self.play_when_ready = None;
            self.play(host, name, sid, None, None);
        }
        if let Some(sid) = self.pending_vlc
            && let Some((addr, _)) = &self.ts.tcp
        {
            self.pending_vlc = None;
            if let Err(e) = external::vlc(&external::http_url(*addr), sid) {
                self.note = e;
            }
        }
        self.sync_outputs(host, name);
    }

    fn sync_outputs(&mut self, host: &Host, name: &str) {
        let mut o = self.outputs_ui.outputs(name, self.tap.clone(), self.pending_vlc.is_some());
        o.carousel = self.player.red.link();
        if self.outputs_sent.as_ref() != Some(&o) {
            host.set_outputs(o.clone());
            self.outputs_sent = Some(o);
        }
    }

    /// Play service `sid` in the player window, with this sound and these
    /// subtitles (None: the first of each).
    pub fn play(&mut self, host: &Host, name: &str, sid: u16, audio: Option<u16>, subs: Option<u16>) {
        let report = Arc::clone(&self.ts.report);
        let Some(p) = report.programmes.iter().find(|p| p.number == sid) else {
            return;
        };
        let sel = player::selection(p, audio, subs);
        if sel.video.is_none() && sel.audio.is_none() {
            self.note = format!("{} has no picture or sound on air right now", p.name.as_deref().unwrap_or("that service"));
            return;
        }
        self.note.clear();
        let (tap, rx) = TsTap::new(2048);
        self.tap = Some(tap.for_video(player::video_pid(p)));
        self.selected = Some(sid);
        self.player.start(p, audio, subs, rx);
        self.sync_outputs(host, name);
    }

    /// A service picked in the list or the guide: play it, unless it is
    /// already playing (its window comes up instead).
    pub fn watch(&mut self, host: &Host, name: &str, sid: u16) {
        self.selected = Some(sid);
        // The guide follows what is watched.
        self.epg_window.select_now(sid);
        if self.player.is_playing(sid) {
            self.player.open = true;
        } else {
            self.play(host, name, sid, None, None);
        }
    }

    fn open_vlc(&mut self, host: &Host, name: &str, sid: u16) {
        self.selected = Some(sid);
        self.outputs_ui.http_on = true;
        self.pending_vlc = Some(sid);
        if let Some((addr, _)) = &self.ts.tcp {
            self.pending_vlc = None;
            if let Err(e) = external::vlc(&external::http_url(*addr), sid) {
                self.note = e;
            }
        }
        self.sync_outputs(host, name);
    }

    fn act(&mut self, host: &Host, name: &str, a: services::Action) {
        match a {
            services::Action::Play(sid) => self.watch(host, name, sid),
            services::Action::Vlc(sid) => self.open_vlc(host, name, sid),
            services::Action::OpenTs => self.ts_window.open = true,
            services::Action::OpenEpg => {
                self.epg_window.open = true;
                self.epg_at = Instant::now() - Duration::from_secs(2);
            }
        }
    }

    /// The services column: the list, its buttons, the outputs.
    pub fn services_ui(&mut self, ui: &mut egui::Ui, host: &Host, name: &str) {
        egui::ScrollArea::vertical().id_salt("tv_services").max_height(ui.available_height()).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("Services").strong());
                ui.label(
                    RichText::new(format!(
                        "{} · {:.2} Mbit/s",
                        self.ts.report.programmes.len(),
                        self.ts.rate_bps / 1e6
                    ))
                    .weak(),
                );
            });
            let mut action = services::list(ui, &self.ts, &mut self.selected);
            ui.add_space(4.0);
            action = services::buttons(ui, self.selected).or(action);
            if !self.note.is_empty() {
                ui.label(RichText::new(&self.note).color(egui::Color32::from_rgb(240, 200, 80)));
            }
            ui.add_space(4.0);
            egui::CollapsingHeader::new("Outputs").default_open(false).show(ui, |ui| {
                self.outputs_ui.show(ui, &self.ts);
            });
            if let Some(a) = action {
                self.act(host, name, a);
            }
        });
    }

    /// The player, guide and TS windows.
    pub fn windows(&mut self, ctx: &egui::Context, host: &Host, name: &str) {
        self.ts_window.show(ctx, &self.ts);
        let report = Arc::clone(&self.ts.report);
        if let Some(sid) = self.epg_window.show(ctx, &self.epg, &report) {
            self.watch(host, name, sid);
        }
        let list = services::ordered(&report);
        match self.player.show(ctx, &list, self.ts.tap_dropped) {
            Some(player::Request::Play(sid, audio, subs)) => self.play(host, name, sid, audio, subs),
            Some(player::Request::Close) => self.tap = None,
            None => {}
        }
    }
}
