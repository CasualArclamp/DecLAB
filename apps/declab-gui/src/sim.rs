// made by CasualArclamp
//! Test helper: runs egui frames headless with made-up mouse input, so the
//! band's and the wheel's gestures can be tested without a window.

use eframe::egui::{self, Event, Modifiers, MouseWheelUnit, PointerButton, Pos2, RawInput, Rect, TouchPhase, pos2, vec2};

pub struct Sim {
    pub ctx: egui::Context,
    time: f64,
    pub screen: Rect,
}

impl Sim {
    pub fn new(w: f32, h: f32) -> Sim {
        Sim { ctx: egui::Context::default(), time: 0.0, screen: Rect::from_min_size(Pos2::ZERO, vec2(w, h)) }
    }

    /// One frame with these events.
    pub fn frame(&mut self, events: Vec<Event>, mut ui_fn: impl FnMut(&mut egui::Ui)) {
        self.time += 1.0 / 60.0;
        let input = RawInput { screen_rect: Some(self.screen), time: Some(self.time), events, ..RawInput::default() };
        let mut out = self.ctx.run_ui(input, |ui| ui_fn(ui));
        // No renderer takes the font atlas: say it was handled (epaint checks).
        out.textures_delta.clear();
    }
}

pub fn moved(x: f32, y: f32) -> Event {
    Event::PointerMoved(pos2(x, y))
}

pub fn button(x: f32, y: f32, button: PointerButton, pressed: bool) -> Event {
    Event::PointerButton { pos: pos2(x, y), button, pressed, modifiers: Modifiers::default() }
}

/// A notch of the wheel: `up` scrolls away from the user.
pub fn scroll(up: bool) -> Event {
    Event::MouseWheel {
        unit: MouseWheelUnit::Line,
        delta: vec2(0.0, if up { 1.0 } else { -1.0 }),
        phase: TouchPhase::Move,
        modifiers: Modifiers::default(),
    }
}
