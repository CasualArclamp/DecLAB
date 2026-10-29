// made by CasualArclamp
//! The digital voice modes' settings panel (see `declab_api::Panel`): one
//! SNR-gate slider per mode, each with its own measured figures in the help
//! text.

use declab_api::{Command, Panel};
use egui::Ui;

/// One mode's gate.
pub struct Gate {
    /// "DMR", "NXDN".
    pub label: &'static str,
    /// The `Command::Set` key its decoder answers to.
    pub key: &'static str,
    pub db: f32,
    /// The best fit noise made, and the median of real signals near the
    /// sensitivity limit (measured; for the help text).
    pub noise_max: f32,
    pub real_median: f32,
}

pub struct GatePanel {
    pub gates: Vec<Gate>,
}

fn set(g: &Gate) -> Command {
    Command::Set {
        key: g.key.into(),
        value: format!("{:.0}", g.db),
    }
}

impl Panel for GatePanel {
    fn settings(&self) -> Vec<Command> {
        self.gates.iter().map(set).collect()
    }

    fn restore(&mut self, key: &str, value: &str) {
        if let (Some(g), Ok(db)) = (
            self.gates.iter_mut().find(|g| g.key == key),
            value.parse::<f32>(),
        ) {
            g.db = db.clamp(0.0, 20.0);
        }
    }

    fn ui(&mut self, ui: &mut Ui, commands: &mut Vec<Command>) {
        ui.label("SNR gate: a sync whose own eye SNR is below this is not believed.");
        ui.add_space(4.0);
        for g in &mut self.gates {
            let r = ui.add(
                egui::Slider::new(&mut g.db, 0.0..=20.0)
                    .step_by(1.0)
                    .text(format!("{} gate, dB", g.label)),
            );
            if r.changed() {
                commands.push(set(g));
            }
            ui.label(
                egui::RichText::new(format!(
                    "{}: noise fitted at most {:.1} dB; real signals at the sensitivity limit fit about {:.0} dB.",
                    g.label, g.noise_max, g.real_median
                ))
                .small(),
            );
            ui.add_space(4.0);
        }
        ui.label("Raise a gate if noise still triggers; lower it for the weakest signals.");
    }
}
