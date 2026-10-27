// made by CasualArclamp
//! DecLAB's window. For now only the mode picker: every registered decoder,
//! grouped by family, and what it is. The front end, spectrum and decoder
//! hosting arrive in M1 (docs/DESIGN.md).

use declab_api::{DecoderFactory, Family};

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("DecLAB")
            .with_inner_size([1100.0, 700.0]),
        ..Default::default()
    };
    eframe::run_native("DecLAB", options, Box::new(|_cc| Ok(Box::new(App::default()))))
}

#[derive(Default)]
struct App {
    /// Index into `declab_decoders::ALL`.
    selected: usize,
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut eframe::egui::Ui, _frame: &mut eframe::Frame) {
        eframe::egui::Panel::left("modes").resizable(false).show(ui, |ui| {
            ui.heading("Mode");
            for family in Family::ALL {
                let members: Vec<_> = declab_decoders::ALL
                    .iter()
                    .enumerate()
                    .filter(|(_, f)| f.info().family == family)
                    .collect();
                if members.is_empty() {
                    continue;
                }
                ui.separator();
                ui.label(family.label());
                for (i, f) in members {
                    ui.selectable_value(&mut self.selected, i, f.info().name);
                }
            }
        });
        eframe::egui::CentralPanel::default().show(ui, |ui| {
            let f: &dyn DecoderFactory = declab_decoders::ALL[self.selected];
            let info = f.info();
            ui.heading(info.name);
            ui.label(info.summary);
            ui.label(format!("Standard: {}", info.standard));
            ui.label(format!("Id: {}", info.id));
        });
    }
}
