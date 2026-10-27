// made by CasualArclamp
//! The smallest useful decoder, and the template for a new one: it measures
//! the channel's power and shows the raw samples on the constellation scope.

use declab_api::{Block, Decoder, DecoderFactory, DecoderInfo, Family, InputSpec, Lock, Sink};

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "level",
            name: "Level meter",
            family: Family::Other,
            standard: "—",
            summary: "Channel power and a raw-sample scope; the example decoder",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        InputSpec { sample_rate: Some(2.048e6), bandwidth_hz: 1.5e6, block_len: 65_536 }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Level::default())
    }
}

#[derive(Default)]
struct Level {
    /// Sum of |x|² and sample count since the last report.
    energy: f64,
    count: u64,
    /// Samples between reports: a tenth of a second.
    report_every: u64,
}

impl Decoder for Level {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        if self.report_every == 0 {
            self.report_every = (block.tuning.sample_rate / 10.0).max(1.0) as u64;
        }
        self.energy += block.iq.iter().map(|s| s.norm_sqr() as f64).sum::<f64>();
        self.count += block.iq.len() as u64;
        if self.count >= self.report_every {
            let mean = self.energy / self.count as f64;
            out.measure("Power", 10.0 * mean.max(1e-20).log10(), "dBFS");
            out.lock = Lock::Locked;
            // A few hundred points are plenty for a scope.
            let step = (block.iq.len() / 256).max(1);
            out.constellation.extend(block.iq.iter().step_by(step));
            self.energy = 0.0;
            self.count = 0;
        }
    }

    fn reset(&mut self) {
        self.energy = 0.0;
        self.count = 0;
    }
}
