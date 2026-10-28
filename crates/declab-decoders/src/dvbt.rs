// made by CasualArclamp
//! DVB-T and DVB-T2: DecDVB-T's receiver (`decdvbt_rx::auto::AutoReceiver`,
//! same author) as a DecLAB decoder. One entry in the wheel: the receiver
//! looks for both standards at once and decodes whichever it finds.
//!
//! The decoder takes the radio's own samples (`InputSpec::sample_rate:
//! None`): the receiver's FFT runs on them directly, which was DecDVB-T's
//! biggest single CPU saving. Its transport stream goes out as `Sink::ts`
//! and through `decdvbt_ts::TsAnalyser`, whose PSI/SI gives the services and
//! their now/next. The card is DecDVB-T's: the mode, the effective SNR
//! against the C/N the mode needs for quasi error free reception (EN 300 744
//! Table A.1 for DVB-T; the PLP's code for T2), and the mode's details.

use declab_api::{
    Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq, Lock,
    Service, Sink,
};
use decdvbt_core::dvbt::{Bandwidth, Mode};
use decdvbt_rx::auto::{AutoReceiver, Standard};
use decdvbt_rx::receiver::ReceiverStatus;
use decdvbt_rx::t2::plp::PlpParams;
use decdvbt_rx::t2::receiver::{FS_NATIVE, T2Status};
use decdvbt_ts::{TsAnalyser, service_type_name};

/// The card and measurements, this often (seconds): 60 a second, as the
/// GUI draws.
const REPORT_S: f64 = 1.0 / 60.0;
/// The equalised cells for the scope, this often: the receiver gathers them
/// only when asked (they cost a copy of a symbol).
const SCOPES_S: f64 = 1.0 / 15.0;
/// The services list from the TS tables, this often.
const SERVICES_S: f64 = 1.0;
/// Points kept for the scope.
const SCOPE_POINTS: usize = 6000;

pub struct Factory;

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "dvbt",
            name: "DVB-T/T2",
            family: Family::Terrestrial,
            standard: "EN 300 744 / EN 302 755",
            summary: "Terrestrial TV, either standard found blind (8 MHz)",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        // The radio's samples, untouched (with the channel centred): an 8 MHz
        // channel's 7.77 MHz (T2 extended) is the width to keep.
        InputSpec { sample_rate: None, bandwidth_hz: 7.77e6, block_len: 131_072 }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Dvbt::default())
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        Some(Box::new(panel::DvbtPanel::default()))
    }
}

/// The PLPs a DVB-T2 multiplex's L1 lists, sent to the panel when they
/// change: (id, a line describing it).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PlpList(pub Vec<(u8, String)>);

/// One PLP from L1-post, described: "data 256-QAM 2/3 rotated".
fn describe_plp(c: &decdvbt_ofdm::t2::l1::PlpConfig) -> String {
    let kind = match c.plp_type {
        0 => "common",
        1 => "data (type 1)",
        2 => "data",
        _ => "other",
    };
    match PlpParams::from_l1(c) {
        Some(p) => format!("{kind} · {} {}{}", p.qam.name(), p.rate, if p.rotated { " rotated" } else { "" }),
        None => format!("{kind} · not decodable here"),
    }
}

#[derive(Default)]
struct Dvbt {
    rx: Option<AutoReceiver>,
    rate: f64,
    ts: Vec<u8>,
    analyser: TsAnalyser,
    /// Samples since the stream began, and when each report last went out.
    samples: u64,
    report_at: u64,
    scopes_at: u64,
    services_at: u64,
    /// Bytes of TS since the last services report, for the stream's rate.
    ts_bytes: u64,
    stream_mbps: Option<f64>,
    last_services: Vec<Service>,
    points: Vec<Iq>,
    /// The PLP asked for (None: the first data PLP); kept across a reset.
    plp: Option<u8>,
    last_plps: PlpList,
}

impl Dvbt {
    fn report(&mut self, out: &mut Sink) {
        let Some(rx) = &mut self.rx else { return };
        let standard = rx.standard();
        let t = rx.dvbt().clone();
        let t2 = rx.dvbt2().clone();
        let (tracking, decoding) = match standard {
            Standard::Dvbt2 => (t2.tracking, t2.decoding),
            Standard::Dvbt => (t.tracking, t.decoding),
            Standard::Searching => (false, false),
        };
        out.lock = if decoding {
            Lock::Locked
        } else if tracking {
            Lock::Synced
        } else {
            Lock::Searching
        };
        let card = match standard {
            Standard::Dvbt2 => t2_card(&t2, self.stream_mbps),
            Standard::Dvbt => t_card(&t, self.stream_mbps),
            Standard::Searching => searching_card(&t, &t2),
        };
        if decoding {
            let (snr, eff) = if standard == Standard::Dvbt2 { (t2.snr_db, t2.effective_snr_db) } else { (t.snr_db, t.effective_snr_db) };
            out.measure("MER", snr as f64, "dB");
            if eff.is_finite() {
                out.measure("effective SNR", eff as f64, "dB");
            }
        }
        out.card = Some(card);
        if !self.points.is_empty() {
            out.constellation.extend_from_slice(&self.points);
        }
        // The PLPs, for the panel, when they change.
        let plps = PlpList(
            t2.l1post.as_ref().map_or_else(Vec::new, |l| l.plps.iter().map(|c| (c.plp_id, describe_plp(c))).collect()),
        );
        if plps != self.last_plps {
            out.custom.push(Box::new(plps.clone()));
            self.last_plps = plps;
        }
    }

    fn services(&mut self, secs: f64, out: &mut Sink) {
        self.stream_mbps = Some(self.ts_bytes as f64 * 8.0 / secs / 1e6);
        self.ts_bytes = 0;
        let report = self.analyser.report();
        let mut services: Vec<Service> = report
            .programmes
            .iter()
            .filter(|p| p.number != 0)
            .map(|p| Service {
                id: u32::from(p.number),
                name: p.name.clone().unwrap_or_default(),
                provider: p.provider.clone().unwrap_or_default(),
                kind: p.service_type.map_or("", service_type_name).to_string(),
            })
            .collect();
        services.sort_by_key(|s| {
            let p = report.programmes.iter().find(|p| u32::from(p.number) == s.id);
            (p.and_then(|p| p.lcn).unwrap_or(u16::MAX), s.id)
        });
        if services != self.last_services {
            out.services = Some(services.clone());
            self.last_services = services;
        }
        for p in &report.programmes {
            let mut line = String::new();
            if let Some(e) = &p.now {
                line = format!("Now: {}", e.name);
            }
            if let Some(e) = &p.next {
                line = format!("{line}{}Next: {}", if line.is_empty() { "" } else { " · " }, e.name);
            }
            if !line.is_empty() {
                out.text.push((u32::from(p.number), line));
            }
        }
    }
}

impl Decoder for Dvbt {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        let rate = block.tuning.sample_rate;
        if self.rx.is_none() || self.rate != rate {
            let mut rx = AutoReceiver::new(rate, 0.0);
            rx.set_plp(self.plp);
            // From a radio the T2 FEC gives up a frame rather than hold the
            // worker when it is behind (a radio's samples do not wait); from
            // a file it waits, and nothing is lost.
            rx.set_realtime(block.live);
            self.rx = Some(rx);
            self.rate = rate;
        }
        let rx = self.rx.as_mut().expect("made above");
        let every = |s: f64| (s * rate) as u64;
        if self.samples - self.scopes_at >= every(SCOPES_S) {
            self.scopes_at = self.samples;
            rx.request_scopes();
        }
        rx.push(block.iq, block.first_sample);
        self.ts.clear();
        rx.take_ts(&mut self.ts);
        if let Some(s) = rx.take_scopes() {
            self.points = decdvbt_rx::scope::thin(s.data.into_iter(), SCOPE_POINTS);
        }
        let (packets, _) = self.ts.as_chunks::<188>();
        for p in packets {
            self.analyser.packet(p);
            out.ts.push(*p);
        }
        self.ts_bytes += self.ts.len() as u64;
        self.samples += block.iq.len() as u64;

        if self.samples - self.report_at >= every(REPORT_S) {
            self.report_at = self.samples;
            self.report(out);
        }
        let since = self.samples - self.services_at;
        if since >= every(SERVICES_S) {
            self.services_at = self.samples;
            self.services(since as f64 / rate, out);
        }
    }

    fn command(&mut self, cmd: Command, out: &mut Sink) {
        if let Command::Set { key, value } = cmd {
            if key != "plp" {
                return;
            }
            self.plp = if value == "auto" { None } else { value.parse().ok() };
            if let Some(rx) = &mut self.rx {
                rx.set_plp(self.plp);
            }
            out.log(format!("PLP: {}", self.plp.map_or("the first data PLP".into(), |p| p.to_string())));
        }
    }

    fn reset(&mut self) {
        // A retune: the receiver starts looking again; the TS tables are the
        // new multiplex's. The PLP asked for stays.
        let plp = self.plp;
        *self = Dvbt::default();
        self.plp = plp;
    }

    fn wants_threads(&self) -> usize {
        // The T2 receiver runs its FEC on threads of its own.
        2
    }
}

fn kv(k: &str, v: String) -> (String, String) {
    (k.to_string(), v)
}

/// The gauge: the effective SNR (else the MER) against the C/N the mode
/// needs, on DecDVB-T's 0-35 dB scale.
fn gauge(snr: f32, eff: f32, qef: Option<f32>) -> Gauge {
    let (value, label) = if eff.is_finite() { (eff, "effective SNR") } else { (snr, "MER") };
    Gauge { label: label.into(), value_db: value, need_db: qef, need_label: "QEF".into(), range: (0.0, 35.0) }
}

fn t_card(t: &ReceiverStatus, mbps: Option<f64>) -> Card {
    let mut details = Vec::new();
    let mode = t.tps.map(|tps| {
        let mut m = Mode::uk(tps.modulation, tps.rate_hp);
        m.bandwidth = Bandwidth::Mhz8;
        m.fft = tps.fft;
        m.guard = tps.guard;
        m.rate_lp = tps.rate_lp;
        m.alpha = tps.alpha;
        m
    });
    if let Some(m) = &mode {
        details.push(kv("FFT size", m.fft.name().to_string()));
        details.push(kv("guard", format!("{} ({:.0} µs)", m.guard.name(), m.guard_s() * 1e6)));
        details.push(kv("carriers", format!("{} ({} data)", m.fft.active_carriers(), m.fft.data_carriers())));
        details.push(kv("bit rate", format!("{:.2} Mbit/s", m.bitrate_hp() / 1e6)));
    }
    if t.decoding {
        let o = &t.outer;
        details.push(kv("MER", format!("{:.1} dB", t.snr_db)));
        if let Some(r) = mbps {
            details.push(kv("stream", format!("{r:.2} Mbit/s")));
        }
        details.push(kv("RS blocks", format!("{} fixed · {} lost", o.corrected + o.uncorrectable, o.uncorrectable)));
        details.push(kv("before RS", format!("{:.1e} BER", o.pre_rs_ber())));
    }
    details.push(kv("carrier offset", format!("{:+.2} kHz", t.cfo_hz / 1e3)));
    details.push(kv("clock", format!("{:+.1} ppm", t.clock_ppm)));
    Card {
        title: "DVB-T".into(),
        headline: mode.map_or("–".into(), |m| format!("{} {}", m.modulation.name(), m.rate_hp.name())),
        gauge: t.decoding.then(|| gauge(t.snr_db, t.effective_snr_db, mode.and_then(|m| m.qef_cn_db()))),
        details,
        text: Vec::new(),
    }
}

fn t2_card(t2: &T2Status, mbps: Option<f64>) -> Card {
    let mut details = Vec::new();
    let fs = t2.bandwidth.map_or(FS_NATIVE, |b| b.sample_rate_hz());
    if let Some(m) = &t2.mode {
        let g = m.guard.samples(m.fft);
        details.push(kv("FFT size", format!("{}{}", m.fft, if m.extended { " ext" } else { "" })));
        details.push(kv("guard", format!("{} ({:.0} µs)", m.guard, g as f64 / fs * 1e6)));
        details.push(kv("pilots", m.pp.to_string()));
        details.push(kv("carriers", m.c_ps().to_string()));
    }
    if t2.plp.is_some() {
        details.push(kv("PLP", format!("{} · {} blocks a frame", t2.plp_id, t2.blocks_per_frame)));
    }
    if t2.decoding {
        details.push(kv("MER", format!("{:.1} dB", t2.snr_db)));
        if let Some(r) = mbps {
            details.push(kv("stream", format!("{r:.2} Mbit/s")));
        }
        let total = (t2.blocks_ok + t2.blocks_bad).max(1);
        details.push(kv(
            "LDPC blocks",
            format!(
                "{} lost of {} · {:.1} iterations",
                t2.blocks_bad,
                total,
                t2.ldpc_iterations as f64 / t2.blocks_ok.max(1) as f64
            ),
        ));
        details.push(kv("BCH fixes", t2.bch_corrected.to_string()));
    }
    details.push(kv("carrier offset", format!("{:+.2} kHz", t2.cfo_hz / 1e3)));
    Card {
        title: "DVB-T2".into(),
        headline: t2.plp.map_or("–".into(), |p| {
            format!("{} {}{}", p.qam.name(), p.rate, if p.rotated { " rotated" } else { "" })
        }),
        gauge: t2.decoding.then(|| gauge(t2.snr_db, t2.effective_snr_db, t2.plp.and_then(|p| p.qef_cn_db()))),
        details,
        text: Vec::new(),
    }
}

fn searching_card(t: &ReceiverStatus, t2: &T2Status) -> Card {
    let mut details = vec![
        kv("DVB-T tries", format!("{} (no prefix {}, not DVB-T {})", t.attempts, t.acq_no_prefix, t.acq_not_dvbt)),
        kv("DVB-T2 tries", format!("{} (no P1 {}, no L1 {})", t2.attempts, t2.acq_no_p1, t2.acq_no_l1)),
    ];
    if let Some(s1) = t2.p1_unsupported {
        details.push(kv("T2", format!("P1 S1 = {s1}: frames this receiver does not decode")));
    }
    Card { title: "DVB-T / T2".into(), headline: "searching".into(), gauge: None, details, text: Vec::new() }
}

/// The DVB-T/T2 settings panel: which PLP of a T2 multiplex to decode.
#[cfg(feature = "ui")]
mod panel {
    use declab_api::{Command, Panel};
    use egui::{RichText, Ui};

    use super::PlpList;

    #[derive(Default)]
    pub struct DvbtPanel {
        /// The PLPs the running decoder last saw.
        plps: PlpList,
        /// None: the first data PLP.
        chosen: Option<u8>,
    }

    impl Panel for DvbtPanel {
        fn receive(&mut self, msg: &(dyn std::any::Any + Send)) {
            if let Some(l) = msg.downcast_ref::<PlpList>() {
                self.plps = l.clone();
            }
        }

        fn ui(&mut self, ui: &mut Ui, commands: &mut Vec<Command>) {
            ui.label("DVB-T2 PLP (physical layer pipe)");
            let label = |c: Option<u8>| c.map_or("first data PLP".to_string(), |p| format!("PLP {p}"));
            let before = self.chosen;
            egui::ComboBox::from_id_salt("plp").selected_text(label(self.chosen)).width(260.0).show_ui(ui, |ui| {
                ui.selectable_value(&mut self.chosen, None, label(None));
                for (id, what) in &self.plps.0 {
                    ui.selectable_value(&mut self.chosen, Some(*id), format!("PLP {id} · {what}"));
                }
            });
            if self.chosen != before {
                let value = self.chosen.map_or("auto".to_string(), |p| p.to_string());
                commands.push(Command::Set { key: "plp".into(), value });
            }
            if self.plps.0.is_empty() {
                ui.label(RichText::new("The PLPs are listed once a T2 signal is read.").size(11.0).weak());
            }
        }

        fn settings(&self) -> Vec<Command> {
            vec![Command::Set { key: "plp".into(), value: self.chosen.map_or("auto".into(), |p| p.to_string()) }]
        }

        fn restore(&mut self, key: &str, value: &str) {
            if key == "plp" {
                self.chosen = value.parse().ok();
            }
        }
    }
}
