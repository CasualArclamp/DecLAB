// made by CasualArclamp
//! Analog voice and broadcast: AM, DSB, USB, LSB, CW, narrow FM and
//! broadcast FM (stereo, with RDS). One decoder, seven entries in the mode
//! picker, one [`Factory`] each.
//!
//! The demodulators are DecSAT's (`decsat-engine::analog`, same author). There
//! they did their own crude pre-filtering and decimation; here the host's
//! down-converter delivers the channel already filtered at the rate asked
//! for (48 kHz for voice, 384 kHz for broadcast FM: room for the 19 kHz
//! pilot, the 38 kHz stereo subcarrier and RDS at 57 kHz).
//!
//! USB/LSB are Weaver demodulators: the channel (centred on the suppressed
//! carrier) is shifted by ∓1700 Hz to put the middle of the 300–3100 Hz voice
//! band at 0, low-passed to ±1550 Hz by a 401-tap FIR (which is what
//! rejects the other sideband: the host's channel filter is symmetric about
//! the carrier), and shifted back, the real part being the audio. CW is the
//! same with a ±250 Hz filter on the carrier and the shift back 800 Hz
//! higher, for an 800 Hz note. DecSAT's version shifted once and never back,
//! so both sidebands folded onto the audio at the wrong pitch. The audio
//! filters elsewhere are three cascaded one-pole sections: cheap, with
//! limited stopband.

use std::f64::consts::TAU;

use declab_api::{
    Audio, Block, Card, Command, Decoder, DecoderFactory, DecoderInfo, Family, Gauge, InputSpec, Iq,
    Lock, Service, Sink,
};

use declab_dsp::{Fir, lowpass};

use crate::analog_rds::{RdsDecoder, pty_name};

/// The audio the decoder hands back, Hz.
const PCM_RATE: u32 = 48_000;
const EPS: f32 = 1e-9;
/// Measurements, scopes and the card are reported this often, seconds: 60
/// times a second, so the GUI's 60 fps has something new each frame.
const REPORT_S: f64 = 1.0 / 60.0;
/// The SSB/CW filter's length at 48 kHz: a transition about 650 Hz wide.
const SSB_TAPS: usize = 401;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Am,
    Dsb,
    Usb,
    Lsb,
    Cw,
    Nfm,
    Wfm,
}

/// One factory per mode: `analog::Factory(Mode::Usb)` in the decoder list.
pub struct Factory(pub Mode);

impl DecoderFactory for Factory {
    fn info(&self) -> DecoderInfo {
        let (id, name, summary) = match self.0 {
            Mode::Am => ("am", "AM", "Envelope detector, 5 kHz audio"),
            Mode::Dsb => ("dsb", "DSB", "Double sideband, suppressed carrier"),
            Mode::Usb => ("usb", "USB", "Upper sideband voice"),
            Mode::Lsb => ("lsb", "LSB", "Lower sideband voice"),
            Mode::Cw => ("cw", "CW", "Morse, 800 Hz tone"),
            Mode::Nfm => ("nfm", "NFM", "Narrow FM voice, 5 kHz deviation"),
            Mode::Wfm => ("wfm", "WFM", "Broadcast FM: stereo and RDS"),
        };
        DecoderInfo { id, name, family: Family::Analog, standard: "—", summary }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        let (rate, bw) = match self.0 {
            Mode::Am | Mode::Dsb => (48_000.0, 10_000.0),
            // Both sidebands' worth either side of the carrier: the decoder
            // picks the side.
            Mode::Usb | Mode::Lsb => (48_000.0, 8_000.0),
            Mode::Cw => (48_000.0, 3_000.0),
            Mode::Nfm => (48_000.0, 12_500.0),
            Mode::Wfm => (384_000.0, 200_000.0),
        };
        InputSpec { sample_rate: Some(rate), bandwidth_hz: bw, block_len: 16_384 }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Analog::new(self.0))
    }

    #[cfg(feature = "ui")]
    fn panel(&self) -> Option<Box<dyn declab_api::Panel>> {
        panel::for_mode(self.0)
    }
}

/// The settings a panel can change, as [`Command::Set`] keys and values.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Settings {
    /// De-emphasis time constant, µs (0: none). `"deemph"`.
    deemph_us: f64,
    /// Broadcast FM: stereo when the pilot allows, or always mono. `"stereo"`
    /// = `"auto"` | `"mono"`.
    stereo: bool,
    /// Mute the audio while the channel is weaker than this, dBFS. `"squelch"`
    /// = a number or `"off"`.
    squelch_dbfs: Option<f64>,
    /// CW: the note a carrier on the dial makes, Hz. `"pitch"`.
    pitch_hz: f64,
}

impl Settings {
    fn for_mode(mode: Mode) -> Self {
        Settings {
            deemph_us: match mode {
                Mode::Wfm => 50.0,
                Mode::Nfm => 300.0,
                _ => 0.0,
            },
            stereo: true,
            squelch_dbfs: None,
            pitch_hz: 800.0,
        }
    }

    fn set(&mut self, key: &str, value: &str) -> Result<(), String> {
        let num = || value.parse::<f64>().map_err(|_| format!("{key}: {value:?} is not a number"));
        match key {
            "deemph" => self.deemph_us = num()?.clamp(0.0, 2000.0),
            "stereo" => self.stereo = value != "mono",
            "squelch" => self.squelch_dbfs = if value == "off" { None } else { Some(num()?) },
            "pitch" => self.pitch_hz = num()?.clamp(200.0, 2000.0),
            _ => return Err(format!("no setting {key:?}")),
        }
        Ok(())
    }
}

#[inline]
fn coeff(hz: f64, rate: f64) -> f32 {
    (1.0 - (-TAU * hz.max(0.01) / rate.max(1.0)).exp()) as f32
}

/// Three cascaded low-pass poles.
#[derive(Default)]
struct LowPass3 {
    a: f32,
    b: f32,
    c: f32,
}

impl LowPass3 {
    #[inline]
    fn tick(&mut self, value: f32, k: f32) -> f32 {
        self.a += k * (value - self.a);
        self.b += k * (self.a - self.b);
        self.c += k * (self.b - self.c);
        self.c
    }
}

struct Analog {
    mode: Mode,
    previous: Iq,
    power: f32,
    dc: f32,
    fm_dc: f32,
    mono: LowPass3,
    difference: LowPass3,
    /// SSB/CW: the Weaver filter, the mixed-down block and its filtered copy.
    ssb_fir: Option<Fir>,
    mixed: Vec<Iq>,
    filtered: Vec<Iq>,
    /// The shift back up (the shift down is `phase`), with CW's note.
    up_phase: f64,
    deemph_l: f32,
    deemph_r: f32,
    phase: f64,
    audio_phase: f64,
    pilot_i: f32,
    pilot_q: f32,
    pilot_phase: f64,
    /// 0 until the first block sets it from the rate.
    pilot_omega: f64,
    /// The same detector at 17.5 kHz, where broadcast FM has nothing (the
    /// guard between the audio and the pilot): the noise the pilot is
    /// judged against. DecSAT judged the pilot's level alone, and FM noise
    /// in a 500 Hz band reads above its threshold, so hiss went stereo.
    guard_i: LowPass3,
    guard_q: LowPass3,
    guard_phase: f64,
    guard_level: f32,
    /// The pilot measured through the same filter as the guard band (the
    /// PLL's own one-pole detector lets the pilot leak 1.5 kHz across).
    meas_i: LowPass3,
    meas_q: LowPass3,
    meas_level: f32,
    rds: RdsDecoder,
    /// Channel power over the report period.
    energy: f64,
    count: u64,
    pcm: Vec<f32>,
    last_station: Option<String>,
    last_text: Option<String>,
    settings: Settings,
    /// How much of the stereo difference is let through, 0..1.
    blend: f32,
    /// RDS groups and samples at the start of the rate's window.
    rate_mark: (u64, u64),
    samples: u64,
    rds_rate: f64,
}

impl Analog {
    fn new(mode: Mode) -> Self {
        Analog {
            mode,
            previous: Iq::new(1.0, 0.0),
            power: 0.1,
            dc: 0.0,
            fm_dc: 0.0,
            mono: LowPass3::default(),
            difference: LowPass3::default(),
            ssb_fir: None,
            mixed: Vec::new(),
            filtered: Vec::new(),
            up_phase: 0.0,
            deemph_l: 0.0,
            deemph_r: 0.0,
            phase: 0.0,
            audio_phase: 0.0,
            pilot_i: 0.0,
            pilot_q: 0.0,
            pilot_phase: 0.0,
            pilot_omega: 0.0,
            guard_i: LowPass3::default(),
            guard_q: LowPass3::default(),
            guard_phase: 0.0,
            guard_level: 0.0,
            meas_i: LowPass3::default(),
            meas_q: LowPass3::default(),
            meas_level: 0.0,
            rds: RdsDecoder::default(),
            energy: 0.0,
            count: 0,
            pcm: Vec::new(),
            last_station: None,
            last_text: None,
            settings: Settings::for_mode(mode),
            blend: 0.0,
            rate_mark: (0, 0),
            samples: 0,
            rds_rate: 0.0,
        }
    }

    /// The card: for WFM everything RDS has said; for the others the level.
    fn card(&self, level: f64) -> Card {
        let name = match self.mode {
            Mode::Am => "AM",
            Mode::Dsb => "DSB",
            Mode::Usb => "USB",
            Mode::Lsb => "LSB",
            Mode::Cw => "CW",
            Mode::Nfm => "NFM",
            Mode::Wfm => "WFM",
        };
        let kv = |k: &str, v: String| (k.to_string(), v);
        if self.mode != Mode::Wfm {
            let mut details = vec![kv("level", format!("{level:.1} dBFS"))];
            if let Some(sq) = self.settings.squelch_dbfs {
                details.push(kv("squelch", format!("{sq:.0} dBFS")));
            }
            if self.settings.deemph_us > 0.0 {
                details.push(kv("de-emphasis", format!("{:.0} µs", self.settings.deemph_us)));
            }
            if self.mode == Mode::Cw {
                details.push(kv("note", format!("{:.0} Hz", self.settings.pitch_hz)));
            }
            return Card {
                title: name.into(),
                headline: format!("{level:.1} dBFS"),
                gauge: Some(Gauge {
                    label: "level".into(),
                    value_db: level as f32 + 100.0,
                    need_db: self.settings.squelch_dbfs.map(|s| s as f32 + 100.0),
                    need_label: "squelch".into(),
                    range: (0.0, 100.0),
                }),
                details,
                text: Vec::new(),
            };
        }
        let r = &self.rds;
        let stereo = self.blend > 0.5;
        let mut details = vec![
            kv("level", format!("{level:.1} dBFS")),
            kv("stereo", format!("{:.0} %", self.blend * 100.0)),
        ];
        let mut text = Vec::new();
        if let Some(pi) = r.pi() {
            details.push(kv("PI", format!("{pi:04X}")));
            details.push(kv("PTY", r.pty().map_or("–", pty_name).to_string()));
            let (tp, ta) = r.tp_ta();
            details.push(kv("TP / TA", format!("{} / {}", u8::from(tp), u8::from(ta))));
            details.push(kv("content", if r.music() { "music" } else { "speech" }.to_string()));
        }
        details.push(kv("RDS sync", if r.synced() { "held" } else { "searching" }.to_string()));
        details.push(kv("RDS SNR", format!("{:.1} dB", r.snr_db())));
        details.push(kv("groups", format!("{}", r.groups())));
        details.push(kv("groups/s", format!("{:.1}", self.rds_rate)));
        if let Some(b) = r.bler() {
            details.push(kv("block errors", format!("{:.1} %", b * 100.0)));
            details.push(kv("corrected", format!("{}", r.corrected())));
        }
        details.push(kv("RDS phase", format!("{:.0}°", r.phase_deg())));
        if let Some(c) = r.clock() {
            details.push(kv("clock", c));
        }
        if let Some(t) = r.radio_text() {
            text.push(kv("RadioText", t));
        }
        let af = r.af_mhz();
        if !af.is_empty() {
            let list: Vec<String> = af.iter().map(|f| format!("{f:.1}")).collect();
            text.push(kv("AF (MHz)", list.join(" ")));
        }
        let types = r.group_types();
        if !types.is_empty() {
            text.push(kv("group types", types));
        }
        Card {
            title: format!("WFM · {}", if stereo { "stereo" } else { "mono" }),
            headline: r.station().unwrap_or_else(|| "–".into()),
            gauge: Some(Gauge {
                label: "pilot".into(),
                value_db: self.pilot_snr_db(),
                // The blend is full at 6x the guard band's noise: 15.6 dB.
                need_db: Some(15.6),
                need_label: "full stereo".into(),
                range: (0.0, 60.0),
            }),
            details,
            text,
        }
    }

    /// The pilot over the guard band's noise, dB.
    fn pilot_snr_db(&self) -> f32 {
        20.0 * (self.meas_level.max(EPS) / self.guard_level.max(EPS)).log10()
    }

    fn report(&mut self, rate: f64, out: &mut Sink) {
        let mean = self.energy / self.count.max(1) as f64;
        let level = 10.0 * mean.max(1e-20).log10();
        out.measure("Level", level, "dBFS");
        out.lock = Lock::Locked;
        self.samples += self.count;
        // RDS groups a second, over windows of about a second.
        if (self.samples - self.rate_mark.1) as f64 >= rate {
            let dt = (self.samples - self.rate_mark.1) as f64 / rate;
            self.rds_rate = (self.rds.groups() - self.rate_mark.0) as f64 / dt;
            self.rate_mark = (self.rds.groups(), self.samples);
        }
        out.card = Some(self.card(level));
        if self.mode == Mode::Wfm && rate >= 120_000.0 {
            out.measure("Pilot", self.pilot_snr_db() as f64, "dB");
            out.measure("RDS groups", self.rds.groups() as f64, "");
            out.measure("RDS phase", self.rds.phase_deg() as f64, "°");
            out.measure("RDS SNR", self.rds.snr_db() as f64, "dB");
            out.scopes.push(("RDS", self.rds.points()));
            let station = self.rds.station();
            if station != self.last_station {
                out.services = Some(vec![Service {
                    id: 0,
                    name: station.clone().unwrap_or_default(),
                    kind: "FM".into(),
                    ..Service::default()
                }]);
                self.last_station = station;
            }
            let text = self.rds.radio_text();
            if text != self.last_text {
                if let Some(t) = &text {
                    out.text.push((0, t.clone()));
                }
                self.last_text = text;
            }
        }
        self.energy = 0.0;
        self.count = 0;
    }
}

impl Decoder for Analog {
    fn process(&mut self, block: &Block<'_>, out: &mut Sink) {
        let iq = block.iq;
        if iq.is_empty() {
            return;
        }
        let rate = block.tuning.sample_rate;
        let mode = self.mode;
        if self.pilot_omega == 0.0 {
            self.pilot_omega = TAU * 19_000.0 / rate;
        }
        self.pcm.clear();
        let is_fm = matches!(mode, Mode::Nfm | Mode::Wfm);
        let audio_hz = match mode {
            Mode::Wfm => 15_000.0,
            Mode::Am | Mode::Dsb => 5_000.0,
            Mode::Cw => 800.0,
            _ => 3_500.0,
        };
        let audio_k = coeff(audio_hz, rate);
        // Weaver: the voice band's middle to 0 Hz; CW listens on the carrier.
        let shift = match mode {
            Mode::Usb => 1_700.0,
            Mode::Lsb => -1_700.0,
            _ => 0.0,
        };
        let pitch = if mode == Mode::Cw { self.settings.pitch_hz } else { 0.0 };
        let diff_k = coeff(15_000.0, rate);
        // 50 µs broadcast FM de-emphasis (Europe); 300 µs for NFM voice.
        // De-emphasis: a pole at 1/(2πτ) (50 µs: 3183 Hz); τ = 0 leaves it out.
        let tau = self.settings.deemph_us * 1e-6;
        let de_k = if tau > 0.0 { coeff(1.0 / (TAU * tau), rate) } else { 1.0 };
        let block_energy: f64 = iq.iter().map(|x| x.norm_sqr() as f64).sum();
        let block_db = 10.0 * (block_energy / iq.len() as f64).max(1e-20).log10();
        let muted = self.settings.squelch_dbfs.is_some_and(|sq| block_db < sq);
        let dc_k = coeff(15.0, rate);
        let agc_k = coeff(12.0, rate);
        let pilot_k = coeff(500.0, rate);
        let pilot_energy_k = coeff(8.0, rate);
        let pll_limit = TAU * 300.0 / rate;
        let guard_step = TAU * 17_500.0 / rate;
        let meas_k = coeff(200.0, rate);
        let stereo = mode == Mode::Wfm && rate >= 120_000.0;
        let allow_stereo = self.settings.stereo;
        let weaver = matches!(mode, Mode::Usb | Mode::Lsb | Mode::Cw);
        if weaver {
            // Down by `shift` and through the low-pass, the whole block at
            // once (the FIR's dot products vectorise).
            let fir = self.ssb_fir.get_or_insert_with(|| {
                let edge = if mode == Mode::Cw { 250.0 } else { 1_550.0 };
                let taps = ((SSB_TAPS as f64 * rate / 48_000.0) as usize).clamp(63, 4095);
                Fir::new(lowpass(edge / rate, taps))
            });
            let step = TAU * shift / rate;
            self.mixed.clear();
            for &x in iq {
                self.phase = (self.phase + step).rem_euclid(TAU);
                self.mixed.push(x * Iq::from_polar(1.0, -self.phase as f32));
            }
            self.filtered.clear();
            fir.process(&self.mixed, &mut self.filtered);
        }
        let up_step = TAU * (shift + pitch) / rate;

        for (i, &x) in iq.iter().enumerate() {
            let amp2 = x.norm_sqr();
            self.energy += amp2 as f64;
            self.power += agc_k * (amp2 - self.power);
            let gain = 0.6 / self.power.max(EPS).sqrt();
            let mut left: f32;
            let mut right: f32;
            if is_fm {
                let delta = x * self.previous.conj();
                let deviation = if mode == Mode::Nfm { 5_000.0 } else { 75_000.0 };
                let multiplex =
                    (delta.im.atan2(delta.re) * (rate / (TAU * deviation)) as f32).clamp(-4.0, 4.0);
                self.fm_dc += dc_k * (multiplex - self.fm_dc);
                let multiplex = multiplex - self.fm_dc;
                if mode == Mode::Wfm && rate >= 140_000.0 {
                    self.rds.feed(multiplex, self.pilot_phase, rate);
                }
                let mono = self.mono.tick(multiplex, audio_k);
                if stereo {
                    // Coherent 19 kHz pilot detector and a simple digital
                    // PLL; twice its phase demodulates the 38 kHz L−R DSB.
                    let cs = self.pilot_phase.cos() as f32;
                    let sn = self.pilot_phase.sin() as f32;
                    self.pilot_i += pilot_k * (multiplex * cs - self.pilot_i);
                    self.pilot_q += pilot_k * (multiplex * -sn - self.pilot_q);
                    let error = self.pilot_q.atan2(self.pilot_i) as f64;
                    self.pilot_omega += (0.000_000_12 * error).clamp(-pll_limit, pll_limit);
                    self.pilot_omega =
                        self.pilot_omega.clamp(TAU * 18_800.0 / rate, TAU * 19_200.0 / rate);
                    self.pilot_phase =
                        (self.pilot_phase + self.pilot_omega + 0.0015 * error).rem_euclid(TAU);
                    let m = self.meas_i.tick(multiplex * cs, meas_k).hypot(self.meas_q.tick(multiplex * -sn, meas_k));
                    self.meas_level += pilot_energy_k * (m - self.meas_level);
                    self.guard_phase = (self.guard_phase + guard_step).rem_euclid(TAU);
                    let (gs, gc) = self.guard_phase.sin_cos();
                    let g = self.guard_i.tick(multiplex * gc as f32, meas_k)
                        .hypot(self.guard_q.tick(multiplex * -gs as f32, meas_k));
                    self.guard_level += pilot_energy_k * (g - self.guard_level);
                    let difference = self.difference.tick(
                        multiplex * (2.0 * self.pilot_phase).cos() as f32 * 2.0,
                        diff_k,
                    );
                    // Fade to mono when the pilot is not clearly above the
                    // noise: none at 2x the guard band, full stereo at 6x.
                    let ratio = self.meas_level / self.guard_level.max(EPS);
                    let blend = if allow_stereo { ((ratio - 2.0) / 4.0).clamp(0.0, 1.0) } else { 0.0 };
                    self.blend = blend;
                    left = mono + difference * blend;
                    right = mono - difference * blend;
                } else {
                    left = mono;
                    right = mono;
                }
                self.deemph_l += de_k * (left - self.deemph_l);
                self.deemph_r += de_k * (right - self.deemph_r);
                left = self.deemph_l;
                right = self.deemph_r;
            } else {
                let y = match mode {
                    Mode::Am => amp2.sqrt() * gain,
                    Mode::Dsb => x.re * gain,
                    Mode::Usb | Mode::Lsb | Mode::Cw => {
                        // Back up by `shift` plus CW's note; the real part is
                        // the audio. (The FIR's delay only rotates the
                        // phase, which the real part does not hear.)
                        self.up_phase = (self.up_phase + up_step).rem_euclid(TAU);
                        let y = self.filtered.get(i).copied().unwrap_or_default();
                        (y * Iq::from_polar(1.0, self.up_phase as f32)).re * gain
                    }
                    _ => 0.0,
                };
                left = match mode {
                    Mode::Am => {
                        self.dc += dc_k * (y - self.dc);
                        self.mono.tick(y - self.dc, audio_k)
                    }
                    Mode::Usb | Mode::Lsb | Mode::Cw => y,
                    _ => self.mono.tick(y, audio_k),
                };
                right = left;
            }
            self.previous = x;
            self.audio_phase += PCM_RATE as f64 / rate;
            if self.audio_phase >= 1.0 {
                self.audio_phase -= 1.0;
                self.pcm.push((left * 0.7).clamp(-1.0, 1.0));
                self.pcm.push((right * 0.7).clamp(-1.0, 1.0));
            }
        }
        self.count += iq.len() as u64;
        if muted {
            self.pcm.fill(0.0);
        }
        if !self.pcm.is_empty() {
            out.audio.push(Audio {
                service: 0,
                sample_rate: PCM_RATE,
                channels: 2,
                samples: self.pcm.clone(),
            });
        }
        // A handful of channel samples for the scope.
        let step = (iq.len() / 128).max(1);
        out.constellation.extend(iq.iter().step_by(step).map(|s| s * (0.5 / self.power.max(EPS).sqrt())));
        if self.count as f64 >= REPORT_S * rate {
            self.report(rate, out);
        }
    }

    fn command(&mut self, cmd: Command, out: &mut Sink) {
        if let Command::Set { key, value } = cmd
            && let Err(e) = self.settings.set(&key, &value)
        {
            out.log(e);
        }
    }

    fn reset(&mut self) {
        // A retune forgets the signal, not what the panel set.
        let settings = self.settings;
        *self = Analog::new(self.mode);
        self.settings = settings;
    }
}

/// The settings panels, drawn by the decoder (see `declab_api::Panel`).
#[cfg(feature = "ui")]
mod panel {
    use declab_api::{Command, Panel};
    use egui::Ui;

    use super::{Mode, Settings};

    pub fn for_mode(mode: Mode) -> Option<Box<dyn Panel>> {
        matches!(mode, Mode::Wfm | Mode::Nfm | Mode::Cw)
            .then(|| Box::new(AnalogPanel { mode, s: Settings::for_mode(mode) }) as Box<dyn Panel>)
    }

    struct AnalogPanel {
        mode: Mode,
        /// What has been sent, so the controls show it.
        s: Settings,
    }

    fn set(commands: &mut Vec<Command>, key: &str, value: String) {
        commands.push(Command::Set { key: key.into(), value });
    }

    fn deemph_label(us: f64) -> String {
        if us == 0.0 { "off".into() } else { format!("{us:.0} µs") }
    }

    impl Panel for AnalogPanel {
        fn ui(&mut self, ui: &mut Ui, commands: &mut Vec<Command>) {
            egui::Grid::new("analog_panel").num_columns(2).spacing([16.0, 8.0]).show(ui, |ui| {
                if matches!(self.mode, Mode::Wfm | Mode::Nfm) {
                    let choices: &[f64] =
                        if self.mode == Mode::Wfm { &[50.0, 75.0, 0.0] } else { &[300.0, 750.0, 0.0] };
                    ui.label("De-emphasis");
                    egui::ComboBox::from_id_salt("deemph")
                        .selected_text(deemph_label(self.s.deemph_us))
                        .show_ui(ui, |ui| {
                            for &us in choices {
                                if ui.selectable_label(self.s.deemph_us == us, deemph_label(us)).clicked() {
                                    self.s.deemph_us = us;
                                    set(commands, "deemph", us.to_string());
                                }
                            }
                        });
                    ui.end_row();
                }
                if self.mode == Mode::Wfm {
                    ui.label("Stereo");
                    egui::ComboBox::from_id_salt("stereo")
                        .selected_text(if self.s.stereo { "auto" } else { "mono" })
                        .show_ui(ui, |ui| {
                            for (on, name) in [(true, "auto"), (false, "mono")] {
                                if ui.selectable_label(self.s.stereo == on, name).clicked() {
                                    self.s.stereo = on;
                                    set(commands, "stereo", name.into());
                                }
                            }
                        });
                    ui.end_row();
                }
                if self.mode == Mode::Nfm {
                    ui.label("Squelch");
                    let mut on = self.s.squelch_dbfs.is_some();
                    let mut level = self.s.squelch_dbfs.unwrap_or(-60.0);
                    ui.horizontal(|ui| {
                        let a = ui.checkbox(&mut on, "").changed();
                        let b = ui
                            .add_enabled(
                                on,
                                egui::Slider::new(&mut level, -100.0..=-20.0).suffix(" dBFS").integer(),
                            )
                            .changed();
                        if a || b {
                            self.s.squelch_dbfs = on.then_some(level);
                            let v = if on { format!("{level:.0}") } else { "off".into() };
                            set(commands, "squelch", v);
                        }
                    });
                    ui.end_row();
                }
                if self.mode == Mode::Cw {
                    ui.label("Note");
                    let mut p = self.s.pitch_hz;
                    if ui
                        .add(egui::Slider::new(&mut p, 300.0..=1200.0).suffix(" Hz").step_by(50.0))
                        .changed()
                    {
                        self.s.pitch_hz = p;
                        set(commands, "pitch", format!("{p:.0}"));
                    }
                    ui.end_row();
                }
            });
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use declab_api::Tuning;

    /// Run `mode` over `iq` at `rate`, returning the left channel.
    fn demod(mode: Mode, iq: &[Iq], rate: f64) -> Vec<f32> {
        let mut d = Analog::new(mode);
        let mut sink = Sink::default();
        let block = Block {
            iq,
            tuning: Tuning { center_hz: 0.0, sample_rate: rate, generation: 0 },
            first_sample: 0,
            dropped_before: 0,
        };
        d.process(&block, &mut sink);
        sink.audio.iter().flat_map(|a| a.samples.iter().step_by(2).copied()).collect()
    }

    /// Strength of `hz` in `x` (48 kHz), by correlation.
    fn tone(x: &[f32], hz: f64) -> f64 {
        let (mut c, mut s) = (0.0, 0.0);
        for (n, v) in x.iter().enumerate() {
            let p = TAU * hz * n as f64 / 48_000.0;
            c += *v as f64 * p.cos();
            s += *v as f64 * p.sin();
        }
        (c * c + s * s).sqrt() / x.len() as f64
    }

    #[test]
    fn nfm_recovers_a_1khz_tone() {
        let rate = 48_000.0;
        let mut ph = 0.0f64;
        let iq: Vec<Iq> = (0..48_000)
            .map(|n| {
                let f = 3_000.0 * (TAU * 1_000.0 * n as f64 / rate).sin();
                ph += TAU * f / rate;
                Iq::new(ph.cos() as f32, ph.sin() as f32) * 0.3
            })
            .collect();
        let audio = demod(Mode::Nfm, &iq, rate);
        let tail = &audio[audio.len() / 2..];
        assert!(tone(tail, 1_000.0) > 5.0 * tone(tail, 1_700.0), "no 1 kHz tone");
    }

    #[test]
    fn usb_puts_a_carrier_offset_tone_at_its_offset() {
        // A tone 1 kHz above the carrier is 1 kHz of audio on USB.
        let rate = 48_000.0;
        let iq: Vec<Iq> = (0..48_000)
            .map(|n| {
                let p = TAU * 1_000.0 * n as f64 / rate;
                Iq::new(p.cos() as f32, p.sin() as f32) * 0.2
            })
            .collect();
        let audio = demod(Mode::Usb, &iq, rate);
        let tail = &audio[audio.len() / 2..];
        let lsb = demod(Mode::Lsb, &iq, rate);
        let lsb_tail = &lsb[lsb.len() / 2..];
        let (u, l) = (tone(tail, 1_000.0), tone(lsb_tail, 1_000.0));
        assert!(u > 5.0 * tone(tail, 700.0), "USB at the wrong pitch");
        // The other sideband is rejected (40 dB or more).
        assert!(u > 100.0 * l, "LSB lets the upper sideband through: {u} vs {l}");
    }

    /// Broadcast FM at 384 kHz carrying `multiplex(t)`, 75 kHz deviation
    /// at full scale, plus complex noise of `noise` RMS.
    fn wfm(seconds: f64, noise: f32, multiplex: impl Fn(f64) -> f64) -> Vec<Iq> {
        let rate = 384_000.0;
        let mut ph = 0.0f64;
        let mut state = 0x9e37_79b9_7f4a_7c15u64;
        let mut rnd = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / (1u64 << 24) as f32 - 0.5
        };
        (0..(seconds * rate) as usize)
            .map(|n| {
                ph += TAU * 75_000.0 * multiplex(n as f64 / rate) / rate;
                Iq::from_polar(0.3, ph as f32) + Iq::new(rnd(), rnd()) * (noise * 2.45)
            })
            .collect()
    }

    fn pilot_db(iq: &[Iq]) -> f64 {
        let mut d = Analog::new(Mode::Wfm);
        let mut sink = Sink::default();
        let block = Block {
            iq,
            tuning: Tuning { center_hz: 0.0, sample_rate: 384_000.0, generation: 0 },
            first_sample: 0,
            dropped_before: 0,
        };
        d.process(&block, &mut sink);
        d.pilot_snr_db() as f64
    }

    #[test]
    fn stereo_pilot_is_told_from_noise() {
        // A 1 kHz tone on the left only, with the 19 kHz pilot at 10 %.
        let stereo = wfm(1.0, 0.01, |t| {
            let l = (TAU * 1_000.0 * t).sin();
            0.45 * l + 0.45 * l * (TAU * 38_000.0 * t).sin() + 0.1 * (TAU * 19_000.0 * t).sin()
        });
        let hiss = wfm(1.0, 0.0, |_| 0.0)
            .into_iter()
            .enumerate()
            .map(|(n, _)| {
                // Noise only: no carrier at all.
                let x = ((n as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 40) as f32;
                let y = ((n as u64).wrapping_mul(0xc2b2_ae3d_27d4_eb4f) >> 40) as f32;
                Iq::new(x / (1u64 << 24) as f32 - 0.5, y / (1u64 << 24) as f32 - 0.5)
            })
            .collect::<Vec<_>>();
        let (p, n) = (pilot_db(&stereo), pilot_db(&hiss));
        assert!(p > 20.0, "pilot only {p:.1} dB over the guard band");
        assert!(n < 6.0, "noise reads as a {n:.1} dB pilot");
    }

    #[test]
    fn cw_sounds_a_carrier_at_800_hz() {
        let iq = vec![Iq::new(0.2, 0.0); 48_000];
        let audio = demod(Mode::Cw, &iq, 48_000.0);
        let tail = &audio[audio.len() / 2..];
        assert!(tone(tail, 800.0) > 5.0 * tone(tail, 1_000.0));
    }
}
