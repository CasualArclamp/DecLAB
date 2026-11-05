// made by CasualArclamp
//! What a device decoder is: its line coding and timings (rtl_433's
//! `r_device`), a decode function over a [`BitBuffer`], and the messages it
//! hands back.
//!
//! The shape follows rtl_433's `r_device.h` and `data.h` (GPL-2.0-or-later)
//! so its decoders port line for line: `data_make("model", …, "id", …)`
//! becomes `Message::new("Nexus-TH").int("id", id)…`, and
//! `decoder_output_data` becomes [`Ctx::output`].

use crate::bits::BitBuffer;

/// Line coding and modulation, as rtl_433's `modulation_types`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Modulation {
    /// OOK, Manchester, hard-coded zero bit first (rising edge 0, falling 1).
    OokManchesterZeroBit,
    /// OOK, NRZ (`short == long`) or RZ (`short < long`): pulse 1, none 0.
    OokPcm,
    /// OOK, pulse position: short gap 0, long gap 1.
    OokPpm,
    /// OOK, pulse width: short pulse 1, long pulse 0.
    OokPwm,
    /// OOK, a level change each bit, raw.
    OokPiwmRaw,
    /// OOK, a level change each bit, DC-free.
    OokPiwmDc,
    /// OOK, differential Manchester.
    OokDmc,
    /// OOK, NRZ-S.
    OokNrzs,
    /// FSK, NRZ or RZ.
    FskPcm,
    /// FSK, pulse width.
    FskPwm,
    /// FSK, Manchester zero bit.
    FskManchesterZeroBit,
}

impl Modulation {
    pub fn is_fsk(self) -> bool {
        matches!(
            self,
            Modulation::FskPcm | Modulation::FskPwm | Modulation::FskManchesterZeroBit
        )
    }

    /// rtl_433's names, as the flex decoder's `m=` takes them.
    pub fn name(self) -> &'static str {
        match self {
            Modulation::OokManchesterZeroBit => "OOK_MC_ZEROBIT",
            Modulation::OokPcm => "OOK_PCM",
            Modulation::OokPpm => "OOK_PPM",
            Modulation::OokPwm => "OOK_PWM",
            Modulation::OokPiwmRaw => "OOK_PIWM_RAW",
            Modulation::OokPiwmDc => "OOK_PIWM_DC",
            Modulation::OokDmc => "OOK_DMC",
            Modulation::OokNrzs => "OOK_NRZS",
            Modulation::FskPcm => "FSK_PCM",
            Modulation::FskPwm => "FSK_PWM",
            Modulation::FskManchesterZeroBit => "FSK_MC_ZEROBIT",
        }
    }

    /// Short labels for the card ("OOK PWM").
    pub fn label(self) -> &'static str {
        match self {
            Modulation::OokManchesterZeroBit => "OOK Manchester",
            Modulation::OokPcm => "OOK PCM",
            Modulation::OokPpm => "OOK PPM",
            Modulation::OokPwm => "OOK PWM",
            Modulation::OokPiwmRaw | Modulation::OokPiwmDc => "OOK PIWM",
            Modulation::OokDmc => "OOK DMC",
            Modulation::OokNrzs => "OOK NRZS",
            Modulation::FskPcm => "FSK PCM",
            Modulation::FskPwm => "FSK PWM",
            Modulation::FskManchesterZeroBit => "FSK Manchester",
        }
    }
}

/// What kind of thing a device is: the panel turns groups on and off, and
/// the devices table can sort by it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Group {
    /// Weather stations, temperature/humidity sensors.
    Weather,
    /// Remotes, door/window and security sensors, doorbells.
    Remote,
    /// Tyre-pressure sensors.
    Tpms,
    /// Energy monitors and utility meters.
    Meter,
    /// User-written flex decoders.
    Flex,
}

impl Group {
    pub const ALL: [Group; 5] = [
        Group::Weather,
        Group::Remote,
        Group::Tpms,
        Group::Meter,
        Group::Flex,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Group::Weather => "Weather and sensors",
            Group::Remote => "Remotes and security",
            Group::Tpms => "Tyre pressure",
            Group::Meter => "Energy and meters",
            Group::Flex => "Flex",
        }
    }

    pub fn key(self) -> &'static str {
        match self {
            Group::Weather => "weather",
            Group::Remote => "remotes",
            Group::Tpms => "tpms",
            Group::Meter => "meters",
            Group::Flex => "flex",
        }
    }
}

/// rtl_433's `decode_return_codes`: a decode function returns the number
/// of messages it output (> 0) or one of these.
pub const DECODE_ABORT_LENGTH: i32 = -1;
pub const DECODE_ABORT_EARLY: i32 = -2;
pub const DECODE_FAIL_MIC: i32 = -3;
pub const DECODE_FAIL_SANITY: i32 = -4;

/// A device decode function: rtl_433's `decode_fn`.
///
/// Rust note: a plain `fn` pointer (not a closure), so a `Device` can be a
/// `static`.
pub type DecodeFn = fn(&mut Ctx, &mut BitBuffer) -> i32;

/// A device decoder: rtl_433's `r_device`. Widths are in microseconds.
#[derive(Clone, Copy)]
pub struct Device {
    /// rtl_433's name (the long one, listing the models it covers).
    pub name: &'static str,
    pub group: Group,
    pub modulation: Modulation,
    pub short_width: f32,
    pub long_width: f32,
    /// A gap longer than this ends a transmission.
    pub reset_limit: f32,
    /// A gap longer than this ends a row (0: none).
    pub gap_limit: f32,
    pub sync_width: f32,
    pub tolerance: f32,
    pub decode: DecodeFn,
    /// Runs after every decoder of a lower priority, and only if none of
    /// them output anything (rtl_433's `priority`).
    pub priority: u32,
    /// Off unless the user turns it on (rtl_433's `disabled = 1`: prone
    /// to false positives).
    pub disabled: bool,
}

impl Device {
    /// A device with every timing 0 and priority 0: fill in with struct
    /// update syntax (`Device { name: …, ..Device::DEFAULT }`).
    pub const DEFAULT: Device = Device {
        name: "",
        group: Group::Weather,
        modulation: Modulation::OokPwm,
        short_width: 0.0,
        long_width: 0.0,
        reset_limit: 0.0,
        gap_limit: 0.0,
        sync_width: 0.0,
        tolerance: 0.0,
        decode: |_, _| DECODE_ABORT_EARLY,
        priority: 0,
        disabled: false,
    };
}

impl std::fmt::Debug for Device {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Device({:?}, {})", self.name, self.modulation.name())
    }
}

/// One field's value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Int(i64),
    /// A number and the decimals to show it with.
    Float(f64, u8),
    Str(String),
    /// Shown in hex with at least this many digits (an ID, a code).
    Hex(u64, u8),
}

impl std::fmt::Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::Int(v) => write!(f, "{v}"),
            Value::Float(v, d) => write!(f, "{v:.*}", *d as usize),
            Value::Str(s) => f.write_str(s),
            Value::Hex(v, d) => write!(f, "0x{v:0w$x}", w = *d as usize),
        }
    }
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Int(v) => Some(*v as f64),
            Value::Float(v, _) => Some(*v),
            Value::Hex(v, _) => Some(*v as f64),
            Value::Str(_) => None,
        }
    }
}

/// One decoded transmission: rtl_433's `data_t` list, in its order.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Message {
    pub model: String,
    /// `(key, value)`, keys as rtl_433's (`id`, `channel`, `battery_ok`,
    /// `temperature_C`, `humidity`, `pressure_kPa`, `mic`, …).
    pub fields: Vec<(&'static str, Value)>,
}

impl Message {
    pub fn new(model: impl Into<String>) -> Self {
        Message {
            model: model.into(),
            fields: Vec::with_capacity(8),
        }
    }

    pub fn int(mut self, key: &'static str, v: impl Into<i64>) -> Self {
        self.fields.push((key, Value::Int(v.into())));
        self
    }

    /// A number shown with `decimals` places (rtl_433's `DATA_FORMAT,
    /// "%.1f C"` becomes `.float("temperature_C", t, 1)`).
    pub fn float(mut self, key: &'static str, v: f64, decimals: u8) -> Self {
        self.fields.push((key, Value::Float(v, decimals)));
        self
    }

    pub fn str(mut self, key: &'static str, v: impl Into<String>) -> Self {
        self.fields.push((key, Value::Str(v.into())));
        self
    }

    /// A number shown in hex, at least `digits` digits (rtl_433's
    /// `DATA_FORMAT, "%08x"`).
    pub fn hex(mut self, key: &'static str, v: u64, digits: u8) -> Self {
        self.fields.push((key, Value::Hex(v, digits)));
        self
    }

    /// rtl_433's `DATA_COND`: the field only when `cond` holds.
    pub fn cond(self, cond: bool, f: impl FnOnce(Self) -> Self) -> Self {
        if cond { f(self) } else { self }
    }

    pub fn get(&self, key: &str) -> Option<&Value> {
        self.fields.iter().find(|(k, _)| *k == key).map(|(_, v)| v)
    }

    /// The device's identity for the heard list: model, `id` and
    /// `channel` as text.
    pub fn id_text(&self) -> String {
        self.get("id").map(|v| v.to_string()).unwrap_or_default()
    }

    pub fn channel_text(&self) -> String {
        self.get("channel")
            .map(|v| v.to_string())
            .unwrap_or_default()
    }

    /// The readings, short, for a table cell: "18.4 °C 57 %".
    pub fn readings(&self) -> String {
        let mut s = String::new();
        for (k, v) in &self.fields {
            if matches!(
                *k,
                "id" | "channel"
                    | "mic"
                    | "model"
                    | "subtype"
                    | "battery_ok"
                    | "flags"
                    | "test"
                    | "status"
            ) {
                continue;
            }
            let (_, unit) = field_label(k);
            if !s.is_empty() {
                s.push(' ');
            }
            s.push_str(&v.to_string());
            if !unit.is_empty() {
                if unit != "%" {
                    s.push(' ');
                }
                s.push_str(unit);
            }
            if s.len() > 60 {
                break;
            }
        }
        if let Some(Value::Int(0)) = self.get("battery_ok") {
            s.push_str(" batt low");
        }
        s
    }
}

/// A field key's label and unit for the card: rtl_433's naming
/// conventions (`temperature_C`, `wind_avg_km_h`) read back.
pub fn field_label(key: &str) -> (&str, &'static str) {
    const UNITS: &[(&str, &str, &str)] = &[
        ("temperature_C", "temperature", "°C"),
        ("temperature_F", "temperature", "°F"),
        ("temperature_1_C", "temperature 1", "°C"),
        ("temperature_2_C", "temperature 2", "°C"),
        ("setpoint_C", "setpoint", "°C"),
        ("humidity", "humidity", "%"),
        ("moisture", "moisture", "%"),
        ("battery_ok", "battery", ""),
        ("battery_V", "battery", "V"),
        ("battery_mV", "battery", "mV"),
        ("pressure_kPa", "pressure", "kPa"),
        ("pressure_PSI", "pressure", "PSI"),
        ("pressure_bar", "pressure", "bar"),
        ("pressure_hPa", "pressure", "hPa"),
        ("wind_avg_km_h", "wind", "km/h"),
        ("wind_max_km_h", "gust", "km/h"),
        ("wind_avg_m_s", "wind", "m/s"),
        ("wind_max_m_s", "gust", "m/s"),
        ("wind_avg_mi_h", "wind", "mph"),
        ("wind_max_mi_h", "gust", "mph"),
        ("wind_dir_deg", "wind dir", "°"),
        ("rain_mm", "rain", "mm"),
        ("rain_in", "rain", "in"),
        ("rain_rate_mm_h", "rain rate", "mm/h"),
        ("rain_rate_in_h", "rain rate", "in/h"),
        ("light_lux", "light", "lux"),
        ("uv", "UV", ""),
        ("uvi", "UV index", ""),
        ("power_W", "power", "W"),
        ("power0_W", "power", "W"),
        ("power1_W", "power 2", "W"),
        ("power2_W", "power 3", "W"),
        ("energy_kWh", "energy", "kWh"),
        ("current_A", "current", "A"),
        ("voltage_V", "voltage", "V"),
        ("consumption_data", "consumption", ""),
        ("strike_count", "strikes", ""),
        ("storm_dist_km", "storm", "km"),
        ("depth_cm", "depth", "cm"),
        ("snr_dB", "SNR", "dB"),
    ];
    for (k, label, unit) in UNITS {
        if *k == key {
            return (label, unit);
        }
    }
    (key, "")
}

/// What a decode function writes to: rtl_433's `decoder_output_data` and
/// `decoder_log` targets.
#[derive(Default)]
pub struct Ctx {
    pub out: Vec<Message>,
    /// Debug lines, kept only when `verbose` (rtl_433's `-v`).
    pub verbose: bool,
    pub log: Vec<String>,
}

impl Ctx {
    pub fn output(&mut self, m: Message) {
        self.out.push(m);
    }

    /// rtl_433's `decoder_log`/`decoder_logf`: kept only when verbose.
    pub fn log(&mut self, f: impl FnOnce() -> String) {
        if self.verbose {
            self.log.push(f());
        }
    }
}

/// Run one device on rows as its slicer would hand them over (tests and
/// the simulator's checks).
pub fn decode_rows(dev: &Device, code: &str) -> Vec<Message> {
    let mut bits = BitBuffer::parse(code);
    let mut ctx = Ctx::default();
    (dev.decode)(&mut ctx, &mut bits);
    ctx.out
}
