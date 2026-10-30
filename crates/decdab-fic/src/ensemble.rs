// made by CasualArclamp
//! What the FIC says about an ensemble: its identity and label, its
//! sub-channels, its services and their components, labels, programme
//! types, user applications and the time.

use std::collections::BTreeMap;

use decdab_fec::{EepSet, Plan};

/// A label (FIG 1, EN 300 401 §8.1.13): 16 characters, and the short form
/// the character flags pick out of them.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Label {
    pub text: String,
    pub short: String,
}

/// A sub-channel's protection (FIG 0/1, §6.2.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Protection {
    /// Unequal: the index into EN 300 401 Table 31.
    Uep(u8),
    /// Equal: profile set and level (1–4).
    Eep(EepSet, u8),
}

/// A sub-channel of the MSC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubChannel {
    pub id: u8,
    /// First capacity unit in the CIF (0–863).
    pub start: u16,
    /// Capacity units it spans.
    pub size: u16,
    pub protection: Protection,
    /// FIG 0/14's FEC scheme for packet mode (0: none, 1: RS).
    pub fec_scheme: u8,
}

impl SubChannel {
    /// Its puncturing plan; `None` if FIG 0/1 named an impossible one.
    pub fn plan(&self) -> Option<Plan> {
        match self.protection {
            Protection::Uep(i) => Plan::uep(i as usize),
            Protection::Eep(set, level) => Plan::eep(set, level, self.size as usize),
        }
    }

    /// Bit rate, kbit/s.
    pub fn bitrate(&self) -> u32 {
        self.plan().map_or(0, |p| p.info_bits() as u32 / 24)
    }

    /// "UEP 3", "EEP 3-A" and so on.
    pub fn protection_name(&self) -> String {
        match self.protection {
            Protection::Uep(i) => {
                let level = decdab_fec::conv::UEP_TABLE
                    .get(i as usize)
                    .map_or(0, |r| r.1);
                format!("UEP {level}")
            }
            Protection::Eep(set, level) => format!(
                "EEP {level}-{}",
                if set == EepSet::A { 'A' } else { 'B' }
            ),
        }
    }
}

/// How a service component is carried (FIG 0/2's TMId, §6.3.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    /// MSC stream audio; ASCTy 0 is MPEG Layer II, 63 is DAB+.
    Audio { ascty: u8, subch: u8 },
    /// MSC stream data.
    StreamData { dscty: u8, subch: u8 },
    /// MSC packet data: the service component identifier (FIG 0/3 tells the
    /// rest).
    Packet { scid: u16 },
    /// Fast information data channel.
    Fidc { fidcid: u8 },
}

/// A user application (FIG 0/13, §6.3.6; types in TS 101 756 Table 16).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UserApp {
    pub kind: u16,
    pub data: Vec<u8>,
}

impl UserApp {
    pub fn name(&self) -> String {
        match self.kind {
            0x002 => "Slideshow".into(),
            0x003 => "Web site".into(),
            0x004 => "TPEG".into(),
            0x005 => "DGPS".into(),
            0x006 => "TMC".into(),
            0x007 => "SPI/EPG".into(),
            0x008 => "DAB-TV".into(),
            0x00D => "Filecasting".into(),
            0x44A => "Journaline".into(),
            k => format!("app 0x{k:03X}"),
        }
    }
}

/// A packet-mode component (FIG 0/3, §6.3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketComponent {
    pub scid: u16,
    pub subch: u8,
    pub address: u16,
    pub dscty: u8,
    /// Whether data groups are used (FIG 0/3's DG flag is 0 when they are).
    pub data_groups: bool,
}

/// One component of a service.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Component {
    pub transport: Transport,
    pub primary: bool,
    pub ca: bool,
    /// Its identifier within the service (FIG 0/8), once known.
    pub scids: Option<u8>,
    pub label: Option<Label>,
    pub apps: Vec<UserApp>,
}

impl Component {
    /// The sub-channel that carries it, for stream components (packet ones
    /// go through the ensemble's FIG 0/3 table).
    pub fn subch(&self) -> Option<u8> {
        match self.transport {
            Transport::Audio { subch, .. } | Transport::StreamData { subch, .. } => Some(subch),
            _ => None,
        }
    }

    pub fn is_dab_plus(&self) -> bool {
        matches!(self.transport, Transport::Audio { ascty: 63, .. })
    }
}

/// A service (FIG 0/2).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Service {
    /// 16 bits for a programme service, 32 for a data service.
    pub sid: u32,
    pub label: Option<Label>,
    /// Programme type (FIG 0/17), the international code.
    pub pty: Option<u8>,
    pub components: Vec<Component>,
}

impl Service {
    pub fn is_programme(&self) -> bool {
        self.sid <= 0xFFFF
    }

    /// The primary component, or the first.
    pub fn primary(&self) -> Option<&Component> {
        self.components
            .iter()
            .find(|c| c.primary)
            .or(self.components.first())
    }

    /// The label to show: the service's own, or its SId in hex.
    pub fn name(&self) -> String {
        self.label
            .as_ref()
            .map(|l| l.text.trim().to_string())
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| format!("{:04X}", self.sid))
    }
}

/// Date and time (FIG 0/10, §8.1.3.1), UTC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DabTime {
    /// Modified Julian Date.
    pub mjd: u32,
    pub hours: u8,
    pub minutes: u8,
    pub seconds: Option<u8>,
}

impl DabTime {
    /// (year, month, day), from the MJD (the algorithm of EN 300 468
    /// Annex C).
    pub fn date(&self) -> (i32, u32, u32) {
        let mjd = self.mjd as f64;
        let yp = ((mjd - 15078.2) / 365.25).floor();
        let mp = ((mjd - 14956.1 - (yp * 365.25).floor()) / 30.6001).floor();
        let d = mjd - 14956.0 - (yp * 365.25).floor() - (mp * 30.6001).floor();
        let k = if mp == 14.0 || mp == 15.0 { 1.0 } else { 0.0 };
        let y = yp + k + 1900.0;
        let m = mp - 1.0 - k * 12.0;
        (y as i32, m as u32, d as u32)
    }
}

/// Programme type names (TS 101 756 Table 12, the European list).
pub fn pty_name(pty: u8) -> &'static str {
    const NAMES: [&str; 32] = [
        "None",
        "News",
        "Current Affairs",
        "Information",
        "Sport",
        "Education",
        "Drama",
        "Culture",
        "Science",
        "Varied",
        "Pop Music",
        "Rock Music",
        "Easy Listening",
        "Light Classical",
        "Serious Classical",
        "Other Music",
        "Weather",
        "Finance",
        "Children's",
        "Social Affairs",
        "Religion",
        "Phone In",
        "Travel",
        "Leisure",
        "Jazz Music",
        "Country Music",
        "National Music",
        "Oldies Music",
        "Folk Music",
        "Documentary",
        "Alarm Test",
        "Alarm",
    ];
    NAMES[(pty & 31) as usize]
}

/// The whole picture.
#[derive(Debug, Clone, Default)]
pub struct Ensemble {
    pub eid: Option<u16>,
    pub label: Option<Label>,
    /// Extended country code and local time offset (FIG 0/9), half hours.
    pub ecc: Option<u8>,
    pub lto_half_hours: Option<i8>,
    pub time: Option<DabTime>,
    /// CIF count (FIG 0/0): 0–4999, mod 5000.
    pub cif_count: Option<u16>,
    pub subchannels: BTreeMap<u8, SubChannel>,
    pub services: BTreeMap<u32, Service>,
    pub packet: BTreeMap<u16, PacketComponent>,
}

impl Ensemble {
    pub fn name(&self) -> String {
        self.label
            .as_ref()
            .map(|l| l.text.trim().to_string())
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| self.eid.map_or(String::new(), |e| format!("{e:04X}")))
    }

    /// The sub-channel that carries a component.
    pub fn subchannel_of(&self, c: &Component) -> Option<&SubChannel> {
        let id = match c.transport {
            Transport::Packet { scid } => self.packet.get(&scid)?.subch,
            _ => c.subch()?,
        };
        self.subchannels.get(&id)
    }

    /// Services sorted by label, as a receiver lists them.
    pub fn services_by_name(&self) -> Vec<&Service> {
        let mut v: Vec<&Service> = self.services.values().collect();
        v.sort_by_key(|s| s.name().to_lowercase());
        v
    }
}
