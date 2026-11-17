// made by CasualArclamp
//! Where LoRa is sent, region by region: the LoRaWAN channel plans and data
//! rates of the LoRaWAN Regional Parameters (LoRa Alliance; the tables as
//! LoRaMac-node's `src/mac/region/Region*.h` have them, Revised BSD), and
//! Meshtastic's regions with the formula its firmware puts a channel on
//! (`RadioInterface.cpp`, GPL-3.0).
//!
//! A frame's RF frequency, bandwidth and SF are enough to say which plan's
//! channel and data rate it was sent on ([`identify`]), and a plan's
//! channels say where to put the VFOs ([`Plan::describe`]). Only LoRa data
//! rates are listed: FSK and LR-FHSS are not received here.

/// A LoRaWAN data rate: DRn is SF `sf` in `bw_hz`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DataRate {
    pub dr: u8,
    pub sf: u8,
    pub bw_hz: f64,
}

const fn dr(dr: u8, sf: u8, bw_khz: u32) -> DataRate {
    DataRate { dr, sf, bw_hz: bw_khz as f64 * 1e3 }
}

/// `count` channels from `first_hz`, `step_hz` apart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    pub first_hz: f64,
    pub step_hz: f64,
    pub count: u32,
    /// "uplink", "uplink 500 kHz", "downlink".
    pub what: &'static str,
}

impl Grid {
    pub fn channel_hz(&self, n: u32) -> f64 {
        self.first_hz + n as f64 * self.step_hz
    }
}

const fn grid(first_hz: u32, step_hz: u32, count: u32, what: &'static str) -> Grid {
    Grid { first_hz: first_hz as f64, step_hz: step_hz as f64, count, what }
}

/// A LoRaWAN region's channel plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plan {
    /// "EU868".
    pub name: &'static str,
    /// The band the network may put channels in, Hz.
    pub band: (f64, f64),
    /// Fixed channel grids (US915, AU915, CN470). Empty for the plans
    /// whose network adds its own channels to [`Plan::defaults`].
    pub grids: &'static [Grid],
    /// The channels every device has from the start (the join channels).
    pub defaults: &'static [f64],
    /// The second receive window's default frequency and data rate.
    pub rx2: (f64, u8),
    pub rates: &'static [DataRate],
}

/// The usual data rates: SF12 to SF7 at 125 kHz, and SF7 at 250 kHz.
const RATES_EU: &[DataRate] =
    &[dr(0, 12, 125), dr(1, 11, 125), dr(2, 10, 125), dr(3, 9, 125), dr(4, 8, 125), dr(5, 7, 125), dr(6, 7, 250)];
/// KR920: no 250 kHz rate.
const RATES_KR: &[DataRate] =
    &[dr(0, 12, 125), dr(1, 11, 125), dr(2, 10, 125), dr(3, 9, 125), dr(4, 8, 125), dr(5, 7, 125)];
/// US915: SF10 to SF7 up at 125 kHz, SF8 up at 500 kHz, SF12 to SF7 down at 500 kHz.
const RATES_US: &[DataRate] = &[
    dr(0, 10, 125),
    dr(1, 9, 125),
    dr(2, 8, 125),
    dr(3, 7, 125),
    dr(4, 8, 500),
    dr(8, 12, 500),
    dr(9, 11, 500),
    dr(10, 10, 500),
    dr(11, 9, 500),
    dr(12, 8, 500),
    dr(13, 7, 500),
];
/// AU915: as EU up at 125 kHz, SF8 up at 500 kHz, SF12 to SF7 down at 500 kHz.
const RATES_AU: &[DataRate] = &[
    dr(0, 12, 125),
    dr(1, 11, 125),
    dr(2, 10, 125),
    dr(3, 9, 125),
    dr(4, 8, 125),
    dr(5, 7, 125),
    dr(6, 8, 500),
    dr(8, 12, 500),
    dr(9, 11, 500),
    dr(10, 10, 500),
    dr(11, 9, 500),
    dr(12, 8, 500),
    dr(13, 7, 500),
];
/// CN470: SF12 to SF7 at 125 kHz.
const RATES_CN470: &[DataRate] =
    &[dr(0, 12, 125), dr(1, 11, 125), dr(2, 10, 125), dr(3, 9, 125), dr(4, 8, 125), dr(5, 7, 125)];

const fn mhz(lo: f64, hi: f64) -> (f64, f64) {
    (lo * 1e6, hi * 1e6)
}

/// AS923's four groups share one plan moved by an offset (Regional
/// Parameters RP002: 0, −1.8, −6.6 and −5.9 MHz).
macro_rules! as923 {
    ($name:literal, $off:literal) => {
        Plan {
            name: $name,
            band: mhz(915.0, 928.0),
            grids: &[],
            defaults: &[923.2e6 + $off * 1e6, 923.4e6 + $off * 1e6],
            rx2: (923.2e6 + $off * 1e6, 2),
            rates: RATES_EU,
        }
    };
}

/// Every LoRaWAN plan, as the Regional Parameters name them.
pub const PLANS: &[Plan] = &[
    Plan {
        name: "EU868",
        band: mhz(863.0, 870.0),
        grids: &[],
        defaults: &[868.1e6, 868.3e6, 868.5e6],
        rx2: (869.525e6, 0),
        rates: RATES_EU,
    },
    Plan {
        name: "US915",
        band: mhz(902.0, 928.0),
        grids: &[
            grid(902_300_000, 200_000, 64, "uplink"),
            grid(903_000_000, 1_600_000, 8, "uplink 500 kHz"),
            grid(923_300_000, 600_000, 8, "downlink"),
        ],
        defaults: &[],
        rx2: (923.3e6, 8),
        rates: RATES_US,
    },
    Plan {
        name: "AU915",
        band: mhz(915.0, 928.0),
        grids: &[
            grid(915_200_000, 200_000, 64, "uplink"),
            grid(915_900_000, 1_600_000, 8, "uplink 500 kHz"),
            grid(923_300_000, 600_000, 8, "downlink"),
        ],
        defaults: &[],
        rx2: (923.3e6, 8),
        rates: RATES_AU,
    },
    as923!("AS923-1", 0.0),
    as923!("AS923-2", -1.8),
    as923!("AS923-3", -6.6),
    as923!("AS923-4", -5.9),
    Plan {
        name: "KR920",
        band: mhz(920.9, 923.3),
        grids: &[],
        defaults: &[922.1e6, 922.3e6, 922.5e6],
        rx2: (921.9e6, 0),
        rates: RATES_KR,
    },
    Plan {
        name: "IN865",
        band: mhz(865.0, 867.0),
        grids: &[],
        defaults: &[865.0625e6, 865.4025e6, 865.985e6],
        rx2: (866.55e6, 2),
        rates: RATES_EU,
    },
    Plan {
        name: "RU864",
        band: mhz(864.0, 870.0),
        grids: &[],
        defaults: &[868.9e6, 869.1e6],
        rx2: (869.1e6, 0),
        rates: RATES_EU,
    },
    Plan {
        name: "CN470",
        band: mhz(470.0, 510.0),
        grids: &[grid(470_300_000, 200_000, 96, "uplink"), grid(500_300_000, 200_000, 48, "downlink")],
        defaults: &[],
        rx2: (505.3e6, 0),
        rates: RATES_CN470,
    },
    Plan {
        name: "CN779",
        band: mhz(779.0, 787.0),
        grids: &[],
        defaults: &[779.5e6, 779.7e6, 779.9e6],
        rx2: (786.0e6, 0),
        rates: RATES_EU,
    },
    Plan {
        name: "EU433",
        band: mhz(433.175, 434.665),
        grids: &[],
        defaults: &[433.175e6, 433.375e6, 433.575e6],
        rx2: (434.665e6, 0),
        rates: RATES_EU,
    },
];

/// A plan by name ("EU868", case ignored).
pub fn plan(name: &str) -> Option<&'static Plan> {
    PLANS.iter().find(|p| p.name.eq_ignore_ascii_case(name))
}

/// How far a frame's measured frequency may be from a channel's: a
/// device's crystal is good to some ±20 ppm (17 kHz at 868 MHz), and the
/// channels are at least 200 kHz apart.
fn tolerance_hz(hz: f64) -> f64 {
    (hz * 25e-6).max(10e3)
}

impl Plan {
    /// The data rate SF `sf` in `bw_hz` is in this plan, if any.
    pub fn data_rate(&self, sf: u8, bw_hz: f64) -> Option<u8> {
        self.rates.iter().find(|r| r.sf == sf && (r.bw_hz - bw_hz).abs() < 1.0).map(|r| r.dr)
    }

    /// Which of this plan's channels `hz` is ("uplink ch 12", "default ch 2",
    /// "RX2"), or "in band" for a channel the network may have added.
    pub fn channel(&self, hz: f64) -> Option<String> {
        let tol = tolerance_hz(hz);
        for g in self.grids {
            let n = ((hz - g.first_hz) / g.step_hz).round();
            if n >= 0.0 && n < g.count as f64 && (hz - g.channel_hz(n as u32)).abs() <= tol {
                return Some(format!("{} ch {}", g.what, n as u32));
            }
        }
        if let Some(i) = self.defaults.iter().position(|&c| (hz - c).abs() <= tol) {
            return Some(format!("default ch {}", i + 1));
        }
        if (hz - self.rx2.0).abs() <= tol {
            return Some("RX2".into());
        }
        // Grid plans have no channels off their grids.
        (self.grids.is_empty() && (self.band.0..=self.band.1).contains(&hz)).then(|| "in band".into())
    }

    /// Where its channels are, in a line or two: what to tune to.
    pub fn describe(&self) -> Vec<String> {
        let mut out = Vec::new();
        for g in self.grids {
            out.push(format!(
                "{}: {} channels, {:.3} to {:.3} MHz every {:.0} kHz",
                g.what,
                g.count,
                g.first_hz / 1e6,
                g.channel_hz(g.count - 1) / 1e6,
                g.step_hz / 1e3
            ));
        }
        if !self.defaults.is_empty() {
            let f: Vec<String> = self.defaults.iter().map(|f| format!("{:.4}", f / 1e6)).collect();
            out.push(format!("default channels: {} MHz", f.join(", ")));
            out.push(format!(
                "more channels set by the network: {:.3} to {:.3} MHz",
                self.band.0 / 1e6,
                self.band.1 / 1e6
            ));
        }
        let rx2 = self.rates.iter().find(|r| r.dr == self.rx2.1).expect("RX2 rate listed");
        out.push(format!(
            "RX2: {:.4} MHz DR{} (SF{} {:.0} kHz)",
            self.rx2.0 / 1e6,
            rx2.dr,
            rx2.sf,
            rx2.bw_hz / 1e3
        ));
        let rates: Vec<String> =
            self.rates.iter().map(|r| format!("DR{} SF{}/{:.0}", r.dr, r.sf, r.bw_hz / 1e3)).collect();
        out.push(format!("rates: {}", rates.join(", ")));
        out
    }
}

/// A frame placed in a LoRaWAN plan.
#[derive(Debug, Clone, PartialEq)]
pub struct Placed {
    pub plan: &'static str,
    /// "uplink ch 12", "default ch 2", "RX2", "in band".
    pub channel: String,
    pub dr: u8,
}

impl std::fmt::Display for Placed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} {} DR{}", self.plan, self.channel, self.dr)
    }
}

/// The LoRaWAN plans a frame at `hz`, SF `sf` in `bw_hz` fits, best first
/// (a named channel before "in band"). `only`: one plan's name.
pub fn identify(hz: f64, sf: u8, bw_hz: f64, only: Option<&str>) -> Vec<Placed> {
    let mut v: Vec<Placed> = PLANS
        .iter()
        .filter(|p| only.is_none_or(|o| p.name.eq_ignore_ascii_case(o)))
        .filter_map(|p| Some(Placed { plan: p.name, dr: p.data_rate(sf, bw_hz)?, channel: p.channel(hz)? }))
        .collect();
    v.sort_by_key(|p| p.channel == "in band");
    v
}

/// A Meshtastic region (`RDEF` in `RadioInterface.cpp`): its band, MHz.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MtRegion {
    pub name: &'static str,
    pub start_mhz: f64,
    pub end_mhz: f64,
    /// Channel spacing beyond the bandwidth, MHz (0 in every region today).
    pub spacing_mhz: f64,
}

const fn mt(name: &'static str, start_mhz: f64, end_mhz: f64) -> MtRegion {
    MtRegion { name, start_mhz, end_mhz, spacing_mhz: 0.0 }
}

/// Meshtastic's regions, in its firmware's order.
pub const MT_REGIONS: &[MtRegion] = &[
    mt("US", 902.0, 928.0),
    mt("EU_433", 433.0, 434.0),
    mt("EU_868", 869.4, 869.65),
    mt("CN", 470.0, 510.0),
    mt("JP", 920.5, 923.5),
    mt("ANZ", 915.0, 928.0),
    mt("ANZ_433", 433.05, 434.79),
    mt("RU", 868.7, 869.2),
    mt("KR", 920.0, 923.0),
    mt("TW", 920.0, 925.0),
    mt("IN", 865.0, 867.0),
    mt("NZ_865", 864.0, 868.0),
    mt("TH", 920.0, 925.0),
    mt("UA_433", 433.0, 434.7),
    mt("UA_868", 868.0, 868.6),
    mt("MY_433", 433.0, 435.0),
    mt("MY_919", 919.0, 924.0),
    mt("SG_923", 917.0, 925.0),
    mt("PH_433", 433.0, 434.7),
    mt("PH_868", 868.0, 869.4),
    mt("PH_915", 915.0, 918.0),
    mt("KZ_433", 433.075, 434.775),
    mt("KZ_863", 863.0, 868.0),
    mt("NP_865", 865.0, 868.0),
    mt("BR_902", 902.0, 907.5),
    mt("LORA_24", 2400.0, 2483.5),
];

/// A Meshtastic region by name ("EU_868", case ignored).
pub fn mt_region(name: &str) -> Option<&'static MtRegion> {
    MT_REGIONS.iter().find(|r| r.name.eq_ignore_ascii_case(name))
}

/// The firmware's channel-name hash (djb2).
pub fn djb2(s: &str) -> u32 {
    s.bytes().fold(5381u32, |h, c| h.wrapping_mul(33).wrapping_add(c as u32))
}

/// The name the firmware hashes for a preset's default channel
/// (`getModemPresetDisplayName`): "LongMod" for LongModerate, and
/// "Invalid" for VLongSlow, which that function no longer lists.
pub fn mt_hash_name(preset: &str) -> &str {
    match preset {
        "LongModerate" => "LongMod",
        "VLongSlow" => "Invalid",
        p => p,
    }
}

impl MtRegion {
    /// Channels of `bw_hz` the band holds.
    pub fn channels(&self, bw_hz: f64) -> u32 {
        ((self.end_mhz - self.start_mhz) / (self.spacing_mhz + bw_hz / 1e6)).floor() as u32
    }

    /// Channel `n` (0-based; the apps show n + 1), Hz.
    pub fn channel_hz(&self, n: u32, bw_hz: f64) -> f64 {
        (self.start_mhz + bw_hz / 2e6 + n as f64 * bw_hz / 1e6) * 1e6
    }

    /// Where a channel named `name` (a preset's default: "LongFast") is put
    /// when the channel number is left at 0: slot djb2(name) mod channels.
    pub fn default_slot(&self, name: &str, bw_hz: f64) -> Option<u32> {
        let n = self.channels(bw_hz);
        (n > 0).then(|| djb2(name) % n)
    }

    /// The slot `hz` is on, if it is one of this region's for `bw_hz`.
    pub fn slot(&self, hz: f64, bw_hz: f64) -> Option<u32> {
        let n = self.channels(bw_hz);
        let k = ((hz / 1e6 - self.start_mhz - bw_hz / 2e6) / (bw_hz / 1e6)).round();
        (k >= 0.0 && k < n as f64 && (hz - self.channel_hz(k as u32, bw_hz)).abs() <= tolerance_hz(hz))
            .then_some(k as u32)
    }
}

/// The Meshtastic regions and slots a frame at `hz` in `bw_hz` fits, with
/// whether the slot is `preset`'s default ("LongFast" → its own slot).
/// `only`: one region.
pub fn identify_mt(hz: f64, bw_hz: f64, preset: Option<&str>, only: Option<&str>) -> Vec<(&'static str, u32, bool)> {
    MT_REGIONS
        .iter()
        .filter(|r| only.is_none_or(|o| r.name.eq_ignore_ascii_case(o)))
        .filter_map(|r| {
            let k = r.slot(hz, bw_hz)?;
            let default = preset.and_then(|p| r.default_slot(mt_hash_name(p), bw_hz)) == Some(k);
            Some((r.name, k, default))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn meshtastic_default_frequencies() {
        // The frequencies the apps show for LongFast on its default channel.
        let us = mt_region("US").unwrap();
        let k = us.default_slot("LongFast", 250e3).unwrap();
        assert_eq!(k + 1, 20);
        assert!((us.channel_hz(k, 250e3) - 906.875e6).abs() < 1.0);
        let eu = mt_region("EU_868").unwrap();
        assert_eq!(eu.channels(250e3), 1);
        assert!((eu.channel_hz(0, 250e3) - 869.525e6).abs() < 1.0);
        let hits = identify_mt(906.875e6 + 4e3, 250e3, Some("LongFast"), None);
        assert!(hits.contains(&("US", 19, true)), "{hits:?}");
        assert!(identify_mt(869.525e6, 250e3, Some("LongFast"), Some("EU_868")) == vec![("EU_868", 0, true)]);
    }

    #[test]
    fn lorawan_channels_and_rates() {
        assert_eq!(
            identify(868.1e6 + 6e3, 12, 125e3, None)[0],
            Placed { plan: "EU868", channel: "default ch 1".into(), dr: 0 }
        );
        // US915 uplink 8 (903.9 MHz) at SF10: DR0, the slowest.
        let us = identify(903.9e6, 10, 125e3, Some("us915"));
        assert_eq!(us, vec![Placed { plan: "US915", channel: "uplink ch 8".into(), dr: 0 }]);
        // SF12 at 125 kHz is no US915 uplink rate.
        assert!(identify(903.9e6, 12, 125e3, Some("US915")).is_empty());
        // A 500 kHz downlink, and the 500 kHz uplinks.
        assert_eq!(identify(923.9e6, 12, 500e3, Some("US915"))[0].channel, "downlink ch 1");
        assert_eq!(identify(904.6e6, 8, 500e3, Some("US915"))[0].dr, 4);
        // Off the grid: not US915.
        assert!(identify(903.95e6, 10, 125e3, Some("US915")).is_empty());
        // AS923-2's first default is AS923-1's less 1.8 MHz.
        assert_eq!(identify(921.4e6, 7, 125e3, Some("AS923-2"))[0].channel, "default ch 1");
        assert_eq!(identify(869.525e6, 12, 125e3, Some("EU868"))[0].channel, "RX2");
        assert_eq!(identify(867.7e6, 9, 125e3, Some("EU868"))[0].channel, "in band");
        for p in PLANS {
            assert!(p.describe().len() >= 2);
            assert!(p.data_rate(p.rates[0].sf, p.rates[0].bw_hz).is_some());
        }
    }
}
