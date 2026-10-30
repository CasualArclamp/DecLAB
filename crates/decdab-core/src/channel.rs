// made by CasualArclamp
//! The Band III DAB block plan, for tuning by block name ("12B").
//!
//! EN 50248 / the CEPT Wiesbaden 1995 (WI95) and Maastricht 2002 plans:
//! four 1.712 MHz blocks (A–D) in each 7 MHz Band III TV channel 5–12, the
//! "N" blocks of the Irish/UK plan in 10–12, and six blocks in channel 13.
//! The UK uses 5A–13F; the national ensembles sit in 11A–12D and the
//! local and small-scale ones across the band.

/// One DAB block: its name and centre frequency in kHz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Block {
    pub name: &'static str,
    pub khz: u32,
}

impl Block {
    pub fn hz(self) -> f64 {
        self.khz as f64 * 1e3
    }
}

const fn b(name: &'static str, khz: u32) -> Block {
    Block { name, khz }
}

/// Every Band III block, lowest first (EN 50248 Table 1).
pub const BAND_III: [Block; 41] = [
    b("5A", 174_928),
    b("5B", 176_640),
    b("5C", 178_352),
    b("5D", 180_064),
    b("6A", 181_936),
    b("6B", 183_648),
    b("6C", 185_360),
    b("6D", 187_072),
    b("7A", 188_928),
    b("7B", 190_640),
    b("7C", 192_352),
    b("7D", 194_064),
    b("8A", 195_936),
    b("8B", 197_648),
    b("8C", 199_360),
    b("8D", 201_072),
    b("9A", 202_928),
    b("9B", 204_640),
    b("9C", 206_352),
    b("9D", 208_064),
    b("10A", 209_936),
    b("10N", 210_096),
    b("10B", 211_648),
    b("10C", 213_360),
    b("10D", 215_072),
    b("11A", 216_928),
    b("11N", 217_088),
    b("11B", 218_640),
    b("11C", 220_352),
    b("11D", 222_064),
    b("12A", 223_936),
    b("12N", 224_096),
    b("12B", 225_648),
    b("12C", 227_360),
    b("12D", 229_072),
    b("13A", 230_784),
    b("13B", 232_496),
    b("13C", 234_208),
    b("13D", 235_776),
    b("13E", 237_488),
    b("13F", 239_200),
];

/// The blocks a scan visits: all but the N blocks, which overlap their
/// neighbours and carry nothing in the UK.
pub fn scan_blocks() -> impl Iterator<Item = Block> {
    // `impl Iterator` in return position: the caller gets a lazy iterator
    // without the concrete adapter type being spelled out.
    BAND_III.iter().copied().filter(|b| !b.name.ends_with('N'))
}

/// Read a block as typed: "12B", "12b", " 12B ".
pub fn parse_block(s: &str) -> Option<Block> {
    let s = s.trim();
    BAND_III
        .iter()
        .copied()
        .find(|b| b.name.eq_ignore_ascii_case(s))
}

/// The block centred within 100 kHz of `hz`, if any (the N blocks lose to
/// the lettered ones, which are what is on air).
pub fn block_at(hz: f64) -> Option<Block> {
    scan_blocks()
        .chain(BAND_III.iter().copied())
        .find(|b| (b.hz() - hz).abs() <= 100e3)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_blocks() {
        assert_eq!(parse_block("12B").unwrap().khz, 225_648);
        assert_eq!(parse_block("11d").unwrap().khz, 222_064);
        assert_eq!(parse_block("5A").unwrap().khz, 174_928);
        assert!(parse_block("14A").is_none());
        assert_eq!(block_at(225.648e6).unwrap().name, "12B");
        assert_eq!(block_at(210.0e6).unwrap().name, "10A");
        assert!(block_at(100e6).is_none());
    }

    #[test]
    fn lettered_blocks_are_1712_khz_apart_within_a_channel() {
        for w in BAND_III.windows(2) {
            let (a, b) = (w[0], w[1]);
            if a.name.len() == b.name.len()
                && a.name[..a.name.len() - 1] == b.name[..b.name.len() - 1]
                && !a.name.ends_with('N')
                && !b.name.ends_with('N')
                && !a.name.starts_with("13")
            {
                assert_eq!(b.khz - a.khz, 1712, "{} {}", a.name, b.name);
            }
        }
        assert_eq!(scan_blocks().count(), 38);
    }
}
