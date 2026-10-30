// made by CasualArclamp
//! Programme-associated data (EN 300 401 §7.4): the bytes at the end of an
//! MP2 audio frame, or in the data_stream_element of a DAB+ access unit
//! (TS 102 563 §5.4), carrying dynamic labels (DLS, with DL Plus tags,
//! TS 102 980) and MOT objects (the slideshow, TS 101 499).
//!
//! **Layout.** The last two bytes are the F-PAD: byte L−1 has the F-PAD type
//! (2 bits, 00) and the X-PAD indicator (2 bits: none, short, variable);
//! byte L has the CI flag (bit 1). The X-PAD comes before the F-PAD in
//! reverse byte order, so it is read backwards from there. With the CI flag
//! set it opens with contents indicators: one byte for a short X-PAD (an
//! application type, then 3 data bytes), up to four for a variable one
//! (a length index, 3 bits, and an application type, 5 bits; type 0 ends
//! the list). Without it, the X-PAD continues the last data subfield and is
//! as long as the previous X-PAD.
//!
//! **Applications** (§7.4.3, Table 11): 1 data group length indicator,
//! 2/3 DLS start/continuation, 12/13 MOT start/continuation.

use crate::crc::crc16_check as crc16_ok;

use crate::charset;
use crate::mot::{MotDecoder, MotHeader, MotOutput};

/// Bytes of a variable X-PAD subfield, by length index (§7.4.2.2 Table 10).
const SUBFIELD_LEN: [usize; 8] = [4, 6, 8, 12, 16, 24, 32, 48];

/// What the PAD carried.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PadEvent {
    /// A complete dynamic label.
    Dls(DynamicLabel),
    /// The broadcaster cleared the label.
    DlsClear,
    /// A slideshow image (or another MOT object).
    Mot {
        transport_id: u16,
        name: String,
        mime: String,
        header: MotHeader,
        data: Vec<u8>,
    },
}

/// A dynamic label and what DL Plus said about it.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct DynamicLabel {
    pub text: String,
    /// DL Plus objects (TS 102 980 Annex A): content type and text.
    pub tags: Vec<(u8, String)>,
}

impl DynamicLabel {
    /// The DL Plus object of a content type (1 ITEM.TITLE, 4 ITEM.ARTIST, …).
    pub fn tag(&self, kind: u8) -> Option<&str> {
        self.tags
            .iter()
            .find(|(k, _)| *k == kind)
            .map(|(_, s)| s.as_str())
    }
}

/// DL Plus content type names (TS 102 980 Table A.1), those a UI shows.
pub fn dl_plus_name(kind: u8) -> &'static str {
    match kind {
        1 => "Title",
        2 => "Album",
        3 => "Track",
        4 => "Artist",
        5 => "Composition",
        6 => "Movement",
        7 => "Conductor",
        8 => "Composer",
        9 => "Band",
        10 => "Comment",
        11 => "Genre",
        12 => "News",
        13 => "Local news",
        31 => "Station",
        32 => "Programme",
        33 => "Now",
        34 => "Next",
        35 => "Part",
        36 => "Host",
        37 => "Editorial staff",
        39 => "Homepage",
        40 => "Studio phone",
        44 => "Hotline",
        48 => "SMS studio",
        _ => "",
    }
}

/// Statistics.
#[derive(Debug, Clone, Copy, Default)]
pub struct PadStats {
    pub pads: u64,
    pub dls_segments: u64,
    pub dls_crc_errors: u64,
    pub mot_groups: u64,
    pub mot_errors: u64,
    /// X-PAD subfields seen, by application type (§7.4.3 Table 11): what a
    /// service sends, whether or not it is decoded here.
    pub apps: [u64; 32],
}

#[derive(Default)]
struct Dls {
    toggle: Option<bool>,
    segments: Vec<Option<Vec<u8>>>,
    charset: u8,
    text: String,
    tags: Vec<(u8, u8, u8)>,
}

pub struct PadDecoder {
    /// The last data subfield's application type and the last X-PAD's
    /// length, for continuations.
    last_app: u8,
    last_len: usize,
    /// Data group being gathered for DLS (2/3) and MOT (12/13).
    dls_buf: Vec<u8>,
    mot_buf: Vec<u8>,
    /// From the last data group length indicator (app 1).
    mot_len: Option<usize>,
    dls: Dls,
    mot: Box<MotDecoder>,
    pub stats: PadStats,
}

impl Default for PadDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Bytes a DLS data group holds, from its prefix (§7.4.5.2).
fn dls_group_len(prefix: &[u8]) -> Option<usize> {
    let b0 = *prefix.first()?;
    let c = b0 & 0x10 != 0;
    let len = (b0 & 0x0F) as usize;
    Some(if !c {
        2 + len + 1 + 2
    } else if len == 0b0010 {
        // DL Plus command: its own length in byte 1.
        2 + (*prefix.get(1)? as usize & 0x0F) + 1 + 2
    } else {
        4
    })
}

impl PadDecoder {
    pub fn new() -> Self {
        PadDecoder {
            last_app: 0,
            last_len: 0,
            dls_buf: Vec::new(),
            mot_buf: Vec::new(),
            mot_len: None,
            dls: Dls::default(),
            mot: Box::new(MotDecoder::new()),
            stats: PadStats::default(),
        }
    }

    /// Take a PAD field ending in its F-PAD: a whole MP2 frame, or a DAB+
    /// AU's data_stream_element bytes.
    pub fn push(&mut self, field: &[u8], events: &mut Vec<PadEvent>) {
        let n = field.len();
        if n < 2 {
            return;
        }
        self.stats.pads += 1;
        let (l1, l) = (field[n - 2], field[n - 1]);
        if l1 >> 6 != 0 {
            return; // F-PAD type other than 00
        }
        let xpad_ind = (l1 >> 4) & 3;
        let ci = l & 0x02 != 0;
        // The X-PAD bytes in their logical order.
        let x = |i: usize| -> Option<u8> { (i + 2 < n).then(|| field[n - 3 - i]) };
        let mut subs: Vec<(u8, usize)> = Vec::new();
        let mut pos;
        match (xpad_ind, ci) {
            (0, _) => return,
            (1, true) => {
                let Some(app) = x(0) else { return };
                subs.push((app & 0x1F, 3));
                pos = 1;
            }
            (1, false) => {
                subs.push((self.continuation(), 4));
                pos = 0;
            }
            (2, true) => {
                pos = 0;
                for _ in 0..4 {
                    let Some(b) = x(pos) else { return };
                    pos += 1;
                    let app = b & 0x1F;
                    if app == 0 {
                        break;
                    }
                    subs.push((app, SUBFIELD_LEN[(b >> 5) as usize]));
                }
            }
            (2, false) => {
                if self.last_len == 0 {
                    return;
                }
                subs.push((self.continuation(), self.last_len));
                pos = 0;
            }
            _ => return,
        }
        let mut total = 0;
        for (app, len) in subs {
            let data: Vec<u8> = (pos..pos + len).map_while(x).collect();
            pos += len;
            total += len;
            self.subfield(app, &data, events);
            self.last_app = app;
        }
        if xpad_ind == 2 && ci {
            self.last_len = total;
        } else if xpad_ind == 1 {
            self.last_len = 0;
        }
    }

    /// The application type a continuation belongs to.
    fn continuation(&self) -> u8 {
        match self.last_app {
            2 => 3,
            12 => 13,
            a => a,
        }
    }

    fn subfield(&mut self, app: u8, data: &[u8], events: &mut Vec<PadEvent>) {
        self.stats.apps[(app & 31) as usize] += 1;
        match app {
            1 => {
                // Data group length indicator: rfa (2), length (14), CRC.
                if data.len() >= 4 && crc16_ok(&data[..4]) {
                    self.mot_len = Some(((data[0] as usize & 0x3F) << 8) | data[1] as usize);
                }
            }
            2 => {
                self.dls_buf.clear();
                self.dls_buf.extend_from_slice(data);
                self.try_dls(events);
            }
            3 if !self.dls_buf.is_empty() => {
                self.dls_buf.extend_from_slice(data);
                self.try_dls(events);
            }
            12 => {
                self.mot_buf.clear();
                self.mot_buf.extend_from_slice(data);
                self.try_mot(events);
            }
            13 if !self.mot_buf.is_empty() => {
                self.mot_buf.extend_from_slice(data);
                self.try_mot(events);
            }
            _ => {}
        }
    }

    fn try_mot(&mut self, events: &mut Vec<PadEvent>) {
        let Some(len) = self.mot_len else {
            return;
        };
        if self.mot_buf.len() < len {
            return;
        }
        let group: Vec<u8> = self.mot_buf.drain(..).take(len).collect();
        self.mot_len = None;
        self.stats.mot_groups += 1;
        let before = self.mot.stats().crc_errors + self.mot.stats().malformed;
        for out in self.mot.push_data_unit(&group) {
            if let MotOutput::Object(obj) = out {
                let data = obj.decompressed_body().unwrap_or_else(|_| obj.body.clone());
                events.push(PadEvent::Mot {
                    transport_id: obj.transport_id,
                    name: obj.name(),
                    mime: obj.mime(),
                    header: obj.header.clone(),
                    data,
                });
            }
        }
        if self.mot.stats().crc_errors + self.mot.stats().malformed > before {
            self.stats.mot_errors += 1;
        }
    }

    fn try_dls(&mut self, events: &mut Vec<PadEvent>) {
        let Some(len) = dls_group_len(&self.dls_buf) else {
            return;
        };
        if self.dls_buf.len() < len {
            return;
        }
        let g: Vec<u8> = self.dls_buf.drain(..).take(len).collect();
        self.dls_buf.clear();
        self.stats.dls_segments += 1;
        if !crc16_ok(&g) {
            self.stats.dls_crc_errors += 1;
            return;
        }
        let (b0, b1) = (g[0], g[1]);
        let toggle = b0 & 0x80 != 0;
        let first = b0 & 0x40 != 0;
        let last = b0 & 0x20 != 0;
        let command = b0 & 0x10 != 0;
        let body = &g[2..g.len() - 2];
        if command {
            match b0 & 0x0F {
                0b0001 => {
                    self.dls = Dls::default();
                    events.push(PadEvent::DlsClear);
                }
                0b0010 => self.dl_plus(body, events),
                _ => {}
            }
            return;
        }
        if self.dls.toggle != Some(toggle) {
            // A new label.
            self.dls.toggle = Some(toggle);
            self.dls.segments.clear();
        }
        let seg = if first {
            self.dls.charset = b1 >> 4;
            0
        } else {
            ((b1 >> 4) & 7) as usize
        };
        if self.dls.segments.len() <= seg {
            self.dls.segments.resize(seg + 1, None);
        }
        self.dls.segments[seg] = Some(body.to_vec());
        let complete = last || self.dls.segments.len() > 8;
        if last && self.dls.segments.iter().all(Option::is_some) && complete {
            let bytes: Vec<u8> = self
                .dls
                .segments
                .iter()
                .flat_map(|s| s.clone().unwrap_or_default())
                .collect();
            let text = charset::decode(self.dls.charset, &bytes);
            if text != self.dls.text {
                self.dls.text = text;
                events.push(PadEvent::Dls(self.label()));
            }
        }
    }

    /// DL Plus tags command (TS 102 980 §7.3): CId 0 (tags), IT, IR, NT,
    /// then NT + 1 tags of content type (7), start (7), length (6) bits.
    fn dl_plus(&mut self, body: &[u8], events: &mut Vec<PadEvent>) {
        let Some(&h) = body.first() else { return };
        if h >> 4 != 0 {
            return;
        }
        let n = (h & 3) as usize + 1;
        self.dls.tags.clear();
        for t in body[1..].as_chunks::<3>().0.iter().take(n) {
            self.dls.tags.push((t[0] & 0x7F, t[1] & 0x7F, t[2] & 0x7F));
        }
        if !self.dls.text.is_empty() {
            events.push(PadEvent::Dls(self.label()));
        }
    }

    fn label(&self) -> DynamicLabel {
        let chars: Vec<char> = self.dls.text.chars().collect();
        let tags = self
            .dls
            .tags
            .iter()
            .filter(|(k, _, _)| *k != 0)
            .filter_map(|&(k, start, len)| {
                let (a, b) = (start as usize, start as usize + len as usize + 1);
                (b <= chars.len()).then(|| (k, chars[a..b].iter().collect::<String>()))
            })
            .collect();
        DynamicLabel {
            text: self.dls.text.clone(),
            tags,
        }
    }

    /// The current label, if any.
    pub fn current_label(&self) -> Option<DynamicLabel> {
        (!self.dls.text.is_empty()).then(|| self.label())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::crc16 as crc16_ccitt;

    /// A DLS data group of one segment.
    fn dls_group(text: &str, toggle: bool) -> Vec<u8> {
        let b = text.as_bytes();
        let mut g = vec![
            (u8::from(toggle) << 7) | 0x60 | (b.len() as u8 - 1),
            0x00, // EBU Latin
        ];
        g.extend_from_slice(b);
        let c = crc16_ccitt(&g);
        g.extend_from_slice(&c.to_be_bytes());
        g
    }

    /// The PAD field that carries `xpad` (logical order) with CI.
    fn field(xpad: &[u8], variable: bool, ci: bool) -> Vec<u8> {
        let mut f: Vec<u8> = xpad.iter().rev().copied().collect();
        f.push(if variable { 0x20 } else { 0x10 });
        f.push(if ci { 0x02 } else { 0x00 });
        f
    }

    #[test]
    fn dls_in_variable_xpad() {
        let g = dls_group("Now: Song", false); // 2 + 9 + 2 = 13 bytes
        let mut dec = PadDecoder::new();
        let mut ev = Vec::new();
        // CI: 16 bytes (index 4) of app 2, end marker; then the group, padded.
        let mut x = vec![(4 << 5) | 2, 0];
        let mut sub = g.clone();
        sub.resize(16, 0);
        x.extend_from_slice(&sub);
        dec.push(&field(&x, true, true), &mut ev);
        assert_eq!(
            ev,
            vec![PadEvent::Dls(DynamicLabel {
                text: "Now: Song".into(),
                tags: vec![]
            })]
        );
    }

    /// A slideshow picture as a station sends it in the X-PAD (TS 101 499):
    /// each MOT data group announced by a data group length indicator (app 1)
    /// and carried in MOT start (12) and continuation (13) subfields of 48
    /// bytes, behind running DLS fields.
    #[test]
    fn slideshow_picture_out_of_variable_xpad() {
        use crate::mot::MotEncoder;
        let mut pic = vec![0xFF, 0xD8, 0xFF, 0xE0];
        pic.extend((0..600u32).map(|i| (i * 7 + 3) as u8));
        pic.extend([0xFF, 0xD9]);
        let mut enc = MotEncoder::header_mode();
        enc.set_segment_size(200).unwrap();
        enc.add_file("cover.jpg", pic.clone()).unwrap();
        let mut dec = PadDecoder::new();
        let mut ev = Vec::new();
        for _ in 0..20 {
            let Some(group) = enc.next_data_group() else { break };
            // The length indicator: rfa 00, 14-bit length, CRC.
            let mut dgli = vec![(group.len() >> 8) as u8 & 0x3F, group.len() as u8];
            let c = crc16_ccitt(&dgli);
            dgli.extend_from_slice(&c.to_be_bytes());
            for (i, chunk) in group.chunks(48).enumerate() {
                let mut x = Vec::new();
                let mut sub = chunk.to_vec();
                sub.resize(48, 0);
                if i == 0 {
                    // CI: 4 bytes of app 1, 48 bytes of app 12, end.
                    x.extend([1, (7 << 5) | 12, 0]);
                    x.extend_from_slice(&dgli);
                } else {
                    x.extend([(7 << 5) | 13, 0]);
                }
                x.extend_from_slice(&sub);
                dec.push(&field(&x, true, true), &mut ev);
            }
            // A label between the groups, as on air.
            let mut l = vec![(4 << 5) | 2, 0];
            let mut g = dls_group("Artist - Title", false);
            g.resize(16, 0);
            l.extend_from_slice(&g);
            dec.push(&field(&l, true, true), &mut ev);
            if ev.iter().any(|e| matches!(e, PadEvent::Mot { .. })) {
                break;
            }
        }
        let mot = ev.iter().find_map(|e| match e {
            PadEvent::Mot { name, mime, data, .. } => Some((name.clone(), mime.clone(), data.clone())),
            _ => None,
        });
        let (name, mime, data) = mot.expect("the picture");
        assert_eq!((name.as_str(), mime.as_str()), ("cover.jpg", "image/jpeg"));
        assert_eq!(data, pic);
        assert_eq!(dec.stats.mot_errors, 0);
        assert!(dec.stats.apps[1] > 0 && dec.stats.apps[12] > 0 && dec.stats.apps[13] > 0);
    }

    #[test]
    fn dls_split_over_short_xpads() {
        let g = dls_group("Hello", true); // 9 bytes
        let mut dec = PadDecoder::new();
        let mut ev = Vec::new();
        // Short X-PAD: CI + 3 bytes, then continuations of 4.
        let mut x = vec![2];
        x.extend_from_slice(&g[..3]);
        dec.push(&field(&x, false, true), &mut ev);
        dec.push(&field(&g[3..7], false, false), &mut ev);
        let mut tail = g[7..].to_vec();
        tail.resize(4, 0);
        dec.push(&field(&tail, false, false), &mut ev);
        assert_eq!(ev.len(), 1);
        assert!(matches!(&ev[0], PadEvent::Dls(l) if l.text == "Hello"));
    }
}
