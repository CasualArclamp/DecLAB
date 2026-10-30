// made by CasualArclamp
//! MPEG-1 and MPEG-2 (low sampling frequency) Audio Layer II, the codec of
//! DAB's original audio services (EN 300 401 §7; ISO/IEC 11172-3 §2.4,
//! ISO/IEC 13818-3).
//!
//! A frame holds 1152 samples a channel: a header, the bit allocation of
//! each sub-band, scale factor selection, scale factors (three per
//! sub-band, one for each 384-sample part), then 12 granules of three
//! samples per sub-band. The polyphase synthesis is the standard's own
//! (Figure 3-A.2): matrixing into V, windowing with D, summing.
//!
//! In DAB a 48 kHz frame fills one logical frame (24 ms) exactly and a
//! 24 kHz frame two; [`Mp2Decoder::push`] finds frames in the byte stream
//! either way.
//!
//! Optionally ([`Mp2Decoder::set_conceal`], off by default) the decoder
//! checks the header CRC and DAB's scale-factor CRCs and conceals what they
//! find damaged; see the notes on error detection further down.

use std::collections::VecDeque;
use std::sync::OnceLock;

use crate::mp2_window::SYNTHESIS_D;
use crate::{CodecError, Pcm};

/// Quantisation classes (Table 3-B.4): levels, grouped, bits per sample
/// or per group of three.
const CLASSES: [(u32, bool, u32); 17] = [
    (3, true, 5),
    (5, true, 7),
    (7, false, 3),
    (9, true, 10),
    (15, false, 4),
    (31, false, 5),
    (63, false, 6),
    (127, false, 7),
    (255, false, 8),
    (511, false, 9),
    (1023, false, 10),
    (2047, false, 11),
    (4095, false, 12),
    (8191, false, 13),
    (16383, false, 14),
    (32767, false, 15),
    (65535, false, 16),
];

/// The allocation rows of Tables 3-B.2a–d and 13818-3 B.1: nbal, and the
/// class of each allocation value (index 0 is "none").
const ROWS: [(u32, [u8; 16]); 8] = [
    (2, [0, 0, 1, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
    (2, [0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
    (3, [0, 0, 1, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0]),
    (3, [0, 0, 1, 2, 3, 4, 5, 16, 0, 0, 0, 0, 0, 0, 0, 0]),
    (4, [0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]),
    (4, [0, 0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    (4, [0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16]),
    (4, [0, 0, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]),
];

/// For each table, sblimit and the row of each sub-band.
const TABLES: [(usize, [u8; 30]); 5] = [
    // 3-B.2a
    (
        27,
        [
            7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0,
        ],
    ),
    // 3-B.2b
    (
        30,
        [
            7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0,
        ],
    ),
    // 3-B.2c
    (
        8,
        [
            5, 5, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ],
    ),
    // 3-B.2d
    (
        12,
        [
            5, 5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ],
    ),
    // 13818-3 B.1 (low sampling frequencies)
    (
        30,
        [
            4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        ],
    ),
];

const BITRATES_MPEG1: [u32; 15] = [
    0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
];
const BITRATES_MPEG2: [u32; 15] = [
    0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160,
];

/// A Layer II frame header (§2.4.2.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// MPEG-1 (else MPEG-2 low sampling frequency).
    pub mpeg1: bool,
    pub crc: bool,
    pub bitrate: u32,
    pub rate: u32,
    pub padding: bool,
    /// 0 stereo, 1 joint stereo, 2 dual channel, 3 mono.
    pub mode: u8,
    pub mode_ext: u8,
}

impl Header {
    pub fn parse(b: &[u8]) -> Option<Header> {
        if b.len() < 4 || b[0] != 0xFF || b[1] & 0xF0 != 0xF0 {
            return None;
        }
        let mpeg1 = b[1] & 0x08 != 0;
        if (b[1] >> 1) & 3 != 2 {
            return None; // not Layer II
        }
        let bi = (b[2] >> 4) as usize;
        let si = ((b[2] >> 2) & 3) as usize;
        if bi == 0 || bi == 15 || si == 3 {
            return None;
        }
        let bitrate = if mpeg1 {
            BITRATES_MPEG1[bi]
        } else {
            BITRATES_MPEG2[bi]
        };
        let rate = if mpeg1 {
            [44_100, 48_000, 32_000][si]
        } else {
            [22_050, 24_000, 16_000][si]
        };
        Some(Header {
            mpeg1,
            crc: b[1] & 1 == 0,
            bitrate,
            rate,
            padding: b[2] & 2 != 0,
            mode: b[3] >> 6,
            mode_ext: (b[3] >> 4) & 3,
        })
    }

    /// Bytes in the frame.
    pub fn frame_len(&self) -> usize {
        (144 * self.bitrate * 1000 / self.rate) as usize + self.padding as usize
    }

    pub fn channels(&self) -> usize {
        if self.mode == 3 { 1 } else { 2 }
    }

    /// Bytes of DAB's scale-factor CRC (EN 300 401 §7.3.2.2, Annex B.3):
    /// one CRC-8 for each group of sub-bands, four (sub-bands 0–3, 4–7, 8–15
    /// and 16 up) from 56 kbit/s a channel and for MPEG-2 LSF, two (0–3 and
    /// 4–7) below that, which is the 8- and 12-band tables. Checked on the
    /// air: LBC (24 kHz, 64 kbit/s mono) and the BBC's 48 kHz services send
    /// 4.
    pub fn scf_crc_len(&self) -> usize {
        if matches!(self.table(), 2 | 3) { 2 } else { 4 }
    }

    /// Which allocation table (ISO/IEC 11172-3 §2.4.3.3.1).
    fn table(&self) -> usize {
        if !self.mpeg1 {
            return 4;
        }
        let per_ch = self.bitrate / self.channels() as u32;
        if (self.rate == 48_000 && per_ch >= 56) || (56..=80).contains(&per_ch) {
            0
        } else if self.rate != 48_000 && per_ch >= 96 {
            1
        } else if self.rate != 32_000 && per_ch <= 48 {
            2
        } else {
            3
        }
    }
}

struct Bits<'a> {
    d: &'a [u8],
    pos: usize,
}

impl Bits<'_> {
    fn get(&mut self, n: u32) -> u32 {
        let mut v = 0;
        for _ in 0..n {
            let byte = self.d.get(self.pos / 8).copied().unwrap_or(0);
            v = (v << 1) | ((byte >> (7 - self.pos % 8)) & 1) as u32;
            self.pos += 1;
        }
        v
    }
}

/// The synthesis matrix N[i][k] = cos((16 + i)(2k + 1)π/64).
fn matrix() -> &'static [[f32; 32]; 64] {
    static N: OnceLock<Box<[[f32; 32]; 64]>> = OnceLock::new();
    N.get_or_init(|| {
        let mut n = Box::new([[0f32; 32]; 64]);
        for (i, row) in n.iter_mut().enumerate() {
            for (k, v) in row.iter_mut().enumerate() {
                *v = (((16 + i) * (2 * k + 1)) as f64 * std::f64::consts::PI / 64.0).cos() as f32;
            }
        }
        n
    })
}

/// One channel's synthesis filterbank state.
struct Synth {
    v: [f32; 1024],
    off: usize,
}

impl Synth {
    fn new() -> Self {
        Synth {
            v: [0.0; 1024],
            off: 0,
        }
    }

    /// 32 sub-band samples in, 32 PCM samples out.
    fn run(&mut self, s: &[f32; 32], out: &mut [f32; 32]) {
        let n = matrix();
        self.off = (self.off + 1024 - 64) % 1024;
        for (i, row) in n.iter().enumerate() {
            let mut acc = 0f32;
            for k in 0..32 {
                acc += row[k] * s[k];
            }
            self.v[self.off + i] = acc;
        }
        // U from V (Figure 3-A.2), windowed and summed in one pass.
        for (j, o) in out.iter_mut().enumerate() {
            let mut acc = 0f32;
            for i in 0..8 {
                let a = (self.off + 128 * i + j) % 1024;
                let b = (self.off + 128 * i + 96 + j) % 1024;
                acc += self.v[a] * SYNTHESIS_D[64 * i + j];
                acc += self.v[b] * SYNTHESIS_D[64 * i + 32 + j];
            }
            *o = acc;
        }
    }
}

// ---- error detection --------------------------------------------------
//
// DAB's Layer II frames carry two checks the standard decoder ignores:
//
// - the CRC-16 over the header's second half and the side information that
//   the bit allocation and SCFSI fields make up (EN 300 401 Annex B.2,
//   ISO/IEC 11172-3 §2.4.3.1): a failure means the parse is garbage;
// - the ScF-CRC (Annex B.3): one CRC-8 per group of sub-bands over the three
//   most significant bits of their scale factors, sent in the *previous*
//   frame (just before its F-PAD) so that the receiver knows a frame's
//   scale factors are damaged before it plays them.
//
// The sample bits themselves are not protected by anything: errors there
// cannot be found and are left alone.

/// CRC-16 table, generator x¹⁶ + x¹⁵ + x² + 1 (0x8005), MSB first: the
/// register's top byte XORed with the data byte picks the entry.
/// (`const fn` runs at compile time, so the table costs nothing at run time;
/// `while` is used because `for` loops are not allowed in `const fn` yet.)
const fn crc16_table() -> [u16; 256] {
    let mut t = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut r = (i as u16) << 8;
        let mut k = 0;
        while k < 8 {
            r = if r & 0x8000 != 0 { (r << 1) ^ 0x8005 } else { r << 1 };
            k += 1;
        }
        t[i] = r;
        i += 1;
    }
    t
}

static CRC16: [u16; 256] = crc16_table();

/// CRC-8, generator x⁸ + x⁴ + x³ + x² + 1 (0x1D), MSB first, for three
/// data bits at a time (the 3 MSBs of a scale factor): the register's top
/// three bits XORed with the data pick the entry, which is those bits
/// clocked out through the generator.
const fn crc8_table3() -> [u8; 8] {
    let mut t = [0u8; 8];
    let mut i = 0;
    while i < 8 {
        let mut r = (i as u8) << 5;
        let mut k = 0;
        while k < 3 {
            r = if r & 0x80 != 0 { (r << 1) ^ 0x1D } else { r << 1 };
            k += 1;
        }
        t[i] = r;
        i += 1;
    }
    t
}

static CRC8_3: [u8; 8] = crc8_table3();

/// Clock the three bits `v` (0..8) into a ScF-CRC register.
#[inline]
fn crc8_step3(reg: u8, v: u32) -> u8 {
    (reg << 3) ^ CRC8_3[((reg >> 5) as u32 ^ v) as usize & 7]
}

/// The header CRC (EN 300 401 Annex B.2, ISO/IEC 11172-3 §2.4.3.1): over
/// header bytes 2–3 (bit rate to emphasis) and every bit from the end of
/// the CRC field (bit 48) to `end_bit`, the end of the SCFSI field;
/// register initialised to 0xFFFF, not inverted. `None` if that span runs
/// off the frame.
fn header_crc(frame: &[u8], end_bit: usize) -> Option<u16> {
    if end_bit < 48 || end_bit > frame.len() * 8 {
        return None;
    }
    let mut crc = 0xFFFFu16;
    let byte = |crc: u16, b: u8| (crc << 8) ^ CRC16[((crc >> 8) as u8 ^ b) as usize];
    for &b in &frame[2..4] {
        crc = byte(crc, b);
    }
    let (whole, rest) = ((end_bit - 48) / 8, (end_bit - 48) % 8);
    for &b in &frame[6..6 + whole] {
        crc = byte(crc, b);
    }
    // The last few bits of the SCFSI field, one at a time.
    for k in 0..rest {
        let bit = (frame[6 + whole] >> (7 - k)) as u16 & 1;
        let top = (crc >> 15) ^ bit;
        crc <<= 1;
        if top != 0 {
            crc ^= 0x8005;
        }
    }
    Some(crc)
}

/// Whether the header CRC stored in bytes 4–5 holds for a span ending at
/// `end_bit`.
fn header_crc_ok(frame: &[u8], end_bit: usize) -> bool {
    header_crc(frame, end_bit) == Some(u16::from_be_bytes([frame[4], frame[5]]))
}

/// The ScF-CRC group a sub-band belongs to: 0–3, 4–7, 8–15, 16 and up, or
/// only the first two where DAB sends two CRCs (Annex B.3).
fn scf_group(sb: usize, groups: usize) -> Option<usize> {
    match sb {
        0..=3 => Some(0),
        4..=7 => Some(1),
        8..=15 if groups == 4 => Some(2),
        16.. if groups == 4 => Some(3),
        _ => None,
    }
}

/// The sub-bands of each ScF-CRC group.
const GROUP_BANDS: [std::ops::Range<usize>; 4] = [0..4, 4..8, 8..16, 16..32];

/// The ScF-CRCs sent for the frame that follows `frame`: the bytes before
/// its F-PAD, in reverse group order (CRC 3 first, CRC 0 last; Annex B.3
/// and EN 300 401 §7.3.2.8). `None` if the frame is too short.
fn scf_crcs_in_tail(frame: &[u8], groups: usize) -> Option<[u8; 4]> {
    let n = frame.len();
    if n < 2 + groups + 6 {
        return None;
    }
    let mut c = [0u8; 4];
    for (g, v) in c.iter_mut().enumerate().take(groups) {
        *v = frame[n - 3 - g];
    }
    Some(c)
}

// ---- the frame's layout ------------------------------------------------

/// What the header says about the frame's shape.
struct Shape {
    nch: usize,
    sblimit: usize,
    rows: [u8; 30],
    /// First sub-band whose samples both channels share (intensity stereo).
    bound: usize,
}

impl Shape {
    fn of(h: &Header) -> Shape {
        let nch = h.channels();
        let (sblimit, rows) = TABLES[h.table()];
        let bound = match h.mode {
            1 => (4 * (h.mode_ext as usize + 1)).min(sblimit),
            3 => 0,
            _ => sblimit,
        };
        Shape {
            nch,
            sblimit,
            rows,
            bound,
        }
    }
}

type Alloc = [[u8; 32]; 2];

/// Bit allocation, then SCFSI (§2.4.1.6, §2.4.3.3).
fn read_side(b: &mut Bits, s: &Shape) -> (Alloc, Alloc) {
    let mut alloc = [[0u8; 32]; 2];
    for sb in 0..s.sblimit {
        let (nbal, _) = ROWS[s.rows[sb] as usize];
        if sb < s.bound || s.nch == 1 {
            for a in alloc.iter_mut().take(s.nch) {
                a[sb] = b.get(nbal) as u8;
            }
        } else {
            let v = b.get(nbal) as u8;
            alloc[0][sb] = v;
            alloc[1][sb] = v;
        }
    }
    let mut scfsi = [[0u8; 32]; 2];
    for sb in 0..s.sblimit {
        for ch in 0..s.nch {
            if alloc[ch][sb] != 0 {
                scfsi[ch][sb] = b.get(2) as u8;
            }
        }
    }
    (alloc, scfsi)
}

/// The scale factor indices, three a sub-band (one for each part of the
/// frame), as the SCFSI says (§2.4.1.6): 0 sends three, 1 sends two (the
/// first is used twice), 2 one (used thrice), 3 two (the second is used
/// twice). With `groups` > 0 the ScF-CRC of each group is worked out as
/// well, into `crc`, from the 3 MSBs of the values actually sent, in stream
/// order (sub-band outer, channel inner).
fn read_scf(
    b: &mut Bits,
    s: &Shape,
    alloc: &Alloc,
    scfsi: &Alloc,
    groups: usize,
    crc: &mut [u8; 4],
) -> [[[u32; 3]; 32]; 2] {
    let mut out = [[[0u32; 3]; 32]; 2];
    for sb in 0..s.sblimit {
        let group = if groups > 0 { scf_group(sb, groups) } else { None };
        for ch in 0..s.nch {
            if alloc[ch][sb] == 0 {
                continue;
            }
            let mode = scfsi[ch][sb] as usize;
            let sent = [3, 2, 1, 2][mode];
            let mut v = [0u32; 3];
            for x in v.iter_mut().take(sent) {
                *x = b.get(6);
            }
            out[ch][sb] = match mode {
                0 => v,
                1 => [v[0], v[0], v[1]],
                2 => [v[0]; 3],
                _ => [v[0], v[1], v[1]],
            };
            if let Some(g) = group {
                for &x in &v[..sent] {
                    crc[g] = crc8_step3(crc[g], x >> 3);
                }
            }
        }
    }
    out
}

/// Table 3-B.1: 2^(1 − i/3); 63 is not allowed.
fn scf_gain(i: u32) -> f32 {
    if i < 63 {
        2f32.powf(1.0 - i as f32 / 3.0)
    } else {
        0.0
    }
}

/// The PAD field of a DAB MP2 frame as the PAD decoder wants it: the bytes
/// before the ScF-CRC (the X-PAD at their end), then the F-PAD. The
/// frame ends X-PAD, ScF-CRC, F-PAD (EN 300 401 Figure 6).
pub fn pad_field(frame: &[u8]) -> Option<Vec<u8>> {
    let h = Header::parse(frame)?;
    let n = frame.len();
    let crc = h.scf_crc_len();
    if n < 2 + crc + 8 {
        return None;
    }
    let mut f = frame[..n - 2 - crc].to_vec();
    f.extend_from_slice(&frame[n - 2..]);
    Some(f)
}

/// Sub-band slots of 32 samples a frame (12 granules of three).
const SLOTS: usize = 36;

/// After a concealed frame the next good one is faded in over this many
/// slots (6 × 32 samples: 4 ms at 48 kHz).
const FADE_IN_SLOTS: usize = 6;

/// What error concealment remembers from the last good frame. Only touched
/// while [`Mp2Decoder::set_conceal`] is on.
struct Memory {
    /// The scale factors the last good frame played with (zero for
    /// sub-bands it did not allocate), by channel and sub-band.
    scf: [[[f32; 3]; 32]; 2],
    /// Its dequantised sub-band samples, before synthesis (and before any
    /// fade-in), by channel and slot.
    sub: [[[f32; 32]; SLOTS]; 2],
    nch: usize,
    /// Its sampling rate; 0 until a frame has passed its header CRC.
    rate: u32,
    /// `sub` has not been repeated yet: the next concealed frame fades it
    /// out, any after that is silent.
    repeat: bool,
    /// The ScF-CRCs the previous frame carried for this one, and how many;
    /// `None` when there is nothing to check against (first frame, frame
    /// after a header failure or after lost sync).
    tail: Option<([u8; 4], usize)>,
    /// The next good frame follows a concealed one: fade it in.
    fade_in: bool,
}

impl Memory {
    fn new() -> Self {
        Memory {
            scf: [[[0.0; 3]; 32]; 2],
            sub: [[[0.0; 32]; SLOTS]; 2],
            nch: 1,
            rate: 0,
            repeat: false,
            tail: None,
            fade_in: false,
        }
    }
}

/// One slot of 32 sub-band samples a channel in, 32 stereo PCM frames out
/// (a mono channel is sent to both sides). An associated function taking
/// the synthesis state rather than `&mut self`, so the caller can keep other
/// fields of the decoder borrowed meanwhile.
fn run_slot(synth: &mut [Synth; 2], nch: usize, x: [&[f32; 32]; 2], out: &mut Vec<i16>) {
    let mut p = [[0f32; 32]; 2];
    for ch in 0..nch {
        synth[ch].run(x[ch], &mut p[ch]);
    }
    for (j, &l) in p[0].iter().enumerate() {
        let r = if nch == 2 { p[1][j] } else { l };
        out.push((l * 32767.0).clamp(-32768.0, 32767.0) as i16);
        out.push((r * 32767.0).clamp(-32768.0, 32767.0) as i16);
    }
}

pub struct Mp2Decoder {
    synth: [Synth; 2],
    buf: Vec<u8>,
    pub frames: u64,
    pub bad: u64,
    /// Frames whose header CRC failed (counted with the switch on only).
    pub hdr_crc_fail: u64,
    /// ScF-CRC groups that failed (one frame can fail several).
    pub scf_crc_fail: u64,
    /// Frames the decoder changed: a header failure repeated and faded, or
    /// scale factors replaced (these frames come out with `Pcm::concealed`).
    pub concealed: u64,
    conceal: bool,
    /// For each frame `push` has found and `decode` has not yet seen: were
    /// bytes skipped before it? Then it is not the frame the previous one's
    /// ScF-CRC was sent for. (`decode` takes the frames in the order `push`
    /// gave them; frames decoded without `push` have no entry.)
    gaps: VecDeque<bool>,
    /// The byte stream is at a frame boundary (else `push` is hunting).
    in_sync: bool,
    mem: Box<Memory>,
}

impl Default for Mp2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Mp2Decoder {
    pub fn new() -> Self {
        Mp2Decoder {
            synth: [Synth::new(), Synth::new()],
            buf: Vec::new(),
            frames: 0,
            bad: 0,
            hdr_crc_fail: 0,
            scf_crc_fail: 0,
            concealed: 0,
            conceal: false,
            gaps: VecDeque::new(),
            in_sync: true,
            mem: Box::new(Memory::new()),
        }
    }

    /// Error concealment on or off (off at first, and then the decoder is
    /// exactly the plain Layer II decoder: no CRC is looked at).
    ///
    /// On, a frame is acted on only when a check *detects* damage:
    /// - header CRC fails: the allocation cannot be trusted, so the last
    ///   good frame's sub-band samples are repeated, fading out, (silence
    ///   if there are several in a row) and the next good frame fades in;
    /// - ScF-CRC fails for a group of sub-bands: their scale factors are
    ///   replaced with the previous frame's (zero where the sub-band had no
    ///   bits), which is what makes damaged scale factors "boil";
    /// - everything else, the unprotected sample bits included, is played
    ///   as received, and a frame that passes is decoded bit for bit as
    ///   with the switch off.
    pub fn set_conceal(&mut self, on: bool) {
        if on != self.conceal {
            self.conceal = on;
            *self.mem = Memory::new();
        }
    }

    pub fn conceal(&self) -> bool {
        self.conceal
    }

    /// Take stream bytes; `on_frame` gets each whole frame found (its bytes,
    /// header included), for PAD extraction and decoding.
    pub fn push(&mut self, bytes: &[u8], mut on_frame: impl FnMut(&[u8])) {
        self.buf.extend_from_slice(bytes);
        let mut i = 0;
        while i + 4 <= self.buf.len() {
            let Some(h) = Header::parse(&self.buf[i..]) else {
                i += 1;
                self.in_sync = false;
                continue;
            };
            let len = h.frame_len();
            if i + len > self.buf.len() {
                break;
            }
            // Keep the queue short if nobody decodes what is found.
            if self.gaps.len() >= 16 {
                self.gaps.pop_front();
            }
            self.gaps.push_back(!self.in_sync);
            self.in_sync = true;
            on_frame(&self.buf[i..i + len]);
            i += len;
        }
        self.buf.drain(..i);
    }

    /// Decode one frame to 1152 samples a channel, always stereo out.
    pub fn decode(&mut self, frame: &[u8]) -> Result<Pcm, CodecError> {
        let resync = self.gaps.pop_front().unwrap_or(false);
        let h = Header::parse(frame).ok_or(CodecError::Bad("not a Layer II frame"))?;
        if frame.len() < h.frame_len() {
            return Err(CodecError::Bad("short frame"));
        }
        let s = Shape::of(&h);
        let nch = s.nch;
        let conceal = self.conceal;
        let mut b = Bits {
            d: frame,
            pos: 32 + if h.crc { 16 } else { 0 },
        };
        let (alloc, scfsi) = read_side(&mut b, &s);
        if conceal && h.crc && !header_crc_ok(frame, b.pos) {
            self.hdr_crc_fail += 1;
            return self.conceal_frame();
        }
        // Scale factors, and their CRCs if they will be checked.
        let groups = if conceal { h.scf_crc_len() } else { 0 };
        let mut crc = [0u8; 4];
        let idx = read_scf(&mut b, &s, &alloc, &scfsi, groups, &mut crc);
        let mut scf = [[[0f32; 3]; 32]; 2];
        for ch in 0..nch {
            for sb in 0..s.sblimit {
                // A sub-band with no bits has no scale factors (they stay 0).
                if alloc[ch][sb] != 0 {
                    for p in 0..3 {
                        scf[ch][sb][p] = scf_gain(idx[ch][sb][p]);
                    }
                }
            }
        }
        let mut repaired = false;
        if conceal {
            let m = &mut *self.mem;
            if !resync && let Some((sent, n)) = m.tail.filter(|&(_, n)| n == groups) {
                for g in 0..n {
                    if crc[g] == sent[g] {
                        continue;
                    }
                    self.scf_crc_fail += 1;
                    repaired = true;
                    for sb in GROUP_BANDS[g].clone().filter(|&sb| sb < s.sblimit) {
                        for ch in 0..nch {
                            // Only sub-bands with bits now: zero stays zero.
                            if alloc[ch][sb] != 0 {
                                scf[ch][sb] = m.scf[ch][sb];
                            }
                        }
                    }
                }
            }
            m.scf = scf;
            m.tail = scf_crcs_in_tail(&frame[..h.frame_len()], groups).map(|c| (c, groups));
        }
        // Samples: 12 granules of three per sub-band, then synthesis.
        let mut out = Vec::with_capacity(1152 * 2);
        for gr in 0..12 {
            let part = gr / 4;
            let mut sub = [[[0f32; 32]; 3]; 2];
            for sb in 0..s.sblimit {
                let row = ROWS[s.rows[sb] as usize].1;
                let chans = if sb < s.bound || nch == 1 { nch } else { 1 };
                for ch in 0..chans {
                    let a = alloc[ch][sb] as usize;
                    if a == 0 {
                        continue;
                    }
                    let (levels, grouped, bits) = CLASSES[row[a] as usize];
                    let mut codes = [0u32; 3];
                    if grouped {
                        let mut c = b.get(bits);
                        for v in &mut codes {
                            *v = c % levels;
                            c /= levels;
                        }
                    } else {
                        for v in &mut codes {
                            *v = b.get(bits);
                        }
                    }
                    // §2.4.3.3.5: the requantised value works out to
                    // (2c − (L − 1)) / L for every class.
                    let fl = levels as f32;
                    for (s, &c) in codes.iter().enumerate() {
                        let v = (2.0 * c as f32 - (fl - 1.0)) / fl;
                        sub[ch][s][sb] = v * scf[ch][sb][part];
                        if chans == 1 && nch == 2 {
                            sub[1][s][sb] = v * scf[1][sb][part];
                        }
                    }
                }
            }
            if conceal {
                let m = &mut *self.mem;
                for (mem, chan) in m.sub.iter_mut().zip(&sub).take(nch) {
                    mem[gr * 3..gr * 3 + 3].copy_from_slice(chan);
                }
                if m.fade_in && gr * 3 < FADE_IN_SLOTS {
                    for chan in sub.iter_mut().take(nch) {
                        for (s, x) in chan.iter_mut().enumerate() {
                            let slot = gr * 3 + s;
                            if slot < FADE_IN_SLOTS {
                                let gain = (slot + 1) as f32 / (FADE_IN_SLOTS + 1) as f32;
                                x.iter_mut().for_each(|v| *v *= gain);
                            }
                        }
                    }
                }
            }
            for (l, r) in sub[0].iter().zip(&sub[1]) {
                run_slot(&mut self.synth, nch, [l, r], &mut out);
            }
        }
        if conceal {
            let m = &mut *self.mem;
            (m.nch, m.rate, m.repeat, m.fade_in) = (nch, h.rate, true, false);
            self.concealed += repaired as u64;
        }
        self.frames += 1;
        Ok(Pcm {
            rate: h.rate,
            channels: 2,
            samples: out,
            concealed: repaired,
        })
    }

    /// A frame whose header CRC failed: its allocation, and so everything
    /// after it, cannot be trusted. Its place is filled from the last good
    /// frame's sub-band samples, faded out across the frame (silence when
    /// that has been used already) through the synthesis filter, so that it
    /// carries on from the sound before without a click and its state stays
    /// as if a frame had been decoded; the next good frame is faded in.
    /// Before any good frame there is nothing to repeat, nor a trusted
    /// sampling rate to play at: that is an error, as for a bad header.
    fn conceal_frame(&mut self) -> Result<Pcm, CodecError> {
        let m = &mut *self.mem;
        m.tail = None;
        m.fade_in = true;
        if m.rate == 0 {
            self.bad += 1;
            return Err(CodecError::Bad("header CRC failed"));
        }
        let nch = m.nch;
        let mut out = Vec::with_capacity(1152 * 2);
        for slot in 0..SLOTS {
            let mut x = [[0f32; 32]; 2];
            if m.repeat {
                let gain = 1.0 - slot as f32 / SLOTS as f32;
                for (xc, mem) in x.iter_mut().zip(&m.sub).take(nch) {
                    for (d, &v) in xc.iter_mut().zip(&mem[slot]) {
                        *d = v * gain;
                    }
                }
            }
            run_slot(&mut self.synth, nch, [&x[0], &x[1]], &mut out);
        }
        m.repeat = false;
        self.frames += 1;
        self.concealed += 1;
        Ok(Pcm {
            rate: m.rate,
            channels: 2,
            samples: out,
            concealed: true,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dab_headers() {
        // MPEG-1 Layer II, no CRC... 128 kbit/s, 48 kHz, joint stereo.
        let h = Header::parse(&[0xFF, 0xFD, 0x84, 0x40]).unwrap();
        assert!(h.mpeg1 && !h.crc);
        assert_eq!((h.bitrate, h.rate, h.mode), (128, 48_000, 1));
        assert_eq!(h.frame_len(), 384); // one 24 ms logical frame
        assert_eq!(h.table(), 0);
        // MPEG-2 LSF, 64 kbit/s, 24 kHz, mono: two logical frames.
        let h = Header::parse(&[0xFF, 0xF5, 0x84, 0xC0]).unwrap();
        assert!(!h.mpeg1);
        assert_eq!((h.bitrate, h.rate, h.channels()), (64, 24_000, 1));
        assert_eq!(h.frame_len(), 384);
        assert_eq!(h.table(), 4);
    }

    #[test]
    fn silence_decodes_to_silence() {
        // A frame with no bits allocated anywhere.
        let mut f = vec![0u8; 384];
        f[..4].copy_from_slice(&[0xFF, 0xFD, 0x84, 0x40]);
        let pcm = Mp2Decoder::new().decode(&f).unwrap();
        assert_eq!(pcm.samples.len(), 2304);
        assert!(pcm.samples.iter().all(|&s| s == 0));
    }

    // ---- error detection and concealment -------------------------------
    //
    // Two consecutive frames of Virgin Radio, as received (MPEG-1, 48 kHz,
    // 80 kbit/s mono, 27-band table, CRC present): the ScF-CRC of B is in
    // the tail of A. Longer streams are made from them by `stream`, which
    // sets every ScF-CRC and header CRC right, so a clean stream passes.
    // Stereo, joint stereo, the 8/12-band tables and MPEG-2 LSF have no
    // real frame here: their ScF-CRC ordering is not tested on air.
    const FRAME_A: &str = "fffc54cc117163344333211249000000009aaea8e8834c30e3113d03904935925964d442e6df5abb35319aec917a566f35ce2ad9bf75c8759672c07c96258c68ec8080ecaae4feab93cb7414cb456e374f0c5b680c5cd19d95cdb41dd249172486bab4b3e43a8aac69b0e82e6d2aa2921b18aca8ee9824b79a8673dc509ab13d1e2e7312544b99e4bba4996df229a8a2b0e235b972b1eb2a1e18527864e2db8a43945ad5b6efbe0aa4af652300938234861e35866ba6a59352d957ccc96eb7222e5b103193bb41d45e9ab4085a2c5b2db70654da46ac4a25e66650dd585c88976259c1c7513e203a676e004d6dd31000";
    const FRAME_B: &str = "fffc54cc8a2e6334423313124a00000000aaeaaba8744c38d4113d049051751458f6152e9aaa65ce469516d44e8e93b5c6590d918288a813117f66a451b6198b11ab8da0690a66895dc6432378521caed6b50ad5ce70632e1ca9f1256e2ad8a16566ba8163282a4ab1ab611b31c0891c6b64f87df964084a278056e7252895cc56736f5a4366f3b0cb21598b86b8cccd788d1d352a4e9e9c1eb14632d91514b8313df77515d066ba42aba34333a64c3471b71daa7d19a33a4a2952de0154ea8d2e987ab60e941a380a718cdf2f55a06cb31a1ad749b216900bb62191399dd451c8d8a3490242eb6e694b004f6ee61000";
    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len() / 2)
            .map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap())
            .collect()
    }

    fn flip(f: &mut [u8], bit: usize) {
        f[bit / 8] ^= 0x80 >> (bit % 8);
    }

    /// Where the parts of a frame lie, in bits.
    struct Layout {
        h: Header,
        /// End of the SCFSI field: the header CRC's span ends here.
        end_side: usize,
        /// Each transmitted scale factor group: sub-band, channel, start
        /// bit, and the number of 6-bit values sent.
        scf: Vec<(usize, usize, usize, usize)>,
        end_scf: usize,
    }

    fn layout(f: &[u8]) -> Layout {
        let h = Header::parse(f).unwrap();
        let s = Shape::of(&h);
        let mut b = Bits { d: f, pos: 48 };
        let (alloc, scfsi) = read_side(&mut b, &s);
        let end_side = b.pos;
        let mut scf = Vec::new();
        for sb in 0..s.sblimit {
            for ch in 0..s.nch {
                if alloc[ch][sb] != 0 {
                    let sent = [3, 2, 1, 2][scfsi[ch][sb] as usize];
                    scf.push((sb, ch, b.pos, sent));
                    b.pos += 6 * sent;
                }
            }
        }
        Layout {
            h,
            end_side,
            scf,
            end_scf: b.pos,
        }
    }

    /// The ScF-CRCs a frame's own scale factors give.
    fn scf_crcs(f: &[u8]) -> [u8; 4] {
        let h = Header::parse(f).unwrap();
        let s = Shape::of(&h);
        let mut b = Bits { d: f, pos: 48 };
        let (alloc, scfsi) = read_side(&mut b, &s);
        let mut crc = [0u8; 4];
        read_scf(&mut b, &s, &alloc, &scfsi, h.scf_crc_len(), &mut crc);
        crc
    }

    /// A stream whose frames carry the right header CRC, and the right
    /// ScF-CRC for the frame after.
    fn stream(frames: &[Vec<u8>]) -> Vec<Vec<u8>> {
        let mut v = frames.to_vec();
        for f in &mut v {
            let end = layout(f).end_side;
            let c = header_crc(f, end).unwrap().to_be_bytes();
            f[4..6].copy_from_slice(&c);
        }
        for i in 1..v.len() {
            let c = scf_crcs(&v[i]);
            let n = v[i - 1].len();
            for g in 0..v[i - 1].len().min(4) {
                v[i - 1][n - 3 - g] = c[g];
            }
        }
        v
    }

    fn ab(n: usize) -> Vec<Vec<u8>> {
        let (a, b) = (unhex(FRAME_A), unhex(FRAME_B));
        stream(&(0..n).map(|i| if i % 2 == 0 { a.clone() } else { b.clone() }).collect::<Vec<_>>())
    }

    fn run(d: &mut Mp2Decoder, frames: &[Vec<u8>]) -> Vec<Pcm> {
        frames.iter().filter_map(|f| d.decode(f).ok()).collect()
    }

    fn conceal_on() -> Mp2Decoder {
        let mut d = Mp2Decoder::new();
        d.set_conceal(true);
        d
    }

    #[test]
    fn crc8_table_matches_the_iso_routine() {
        // ISO/IEC 11172-3 Annex B's bit-at-a-time routine, polynomial 0x1D.
        fn iso(mut reg: u8, bits: u32, n: u32) -> u8 {
            for k in (0..n).rev() {
                let carry = reg & 0x80;
                reg <<= 1;
                if (carry == 0) != ((bits >> k) & 1 == 0) {
                    reg ^= 0x1D;
                }
            }
            reg
        }
        for reg in 0..=255u8 {
            for v in 0..8 {
                assert_eq!(crc8_step3(reg, v), iso(reg, v, 3), "reg {reg:02x} v {v}");
            }
        }
    }

    #[test]
    fn real_frames_pass_both_checks() {
        let (a, b) = (unhex(FRAME_A), unhex(FRAME_B));
        let (la, lb) = (layout(&a), layout(&b));
        for (f, l) in [(&a, &la), (&b, &lb)] {
            assert_eq!(f.len(), 240);
            assert!(l.h.crc && l.h.mpeg1 && l.h.channels() == 1);
            assert_eq!((l.h.bitrate, l.h.rate, l.h.scf_crc_len()), (80, 48_000, 4));
            assert!(header_crc_ok(f, l.end_side));
        }
        // The ScF-CRC of B is in A's tail, group 3 first.
        assert_eq!(scf_crcs(&b), scf_crcs_in_tail(&a, 4).unwrap());
        assert_eq!(a[a.len() - 6], scf_crcs(&b)[3]);
        assert_eq!(a[a.len() - 3], scf_crcs(&b)[0]);
        // Fed to the decoder in order, nothing is found wrong.
        let mut d = conceal_on();
        let out = run(&mut d, &[a, b]);
        assert_eq!(out.len(), 2);
        assert_eq!((d.hdr_crc_fail, d.scf_crc_fail, d.concealed), (0, 0, 0));
        assert!(out.iter().all(|p| !p.concealed));
    }

    #[test]
    fn header_crc_sees_every_flipped_bit_of_its_span() {
        let b = unhex(FRAME_B);
        let l = layout(&b);
        let mut checked = 0;
        // Header bytes 2–3 and the CRC itself, then allocation and SCFSI.
        for bit in (16..l.end_side).filter(|&k| !(32..48).contains(&k)).chain(32..48) {
            let mut f = b.clone();
            flip(&mut f, bit);
            assert!(!header_crc_ok(&f, l.end_side), "bit {bit}");
            checked += 1;
        }
        assert!(checked > 100);
        // Past the span (scale factors, samples) it has nothing to say.
        for bit in [l.end_side, l.end_scf, l.end_scf + 40, 239 * 8] {
            let mut f = b.clone();
            flip(&mut f, bit);
            assert!(header_crc_ok(&f, l.end_side), "bit {bit}");
        }
    }

    #[test]
    fn scf_crc_fails_only_for_the_damaged_group() {
        let b = unhex(FRAME_B);
        let clean = scf_crcs(&b);
        let l = layout(&b);
        let mut groups_hit = [false; 4];
        for &(sb, _, bit, sent) in &l.scf {
            let g = scf_group(sb, 4).unwrap();
            // Any of the 3 MSBs of any value sent; the 3 LSBs are not covered.
            for v in 0..sent {
                for k in 0..6 {
                    let mut f = b.clone();
                    flip(&mut f, bit + 6 * v + k);
                    let c = scf_crcs(&f);
                    for h in 0..4 {
                        assert_eq!(c[h] != clean[h], h == g && k < 3, "sb {sb} v {v} k {k} group {h}");
                    }
                }
            }
            groups_hit[g] = true;
        }
        assert!(groups_hit[..3].iter().all(|&x| x), "{groups_hit:?}");
    }

    /// FNV-1a over the samples.
    fn hash(h: &mut u64, p: &Pcm) {
        for s in &p.samples {
            *h = (*h ^ *s as u16 as u64).wrapping_mul(0x100000001b3);
        }
    }

    #[test]
    fn switch_off_is_the_plain_decoder() {
        // 400 frames of A and B with up to 18 bits wrong anywhere (so also
        // in the header, allocation and CRC): hashed on the decoder as it
        // was before the checks were added, 365 of them decodable. With
        // the switch off the output must stay exactly that.
        let (a, b) = (unhex(FRAME_A), unhex(FRAME_B));
        let mut d = Mp2Decoder::new();
        let (mut h, mut x, mut n) = (0xcbf29ce484222325u64, 0x9E3779B97F4A7C15u64, 0);
        for i in 0..400 {
            let mut f = if i % 2 == 0 { a.clone() } else { b.clone() };
            if i >= 2 {
                for _ in 0..(i % 7) * 3 {
                    x ^= x << 13;
                    x ^= x >> 7;
                    x ^= x << 17;
                    let bit = (x % (f.len() as u64 * 8)) as usize;
                    flip(&mut f, bit);
                }
            }
            if let Ok(p) = d.decode(&f) {
                n += 1;
                assert!(!p.concealed);
                hash(&mut h, &p);
            }
        }
        assert_eq!((n, h), (365, 0x8e127375c692be22));
        assert_eq!((d.hdr_crc_fail, d.scf_crc_fail, d.concealed), (0, 0, 0));
    }

    #[test]
    fn switch_on_changes_nothing_in_clean_frames() {
        let s = ab(40);
        let off = run(&mut Mp2Decoder::new(), &s);
        let mut d = conceal_on();
        let on = run(&mut d, &s);
        assert_eq!(off.len(), 40);
        for (x, y) in off.iter().zip(&on) {
            assert_eq!(x.samples, y.samples);
            assert!(!y.concealed);
        }
        assert_eq!((d.hdr_crc_fail, d.scf_crc_fail, d.concealed), (0, 0, 0));
    }

    #[test]
    fn errors_in_the_sample_bits_are_left_alone() {
        // Nothing detects them, so nothing is done: same output as off.
        let mut s = ab(30);
        let end = layout(&s[0]).end_scf;
        let mut x = 12345u64;
        for f in s.iter_mut().skip(1) {
            for _ in 0..25 {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                // Samples run from the scale factors to the PAD.
                flip(f, end + (x % (200 * 8 - end as u64)) as usize);
            }
        }
        let off = run(&mut Mp2Decoder::new(), &s);
        let mut d = conceal_on();
        let on = run(&mut d, &s);
        assert_eq!(off.len(), on.len());
        for (x, y) in off.iter().zip(&on) {
            assert_eq!(x.samples, y.samples);
        }
        assert_eq!((d.hdr_crc_fail, d.scf_crc_fail, d.concealed), (0, 0, 0));
    }

    #[test]
    fn damaged_scale_factors_are_replaced_in_their_group_only() {
        let clean = ab(6);
        let target = 3; // a B frame, after an A
        let l = layout(&clean[target]);
        let mut tested = 0;
        for g in 0..4 {
            let Some(&(_, _, bit, _)) = l.scf.iter().find(|(sb, ..)| scf_group(*sb, 4) == Some(g)) else {
                continue;
            };
            let mut bad = clean.clone();
            flip(&mut bad[target], bit); // the MSB of the group's first value
            // A reference decoder on the clean stream, and one on the damaged.
            let (mut x, mut y) = (conceal_on(), conceal_on());
            for f in &clean[..target] {
                x.decode(f).unwrap();
                y.decode(f).unwrap();
            }
            let prev_scf = x.mem.scf;
            let px = x.decode(&clean[target]).unwrap();
            let py = y.decode(&bad[target]).unwrap();
            assert!(!px.concealed && py.concealed, "group {g}");
            assert_eq!((y.hdr_crc_fail, y.scf_crc_fail, y.concealed), (0, 1, 1), "group {g}");
            assert_eq!((x.scf_crc_fail, x.concealed), (0, 0));
            let mut differs = false;
            for (sb, prev) in prev_scf[0].iter().enumerate() {
                let in_group = scf_group(sb, 4) == Some(g);
                // The scale factors played: the previous frame's in the
                // group (where bits were sent now), the frame's own
                // elsewhere.
                let own = x.mem.scf[0][sb];
                let want = if !in_group || own == [0.0; 3] { own } else { *prev };
                assert_eq!(y.mem.scf[0][sb], want, "group {g} sb {sb}");
                let sub_same = (0..SLOTS).all(|t| x.mem.sub[0][t][sb] == y.mem.sub[0][t][sb]);
                if in_group {
                    differs |= !sub_same;
                } else {
                    assert!(sub_same, "group {g}: sub-band {sb} changed");
                }
            }
            assert!(differs, "group {g}: nothing replaced, test is vacuous");
            assert_ne!(px.samples, py.samples);
            // Switched off, the damaged factors play as they come.
            let mut z = Mp2Decoder::new();
            for f in &bad[..target] {
                z.decode(f).unwrap();
            }
            assert!(!z.decode(&bad[target]).unwrap().concealed);
            tested += 1;
        }
        assert!(tested >= 3, "groups tested: {tested}");
    }

    #[test]
    fn a_failed_header_is_filled_from_the_last_good_frame() {
        let clean = ab(10);
        let mut bad = clean.clone();
        flip(&mut bad[4], 50); // a bit of the bit allocation
        flip(&mut bad[5], 23); // and of the header's last byte but one: two in a row
        let mut d = conceal_on();
        let out = run(&mut d, &bad);
        assert_eq!(out.len(), 10);
        assert_eq!((d.hdr_crc_fail, d.scf_crc_fail, d.concealed, d.frames), (2, 0, 2, 10));
        let flags: Vec<bool> = out.iter().map(|p| p.concealed).collect();
        assert_eq!(flags, [false, false, false, false, true, true, false, false, false, false]);
        let energy = |s: &[i16]| s.iter().map(|&v| (v as f64).powi(2)).sum::<f64>();
        // The first fills with the sound that was playing, fading out...
        let c = &out[4].samples;
        assert_eq!((c.len(), out[4].rate, out[4].channels), (2304, 48_000, 2));
        assert!(energy(c) > 0.0);
        assert!(energy(&c[..1152]) > 2.0 * energy(&c[1152..]));
        // ...the second has only the filter's ringing left, then silence.
        assert!(out[5].samples[17 * 64..].iter().all(|&v| v == 0));
        // The frame after is not checked (nothing to check it against) and
        // the one after that is again.
        assert!(d.mem.tail.is_some());
        let mut d2 = conceal_on();
        for f in &bad[..5] {
            let _ = d2.decode(f);
        }
        assert!(d2.mem.tail.is_none() && d2.mem.fade_in);
        // Off, the same frames play as garbage and nothing is counted.
        let mut o = Mp2Decoder::new();
        let off = run(&mut o, &bad);
        assert!(off.iter().all(|p| !p.concealed));
        assert_eq!((o.hdr_crc_fail, o.concealed), (0, 0));
    }

    #[test]
    fn a_failed_header_before_any_good_frame_is_an_error() {
        let mut f = ab(1).remove(0);
        flip(&mut f, 50);
        let mut d = conceal_on();
        assert!(d.decode(&f).is_err());
        assert_eq!(d.hdr_crc_fail, 1);
        // A frame without a CRC in its header (protection bit set) is
        // taken as it is.
        let mut g = unhex(FRAME_A);
        g[1] |= 1;
        assert!(conceal_on().decode(&g).is_ok());
    }

    #[test]
    fn the_frame_after_a_concealed_one_fades_in() {
        let s = ab(3);
        let plain = Mp2Decoder::new().decode(&s[1]).unwrap();
        let mut d = conceal_on();
        d.mem.fade_in = true;
        let faded = d.decode(&s[1]).unwrap();
        assert!(!d.mem.fade_in);
        let energy = |s: &[i16]| s.iter().map(|&v| (v as f64).powi(2)).sum::<f64>();
        // The ramp covers 6 slots; the filter smears it over 16 more.
        assert!(energy(&faded.samples[..6 * 64]) < 0.5 * energy(&plain.samples[..6 * 64]));
        assert_eq!(faded.samples[22 * 64..], plain.samples[22 * 64..]);
    }

    #[test]
    fn lost_sync_does_not_fail_the_next_frames_scale_factors() {
        let s = ab(4);
        // Frame 2 never arrives: 3 follows 1 and its ScF-CRC (in 2) is lost.
        let gap = |junk: usize| {
            let mut bytes = s[0].clone();
            bytes.extend_from_slice(&s[1]);
            bytes.extend(std::iter::repeat_n(0u8, junk));
            bytes.extend_from_slice(&s[3]);
            let mut d = conceal_on();
            let mut frames = Vec::new();
            d.push(&bytes, |f| frames.push(f.to_vec()));
            assert_eq!(frames.len(), 3);
            for f in &frames {
                d.decode(f).unwrap();
            }
            d.scf_crc_fail
        };
        assert_eq!(gap(240), 0); // bytes skipped: the check is dropped
        assert!(gap(0) > 0); // nothing skipped: the mismatch is believed
    }

    #[test]
    fn switching_resets_what_is_remembered() {
        let s = ab(6);
        let mut d = Mp2Decoder::new();
        run(&mut d, &s[..3]);
        d.set_conceal(true);
        assert!(d.conceal() && d.mem.rate == 0 && d.mem.tail.is_none());
        let out = run(&mut d, &s[3..]);
        assert!(out.iter().all(|p| !p.concealed));
        d.set_conceal(false);
        assert!(!d.conceal());
        assert_eq!((d.hdr_crc_fail, d.scf_crc_fail), (0, 0));
    }
}
