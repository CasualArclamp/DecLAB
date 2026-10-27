// made by CasualArclamp
//! FM RDS (EN 62106), from DecSAT's `decsat-engine::analog_rds` (same author).
//!
//! The 57 kHz subcarrier is brought to baseband as I and Q with the
//! pilot's third harmonic: EN 62106 lets a station lock it either in phase
//! or in quadrature to that harmonic, and DecSAT's version kept the in-phase
//! part only, so a quadrature station decoded on noise. The BPSK symbols'
//! phase is followed (the angle of their mean square, halved) and the chips
//! are decided on the derotated signal. Then biphase chip integration,
//! differential decoding, the RDS CRC and basic groups. One complex point
//! per symbol, derotated, goes to the "RDS" constellation scope.

use std::collections::VecDeque;

use num_complex::Complex32;

const TAU:f64=std::f64::consts::TAU;
const POLY:u16=0x5B9;

/// The programme type's name (EN 62106 annex F, the European RDS table).
pub fn pty_name(pty:u8)->&'static str {
    const NAMES:[&str;32]=["None","News","Current affairs","Information","Sport","Education","Drama","Culture",
        "Science","Varied","Pop music","Rock music","Easy listening","Light classical","Serious classical",
        "Other music","Weather","Finance","Children's","Social affairs","Religion","Phone-in","Travel",
        "Leisure","Jazz","Country","National music","Oldies","Folk music","Documentary","Alarm test","Alarm"];
    NAMES[usize::from(pty&31)]
}

/// Year, month, day of a Modified Julian Day (EN 62106 annex G).
fn mjd_date(mjd:u32)->(u32,u32,u32) {
    let mjd=f64::from(mjd);
    let y1=((mjd-15078.2)/365.25).floor();
    let m1=((mjd-14956.1-(y1*365.25).floor())/30.6001).floor();
    let d=mjd-14956.0-(y1*365.25).floor()-(m1*30.6001).floor();
    let k=if m1==14.0||m1==15.0 {1.0} else {0.0};
    ((y1+k+1900.0) as u32,(m1-1.0-k*12.0) as u32,d as u32)
}
const OFFSETS:[u16;5]=[0x0FC,0x198,0x168,0x1B4,0x350];
/// Symbols kept for the scope between reports.
const SCOPE_POINTS: usize = 400;
fn syndrome(mut v:u32)->u16 {
    for bit in (10..=25).rev() {
        if v&(1<<bit)!=0 {v^=(POLY as u32)<<(bit-10);}
    }
    (v&0x3ff) as u16
}
pub struct RdsDecoder {
    /// The subcarrier at baseband, low-passed.
    lp:Complex32,
    /// The derotated real part of `lp`, which the chips are decided on.
    lp_re:f32,
    chip_sum:Complex32,
    chip_clock:f64,
    chip_prev:Option<(bool, Complex32)>,
    last_sign: bool,
    chip_phase_locked: bool,
    last_symbol:Option<bool>,
    /// Mean of the symbols squared (BPSK: its angle is twice theirs).
    square:Complex32,
    /// The symbols' phase, radians, taken off before deciding.
    theta:f32,
    /// Derotated symbols for the scope.
    points:VecDeque<Complex32>,
    bits:u32,
    bits_seen:usize,
    block:usize,
    words:[u16;4],
    pi:Option<u16>,
    pty:Option<u8>,
    ps:[u8;8],
    text:[u8;64],
    groups:u64,
    /// Traffic programme / announcement, music (vs speech), from block B.
    tp:bool,
    ta:bool,
    music:bool,
    /// RadioText's A/B flag: a change clears the text (EN 62106 3.1.5.3).
    rt_ab:Option<bool>,
    /// Alternative frequencies heard (group 0A), as codes 1-204.
    af:std::collections::BTreeSet<u8>,
    /// Clock time (group 4A): MJD, hour, minute (UTC), local offset in
    /// half hours.
    ct:Option<(u32,u8,u8,i8)>,
    /// Groups heard of each type, index type*2 + version (0A, 0B, 1A…).
    types:[u64;32],
    /// Blocks that passed and failed the CRC while in sync.
    blocks_ok:u64,
    blocks_bad:u64,
}
impl Default for RdsDecoder {
    fn default() -> Self {
        Self {
            lp: Complex32::default(),
            lp_re: 0.0,
            chip_sum: Complex32::default(),
            chip_clock: 0.0,
            chip_prev: None,
            last_sign: false,
            chip_phase_locked: false,
            last_symbol: None,
            square: Complex32::default(),
            theta: 0.0,
            points: VecDeque::with_capacity(SCOPE_POINTS),
            bits: 0,
            bits_seen: 0,
            block: 0,
            words: [0; 4],
            pi: None,
            pty: None,
            ps: [0; 8],
            text: [0; 64],
            groups: 0,
            tp: false,
            ta: false,
            music: false,
            rt_ab: None,
            af: std::collections::BTreeSet::new(),
            ct: None,
            types: [0; 32],
            blocks_ok: 0,
            blocks_bad: 0,
        }
    }
}
impl RdsDecoder {
    /// The station name (PS), once any of it has come.
    pub fn station(&self)->Option<String> {
        self.pi?;
        let ps=String::from_utf8_lossy(&self.ps).trim_matches('\0').trim().to_string();
        (!ps.is_empty()).then_some(ps)
    }
    /// RadioText, once any of it has come.
    pub fn radio_text(&self)->Option<String> {
        let t=String::from_utf8_lossy(&self.text).trim_matches('\0').trim().to_string();
        (!t.is_empty()).then_some(t)
    }
    pub fn groups(&self)->u64 { self.groups }
    /// The last few hundred symbols, derotated (on the real axis when the
    /// phase is followed), for the constellation scope. Kept, not drained:
    /// reports come 60 times a second, far faster than the symbols.
    pub fn points(&self)->Vec<Complex32> {
        self.points.iter().copied().collect()
    }
    pub fn pi(&self)->Option<u16> { self.pi }
    pub fn pty(&self)->Option<u8> { self.pty }
    pub fn tp_ta(&self)->(bool,bool) { (self.tp,self.ta) }
    pub fn music(&self)->bool { self.music }
    /// Block error rate while in sync, 0..1 (None before any block).
    pub fn bler(&self)->Option<f64> {
        let n=self.blocks_ok+self.blocks_bad;
        (n>0).then(|| self.blocks_bad as f64/n as f64)
    }
    /// The alternative frequencies heard, MHz (EN 62106 3.2.1.6.1: code n is
    /// 87.5 + 0.1 n MHz).
    pub fn af_mhz(&self)->Vec<f64> {
        self.af.iter().map(|&c| 87.5+0.1*f64::from(c)).collect()
    }
    /// Clock time as "YYYY-MM-DD hh:mm UTC+h:mm", if group 4A was heard.
    pub fn clock(&self)->Option<String> {
        let (mjd,h,m,off)=self.ct?;
        let (y,mo,d)=mjd_date(mjd);
        let sign=if off<0 {'-'} else {'+'};
        let half=off.unsigned_abs();
        Some(format!("{y:04}-{mo:02}-{d:02} {h:02}:{m:02} UTC{sign}{}:{:02}",half/2,(half%2)*30))
    }
    /// "0A 120 · 2A 40 · 4A 1": the group types heard, most first.
    pub fn group_types(&self)->String {
        let mut v:Vec<(usize,u64)>=self.types.iter().copied().enumerate().filter(|(_,n)| *n>0).collect();
        v.sort_by_key(|x| std::cmp::Reverse(x.1));
        v.iter().map(|(i,n)| format!("{}{} {n}",i/2,if i%2==0 {'A'} else {'B'})).collect::<Vec<_>>().join(" · ")
    }
    /// The subcarrier's phase against the pilot's third harmonic, degrees
    /// (0 or 180: in phase; ±90: in quadrature).
    pub fn phase_deg(&self)->f32 {
        self.theta.to_degrees()
    }
    #[allow(dead_code)] // kept for a one-line status
    pub fn summary(&self)->String {
        if let Some(pi)=self.pi {
            let ps=String::from_utf8_lossy(&self.ps).trim_matches('\0').trim().to_string();
            let text=String::from_utf8_lossy(&self.text).trim_matches('\0').trim().to_string();
            format!("PI={pi:04X} PTY={} PS={} RT={} ({} groups)",
                self.pty.unwrap_or(0),ps,text,self.groups)
        } else {"searching for RDS groups".to_string()}
    }
    pub fn feed(&mut self, multiplex:f32, pilot_phase:f64, rate:f64) {
        let (s,c)=(3.0*pilot_phase).sin_cos();
        let mixed=Complex32::new(multiplex*c as f32, -multiplex*s as f32);
        let k=(TAU*2_400.0/rate).min(1.0) as f32;
        self.lp+=(mixed-self.lp)*k;
        // Take the symbols' phase off: z·e^(−jθ), real part.
        let (st,ct)=self.theta.sin_cos();
        self.lp_re=self.lp.re*ct+self.lp.im*st;
        // Manchester transitions provide a clock reference. Apply a small,
        // bounded phase correction instead of letting a free-running 2375-Hz
        // chip clock drift indefinitely relative to the transmitter.
        let sign=self.lp_re>=0.0;
        if sign!=self.last_sign && self.lp_re.abs()>0.002 {
            let phase=self.chip_clock;
            // A Manchester mid-chip transition normally occurs at the
            // sampling boundary. Other transitions are accepted with lower
            // confidence; avoid large jumps which erase decoded groups.
            let error=if phase<0.5 { -phase } else { 1.0-phase };
            if error.abs()<0.30 {
                self.chip_clock=(self.chip_clock+0.05*error).rem_euclid(1.0);
                self.chip_phase_locked=true;
            }
        }
        self.last_sign=sign;
        self.chip_sum+=self.lp;
        self.chip_clock+=2375.0/rate;
        if self.chip_clock>=1.0 {
            self.chip_clock-=1.0;
            let z=self.chip_sum;
            self.chip_sum=Complex32::default();
            let chip=z.re*ct+z.im*st>=0.0;
            if let Some((prev, pz))=self.chip_prev.take() {
                // The two biphase chips should have opposite sign.
                if chip==prev {
                    // Invalid Manchester pair: re-use the last chip as a
                    // possible first chip of the opposite phase alignment.
                    self.chip_prev=Some((chip, z));
                    self.last_symbol=None;
                    self.bits_seen=0;
                    self.block=0;
                    return;
                }
                // The symbol: half the difference of its two chips.
                let sym=(pz-z)*0.5;
                let mag=sym.norm().max(1e-12);
                // Follow the phase from the symbols squared (the BPSK sign
                // drops out), slowly: a few hundred symbols.
                self.square+=(sym*sym/mag-self.square)*0.01;
                self.theta=0.5*self.square.im.atan2(self.square.re);
                if self.points.len()>=SCOPE_POINTS { self.points.pop_front(); }
                self.points.push_back(sym*Complex32::from_polar(1.0,-self.theta));
                let symbol=chip;
                if let Some(last)=self.last_symbol {
                    self.bit(symbol^last);
                }
                self.last_symbol=Some(symbol);
            } else {self.chip_prev=Some((chip, z));}
        }
    }
    fn bit(&mut self,b:bool){
        self.bits=(self.bits<<1)|u32::from(b);
        self.bits_seen+=1;
        if self.bits_seen<26 {return;}
        let word=self.bits&0x03ff_ffff;
        let wanted=if self.block==2 {OFFSETS[2]} else {OFFSETS[self.block]};
        let crc=syndrome(word);
        if self.block==0 {
            if crc!=OFFSETS[0] {return;}
            self.words[0]=(word>>10) as u16;
            self.block=1;
            self.bits_seen=0;
            self.blocks_ok+=1;
        } else if crc==wanted || (self.block==2 && crc==OFFSETS[4]) {
            self.words[self.block]=(word>>10) as u16;
            self.block+=1;
            self.bits_seen=0;
            self.blocks_ok+=1;
            if self.block==4 {self.group();self.block=0;}
        } else {
            self.block=0;
            self.bits_seen=0;
            self.blocks_bad+=1;
        }
    }
    fn group(&mut self){
        self.groups+=1;
        self.pi=Some(self.words[0]);
        let b=self.words[1];
        self.pty=Some(((b>>5)&0x1f) as u8);
        let typ=(b>>12)&0xf;
        let ver=(b>>11)&1;
        let addr=(b&0xf) as usize;
        self.types[(typ*2+ver) as usize]+=1;
        self.tp=(b>>10)&1==1;
        if typ==0 {
            // EN 62106 3.1.5.1: TA, M/S, then the PS segment address.
            self.ta=(b>>4)&1==1;
            self.music=(b>>3)&1==1;
            let seg=(b&3) as usize;
            self.ps[seg*2]=(self.words[3]>>8) as u8;
            self.ps[seg*2+1]=self.words[3] as u8;
            if ver==0 {
                for code in [(self.words[2]>>8) as u8, self.words[2] as u8] {
                    if (1..=204).contains(&code) { self.af.insert(code); }
                }
            }
        } else if typ==4 && ver==0 {
            // EN 62106 3.1.5.6: MJD over blocks B-C, then hour, minute, offset.
            let c=u32::from(self.words[2]);
            let d=u32::from(self.words[3]);
            let mjd=((u32::from(b)&3)<<15)|(c>>1);
            let hour=(((c&1)<<4)|(d>>12)) as u8;
            let minute=((d>>6)&0x3f) as u8;
            let off=(d&0x1f) as i8*if d&0x20!=0 {-1} else {1};
            if hour<24 && minute<60 { self.ct=Some((mjd,hour,minute,off)); }
        } else if typ==2 {
            let ab=(b>>4)&1==1;
            if self.rt_ab.is_some_and(|x| x!=ab) { self.text=[0;64]; }
            self.rt_ab=Some(ab);
            if ver==0 && addr<16 {
                let i=addr*4;
                self.text[i]=(self.words[2]>>8) as u8;
                self.text[i+1]=self.words[2] as u8;
                self.text[i+2]=(self.words[3]>>8) as u8;
                self.text[i+3]=self.words[3] as u8;
            } else if ver==1 && addr<16 {
                let i=addr*2;
                self.text[i]=(self.words[3]>>8) as u8;
                self.text[i+1]=self.words[3] as u8;
            }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn parity_syndrome(){
        assert_eq!(syndrome(0),0);
        // Multiplying generator by a polynomial gives zero syndrome.
        assert_eq!(syndrome((POLY as u32)<<10),0);
    }

    /// A 26-bit block: 16 bits of information and the checkword with
    /// offset word `offset` added (EN 62106 annex B: the syndrome of a good
    /// block is its offset word).
    fn block(info: u16, offset: u16) -> u32 {
        let crc = syndrome(u32::from(info) << 10);
        (u32::from(info) << 10) | u32::from(crc ^ offset)
    }

    /// Group 0A segments carrying `ps` for station `pi`, as bits.
    fn ps_groups(pi: u16, ps: &[u8; 8]) -> Vec<bool> {
        let mut bits = Vec::new();
        for seg in 0..4u16 {
            let b = seg; // group 0A, PTY 0, segment address in the low bits
            let d = (u16::from(ps[2 * seg as usize]) << 8) | u16::from(ps[2 * seg as usize + 1]);
            for (info, off) in [(pi, OFFSETS[0]), (b, OFFSETS[1]), (0xE0CD, OFFSETS[2]), (d, OFFSETS[3])] {
                let w = block(info, off);
                bits.extend((0..26).rev().map(|i| (w >> i) & 1 == 1));
            }
        }
        bits
    }

    /// Feed `seconds` of RDS (the groups repeated) at 384 kS/s, its 57 kHz
    /// subcarrier `phase` radians from the pilot's third harmonic.
    fn decode(phase: f64, seconds: f64) -> RdsDecoder {
        let rate = 384_000.0;
        let bits = ps_groups(0xC123, b"DECLAB  ");
        let mut d = RdsDecoder::default();
        // Differential coding, then biphase: two chips a bit, opposite signs.
        let mut level = false;
        let mut chips = Vec::new();
        for i in 0..((seconds * 1187.5) as usize) {
            level ^= bits[i % bits.len()];
            chips.push(if level { 1.0f32 } else { -1.0 });
            chips.push(if level { -1.0 } else { 1.0 });
        }
        let n = (seconds * rate) as usize;
        for k in 0..n {
            let t = k as f64 / rate;
            let chip = chips[((t * 2375.0) as usize).min(chips.len() - 1)];
            let pilot = TAU * 19_000.0 * t;
            let m = 0.05 * chip * (3.0 * pilot + phase).cos() as f32;
            d.feed(m, pilot.rem_euclid(TAU), rate);
        }
        d
    }

    #[test]
    fn reads_the_station_name_in_phase_and_in_quadrature() {
        for phase in [0.0, TAU / 4.0, TAU / 2.0, 3.0 * TAU / 4.0] {
            let d = decode(phase, 3.0);
            assert_eq!(d.station().as_deref(), Some("DECLAB"), "subcarrier at {:.0}°", phase.to_degrees());
            // About 11.4 groups a second; most of 3 s once locked.
            assert!(d.groups() > 20, "only {} groups at {:.0}°", d.groups(), phase.to_degrees());
        }
    }

    #[test]
    fn group_types_and_errors_are_counted() {
        let d = decode(0.0, 3.0);
        assert!(d.group_types().starts_with("0A "), "{}", d.group_types());
        assert_eq!(d.pi(), Some(0xC123));
        assert!(d.bler().is_some_and(|b| b < 0.05), "{:?}", d.bler());
        // Block C of the test groups is 0xE0CD: AF codes 0xE0 (a filler,
        // 224) and 0xCD (205, not a frequency): none kept.
        assert!(d.af_mhz().is_empty());
    }

    #[test]
    fn modified_julian_day_to_date() {
        // EN 62106 annex G's example: MJD 45218 is 1982-09-06.
        assert_eq!(mjd_date(45218), (1982, 9, 6));
        assert_eq!(mjd_date(61340), (2026, 10, 27));
    }

    #[test]
    fn the_scope_points_lie_on_the_real_axis_once_derotated() {
        let d = decode(TAU / 4.0, 2.0);
        let pts = d.points();
        assert!(pts.len() > 100);
        let tail = &pts[pts.len() / 2..];
        let (re, im): (f32, f32) = tail.iter().fold((0.0, 0.0), |(a, b), z| (a + z.re.abs(), b + z.im.abs()));
        assert!(re > 10.0 * im, "re {re} im {im}: not derotated");
    }
}
