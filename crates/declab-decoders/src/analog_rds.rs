// made by CasualArclamp
//! FM RDS (EN 62106). Started from DecSAT's `decsat-engine::analog_rds`
//! (same author); the demodulator was rewritten for weak signals.
//!
//! **Subcarrier.** The 57 kHz subcarrier is brought to baseband with the
//! pilot's third harmonic as I and Q (a station may lock it in phase or in
//! quadrature, EN 62106 3.1.2), then filtered to ±2.4 kHz and decimated to
//! about 12 kS/s by the shared down-converter (`declab_dsp::Ddc`): the old
//! single-pole filter let in many times the noise and the stereo
//! subcarrier's sidebands.
//!
//! **Symbols.** A matched filter for the biphase symbol (half a symbol +1,
//! half −1), Gardner timing at 1187.5 symbols a second, the BPSK phase from
//! the symbols squared (the sign drops out), and soft decisions: a bit is a
//! change of sign between symbols (differential coding, 3.1.3), so no
//! absolute phase is needed. The old decoder decided each half-symbol hard
//! and lost block sync on any single bad pair.
//!
//! **Blocks.** Sync is taken on two good blocks 26 bits apart in the right
//! order (A B C/C′ D, 3.1.4), then held through bad blocks until 30 of the
//! last 50 fail (a flywheel). A block that fails its check is corrected
//! when its syndrome is that of a burst of one or two adjacent bit errors
//! (the code corrects bursts; two is a conservative use of it). A group
//! uses whatever blocks came good: a PS segment needs B and D, not C. At
//! the block error rates of a weak station a wrong correction slips through
//! now and then, so a PS or RadioText segment is shown only once the same
//! value has come twice in a row (the usual receiver practice).
//!
//! One point per symbol, derotated, goes to the "RDS" constellation scope.

use std::collections::{BTreeSet, HashMap, VecDeque};

use declab_dsp::Ddc;
use num_complex::Complex32;

const POLY:u16=0x5B9;
/// Offset words A, B, C, D, C′ (EN 62106 annex A).
const OFFSETS:[u16;5]=[0x0FC,0x198,0x168,0x1B4,0x350];
/// RDS symbol (biphase bit) rate, Hz: 57 kHz / 48.
const SYMBOL_RATE:f64=1187.5;
/// The subcarrier's band after filtering, Hz (±2.4 kHz, 3.1.2).
const BAND_HZ:f64=4800.0;
/// Symbols kept for the scope.
const SCOPE_POINTS: usize = 400;
/// Blocks remembered for the flywheel, and how many may fail before sync
/// is given up.
const FLYWHEEL:usize=50;
const FLYWHEEL_BAD:usize=30;

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

/// RDS characters as text: control characters become spaces, ASCII stays,
/// the rest (EBU Latin, annex E) the replacement character for now.
fn printable(b:&[u8])->String {
    let s:String=b.iter().map(|&c| match c {
        0x20..=0x7e => c as char,
        c if c<0x20 => ' ',
        _ => '\u{fffd}',
    }).collect();
    s.trim().to_string()
}

fn syndrome(mut v:u32)->u16 {
    for bit in (10..=25).rev() {
        if v&(1<<bit)!=0 {v^=(POLY as u32)<<(bit-10);}
    }
    (v&0x3ff) as u16
}

/// The syndromes of every burst of one or two adjacent bit errors in a
/// 26-bit block, and the burst. A syndrome names at most one of them (the
/// code's distance is enough for that), so a match is a correction.
fn burst_table()->HashMap<u16,u32> {
    let mut t=HashMap::new();
    for i in 0..26 {
        t.insert(syndrome(1<<i),1<<i);
        if i<25 { t.insert(syndrome(3<<i),3<<i); }
    }
    t
}

pub struct RdsDecoder {
    // ---- subcarrier
    ddc:Option<Ddc>,
    mixed:Vec<Complex32>,
    base:Vec<Complex32>,
    /// Samples per symbol at the baseband rate.
    sps:f64,
    // ---- symbols
    /// The matched filter's taps and the baseband samples it spans.
    taps:Vec<f32>,
    hist:VecDeque<Complex32>,
    /// Matched-filter outputs, the newest last (enough for a symbol back).
    mf:VecDeque<Complex32>,
    /// Baseband samples seen, and where the next symbol strobe falls.
    n:u64,
    next:f64,
    prev_sym:Option<Complex32>,
    prev_soft:Option<f32>,
    /// Mean of the symbols squared (BPSK: its angle is twice theirs).
    square:Complex32,
    theta:f32,
    /// Mean symbol power, for normalising the timing error.
    power:f32,
    /// Derotated symbol statistics for the SNR: mean of re², of im².
    re2:f32,
    im2:f32,
    points:VecDeque<Complex32>,
    // ---- blocks
    reg:u32,
    bits_seen:u32,
    synced:bool,
    /// While searching: where (bits) and which block the last good one was.
    cand:Option<(u64,usize)>,
    bit_no:u64,
    /// While synced: the next block (0 A, 1 B, 2 C/C′, 3 D) and its bits so far.
    expect:usize,
    in_block:u32,
    recent:VecDeque<bool>,
    words:[u16;4],
    valid:[bool;4],
    /// Block C carried C′ (a version B group).
    c_prime:bool,
    bursts:HashMap<u16,u32>,
    // ---- what was received
    pi:Option<u16>,
    pty:Option<u8>,
    ps:[u8;8],
    text:[u8;64],
    /// The last value of each PS segment and RadioText half-segment, not yet
    /// confirmed by a second.
    ps_cand:[Option<u16>;4],
    rt_cand:[Option<u16>;32],
    groups:u64,
    tp:bool,
    ta:bool,
    music:bool,
    rt_ab:Option<bool>,
    af:BTreeSet<u8>,
    ct:Option<(u32,u8,u8,i8)>,
    types:[u64;32],
    blocks_ok:u64,
    blocks_fixed:u64,
    blocks_bad:u64,
}

impl Default for RdsDecoder {
    fn default() -> Self {
        Self {
            ddc: None,
            mixed: Vec::with_capacity(4096),
            base: Vec::new(),
            sps: 0.0,
            taps: Vec::new(),
            hist: VecDeque::new(),
            mf: VecDeque::new(),
            n: 0,
            next: 0.0,
            prev_sym: None,
            prev_soft: None,
            square: Complex32::default(),
            theta: 0.0,
            power: 1e-6,
            re2: 0.0,
            im2: 0.0,
            points: VecDeque::with_capacity(SCOPE_POINTS),
            reg: 0,
            bits_seen: 0,
            synced: false,
            cand: None,
            bit_no: 0,
            expect: 0,
            in_block: 0,
            recent: VecDeque::with_capacity(FLYWHEEL),
            words: [0; 4],
            valid: [false; 4],
            c_prime: false,
            bursts: burst_table(),
            pi: None,
            pty: None,
            ps: [0; 8],
            text: [0; 64],
            ps_cand: [None; 4],
            rt_cand: [None; 32],
            groups: 0,
            tp: false,
            ta: false,
            music: false,
            rt_ab: None,
            af: BTreeSet::new(),
            ct: None,
            types: [0; 32],
            blocks_ok: 0,
            blocks_fixed: 0,
            blocks_bad: 0,
        }
    }
}

impl RdsDecoder {
    /// The station name (PS), once any of it has come.
    pub fn station(&self)->Option<String> {
        self.pi?;
        let ps=printable(&self.ps);
        (!ps.is_empty()).then_some(ps)
    }
    /// RadioText, once any of it has come (to its end mark, 0x0D).
    pub fn radio_text(&self)->Option<String> {
        let end=self.text.iter().position(|&c| c==0x0D).unwrap_or(self.text.len());
        let t=printable(&self.text[..end]);
        (!t.is_empty()).then_some(t)
    }
    pub fn groups(&self)->u64 { self.groups }
    /// The last few hundred symbols, derotated (on the real axis when the
    /// phase is followed), for the constellation scope.
    pub fn points(&self)->Vec<Complex32> {
        self.points.iter().copied().collect()
    }
    /// The subcarrier's phase against the pilot's third harmonic, degrees
    /// (0 or 180: in phase; ±90: in quadrature).
    pub fn phase_deg(&self)->f32 { self.theta.to_degrees() }
    /// The symbols' SNR, dB: their power on the decision axis over the
    /// power across it (Es/N0 of the BPSK symbols, about).
    pub fn snr_db(&self)->f32 {
        let noise=self.im2.max(1e-12);
        10.0*((self.re2-noise).max(1e-12)/noise).log10()
    }
    /// Block sync held.
    pub fn synced(&self)->bool { self.synced }
    pub fn pi(&self)->Option<u16> { self.pi }
    pub fn pty(&self)->Option<u8> { self.pty }
    pub fn tp_ta(&self)->(bool,bool) { (self.tp,self.ta) }
    pub fn music(&self)->bool { self.music }
    /// Block error rate while in sync, 0..1 (None before any block);
    /// corrected blocks count as good.
    pub fn bler(&self)->Option<f64> {
        let n=self.blocks_ok+self.blocks_fixed+self.blocks_bad;
        (n>0).then(|| self.blocks_bad as f64/n as f64)
    }
    /// Blocks put right by the burst correction.
    pub fn corrected(&self)->u64 { self.blocks_fixed }
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

    /// One multiplex sample at `rate`, with the pilot's phase.
    pub fn feed(&mut self, multiplex:f32, pilot_phase:f64, rate:f64) {
        let (s,c)=(3.0*pilot_phase).sin_cos();
        self.mixed.push(Complex32::new(multiplex*c as f32, -multiplex*s as f32));
        if self.mixed.len()>=2048 {
            self.flush(rate);
        }
    }

    /// Filter and decimate what has been fed, and run the symbols.
    fn flush(&mut self, rate:f64) {
        let ddc=self.ddc.get_or_insert_with(|| Ddc::new(rate, 0.0, BAND_HZ));
        if self.sps==0.0 {
            let out=ddc.out_rate();
            self.sps=out/SYMBOL_RATE;
            // The biphase symbol: +1 for its first half, −1 for its second.
            let len=self.sps.round() as usize;
            self.taps=(0..len).map(|k| if 2*k<len {1.0} else {-1.0}).collect();
            self.next=self.sps;
        }
        self.base.clear();
        ddc.process(&self.mixed, &mut self.base);
        self.mixed.clear();
        let base=std::mem::take(&mut self.base);
        for &x in &base {
            self.sample(x);
        }
        self.base=base;
    }

    /// One baseband sample: the matched filter, and a symbol when one is due.
    fn sample(&mut self, x:Complex32) {
        let len=self.taps.len();
        if self.hist.len()==len { self.hist.pop_front(); }
        self.hist.push_back(x);
        let mut y=Complex32::default();
        for (h,t) in self.hist.iter().zip(&self.taps) { y+=*h**t; }
        let keep=(self.sps.ceil() as usize)+3;
        if self.mf.len()==keep { self.mf.pop_front(); }
        self.mf.push_back(y);
        self.n+=1;
        if (self.n as f64)<self.next || self.mf.len()<keep { return; }

        // Strobe at `next` (fractional), and half a symbol before it.
        let at=|back:f64, mf:&VecDeque<Complex32>, n:u64| -> Complex32 {
            // Sample index t = n − 1 − back is mf[len − 1 − back]; interpolate.
            let k=back.floor();
            let f=(back-k) as f32;
            let i=mf.len()-1-(k as usize);
            let a=mf[i];
            let b=if i>0 { mf[i-1] } else { a };
            let _=n;
            a*(1.0-f)+b*f
        };
        let back=(self.n as f64-1.0-self.next).max(0.0);
        let sym=at(back,&self.mf,self.n);
        let mid=at((back+self.sps/2.0).min((keep-2) as f64),&self.mf,self.n);

        // Phase from the symbols squared (BPSK), slowly.
        let mag=sym.norm().max(1e-12);
        self.square+=(sym*sym/mag-self.square)*0.01;
        self.theta=0.5*self.square.im.atan2(self.square.re);
        let rot=Complex32::from_polar(1.0,-self.theta);
        let d=sym*rot;
        self.power+=(d.norm_sqr()-self.power)*0.01;
        self.re2+=(d.re*d.re-self.re2)*0.01;
        self.im2+=(d.im*d.im-self.im2)*0.01;
        if self.points.len()>=SCOPE_POINTS { self.points.pop_front(); }
        self.points.push_back(d);

        // Gardner timing on the derotated real parts: the midpoint between
        // two symbols of opposite sign should be zero.
        if let Some(p)=self.prev_sym {
            let e=((d.re-(p*rot).re)*(mid*rot).re)/self.power.max(1e-12);
            self.next-=0.05*f64::from(e.clamp(-1.0,1.0));
        }
        self.prev_sym=Some(sym);
        self.next+=self.sps;

        // Differential decoding: a bit is a change of sign.
        if let Some(p)=self.prev_soft {
            self.bit(p*d.re<0.0);
        }
        self.prev_soft=Some(d.re);
    }

    /// The block index a good syndrome names (C′ counts as C).
    fn block_of(syn:u16)->Option<(usize,bool)> {
        match OFFSETS.iter().position(|&o| o==syn)? {
            4=>Some((2,true)),
            i=>Some((i,false)),
        }
    }

    fn bit(&mut self,b:bool){
        self.reg=((self.reg<<1)|u32::from(b))&0x03ff_ffff;
        self.bits_seen=self.bits_seen.saturating_add(1);
        self.bit_no+=1;
        if !self.synced {
            if self.bits_seen<26 { return; }
            let Some((i,cp))=Self::block_of(syndrome(self.reg)) else { return };
            // Two good blocks a block apart, in order: sync.
            if let Some((at,j))=self.cand
                && self.bit_no-at==26 && i==(j+1)%4
            {
                self.synced=true;
                self.recent.clear();
                self.valid=[false;4];
                self.expect=i;
                self.in_block=0;
                self.take(i,(self.reg>>10) as u16,cp,true);
                return;
            }
            self.cand=Some((self.bit_no,i));
            return;
        }
        self.in_block+=1;
        if self.in_block<26 { return; }
        self.in_block=0;
        let i=(self.expect+1)%4;
        self.expect=i;
        let syn=syndrome(self.reg);
        let offs:&[(u16,bool)]=if i==2 {&[(OFFSETS[2],false),(OFFSETS[4],true)]} else {&[(OFFSETS[i],false)]};
        let mut got=None;
        for &(o,cp) in offs {
            if syn==o { got=Some(((self.reg>>10) as u16,cp,false)); break; }
        }
        if got.is_none() {
            for &(o,cp) in offs {
                if let Some(&e)=self.bursts.get(&(syn^o)) {
                    got=Some((((self.reg^e)>>10) as u16,cp,true));
                    break;
                }
            }
        }
        match got {
            Some((w,cp,fixed)) => {
                if fixed { self.blocks_fixed+=1; } else { self.blocks_ok+=1; }
                self.take(i,w,cp,true);
            }
            None => {
                self.blocks_bad+=1;
                self.take(i,0,false,false);
            }
        }
        if self.recent.len()==FLYWHEEL { self.recent.pop_front(); }
        self.recent.push_back(got.is_none());
        if self.recent.iter().filter(|&&bad| bad).count()>FLYWHEEL_BAD {
            self.synced=false;
            self.cand=None;
            self.bits_seen=0;
        }
    }

    /// Block `i` of the group (good or not); a group is handled at its D.
    fn take(&mut self,i:usize,word:u16,c_prime:bool,good:bool){
        if i==0 { self.valid=[false;4]; }
        self.words[i]=word;
        self.valid[i]=good;
        if i==2 { self.c_prime=c_prime; }
        if i==3 { self.group(); }
    }

    fn group(&mut self){
        if self.valid[0] { self.pi=Some(self.words[0]); }
        if !self.valid[1] { return; }
        // A version B group repeats PI in C′.
        if self.c_prime && self.valid[2] { self.pi=Some(self.words[2]); }
        self.groups+=1;
        let b=self.words[1];
        let typ=(b>>12)&0xf;
        let ver=(b>>11)&1;
        self.types[(typ*2+ver) as usize]+=1;
        self.pty=Some(((b>>5)&0x1f) as u8);
        self.tp=(b>>10)&1==1;
        let (c,d)=(self.words[2],self.words[3]);
        let (cv,dv)=(self.valid[2],self.valid[3]);
        if typ==0 {
            // EN 62106 3.1.5.1: TA, M/S, then the PS segment address.
            self.ta=(b>>4)&1==1;
            self.music=(b>>3)&1==1;
            let seg=(b&3) as usize;
            if dv {
                if self.ps_cand[seg]==Some(d) {
                    self.ps[seg*2]=(d>>8) as u8;
                    self.ps[seg*2+1]=d as u8;
                }
                self.ps_cand[seg]=Some(d);
            }
            if ver==0 && cv {
                for code in [(c>>8) as u8, c as u8] {
                    if (1..=204).contains(&code) { self.af.insert(code); }
                }
            }
        } else if typ==2 {
            let ab=(b>>4)&1==1;
            if self.rt_ab.is_some_and(|x| x!=ab) { self.text=[0;64]; self.rt_cand=[None;32]; }
            self.rt_ab=Some(ab);
            let addr=(b&0xf) as usize;
            // Two characters at text[i], once seen twice (candidate `k`).
            let (text,cand)=(&mut self.text,&mut self.rt_cand);
            let mut put=|k:usize,i:usize,w:u16| {
                if cand[k]==Some(w) {
                    text[i]=(w>>8) as u8;
                    text[i+1]=w as u8;
                }
                cand[k]=Some(w);
            };
            if ver==0 {
                let i=addr*4;
                if cv { put(addr*2,i,c); }
                if dv { put(addr*2+1,i+2,d); }
            } else if dv {
                put(addr,addr*2,d);
            }
        } else if typ==4 && ver==0 && cv && dv {
            // EN 62106 3.1.5.6: MJD over blocks B-C, then hour, minute, offset.
            let (c,d)=(u32::from(c),u32::from(d));
            let mjd=((u32::from(b)&3)<<15)|(c>>1);
            let hour=(((c&1)<<4)|(d>>12)) as u8;
            let minute=((d>>6)&0x3f) as u8;
            let off=(d&0x1f) as i8*if d&0x20!=0 {-1} else {1};
            if hour<24 && minute<60 { self.ct=Some((mjd,hour,minute,off)); }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;
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
        decode_noisy(phase, seconds, 0.0)
    }

    /// As `decode`, with white noise of standard deviation `sigma` added to
    /// the multiplex. RDS here is 0.05 on its subcarrier: in the ±2.4 kHz
    /// band the noise is sigma²/80 against the subcarrier's 6.25e-4, so
    /// sigma 0.224 is 0 dB in the band, Eb/N0 about 6 dB.
    fn decode_noisy(phase: f64, seconds: f64, sigma: f32) -> RdsDecoder {
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
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut gauss = move || {
            // Sum of four uniforms: near enough Gaussian for a test.
            let mut acc = 0.0f32;
            for _ in 0..4 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                acc += (state >> 40) as f32 / (1u64 << 24) as f32 - 0.5;
            }
            acc * 3.0f32.sqrt()
        };
        for k in 0..n {
            let t = k as f64 / rate;
            let chip = chips[((t * 2375.0) as usize).min(chips.len() - 1)];
            let pilot = TAU * 19_000.0 * t;
            let m = 0.05 * chip * (3.0 * pilot + phase).cos() as f32 + sigma * gauss();
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
    fn decodes_down_to_about_0_db_in_band() {
        // sigma 0.2: about +1 dB in the RDS band (Eb/N0 about 7 dB).
        let d = decode_noisy(TAU / 4.0, 6.0, 0.2);
        assert_eq!(d.station().as_deref(), Some("DECLAB"));
        assert!(d.groups() > 30, "{} groups in 6 s", d.groups());
    }

    /// Not a check: prints groups decoded against noise (--nocapture).
    #[test]
    fn noise_sweep() {
        for sigma in [0.1f32, 0.2, 0.25, 0.3, 0.35, 0.4] {
            let d = decode_noisy(0.7, 6.0, sigma);
            let band_db = 10.0 * (6.25e-4f32 / (sigma * sigma / 80.0)).log10();
            eprintln!(
                "sigma {sigma:.2}: band SNR {band_db:+.1} dB, groups {} / ~68, BLER {:.1} %, fixed {}, PS {:?}, sym SNR {:.1} dB",
                d.groups(),
                d.bler().unwrap_or(1.0) * 100.0,
                d.corrected(),
                d.station(),
                d.snr_db()
            );
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
