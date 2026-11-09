// made by CasualArclamp
//! Every DAB+ service of an ensemble at once, for what their PAD carries:
//! the X-PAD applications each sends, its MOT data groups and errors, and
//! the slideshow pictures, written to a folder. The sound is not decoded.
//!
//! `cargo run --release -p declab-cli --example dab_slides -- FILE OFFSET_HZ OUT_DIR [SECONDS]`

use std::collections::hash_map::DefaultHasher;
use std::collections::{BTreeMap, HashSet};
use std::fs::File;
use std::hash::{Hash, Hasher};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::Duration;

use declab_api::{Block, Decoder, DecoderFactory, DecoderInfo, Family, InputSpec, Sink};
use declab_engine::{Host, HostOptions};
use declab_io::{IqFileReader, format_from_path, name_hint};

use decdab_data::pad::{PadDecoder, PadEvent};
use decdab_fic::FicDecoder;
use decdab_msc::{SubChannelDecoder, SuperframeDecoder, au_pad};
use decdab_ofdm::Demod;
use decdab_ofdm::params::{CIF_BITS, CIFS, FIC_BITS};

const RATE: f64 = 2.048e6;

/// Rust note: a factory must be `'static`, so the output folder reaches it
/// through this once-set global rather than a field.
static OUT: OnceLock<PathBuf> = OnceLock::new();

struct Survey;

impl DecoderFactory for Survey {
    fn info(&self) -> DecoderInfo {
        DecoderInfo {
            id: "dab_slides",
            name: "DAB PAD survey",
            family: Family::Radio,
            standard: "EN 300 401",
            summary: "",
        }
    }

    fn input(&self, _source_rate: f64) -> InputSpec {
        InputSpec {
            sample_rate: Some(RATE),
            bandwidth_hz: 1.6e6,
            block_len: 49_152,
        }
    }

    fn create(&self) -> Box<dyn Decoder> {
        Box::new(Worker {
            demod: Demod::new(),
            fic: FicDecoder::new(),
            generation: u64::MAX,
            services: BTreeMap::new(),
            frames: Vec::new(),
            mer: Vec::new(),
            t: 0.0,
        })
    }
}

struct Svc {
    name: String,
    sub_id: u8,
    sub: SubChannelDecoder,
    sf: SuperframeDecoder,
    pad: PadDecoder,
    seen: HashSet<u64>,
    pictures: u64,
    repeats: u64,
    other_mot: u64,
    label: Option<String>,
    dump: Option<File>,
}

struct Worker {
    demod: Demod,
    fic: FicDecoder,
    generation: u64,
    services: BTreeMap<u32, Svc>,
    frames: Vec<Vec<i8>>,
    mer: Vec<f32>,
    t: f64,
}

/// A name safe to use in a file name.
fn clean(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '.' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

impl Worker {
    /// Follow the FIC: services appear over the first seconds, labels later.
    fn sync_services(&mut self) {
        if self.fic.generation == self.generation {
            return;
        }
        self.generation = self.fic.generation;
        let e = &self.fic.ensemble;
        for (sid, svc) in &e.services {
            let Some(c) = svc.primary().filter(|c| c.is_dab_plus()) else {
                continue;
            };
            let Some(sc) = e.subchannel_of(c) else {
                continue;
            };
            if let Some(s) = self.services.get_mut(sid) {
                s.name = svc.name();
                if s.sub_id == sc.id {
                    continue;
                }
            }
            let Some(sub) = SubChannelDecoder::new(*sc) else {
                continue;
            };
            let sf = SuperframeDecoder::new(sub.frame_bytes());
            self.services.insert(
                *sid,
                Svc {
                    name: svc.name(),
                    sub_id: sc.id,
                    sub,
                    sf,
                    pad: PadDecoder::new(),
                    seen: HashSet::new(),
                    pictures: 0,
                    repeats: 0,
                    other_mot: 0,
                    label: None,
                    dump: std::env::var("DAB_DUMP")
                        .ok()
                        .filter(|l| l.split(',').any(|x| u32::from_str_radix(x, 16) == Ok(*sid)))
                        .and_then(|_| {
                            File::create(OUT.get()?.join(format!("pad_{sid:04X}.txt"))).ok()
                        }),
                },
            );
        }
    }
}

impl Svc {
    fn event(&mut self, sid: u32, t: f64, ev: PadEvent) {
        match ev {
            PadEvent::Dls(l) => self.label = Some(l.text),
            PadEvent::DlsClear => {}
            PadEvent::Mot {
                name, mime, data, ..
            } if mime.starts_with("image/") => {
                let mut h = DefaultHasher::new();
                data.hash(&mut h);
                if !self.seen.insert(h.finish()) {
                    self.repeats += 1;
                    return;
                }
                self.pictures += 1;
                let ext = if data.starts_with(&[0x89, b'P', b'N', b'G']) {
                    "png"
                } else {
                    "jpg"
                };
                let path = OUT.get().map(|d| {
                    d.join(format!(
                        "{sid:04X}_{}_{:03}_{}.{ext}",
                        clean(&self.name),
                        self.pictures,
                        clean(&name)
                    ))
                });
                if let Some(p) = &path
                    && let Err(e) = std::fs::write(p, &data)
                {
                    println!("  cannot write {}: {e}", p.display());
                }
                println!(
                    "{t:7.1} s  {sid:04X} {:<18} picture {name:?} {mime} {} bytes",
                    self.name,
                    data.len()
                );
            }
            PadEvent::Mot {
                name, mime, data, ..
            } => {
                self.other_mot += 1;
                println!(
                    "{t:7.1} s  {sid:04X} {:<18} other MOT {name:?} {mime:?} {} bytes",
                    self.name,
                    data.len()
                );
            }
        }
    }
}

impl Decoder for Worker {
    fn process(&mut self, block: &Block<'_>, _out: &mut Sink) {
        self.t = block.first_sample as f64 / RATE;
        let (frames, mer) = (&mut self.frames, &mut self.mer);
        self.demod.push(block.iq, |f| {
            frames.push(f.soft.to_vec());
            mer.push(f.stats.mer_db);
        });
        for soft in std::mem::take(&mut self.frames) {
            self.fic.push_frame(&soft[..FIC_BITS]);
            self.sync_services();
            let t = self.t;
            for (&sid, s) in &mut self.services {
                let mut events = Vec::new();
                for c in 0..CIFS {
                    let a = FIC_BITS + c * CIF_BITS;
                    if let Some(lf) = s.sub.push_cif(&soft[a..a + CIF_BITS]) {
                        let pad = &mut s.pad;
                        let dump = &mut s.dump;
                        s.sf.push(lf, |_, au| {
                            if au.ok
                                && let Some(field) = au_pad(au.data)
                            {
                                if let Some(f) = dump {
                                    // `DAB_DUMP`: the raw PAD field, hex, one AU a line.
                                    let hex: String =
                                        field.iter().map(|b| format!("{b:02x}")).collect();
                                    let _ = writeln!(f, "{hex}");
                                }
                                pad.push(field, &mut events);
                            }
                        });
                    }
                }
                for ev in events {
                    s.event(sid, t, ev);
                }
            }
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        if self.mer.is_empty() {
            println!("no frames decoded");
            return;
        }
        let e = &self.fic.ensemble;
        let mean = self.mer.iter().sum::<f32>() / self.mer.len() as f32;
        println!(
            "\n{}  {} frames, mean MER {mean:.1} dB",
            e.name(),
            self.mer.len()
        );
        println!(
            "{:<4} {:<18} {:>7} {:>6} {:>7} {:>6} {:>6} {:>5} {:>5}  X-PAD apps",
            "SId", "service", "AU ok", "bad", "groups", "MOTerr", "pics", "rep", "other"
        );
        for (sid, s) in &self.services {
            let st = s.pad.stats;
            let apps: Vec<String> = st
                .apps
                .iter()
                .enumerate()
                .filter(|(_, n)| **n > 0)
                .map(|(a, n)| format!("{a}:{n}"))
                .collect();
            println!(
                "{sid:04X} {:<18} {:>7} {:>6} {:>7} {:>6} {:>6} {:>5} {:>5}  {}",
                s.name,
                s.sf.stats.au_ok,
                s.sf.stats.au_bad,
                st.mot_groups,
                st.mot_errors,
                s.pictures,
                s.repeats,
                s.other_mot,
                apps.join(" ")
            );
        }
    }
}

static SURVEY: Survey = Survey;

fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (path, offset, out) = (Path::new(&a[0]), a[1].parse::<f64>()?, PathBuf::from(&a[2]));
    let limit: f64 = a.get(3).map_or(Ok(f64::MAX), |s| s.parse())?;
    std::fs::create_dir_all(&out)?;
    let _ = OUT.set(out);
    let hint = name_hint(path);
    let fmt = format_from_path(path).expect("format from the extension");
    let rate = hint.sample_rate.expect("rate in the file name");
    let mut r = IqFileReader::open(path, fmt, rate, 131_072)?;
    if let Some(c) = hint.center_hz {
        r = r.with_center_freq(c);
    }
    let mut host = Host::start(
        Box::new(r),
        HostOptions {
            pace_files: false,
            loop_files: false,
            waterfall: false,
            ..HostOptions::default()
        },
    );
    host.set_decoder(Some(&SURVEY));
    host.set_volume(0.0);
    host.set_channel(offset, 1.6e6);
    loop {
        std::thread::sleep(Duration::from_millis(250));
        let s = host.stats();
        if host.finished() || s.samples as f64 / s.sample_rate.max(1.0) >= limit {
            if s.handoff_dropped > 0 {
                println!("dropped {} blocks", s.handoff_dropped);
            }
            break;
        }
    }
    // Dropping the host ends its worker, and the worker's `Drop` prints the table.
    drop(host);
    Ok(())
}
