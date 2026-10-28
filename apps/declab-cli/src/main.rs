// made by CasualArclamp
//! The DecLAB command line.
//!
//! ```text
//! declab list
//! declab selftest <decoder>
//! declab run <decoder> --file <capture> [--rate HZ] [--freq HZ] [--offset HZ]
//!                      [--bandwidth HZ] [--seconds S] [--realtime]
//! declab run <decoder> --radio hackrf|airspy --freq HZ [--rate HZ] [--offset HZ]
//!                      [--gain AMP,LNA,VGA] [--seconds S] [--record OUT.cs8]
//!                      [--ts OUT.ts] [--card]
//! ```
//!
//! `run` prints the decoder's lock and measurements once a second and plays
//! any audio it makes. A file is decoded as fast as the decoder goes unless
//! `--realtime` is given (which you want to listen to it).

use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, bail};
use declab_api::{Block, Iq, Sink, Tuning};
use declab_engine::{Host, HostOptions};
use declab_io::{IqFileReader, IqSource, format_from_path, name_hint};

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("list") | None => {
            for f in declab_decoders::ALL {
                let i = f.info();
                println!("{:<8} {:<12} {:<15} {}", i.id, i.name, i.family.label(), i.summary);
            }
            Ok(())
        }
        Some("selftest") => {
            let id = args.get(1).context("usage: declab selftest <decoder id>")?;
            selftest(id)
        }
        Some("run") => run(&args[1..]),
        Some(other) => bail!("unknown command {other:?} (try `list`, `selftest <id>` or `run <id> …`)"),
    }
}

/// `--name value` options, parsed loosely.
struct Opts<'a>(&'a [String]);

impl Opts<'_> {
    fn get(&self, name: &str) -> Option<&str> {
        self.0.iter().position(|a| a == name).and_then(|i| self.0.get(i + 1)).map(String::as_str)
    }

    fn flag(&self, name: &str) -> bool {
        self.0.iter().any(|a| a == name)
    }

    /// A frequency or rate: plain Hz, or with a k/M/G suffix ("98.8M").
    fn hz(&self, name: &str) -> anyhow::Result<Option<f64>> {
        let Some(v) = self.get(name) else { return Ok(None) };
        let (num, scale) = match v.chars().last() {
            Some('k' | 'K') => (&v[..v.len() - 1], 1e3),
            Some('M' | 'm') => (&v[..v.len() - 1], 1e6),
            Some('G' | 'g') => (&v[..v.len() - 1], 1e9),
            _ => (v, 1.0),
        };
        let n: f64 = num.parse().with_context(|| format!("{name} {v}: not a number"))?;
        Ok(Some(n * scale))
    }
}

fn run(args: &[String]) -> anyhow::Result<()> {
    let id = args.first().context("usage: declab run <decoder> --file <capture> | --radio <name>")?;
    let factory = declab_decoders::find(id).with_context(|| format!("no decoder {id:?} (see `declab list`)"))?;
    let o = Opts(&args[1..]);
    let seconds = o.get("--seconds").map(str::parse::<f64>).transpose().context("--seconds")?;

    let source: Box<dyn IqSource> = if let Some(path) = o.get("--file") {
        let path = PathBuf::from(path);
        let hint = name_hint(&path);
        let fmt = format_from_path(&path).context("cannot tell the sample format from the file's extension (.cs8, .cs16, .cf32, .cu8, .wav)")?;
        let rate = o.hz("--rate")?.or(hint.sample_rate).context("give --rate: the file name does not say")?;
        let mut r = IqFileReader::open(&path, fmt, rate, 131_072)?;
        if let Some(f) = o.hz("--freq")?.or(hint.center_hz) {
            r = r.with_center_freq(f);
        }
        Box::new(r)
    } else if let Some(radio) = o.get("--radio") {
        open_radio(radio, &o)?
    } else {
        bail!("give --file <capture> or --radio hackrf|airspy");
    };

    let live = source.is_live();
    let verbose = o.flag("--card");
    println!("{}  {:.3} MS/s", source.describe(), source.sample_rate() / 1e6);
    let mut host = Host::start(
        source,
        HostOptions {
            pace_files: o.flag("--realtime"),
            loop_files: false,
            waterfall: false,
            ..HostOptions::default()
        },
    );
    host.set_decoder(Some(factory));
    if let Some(path) = o.get("--ts") {
        // The decoder's transport stream (DVB-T/T2).
        host.write_ts(Some(PathBuf::from(path)))?;
    }
    if let Some(path) = o.get("--record") {
        // The raw samples, as they come from the source (format from the extension).
        host.record(Some(PathBuf::from(path)));
    }
    let spec = factory.input(host.source_rate());
    let offset = o.hz("--offset")?.unwrap_or(0.0);
    let bandwidth = o.hz("--bandwidth")?.unwrap_or(spec.bandwidth_hz);
    if offset != 0.0 || bandwidth != spec.bandwidth_hz {
        host.set_channel(offset, bandwidth);
    }

    let t0 = Instant::now();
    let mut printed_log = 0usize;
    let mut last_text = String::new();
    let mut audio_said = false;
    loop {
        std::thread::sleep(Duration::from_millis(if live { 1000 } else { 250 }));
        let s = host.stats();
        let v = host.view();
        let ch = v.channel;
        let meas: Vec<String> = v
            .measurements
            .iter()
            .map(|m| format!("{} {:.2}{}{}", m.name, m.value, if m.unit.is_empty() { "" } else { " " }, m.unit))
            .collect();
        println!(
            "{:6.1} s  in {:5.1} dBFS{}  ch {:.0} kS/s (/{}{})  {:?}  {}  cpu {:.0}%{}",
            s.samples as f64 / s.sample_rate.max(1.0),
            s.rms_dbfs,
            if s.clip_percent > 0.1 { format!(" CLIP {:.1}%", s.clip_percent) } else { String::new() },
            ch.out_rate / 1e3,
            ch.decimation,
            ch.resample.map(|(u, d)| format!(" ×{u}/{d}")).unwrap_or_default(),
            v.lock,
            meas.join(", "),
            v.cpu_percent,
            if v.ts_packets > 0 { format!("  TS {} packets", v.ts_packets) } else { String::new() },
        );
        if let Some(c) = &v.card
            && verbose
        {
            println!("    card: {} | {} | {}", c.title, c.headline, c.details.iter().map(|(k, v)| format!("{k} {v}")).collect::<Vec<_>>().join(", "));
        }
        // Rust note: `skip` on a VecDeque's iterator resumes where the last
        // print stopped (the log only grows until it is trimmed at 300).
        for line in v.log.iter().skip(printed_log) {
            println!("    {line}");
        }
        printed_log = v.log.len();
        if let Some(t) = v.text.values().next()
            && *t != last_text
        {
            println!("    text: {t}");
            last_text = t.clone();
        }
        if let Some(sv) = v.services.first()
            && !sv.name.is_empty()
        {
            println!("    service: {}", sv.name);
        }
        if !audio_said && let Some(a) = &v.audio {
            println!("    sound: {a}");
            audio_said = true;
        }
        if let Some(e) = &s.error {
            println!("    error: {e}");
        }
        if host.finished() || seconds.is_some_and(|lim| t0.elapsed().as_secs_f64() >= lim && live)
            || seconds.is_some_and(|lim| !live && s.samples as f64 / s.sample_rate.max(1.0) >= lim)
        {
            break;
        }
    }
    drop(host);
    Ok(())
}

#[cfg(feature = "hackrf")]
fn open_hackrf(o: &Opts) -> anyhow::Result<Box<dyn IqSource>> {
    use declab_io::{HackRfGains, HackRfSettings, HackRfSource};
    let mut gains = HackRfGains::default();
    if let Some(g) = o.get("--gain") {
        let p: Vec<u16> = g.split(',').map(str::parse).collect::<Result<_, _>>().context("--gain AMP,LNA,VGA")?;
        let [amp, lna, vga] = p[..] else { bail!("--gain AMP,LNA,VGA (e.g. 1,32,20)") };
        gains = HackRfGains { amp: amp != 0, lna_db: lna, vga_db: vga };
    }
    let s = HackRfSettings {
        center_hz: o.hz("--freq")?.context("--freq")? as u64,
        sample_rate: o.hz("--rate")?.unwrap_or(10e6) as u32,
        gains,
    };
    Ok(Box::new(HackRfSource::open(s)?))
}

#[cfg(not(feature = "hackrf"))]
fn open_hackrf(_: &Opts) -> anyhow::Result<Box<dyn IqSource>> {
    bail!("built without the hackrf feature")
}

#[cfg(feature = "airspy")]
fn open_airspy(o: &Opts) -> anyhow::Result<Box<dyn IqSource>> {
    use declab_io::{AirspyGains, AirspySettings, AirspySource};
    let mut gains = AirspyGains::default();
    if let Some(g) = o.get("--gain") {
        let p: Vec<u8> = g.split(',').map(str::parse).collect::<Result<_, _>>().context("--gain LNA,MIXER,VGA")?;
        let [lna, mixer, vga] = p[..] else { bail!("--gain LNA,MIXER,VGA (each 0-15)") };
        gains = AirspyGains { lna, mixer, vga }.clamped();
    }
    let s = AirspySettings {
        center_hz: o.hz("--freq")?.context("--freq")? as u64,
        gains,
        ..AirspySettings::default()
    };
    Ok(Box::new(AirspySource::open(s)?))
}

#[cfg(not(feature = "airspy"))]
fn open_airspy(_: &Opts) -> anyhow::Result<Box<dyn IqSource>> {
    bail!("built without the airspy feature")
}

fn open_radio(name: &str, o: &Opts) -> anyhow::Result<Box<dyn IqSource>> {
    match name {
        "hackrf" => open_hackrf(o),
        "airspy" => open_airspy(o),
        other => bail!("unknown radio {other:?} (hackrf or airspy)"),
    }
}

fn selftest(id: &str) -> anyhow::Result<()> {
    let factory = declab_decoders::find(id).with_context(|| format!("no decoder {id:?}"))?;
    let spec = factory.input(20e6);
    let rate = spec.sample_rate.unwrap_or(20e6);
    let mut dec = factory.create();
    let mut sink = Sink::default();

    // White noise at -20 dBFS from a small xorshift, so no rand dependency.
    let mut state = 0x2545_f491_4f6c_dd1du64;
    let mut noise = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 40) as f32 / (1u64 << 24) as f32 - 0.5
    };
    let scale = 0.1 * 12f32.sqrt() / 2f32.sqrt();
    let mut iq = vec![Iq::default(); spec.block_len];
    let tuning = Tuning { center_hz: 0.0, sample_rate: rate, generation: 0 };

    let mut first = 0u64;
    while (first as f64) < rate {
        for s in iq.iter_mut() {
            *s = Iq::new(noise() * scale, noise() * scale);
        }
        let block = Block { iq: &iq, tuning, first_sample: first, dropped_before: 0, live: false };
        sink.clear();
        dec.process(&block, &mut sink);
        for m in &sink.measurements {
            println!("{:>10.0}  {:?}  {} = {:.2} {}", first, sink.lock, m.name, m.value, m.unit);
        }
        first += iq.len() as u64;
    }
    Ok(())
}
