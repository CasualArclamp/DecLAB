// made by CasualArclamp
//! Retune a running decoder from one carrier to another in a recording,
//! as a click in the GUI does, and print what the view shows second by
//! second: for looking into "it will not lock after a retune".
//!
//! `cargo run --release -p declab-cli --example retune_probe -- FILE DECODER HZ_A HZ_B [SECONDS_ON_A]`

use std::time::Duration;

use declab_engine::{Host, HostOptions};
use declab_io::{IqFileReader, format_from_path, name_hint};

fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (path, dec, hz_a, hz_b) = (&a[0], &a[1], a[2].parse::<f64>()?, a[3].parse::<f64>()?);
    let on_a: f64 = a.get(4).map_or(Ok(3.0), |s| s.parse())?;
    let hint = name_hint(std::path::Path::new(path));
    let fmt = format_from_path(std::path::Path::new(path)).expect("format");
    let rate = hint.sample_rate.expect("rate in the name");
    let mut r = IqFileReader::open(std::path::Path::new(path), fmt, rate, 131_072)?;
    if let Some(c) = hint.center_hz {
        r = r.with_center_freq(c);
    }
    let factory = declab_decoders::find(dec).expect("decoder");
    let mut host = Host::start(
        Box::new(r),
        HostOptions { pace_files: true, loop_files: false, waterfall: false, ..HostOptions::default() },
    );
    host.set_decoder(Some(factory));
    host.set_volume(0.0);
    let bw = factory.input(rate).bandwidth_hz;
    host.set_channel_at(hz_a, bw);
    let mut t = 0.0;
    let mut retuned = false;
    while !host.finished() && t < on_a + 8.0 {
        std::thread::sleep(Duration::from_millis(250));
        t += 0.25;
        if !retuned && t >= on_a {
            host.set_channel_at(hz_b, bw);
            retuned = true;
            println!("--- retune to {:.4} MHz", hz_b / 1e6);
        }
        let v = host.view();
        let card = v.card.as_ref().map_or(String::new(), |c| {
            let pick = |k: &str| c.details.iter().find(|(n, _)| n == k).map_or(String::new(), |(_, v)| v.clone());
            format!("{} | {} | {} | off {} | {}", c.title, c.headline, pick("cell"), pick("offset"), pick("slots"))
        });
        println!("{t:5.2} s  {:?}  {card}", v.lock);
    }
    Ok(())
}
