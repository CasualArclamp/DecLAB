// made by CasualArclamp
//! Two VFOs on one recording, as the GUI runs them: a primary decoder at
//! one frequency and a second VFO at another, printing both views each
//! half second. For looking into "the second VFO will not lock".
//!
//! `cargo run --release -p declab-cli --example vfo_probe -- FILE DEC_1 HZ_1 DEC_2 HZ_2`

use std::path::Path;
use std::time::Duration;

use declab_engine::{Host, HostOptions};
use declab_io::{IqFileReader, format_from_path, name_hint};

fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let path = Path::new(&a[0]);
    let (d1, hz1, d2, hz2) = (&a[1], a[2].parse::<f64>()?, &a[3], a[4].parse::<f64>()?);
    let hint = name_hint(path);
    let fmt = format_from_path(path).expect("format");
    let rate = hint.sample_rate.expect("rate in the name");
    let mut r = IqFileReader::open(path, fmt, rate, 131_072)?;
    if let Some(c) = hint.center_hz {
        r = r.with_center_freq(c);
    }
    let f1 = declab_decoders::find(d1).expect("decoder 1");
    let f2 = declab_decoders::find(d2).expect("decoder 2");
    let mut host = Host::start(
        Box::new(r),
        HostOptions { pace_files: true, loop_files: false, waterfall: false, ..HostOptions::default() },
    );
    host.set_decoder(Some(f1));
    host.set_volume(0.0);
    host.set_channel_at(hz1, f1.input(rate).bandwidth_hz);
    let v2 = host.add_vfo(f2, hz2, f2.input(rate).bandwidth_hz).expect("second VFO");
    let mut t = 0.0;
    while !host.finished() && t < 7.0 {
        std::thread::sleep(Duration::from_millis(500));
        t += 0.5;
        let show = |v: Option<declab_engine::DecoderView>| {
            v.map_or("-".into(), |v| {
                let c = v.card.as_ref().map_or(String::new(), |c| format!("{} | {}", c.title, c.headline));
                format!("{:?} {} [ch {:.0} kS/s]", v.lock, c, v.channel.out_rate / 1e3)
            })
        };
        println!("{t:4.1} s  1: {}", show(Some(host.view())));
        println!("        2: {}", show(host.vfo_view(v2)));
    }
    Ok(())
}
