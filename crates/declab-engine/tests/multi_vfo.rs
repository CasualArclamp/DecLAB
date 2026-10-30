// made by CasualArclamp
//! Two VFOs on one file, each with its own decoder, channel and WAV output.
//!
//! Uses the synthetic FM band from `tools/make_fm.py` (`out/` is not
//! committed): the stations at 94.5 and 100.3 MHz carry 600 Hz and 300 Hz
//! tones, so each VFO's sound shows which station it was on.

use std::path::PathBuf;

use declab_engine::{Host, HostOptions};
use declab_core::SampleFormat;
use declab_io::IqFileReader;

/// The tone's frequency from the first channel of a 16-bit WAV, by zero
/// crossings over the part after the first 0.3 s (the filters settle).
fn tone_hz(path: &std::path::Path) -> f64 {
    let b = std::fs::read(path).unwrap();
    let rate = u32::from_le_bytes(b[24..28].try_into().unwrap()) as usize;
    let ch = u16::from_le_bytes(b[22..24].try_into().unwrap()) as usize;
    let x: Vec<f32> = b[44..].chunks_exact(2 * ch).map(|f| i16::from_le_bytes([f[0], f[1]]) as f32).collect();
    let x = &x[rate * 3 / 10..];
    let crossings = x.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
    crossings as f64 * rate as f64 / x.len() as f64
}

#[test]
fn two_vfos_decode_two_stations_at_once() {
    let file = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../out/sim_fm_98.000MHz_10Msps.cs8"));
    if !file.exists() {
        eprintln!("skipped: make {file:?} with tools/make_fm.py");
        return;
    }
    let src = IqFileReader::open(&file, SampleFormat::Cs8, 10e6, 131_072).unwrap().with_center_freq(98.0e6);
    let mut host = Host::start(
        Box::new(src),
        HostOptions { pace_files: false, loop_files: false, waterfall: false, ..HostOptions::default() },
    );
    host.set_volume(0.0);
    let wfm = declab_decoders::find("wfm").unwrap();
    let bw = wfm.input(10e6).bandwidth_hz;
    let a = host.add_vfo(wfm, 94.5e6, bw).unwrap();
    let b = host.add_vfo(wfm, 100.3e6, bw).unwrap();
    assert_eq!(host.vfo_ids(), vec![a, b]);
    let dir = std::env::temp_dir();
    let (wa, wb) = (dir.join("declab_vfo_a.wav"), dir.join("declab_vfo_b.wav"));
    host.write_vfo_audio(a, Some(wa.clone())).unwrap();
    host.write_vfo_audio(b, Some(wb.clone())).unwrap();

    let t0 = std::time::Instant::now();
    while !host.finished() && t0.elapsed().as_secs() < 60 {
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(host.finished());
    host.write_vfo_audio(a, None).unwrap();
    host.write_vfo_audio(b, None).unwrap();

    let (fa, fb) = (tone_hz(&wa), tone_hz(&wb));
    assert!((fa - 600.0).abs() < 30.0, "VFO a heard {fa} Hz");
    assert!((fb - 300.0).abs() < 30.0, "VFO b heard {fb} Hz");
    // Each has its own view and channel.
    let (va, vb) = (host.vfo_view(a).unwrap(), host.vfo_view(b).unwrap());
    assert!((va.channel.offset_hz + 3.5e6).abs() < 1.0, "{}", va.channel.offset_hz);
    assert!((vb.channel.offset_hz - 2.3e6).abs() < 1.0, "{}", vb.channel.offset_hz);

    // The methods without `_vfo` follow the focus (the first VFO until told).
    assert_eq!(host.focused_id(), Some(a));
    assert_eq!(host.view().channel.offset_hz, va.channel.offset_hz);
    host.set_focus(b);
    assert_eq!(host.focused_id(), Some(b));
    assert_eq!(host.view().channel.offset_hz, vb.channel.offset_hz);
    // A new decoder takes the focused VFO's place, in the same position.
    host.set_decoder(Some(wfm));
    let ids = host.vfo_ids();
    assert_eq!(ids.len(), 2);
    assert_eq!(ids[0], a);
    assert_ne!(ids[1], b, "replaced, not kept");
    assert_eq!(host.focused_id(), Some(ids[1]));
    let b = ids[1];

    // Removing the primary hands the radio to the next VFO and leaves it running.
    host.remove_vfo(a);
    assert_eq!(host.vfo_ids(), vec![b]);
    assert!(host.vfo_view(a).is_none());
    assert_eq!(host.focused_id(), Some(b));
    let _ = std::fs::remove_file(wa);
    let _ = std::fs::remove_file(wb);
}
