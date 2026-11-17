// made by CasualArclamp
//! Auto-recording end to end: two NFM transmissions in noise, received by
//! the NFM decoder with its squelch on, must leave two WAV files (named for
//! the channel and the decoder), each about as long as its transmission
//! plus the hang time, and nothing for the noise between.

use declab_api::Command;
use declab_core::SampleFormat;
use declab_engine::{AutoRecord, Host, HostOptions};
use declab_io::IqFileReader;

const RATE: f64 = 250e3;
const RF_HZ: f64 = 446.00625e6;

/// `(seconds, carrier on)` pieces: FM with a 1 kHz tone at 3 kHz
/// deviation when on, noise throughout, as cf32 bytes.
fn signal(pieces: &[(f64, bool)]) -> Vec<u8> {
    let mut seed = 12345u32;
    let mut noise = move || {
        // Uniform noise, two draws summed: plenty for a squelch test.
        let mut u = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        0.02 * (u() + u())
    };
    let mut out = Vec::new();
    let (mut phase, mut t) = (0.0f64, 0.0f64);
    for &(secs, on) in pieces {
        for _ in 0..(secs * RATE) as usize {
            t += 1.0 / RATE;
            phase += std::f64::consts::TAU * 3e3 * (std::f64::consts::TAU * 1e3 * t).sin() / RATE;
            let a = if on { 0.3 } else { 0.0 };
            let i = a * phase.cos() as f32 + noise();
            let q = a * phase.sin() as f32 + noise();
            out.extend_from_slice(&i.to_le_bytes());
            out.extend_from_slice(&q.to_le_bytes());
        }
    }
    out
}

#[test]
fn two_transmissions_two_files() {
    let tmp = std::env::temp_dir().join(format!("declab-autorec-e2e-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(&tmp).unwrap();
    let file = tmp.join("nfm.cf32");
    std::fs::write(&file, signal(&[(1.5, false), (2.0, true), (3.0, false), (1.5, true), (3.0, false)])).unwrap();

    let src = IqFileReader::open(&file, SampleFormat::Cf32, RATE, 16_384).unwrap().with_center_freq(RF_HZ);
    let mut host = Host::start(
        Box::new(src),
        HostOptions { pace_files: false, loop_files: false, waterfall: false, ..HostOptions::default() },
    );
    host.set_volume(0.0);
    let dir = tmp.join("recordings");
    host.set_auto_record(Some(AutoRecord { dir: dir.clone(), hang_s: 1.0, ..AutoRecord::default() }));
    let nfm = declab_decoders::find("nfm").unwrap();
    let id = host.add_vfo(nfm, RF_HZ, nfm.input(RATE).bandwidth_hz).unwrap();
    host.vfo_command(id, Command::Set { key: "squelch".into(), value: "snr".into() });

    let t0 = std::time::Instant::now();
    while !host.finished() && t0.elapsed().as_secs() < 60 {
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    assert!(host.finished());
    let view = host.vfo_view(id).unwrap();
    // Stopping finishes a recording still open.
    host.set_auto_record(None);
    drop(host);

    let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.path()).collect();
    files.sort();
    let lens: Vec<f64> = files
        .iter()
        .map(|p| {
            let b = std::fs::read(p).unwrap();
            let rate = u32::from_le_bytes(b[24..28].try_into().unwrap()) as f64;
            let ch = u16::from_le_bytes(b[22..24].try_into().unwrap()) as f64;
            (b.len() - 44) as f64 / (2.0 * rate * ch)
        })
        .collect();
    eprintln!("{files:?} {lens:?} {:?}", view.record_error);
    assert_eq!(view.record_error, None);
    assert_eq!((view.recorded, view.recording.as_ref()), (2, None));
    assert_eq!(files.len(), 2, "{files:?}");
    for f in &files {
        let name = f.file_name().unwrap().to_string_lossy().into_owned();
        assert!(name.ends_with("_446.00625MHz_nfm.wav") || name.ends_with("_446.00625MHz_nfm-2.wav"), "{name}");
    }
    // Each transmission plus the hang, and the squelch's tail: through the
    // channel filter the NFM squelch lets some noise by for about a second
    // after the carrier goes, which counts as sound.
    let mut lens = lens;
    lens.sort_by(f64::total_cmp);
    assert!((2.4..=4.2).contains(&lens[0]), "1.5 s over: {lens:?}");
    assert!((2.9..=4.7).contains(&lens[1]), "2 s over: {lens:?}");
    std::fs::remove_dir_all(&tmp).unwrap();
}
