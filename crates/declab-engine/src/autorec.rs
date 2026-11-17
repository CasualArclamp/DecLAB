// made by CasualArclamp
//! Recording a VFO's sound by itself, one file a transmission: a file opens
//! when the decoder's audio rises above silence (an analog squelch opening,
//! a digital voice call starting) and closes once it has been quiet for the
//! hang time. A squelch that is shut gives silence and a digital decoder
//! between calls gives nothing, so the sound alone says when something is
//! heard, whatever the decoder.
//!
//! Time is the stream's (samples over rate), so a file played faster than
//! real time cuts its recordings where a radio would have. Files are named
//! `YYYYMMDD-HHMMSSZ_<MHz>_<decoder>.wav`, UTC, from the moment they open.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::host::WavOut;

/// What the auto-recorder is set to (`Host::set_auto_record`).
#[derive(Debug, Clone, PartialEq)]
pub struct AutoRecord {
    /// Where the files go (made if missing).
    pub dir: PathBuf,
    /// Quiet after which a recording ends, seconds: the pauses within one
    /// over stay in its file.
    pub hang_s: f64,
    /// Recordings with less sound than this (seconds above the threshold)
    /// are deleted: a squelch tail or a click.
    pub min_s: f64,
    /// The level (RMS of a chunk, dBFS) above which the sound counts as
    /// something heard.
    pub threshold_dbfs: f32,
}

impl Default for AutoRecord {
    fn default() -> Self {
        AutoRecord { dir: PathBuf::from("recordings"), hang_s: 2.0, min_s: 0.5, threshold_dbfs: -50.0 }
    }
}

/// One recording being written.
struct Current {
    wav: WavOut,
    path: PathBuf,
    /// Stream time of the last chunk above the threshold.
    last_heard_s: f64,
    /// Seconds of sound above the threshold so far.
    heard_s: f64,
}

/// A VFO's auto-recorder: lives on its worker thread.
#[derive(Default)]
pub(crate) struct Recorder {
    current: Option<Current>,
    /// Recordings kept since the VFO started.
    pub kept: u64,
    /// The last file kept, and the last error.
    pub last: Option<PathBuf>,
    pub error: Option<String>,
}

impl Recorder {
    /// A chunk of the decoder's sound at stream time `now_s`, on RF `hz`,
    /// from decoder `decoder`.
    #[allow(clippy::too_many_arguments)] // the chunk, and where it is from
    pub fn audio(
        &mut self,
        cfg: &AutoRecord,
        now_s: f64,
        hz: f64,
        decoder: &str,
        rate: u32,
        channels: u16,
        samples: &[f32],
    ) {
        if samples.is_empty() || rate == 0 {
            return;
        }
        let ms = samples.iter().map(|x| x * x).sum::<f32>() / samples.len() as f32;
        let heard = 10.0 * ms.max(1e-20).log10() > cfg.threshold_dbfs;
        if heard && self.current.is_none() {
            let path = unused(cfg.dir.join(file_name(SystemTime::now(), hz, decoder)));
            match std::fs::create_dir_all(&cfg.dir).and_then(|_| WavOut::create(&path)) {
                Ok(wav) => {
                    self.current = Some(Current { wav, path, last_heard_s: now_s, heard_s: 0.0 });
                    self.error = None;
                }
                Err(e) => self.error = Some(format!("{}: {e}", path.display())),
            }
        }
        if let Some(c) = &mut self.current {
            c.wav.push(rate, channels, samples);
            if heard {
                c.last_heard_s = now_s;
                c.heard_s += samples.len() as f64 / (rate as f64 * channels.max(1) as f64);
            }
        }
    }

    /// Stream time has reached `now_s`: end a recording quiet for longer
    /// than the hang time.
    pub fn tick(&mut self, cfg: &AutoRecord, now_s: f64) {
        if self.current.as_ref().is_some_and(|c| now_s - c.last_heard_s > cfg.hang_s) {
            self.finish(cfg.min_s);
        }
    }

    /// End the recording under way, keeping it if it has `min_s` of sound.
    pub fn finish(&mut self, min_s: f64) {
        let Some(c) = self.current.take() else { return };
        let keep = c.heard_s >= min_s;
        if let Err(e) = c.wav.finish() {
            self.error = Some(format!("{}: {e}", c.path.display()));
        }
        if keep {
            self.kept += 1;
            self.last = Some(c.path);
        } else {
            let _ = std::fs::remove_file(&c.path);
        }
    }

    /// The file being written, if any.
    pub fn recording(&self) -> Option<&Path> {
        self.current.as_ref().map(|c| c.path.as_path())
    }
}

/// `20261117-064501Z_446.00625MHz_analog.wav`.
fn file_name(t: SystemTime, hz: f64, decoder: &str) -> String {
    let s = t.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs()) as i64;
    let (y, mo, d) = civil(s.div_euclid(86_400));
    let r = s.rem_euclid(86_400);
    let mhz = format!("{:.5}", hz / 1e6);
    let mhz = mhz.trim_end_matches('0').trim_end_matches('.');
    let dec: String = decoder.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' { c } else { '_' }).collect();
    format!("{y:04}{mo:02}{d:02}-{:02}{:02}{:02}Z_{mhz}MHz_{dec}.wav", r / 3600, r / 60 % 60, r % 60)
}

/// `path`, or `path` with "-2", "-3" … before its extension if it exists:
/// two transmissions in the same second keep a file each.
fn unused(path: PathBuf) -> PathBuf {
    if !path.exists() {
        return path;
    }
    let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    (2..)
        .map(|n| path.with_file_name(format!("{stem}-{n}.wav")))
        .find(|p| !p.exists())
        .expect("a free name")
}

/// Days since 1970-01-01 to a date (Howard Hinnant's `civil_from_days`).
fn civil(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn names_say_when_where_and_what() {
        assert_eq!(civil(0), (1970, 1, 1));
        assert_eq!(civil(20_774), (2026, 11, 17));
        let t = UNIX_EPOCH + Duration::from_secs(20_774 * 86_400 + 6 * 3600 + 45 * 60 + 1);
        assert_eq!(file_name(t, 446.00625e6, "analog"), "20261117-064501Z_446.00625MHz_analog.wav");
        assert_eq!(file_name(t, 145.5e6, "dmr/x"), "20261117-064501Z_145.5MHz_dmr_x.wav");
    }

    #[test]
    fn one_file_a_transmission() {
        let dir = std::env::temp_dir().join(format!("declab-autorec-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let cfg = AutoRecord { dir: dir.clone(), ..AutoRecord::default() };
        let mut r = Recorder::default();
        let rate = 8000;
        let chunk = 800; // 0.1 s
        let tone: Vec<f32> = (0..chunk).map(|k| 0.3 * (k as f32 * 0.3).sin()).collect();
        let quiet = vec![0.0f32; chunk];
        let mut t = 0.0;
        // Feed `secs` of `x`, ticking as the worker does.
        let feed = |r: &mut Recorder, x: &[f32], secs: f64, t: &mut f64| {
            for _ in 0..(secs * 10.0).round() as usize {
                r.audio(&cfg, *t, 446.1e6, "analog", rate, 1, x);
                *t += 0.1;
                r.tick(&cfg, *t);
            }
        };
        // Silence: nothing. 3 s of sound with a 1 s pause: one file.
        feed(&mut r, &quiet, 2.0, &mut t);
        assert!(r.recording().is_none());
        feed(&mut r, &tone, 1.5, &mut t);
        feed(&mut r, &quiet, 1.0, &mut t);
        feed(&mut r, &tone, 1.5, &mut t);
        assert!(r.recording().is_some());
        feed(&mut r, &quiet, 2.5, &mut t);
        assert!(r.recording().is_none());
        assert_eq!(r.kept, 1);
        // A click: opened, then deleted.
        feed(&mut r, &tone, 0.2, &mut t);
        feed(&mut r, &quiet, 2.5, &mut t);
        assert_eq!(r.kept, 1);
        let files: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().collect();
        assert_eq!(files.len(), 1);
        // The one kept: 3 s of sound, the pause, and the hang until it closed.
        let len = files[0].metadata().unwrap().len();
        let secs = (len - 44) as f64 / (2.0 * rate as f64);
        assert!((5.0..=6.2).contains(&secs), "{secs} s");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
