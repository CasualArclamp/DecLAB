// made by CasualArclamp
//! Sound out: the [`Audio`](declab_api::Audio) a decoder hands back, to the
//! default sound device. The cpal output follows DecSAT's player
//! (`decsat-audio::player`, same author).
//!
//! Decoders send audio at whatever rate suits them (48 kHz analog, DAB's
//! 48/32/24 kHz); it is resampled here, linearly, to the device's rate and
//! kept in a ring the device's callback empties. The ring primes with
//! [`START_MS`] before playing, and is cut back to [`MAX_MS`] if a source
//! runs ahead of the sound card (a file played faster than real time).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};

/// Audio held before playing starts, ms.
const START_MS: u32 = 150;
/// Most audio held: past this the oldest is dropped, ms.
const MAX_MS: u32 = 600;

#[derive(Default)]
struct Ring {
    /// Interleaved stereo at the device's rate.
    buf: VecDeque<f32>,
    primed: bool,
}

/// Linear resampling state for one input rate.
#[derive(Default)]
struct Resample {
    in_rate: u32,
    /// Position of the next output between `last` and the next input frame.
    frac: f64,
    last: [f32; 2],
}

/// The sound output. Dropping it stops the sound.
pub struct AudioOut {
    ring: Arc<Mutex<Ring>>,
    /// Volume, 0..1, as f32 bits (an atomic float).
    volume: Arc<AtomicU32>,
    rate: u32,
    resample: Resample,
    name: String,
    /// Held so the device keeps playing.
    _stream: cpal::Stream,
}

impl AudioOut {
    /// Open the default output device.
    pub fn open() -> Result<AudioOut, String> {
        let host = cpal::default_host();
        let dev = host.default_output_device().ok_or("no sound output device")?;
        let name = dev
            .description()
            .map(|d| d.name().to_string())
            .unwrap_or_else(|_| "sound output".into());
        let cfg = dev.default_output_config().map_err(|e| e.to_string())?;
        let (rate, ch, fmt) = (cfg.sample_rate(), cfg.channels() as usize, cfg.sample_format());
        let config: cpal::StreamConfig = cfg.into();
        let ring = Arc::new(Mutex::new(Ring::default()));
        let volume = Arc::new(AtomicU32::new(0.8f32.to_bits()));
        let start = (rate * START_MS / 1000) as usize * 2;
        let err = |e| eprintln!("audio output: {e}");
        // Rust note: `callback::<f32>` picks the generic function's sample
        // type; each arm builds the same closure for a different format.
        let (r, v) = (ring.clone(), volume.clone());
        let stream = match fmt {
            cpal::SampleFormat::F32 => {
                dev.build_output_stream(config, callback::<f32>(r, v, ch, start), err, None)
            }
            cpal::SampleFormat::I16 => {
                dev.build_output_stream(config, callback::<i16>(r, v, ch, start), err, None)
            }
            cpal::SampleFormat::I32 => {
                dev.build_output_stream(config, callback::<i32>(r, v, ch, start), err, None)
            }
            f => return Err(format!("sound device format {f} is not handled")),
        }
        .map_err(|e| e.to_string())?;
        stream.play().map_err(|e| e.to_string())?;
        Ok(AudioOut {
            ring,
            volume,
            rate,
            resample: Resample::default(),
            name,
            _stream: stream,
        })
    }

    pub fn describe(&self) -> String {
        format!("{} · {} kHz", self.name, self.rate as f64 / 1000.0)
    }

    pub fn set_volume(&self, v: f32) {
        self.volume.store(v.clamp(0.0, 1.0).to_bits(), Ordering::Relaxed);
    }

    /// Queue `samples` (interleaved, `channels` of them) at `rate`.
    pub fn push(&mut self, rate: u32, channels: u16, samples: &[f32]) {
        let ch = channels.max(1) as usize;
        let rs = &mut self.resample;
        if rs.in_rate != rate {
            *rs = Resample { in_rate: rate, ..Resample::default() };
        }
        let step = rate as f64 / self.rate as f64;
        let mut out = Vec::with_capacity((samples.len() / ch) * 2 * self.rate as usize / rate.max(1) as usize + 4);
        for frame in samples.chunks_exact(ch) {
            let cur = [frame[0], frame[if ch > 1 { 1 } else { 0 }]];
            // Outputs falling between the previous input frame and this one.
            while rs.frac < 1.0 {
                let t = rs.frac as f32;
                out.push(rs.last[0] + t * (cur[0] - rs.last[0]));
                out.push(rs.last[1] + t * (cur[1] - rs.last[1]));
                rs.frac += step;
            }
            rs.frac -= 1.0;
            rs.last = cur;
        }
        let mut ring = self.ring.lock().unwrap();
        ring.buf.extend(out);
        let max = (self.rate * MAX_MS / 1000) as usize * 2;
        if ring.buf.len() > max {
            let extra = ring.buf.len() - max;
            ring.buf.drain(..extra);
        }
    }

    /// Drop what is queued (a new station).
    pub fn clear(&mut self) {
        let mut ring = self.ring.lock().unwrap();
        ring.buf.clear();
        ring.primed = false;
    }
}

/// The sound card's callback: stereo frames from the ring to `ch` channels.
fn callback<T>(
    ring: Arc<Mutex<Ring>>,
    volume: Arc<AtomicU32>,
    ch: usize,
    start: usize,
) -> impl FnMut(&mut [T], &cpal::OutputCallbackInfo) + Send + 'static
where
    T: cpal::SizedSample + cpal::FromSample<f32>,
{
    move |data: &mut [T], _| {
        let g = f32::from_bits(volume.load(Ordering::Relaxed)).powi(3);
        let mut ring = ring.lock().unwrap();
        if !ring.primed && ring.buf.len() >= start {
            ring.primed = true;
        }
        for frame in data.chunks_mut(ch.max(1)) {
            let (l, r) = if ring.primed {
                match (ring.buf.pop_front(), ring.buf.pop_front()) {
                    (Some(l), Some(r)) => (l * g, r * g),
                    _ => {
                        ring.primed = false;
                        (0.0, 0.0)
                    }
                }
            } else {
                (0.0, 0.0)
            };
            for (c, s) in frame.iter_mut().enumerate() {
                let v = match (ch, c) {
                    (1, _) => 0.5 * (l + r),
                    (_, 0) => l,
                    (_, 1) => r,
                    _ => 0.0,
                };
                *s = T::from_sample(v);
            }
        }
    }
}
