// made by CasualArclamp
//! The DAB+ audio decoder: libxaac (Apache-2.0, `decsat-xaac-sys`), which
//! sits with DecLAB's GPL where DecDAB's FDK-AAC did not. The libxaac
//! wrapper is DecSAT's (decsat-audio/src/xaac.rs).
//!
//! libxaac runs in its MP4 (raw) mode: the superframe's AudioSpecificConfig
//! first, which signals SBR and PS explicitly and DAB+'s 960-sample frames
//! (TS 102 563 §5.1, frameLengthFlag), then each access unit bare. (Wrapped
//! in ADTS, as DecSAT does, libxaac decoded 1024-sample frames whatever its
//! frame-length flag said: wrong for DAB+.)

use std::alloc::{Layout, alloc_zeroed, dealloc};
use std::ffi::c_void;
use std::ptr::NonNull;

use decsat_xaac_sys as x;

use crate::{CodecError, Pcm};

/// One block of memory handed to libxaac, freed with the decoder.
struct Mem {
    ptr: NonNull<u8>,
    layout: Layout,
}

impl Mem {
    fn new(size: usize, align: usize) -> Result<Mem, CodecError> {
        let align = align.max(16).next_power_of_two();
        let layout =
            Layout::from_size_align(size.max(1), align).map_err(|_| CodecError::Bad("layout"))?;
        // SAFETY: the layout has a non-zero size.
        let p = unsafe { alloc_zeroed(layout) };
        NonNull::new(p)
            .map(|ptr| Mem { ptr, layout })
            .ok_or(CodecError::Bad("libxaac: out of memory"))
    }
}

impl Drop for Mem {
    fn drop(&mut self) {
        // SAFETY: allocated in `Mem::new` with this layout, freed only here.
        unsafe { dealloc(self.ptr.as_ptr(), self.layout) }
    }
}

fn check(what: &'static str, code: x::IaErrorCode) -> Result<x::IaErrorCode, CodecError> {
    if code & x::IA_FATAL_ERROR != 0 {
        Err(CodecError::Xaac(code, what))
    } else {
        Ok(code)
    }
}

/// libxaac fed an AudioSpecificConfig, then raw access units.
struct Xaac {
    api: Mem,
    /// The memory tables and memories (kept alive; libxaac holds pointers).
    _mems: Vec<Mem>,
    input: (NonNull<u8>, usize),
    output: NonNull<u8>,
    /// The stream's header has been read (INIT_PROCESS done).
    ready: bool,
    pending: Vec<u8>,
    rate: u32,
    channels: u32,
}

// SAFETY: the decoder's memory belongs to this value alone and libxaac keeps
// no per-thread or shared state for it, so it may move between threads.
unsafe impl Send for Xaac {}

impl Xaac {
    fn new() -> Result<Xaac, CodecError> {
        let mut size: i32 = 0;
        // SAFETY: GET_API_SIZE takes no object and writes one 32-bit value.
        check("API size", unsafe {
            x::ixheaacd_dec_api(
                std::ptr::null_mut(),
                x::IA_API_CMD_GET_API_SIZE,
                0,
                (&mut size as *mut i32).cast(),
            )
        })?;
        let api = Mem::new(size as usize, 16)?;
        let mut d = Xaac {
            input: (api.ptr, 0),
            output: api.ptr,
            api,
            _mems: Vec::new(),
            ready: false,
            pending: Vec::new(),
            rate: 0,
            channels: 0,
        };
        d.call(
            "pre-config",
            x::IA_API_CMD_INIT,
            x::IA_CMD_TYPE_INIT_API_PRE_CONFIG_PARAMS,
            std::ptr::null_mut(),
        )?;
        d.set(x::IA_XHEAAC_DEC_CONFIG_PARAM_MP4FLAG, 1)?; // raw, after an ASC
        d.set(x::IA_XHEAAC_DEC_CONFIG_PARAM_FRAMELENGTH_FLAG, 1)?; // 960
        let tabs = d.get_cmd(x::IA_API_CMD_GET_MEMTABS_SIZE, 0)?;
        let tabs = Mem::new(tabs as usize, 16)?;
        d.call(
            "memory tables",
            x::IA_API_CMD_SET_MEMTABS_PTR,
            0,
            tabs.ptr.as_ptr().cast(),
        )?;
        d._mems.push(tabs);
        d.call(
            "post-config",
            x::IA_API_CMD_INIT,
            x::IA_CMD_TYPE_INIT_API_POST_CONFIG_PARAMS,
            std::ptr::null_mut(),
        )?;
        let n = d.get_cmd(x::IA_API_CMD_GET_N_MEMTABS, 0)?;
        for i in 0..n {
            let size = d.get_cmd(x::IA_API_CMD_GET_MEM_INFO_SIZE, i)?;
            let align = d.get_cmd(x::IA_API_CMD_GET_MEM_INFO_ALIGNMENT, i)?;
            let kind = d.get_cmd(x::IA_API_CMD_GET_MEM_INFO_TYPE, i)?;
            let m = Mem::new(size as usize, align as usize)?;
            d.call("memory", x::IA_API_CMD_SET_MEM_PTR, i, m.ptr.as_ptr().cast())?;
            match kind {
                x::IA_MEMTYPE_INPUT => d.input = (m.ptr, size as usize),
                x::IA_MEMTYPE_OUTPUT => d.output = m.ptr,
                _ => {}
            }
            d._mems.push(m);
        }
        if d.input.1 == 0 {
            return Err(CodecError::Bad("libxaac: no input buffer"));
        }
        Ok(d)
    }

    fn call(
        &mut self,
        what: &'static str,
        cmd: i32,
        idx: i32,
        value: *mut c_void,
    ) -> Result<i32, CodecError> {
        // SAFETY: the object is live and `value` points where `cmd` expects
        // (a 32-bit value, or memory of the size libxaac asked for).
        check(what, unsafe {
            x::ixheaacd_dec_api(self.api.ptr.as_ptr().cast(), cmd, idx, value)
        })
    }

    fn get_cmd(&mut self, cmd: i32, idx: i32) -> Result<i32, CodecError> {
        let mut v: i32 = 0;
        self.call("query", cmd, idx, (&mut v as *mut i32).cast())?;
        Ok(v)
    }

    fn set(&mut self, param: i32, mut v: i32) -> Result<(), CodecError> {
        self.call(
            "configuration",
            x::IA_API_CMD_SET_CONFIG_PARAM,
            param,
            (&mut v as *mut i32).cast(),
        )
        .map(|_| ())
    }

    fn load(&mut self) -> Result<(), CodecError> {
        let n = self.pending.len().min(self.input.1);
        // SAFETY: the input buffer holds `input.1` bytes; `n` is no more.
        unsafe {
            std::ptr::copy_nonoverlapping(self.pending.as_ptr(), self.input.0.as_ptr(), n);
        }
        let mut n = n as i32;
        self.call(
            "input",
            x::IA_API_CMD_SET_INPUT_BYTES,
            0,
            (&mut n as *mut i32).cast(),
        )?;
        Ok(())
    }

    /// Drop what libxaac used of the pending bytes (all of them if it used
    /// none: they cannot be read and must not pile up).
    fn used(&mut self) -> Result<(), CodecError> {
        let used = self.get_cmd(x::IA_API_CMD_GET_CURIDX_INPUT_BUF, 0)?.max(0) as usize;
        if used == 0 {
            self.pending.clear();
        } else {
            self.pending.drain(..used.min(self.pending.len()));
        }
        Ok(())
    }

    fn query(&mut self) -> Result<(), CodecError> {
        self.rate = self
            .get_cmd(x::IA_API_CMD_GET_CONFIG_PARAM, x::IA_XHEAAC_DEC_CONFIG_PARAM_SAMP_FREQ)?
            .max(0) as u32;
        self.channels = self
            .get_cmd(x::IA_API_CMD_GET_CONFIG_PARAM, x::IA_XHEAAC_DEC_CONFIG_PARAM_NUM_CHANNELS)?
            .max(0) as u32;
        Ok(())
    }

    /// One step of INIT_PROCESS on `bytes` (the AudioSpecificConfig, then
    /// access units until libxaac says it is ready, as its test app does).
    fn init_step(&mut self, bytes: &[u8]) -> Result<(), CodecError> {
        self.pending.clear();
        self.pending.extend_from_slice(bytes);
        self.load()?;
        self.call(
            "header",
            x::IA_API_CMD_INIT,
            x::IA_CMD_TYPE_INIT_PROCESS,
            std::ptr::null_mut(),
        )?;
        let done = self.get_cmd(x::IA_API_CMD_INIT, x::IA_CMD_TYPE_INIT_DONE_QUERY)?;
        self.pending.clear();
        if done != 0 {
            self.ready = true;
            self.query()?;
        }
        Ok(())
    }

    /// One raw access unit; its audio, interleaved, into `out` (empty while
    /// libxaac is still initialising).
    fn decode(&mut self, frame: &[u8], out: &mut Vec<i16>) -> Result<(), CodecError> {
        out.clear();
        if !self.ready {
            return self.init_step(frame);
        }
        self.pending.extend_from_slice(frame);
        while !self.pending.is_empty() {
            self.load()?;
            self.call(
                "decode",
                x::IA_API_CMD_EXECUTE,
                x::IA_CMD_TYPE_DO_EXECUTE,
                std::ptr::null_mut(),
            )?;
            self.used()?;
            let bytes = self.get_cmd(x::IA_API_CMD_GET_OUTPUT_BYTES, 0)?.max(0) as usize;
            self.query()?;
            if self.channels == 0 || bytes == 0 {
                continue;
            }
            // SAFETY: libxaac wrote `bytes` bytes of 16-bit PCM to its
            // output buffer, which is aligned for i16.
            let pcm = unsafe {
                std::slice::from_raw_parts(self.output.as_ptr().cast::<i16>(), bytes / 2)
            };
            out.extend_from_slice(pcm);
        }
        Ok(())
    }
}

impl Drop for Xaac {
    fn drop(&mut self) {
        // Tell libxaac the input is over before its memory goes.
        let _ = self.call("end", x::IA_API_CMD_INPUT_OVER, 0, std::ptr::null_mut());
    }
}

pub struct AacDecoder {
    dec: Xaac,
    asc: Vec<u8>,
    out: Vec<i16>,
    /// The last frame's shape, for silence in place of a lost one.
    last: (u32, u16, usize),
}

impl AacDecoder {
    /// A decoder for the stream an AudioSpecificConfig describes.
    pub fn new(asc: &[u8]) -> Result<Self, CodecError> {
        let mut dec = Xaac::new()?;
        dec.init_step(asc)?;
        Ok(AacDecoder {
            dec,
            asc: asc.to_vec(),
            out: Vec::with_capacity(8192),
            last: (0, 2, 0),
        })
    }

    /// The configuration it was opened with.
    pub fn asc(&self) -> &[u8] {
        &self.asc
    }

    /// Decode one access unit; `None` (after a failed CRC) conceals with
    /// a frame of silence. Always stereo out (mono doubled).
    pub fn decode(&mut self, au: Option<&[u8]>) -> Result<Pcm, CodecError> {
        let (rate, channels, n) = self.last;
        let silence = |n: usize| Pcm {
            rate,
            channels,
            samples: vec![0; n],
            concealed: true,
        };
        let Some(au) = au.filter(|a| !a.is_empty()) else {
            return Ok(silence(n));
        };
        if let Err(e) = self.dec.decode(au, &mut self.out) {
            // A fatal error leaves libxaac unusable: start afresh.
            let mut dec = Xaac::new()?;
            dec.init_step(&self.asc)?;
            self.dec = dec;
            return Err(e);
        }
        if self.out.is_empty() || self.dec.channels == 0 {
            return Ok(silence(n));
        }
        let samples = if self.dec.channels == 1 {
            self.out.iter().flat_map(|&s| [s, s]).collect()
        } else {
            self.out.clone()
        };
        self.last = (self.dec.rate, 2, samples.len());
        Ok(Pcm {
            rate: self.dec.rate,
            channels: 2,
            samples,
            concealed: false,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// libxaac takes DAB+'s AudioSpecificConfigs (as `AudioParams` builds
    /// them: 960-sample frames, SBR/PS signalled explicitly) and conceals a
    /// lost unit before the first with nothing.
    #[test]
    fn opens_for_dab_plus_configs() {
        // AAC-LC, 48 kHz stereo: AOT 2, index 3, 2 channels, frameLength 1.
        // HE-AAC v2 over a 16 kHz mono core: AOT 29, index 8, 1 channel,
        // extension index 5 (32 kHz), AOT 2, frameLength 1.
        for asc in [&[0x11u8, 0x94][..], &[0xec, 0x0a, 0x8a, 0x00][..]] {
            let mut d = AacDecoder::new(asc).expect("libxaac opens");
            assert_eq!(d.asc(), asc);
            let p = d.decode(None).expect("conceals");
            assert!(p.concealed && p.samples.is_empty());
        }
    }
}
