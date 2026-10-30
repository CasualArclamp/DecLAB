// made by CasualArclamp
//! TETRA speech (TCH/S): the ACELP codec of ETSI EN 300 395-2 with its
//! speech channel decoding, loaded at run time from `tetra_acelp.dll`.
//!
//! The codec is a port of ETSI's reference C code, which ETSI licenses on
//! its own terms, so it is kept out of this GPL tree in a separate library.
//! Without it TETRA is decoded in full except that speech is not played.
//!
//! The library's C ABI:
//!
//! ```c
//! void *tetra_acelp_new(void);
//! void  tetra_acelp_free(void *ctx);
//! /* `n` soft type-4 bits (LLRs, positive = 0) of a TCH/S slot: 432 for a
//!  * full slot, 216 for block 2 alone (the first half stolen). Writes two
//!  * 30 ms frames of 8 kHz speech (480 samples, the first concealed when
//!  * stolen). Returns the bad-frame flags: bit 0 frame 1, bit 1 frame 2. */
//! int   tetra_acelp_decode(void *ctx, const float *llr, int n, short *pcm);
//! ```
//!
//! Searched for next to the executable, then in the working directory, or
//! at the path in `DECLAB_ACELP`.

/// Samples of 8 kHz speech a slot decodes to (two 30 ms frames).
pub const SLOT_SAMPLES: usize = 480;

#[cfg(windows)]
mod sys {
    use std::ffi::c_void;

    #[link(name = "kernel32")]
    unsafe extern "system" {
        pub fn LoadLibraryW(name: *const u16) -> *mut c_void;
        pub fn GetProcAddress(module: *mut c_void, name: *const u8) -> *mut c_void;
    }
}

type NewFn = unsafe extern "C" fn() -> *mut std::ffi::c_void;
type FreeFn = unsafe extern "C" fn(*mut std::ffi::c_void);
type DecodeFn = unsafe extern "C" fn(*mut std::ffi::c_void, *const f32, i32, *mut i16) -> i32;

/// One decoder instance (the codec keeps state between frames).
pub struct Acelp {
    ctx: *mut std::ffi::c_void,
    free: FreeFn,
    decode: DecodeFn,
}

// Rust note: a raw pointer makes a type neither `Send` nor `Sync` by
// default. The instance is used from one thread at a time (the decoder that
// owns it), which is what `Send` promises.
unsafe impl Send for Acelp {}

impl Acelp {
    /// Load the library and make an instance, or say why not.
    pub fn load() -> Result<Acelp, String> {
        #[cfg(windows)]
        {
            let mut tried = Vec::new();
            let mut paths = Vec::new();
            if let Ok(p) = std::env::var("DECLAB_ACELP") {
                paths.push(std::path::PathBuf::from(p));
            }
            if let Ok(exe) = std::env::current_exe()
                && let Some(dir) = exe.parent()
            {
                paths.push(dir.join("tetra_acelp.dll"));
            }
            paths.push(std::path::PathBuf::from("tetra_acelp.dll"));
            for p in paths {
                if !p.exists() {
                    tried.push(p.display().to_string());
                    continue;
                }
                use std::os::windows::ffi::OsStrExt;
                let wide: Vec<u16> = p.as_os_str().encode_wide().chain([0]).collect();
                // SAFETY: a NUL-terminated wide path; the module is never
                // unloaded, so the function pointers stay valid.
                unsafe {
                    let m = sys::LoadLibraryW(wide.as_ptr());
                    if m.is_null() {
                        return Err(format!("{} would not load", p.display()));
                    }
                    let sym = |name: &[u8]| sys::GetProcAddress(m, name.as_ptr());
                    let (n, f, d) = (
                        sym(b"tetra_acelp_new\0"),
                        sym(b"tetra_acelp_free\0"),
                        sym(b"tetra_acelp_decode\0"),
                    );
                    if n.is_null() || f.is_null() || d.is_null() {
                        return Err(format!("{} lacks the tetra_acelp_* functions", p.display()));
                    }
                    let new: NewFn = std::mem::transmute::<*mut std::ffi::c_void, NewFn>(n);
                    let ctx = new();
                    if ctx.is_null() {
                        return Err("tetra_acelp_new failed".into());
                    }
                    return Ok(Acelp {
                        ctx,
                        free: std::mem::transmute::<*mut std::ffi::c_void, FreeFn>(f),
                        decode: std::mem::transmute::<*mut std::ffi::c_void, DecodeFn>(d),
                    });
                }
            }
            Err(format!(
                "tetra_acelp.dll not found (looked at {})",
                tried.join(", ")
            ))
        }
        #[cfg(not(windows))]
        {
            Err("the TETRA speech library is only loaded on Windows".into())
        }
    }

    /// Decode a slot's soft type-4 bits (432, or 216 when the first half
    /// was stolen) to 480 samples of 8 kHz speech in ±1. Returns the
    /// bad-frame flags (bit 0, bit 1).
    pub fn decode(&mut self, llr: &[f32], out: &mut [f32; SLOT_SAMPLES]) -> u32 {
        let mut pcm = [0i16; SLOT_SAMPLES];
        let n = if llr.len() >= 432 { 432 } else { 216 };
        assert!(llr.len() >= n, "a TCH/S slot is 432 or 216 soft bits");
        // SAFETY: `llr` holds `n` floats and `pcm` 480 shorts, as the ABI
        // above requires; `ctx` came from this library's `new`.
        let bfi = unsafe { (self.decode)(self.ctx, llr.as_ptr(), n as i32, pcm.as_mut_ptr()) };
        for (o, &s) in out.iter_mut().zip(&pcm) {
            *o = f32::from(s) / 32768.0;
        }
        bfi as u32
    }
}

impl Drop for Acelp {
    fn drop(&mut self) {
        // SAFETY: `ctx` came from `new` and is freed once.
        unsafe { (self.free)(self.ctx) };
    }
}
