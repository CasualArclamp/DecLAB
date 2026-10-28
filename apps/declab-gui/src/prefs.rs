// made by CasualArclamp
//! What the app remembers between runs: a few `key=value` lines in
//! `%APPDATA%\DecLAB\prefs.txt` (`~/.config/declab/prefs.txt` elsewhere).
//! Nothing here is needed to run: a file that cannot be read or written is
//! the same as an empty one. From DecDVB-T (same author).

use std::path::PathBuf;

fn file() -> Option<PathBuf> {
    let base = if cfg!(windows) {
        std::env::var_os("APPDATA").map(|d| PathBuf::from(d).join("DecLAB"))
    } else {
        std::env::var_os("HOME").map(|d| PathBuf::from(d).join(".config").join("declab"))
    }?;
    Some(base.join("prefs.txt"))
}

/// Every key and value, read once at start.
pub fn load() -> Vec<(String, String)> {
    let Some(text) = file().and_then(|f| std::fs::read_to_string(f).ok()) else {
        return Vec::new();
    };
    text.lines()
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| (k.to_string(), v.trim().to_string()))
        .collect()
}

/// Write these keys (an empty value removes one), keeping the others.
pub fn set_many(pairs: &[(&str, String)]) {
    let Some(path) = file() else { return };
    let old = std::fs::read_to_string(&path).unwrap_or_default();
    let mut lines: Vec<String> = old
        .lines()
        .filter(|l| l.split_once('=').is_none_or(|(k, _)| !pairs.iter().any(|(key, _)| k == *key)))
        .map(String::from)
        .collect();
    for (key, value) in pairs {
        if !value.is_empty() {
            lines.push(format!("{key}={value}"));
        }
    }
    if let Some(dir) = path.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let _ = std::fs::write(path, lines.join("\n") + "\n");
}

/// A loaded value by key.
pub fn get<'a>(all: &'a [(String, String)], key: &str) -> Option<&'a str> {
    all.iter().rev().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

pub fn num<T: std::str::FromStr>(all: &[(String, String)], key: &str) -> Option<T> {
    get(all, key)?.parse().ok()
}

// The player's settings (its window was ported from DecDVB-T and reads
// them one at a time).

fn read_one(key: &str) -> Option<String> {
    get(&load(), key).map(String::from)
}

fn flag(key: &str, default: bool) -> bool {
    read_one(key).map_or(default, |v| v == "1")
}

pub fn volume() -> f32 {
    read_one("volume").and_then(|v| v.parse().ok()).map_or(0.8, |v: f32| v.clamp(0.0, 1.0))
}

pub fn set_volume(v: f32) {
    set_many(&[("volume", format!("{v:.2}"))]);
}

/// Bob deinterlacing (50 fields a second) rather than one frame of two.
pub fn bob() -> bool {
    flag("bob", false)
}

pub fn set_bob(on: bool) {
    set_many(&[("bob", if on { "1" } else { "0" }.to_string())]);
}

/// Video decoded on the graphics card.
pub fn gpu() -> bool {
    flag("gpu", false)
}

pub fn set_gpu(on: bool) {
    set_many(&[("gpu", if on { "1" } else { "0" }.to_string())]);
}

pub fn ffmpeg_path() -> Option<PathBuf> {
    read_one("ffmpeg").filter(|s| !s.is_empty()).map(PathBuf::from)
}

pub fn set_ffmpeg_path(p: Option<&std::path::Path>) {
    set_many(&[("ffmpeg", p.map_or(String::new(), |p| p.display().to_string()))]);
}
