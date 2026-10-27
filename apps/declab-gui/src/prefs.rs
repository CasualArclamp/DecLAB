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
