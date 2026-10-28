// made by CasualArclamp
//! Draws the app's icon: `cargo run -p declab-gui --example make_icon`
//! writes `assets/icon.ico` (16 to 256 px, PNG inside) and `assets/icon-256.png`
//! (the window icon, compiled into the app). After DecDAB's `make_icon`.
//!
//! A rounded near-black square holding a spectrum with the three kinds of
//! signal DecLAB receives side by side: a wide flat-topped TV multiplex
//! (blue), a narrower DAB block (green) and a peaked FM carrier (orange),
//! each tinted underneath, over a faint noise floor. Each size is drawn from
//! distance functions (not scaled down from the big one), so the trace is as
//! sharp at 16 px as at 256; below 48 px the floor is smooth.

use std::io::Cursor;

use image::{ImageFormat, Rgba, RgbaImage};

const SIZES: [u32; 7] = [16, 24, 32, 48, 64, 128, 256];

const BLUE: [f32; 3] = [80.0, 160.0, 255.0];
const GREEN: [f32; 3] = [90.0, 235.0, 120.0];
const ORANGE: [f32; 3] = [255.0, 170.0, 50.0];
const FLOOR: [f32; 3] = [110.0, 132.0, 122.0];
const TOP: [f32; 3] = [16.0, 22.0, 26.0];
const BOTTOM: [f32; 3] = [4.0, 7.0, 9.0];
const RIM: [f32; 3] = [52.0, 80.0, 64.0];

/// Where the trace runs, across the unit square.
const X0: f32 = 0.15;
const X1: f32 = 0.85;
/// The noise floor's height (down from the top).
const BASE: f32 = 0.71;

/// One signal: its centre and width (fractions of the icon), its height
/// above the floor, and whether it is flat-topped (OFDM) or peaked (FM).
struct Signal {
    centre: f32,
    width: f32,
    height: f32,
    flat: bool,
    colour: [f32; 3],
}

const SIGNALS: [Signal; 3] = [
    Signal {
        centre: 0.31,
        width: 0.20,
        height: 0.42,
        flat: true,
        colour: BLUE,
    },
    Signal {
        centre: 0.565,
        width: 0.11,
        height: 0.30,
        flat: true,
        colour: GREEN,
    },
    Signal {
        centre: 0.745,
        width: 0.055,
        height: 0.47,
        flat: false,
        colour: ORANGE,
    },
];

impl Signal {
    /// Its height at `x`: a flat top with soft (logistic) skirts, or a
    /// Gaussian peak.
    fn at(&self, x: f32, edge: f32) -> f32 {
        if self.flat {
            let lo = 1.0 + (-(x - (self.centre - self.width / 2.0)) / edge).exp();
            let hi = 1.0 + ((x - (self.centre + self.width / 2.0)) / edge).exp();
            self.height / (lo * hi)
        } else {
            let t = (x - self.centre) / (self.width / 2.2);
            self.height * (-t * t).exp()
        }
    }
}

/// The noise floor's ripple: a few fixed sines, so the icon is the same every
/// time (and smooth when `rough` is false).
fn ripple(x: f32, rough: bool) -> f32 {
    if !rough {
        return 0.0;
    }
    0.005 * (x * 97.0).sin() + 0.004 * (x * 173.0 + 1.3).sin() + 0.003 * (x * 281.0 + 0.4).sin()
}

/// The trace as a polyline in pixels, each point with the colour of the
/// signal it belongs to (the floor's where none stands out).
fn trace(size: u32) -> Vec<(f32, f32, [f32; 3])> {
    let s = size as f32;
    let rough = size >= 48;
    // Skirts no sharper than about a pixel, so small sizes stay smooth.
    let edge = (0.012f32).max(0.8 / s);
    let n = (size * 3).max(120);
    (0..=n)
        .map(|i| {
            let x = X0 + (X1 - X0) * i as f32 / n as f32;
            let parts: Vec<f32> = SIGNALS.iter().map(|g| g.at(x, edge)).collect();
            let sum: f32 = parts.iter().sum();
            // The ripple only on the floor: the signals' tops stay clean.
            let y = BASE + ripple(x, rough) * (1.0 - sum / 0.03).max(0.0) - sum;
            let (k, best) = parts
                .iter()
                .enumerate()
                .fold((0, 0.0f32), |a, (k, &p)| if p > a.1 { (k, p) } else { a });
            let colour = if best > 0.03 {
                SIGNALS[k].colour
            } else {
                FLOOR
            };
            (x * s, y * s, colour)
        })
        .collect()
}

/// Coverage of a shape whose signed distance (in pixels, negative inside) is
/// `d`: one pixel of anti-aliasing across the edge.
fn cover(d: f32) -> f32 {
    (0.5 - d).clamp(0.0, 1.0)
}

fn mix(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// Distance from (px, py) to the segment a–b, and how far along it (0–1).
fn segment(px: f32, py: f32, a: (f32, f32), b: (f32, f32)) -> (f32, f32) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((px - a.0) * dx + (py - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    ((px - a.0 - t * dx).hypot(py - a.1 - t * dy), t)
}

fn render(size: u32) -> RgbaImage {
    let s = size as f32;
    let mut img = RgbaImage::new(size, size);
    // The square: half-side and corner radius, as fractions of the size.
    let (half, radius) = (0.47, 0.21);
    let line = trace(size);
    // The trace's half-width: a fraction of the icon, never under 0.7 px.
    let w = (0.011 * s).max(0.7);
    let edge = (0.012f32).max(0.8 / s);
    for (x, y, px) in img.enumerate_pixels_mut() {
        let (u, v) = ((x as f32 + 0.5) / s, (y as f32 + 0.5) / s);
        // Signed distance to the rounded square, in pixels.
        let (qx, qy) = (
            (u - 0.5).abs() - (half - radius),
            (v - 0.5).abs() - (half - radius),
        );
        let sd = ((qx.max(0.0)).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - radius) * s;
        let alpha = cover(sd);
        if alpha <= 0.0 {
            *px = Rgba([0, 0, 0, 0]);
            continue;
        }
        // Diagonal gradient, a thin lighter rim just inside the edge.
        let mut c = mix(TOP, BOTTOM, ((u + v) * 0.5).clamp(0.0, 1.0));
        let rim = (1.0 - (-sd / (0.018 * s).max(1.0)).clamp(0.0, 1.0)) * 0.55;
        c = mix(c, RIM, rim);
        // Each signal's tint, from its own curve down to the floor.
        if (X0..=X1).contains(&u) {
            let floor = BASE;
            for g in &SIGNALS {
                let h = g.at(u, edge);
                if h > 0.004 {
                    // Inside: between the floor and this signal's own height.
                    let d = ((floor - h) - v).max(v - floor) * s;
                    c = mix(c, g.colour, cover(d) * 0.30);
                }
            }
        }
        // The trace: the nearest segment within reach sets the colour.
        let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
        let mut best = (f32::MAX, FLOOR);
        for pair in line.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if fx < a.0.min(b.0) - w - 1.0 || fx > a.0.max(b.0) + w + 1.0 {
                continue;
            }
            let (d, t) = segment(fx, fy, (a.0, a.1), (b.0, b.1));
            if d < best.0 {
                best = (d, if t < 0.5 { a.2 } else { b.2 });
            }
        }
        c = mix(c, best.1, cover(best.0 - w));
        *px = Rgba([
            c[0].round() as u8,
            c[1].round() as u8,
            c[2].round() as u8,
            (alpha * 255.0).round() as u8,
        ]);
    }
    img
}

fn png(img: &RgbaImage) -> Vec<u8> {
    let mut out = Cursor::new(Vec::new());
    img.write_to(&mut out, ImageFormat::Png).expect("PNG");
    out.into_inner()
}

/// An .ico holding each image as a PNG (Windows Vista and later).
fn ico(images: &[(u32, Vec<u8>)]) -> Vec<u8> {
    let mut out = vec![0, 0, 1, 0];
    out.extend((images.len() as u16).to_le_bytes());
    let mut offset = 6 + 16 * images.len() as u32;
    for (size, data) in images {
        // A width of 0 means 256.
        let b = if *size >= 256 { 0 } else { *size as u8 };
        out.extend([b, b, 0, 0, 1, 0, 32, 0]);
        out.extend((data.len() as u32).to_le_bytes());
        out.extend(offset.to_le_bytes());
        offset += data.len() as u32;
    }
    for (_, data) in images {
        out.extend(data);
    }
    out
}

fn main() -> std::io::Result<()> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
    std::fs::create_dir_all(&dir)?;
    let images: Vec<(u32, Vec<u8>)> = SIZES.iter().map(|&s| (s, png(&render(s)))).collect();
    std::fs::write(dir.join("icon.ico"), ico(&images))?;
    let big = &images.last().expect("sizes").1;
    std::fs::write(dir.join("icon-256.png"), big)?;
    // A contact sheet for a look: every size on a grey ground.
    let mut sheet = RgbaImage::from_pixel(680, 272, Rgba([58, 58, 62, 255]));
    let mut x = 8;
    for s in [256, 128, 64, 48, 32, 24, 16] {
        image::imageops::overlay(&mut sheet, &render(s), x as i64, 8);
        x += s as i32 + 12;
    }
    let out = dir.join("..").join("..").join("..").join("out");
    std::fs::create_dir_all(&out)?;
    sheet.save(out.join("icon-preview.png")).ok();
    println!("wrote {}", dir.display());
    Ok(())
}
