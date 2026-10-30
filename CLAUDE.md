<!-- made by CasualArclamp -->
# DecLAB — working notes

Read [`docs/DESIGN.md`](docs/DESIGN.md) first: the agreed scope, the plug-in
decoder API and the milestones. Update it when a decision changes.

DecLAB brings together the DVB modes of DecDVB-T (`..\DecDVB-T`), DecSAT
(`..\DecDVB`) and DecDAB (`..\DecDAB`). Those projects stay where they are;
code is copied in, not linked.

## Commands

```bash
cargo check --workspace --all-targets
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo run --release -p declab-gui
cargo run --release -p declab-gui -- out/sim_fm_98.000MHz_10Msps.cs8 --decoder wfm --chan 98.8M --screenshot out/x.png --after 4
cargo run --release -p declab-gui -- --radio airspy --freq 98M --decoder wfm --wheel   # opens with the decoder wheel up
cargo run --release -p declab-gui -- --radio airspy --freq 546M --chan 546M --decoder dvbt --optimise --screenshot out/x.png --after 90   # Airspy gain optimiser (also "Optimise gain" in the gain menu); logs each setting to stderr
cargo run --release -p declab-gui -- ..\DecPagers\out\sim_pagers_153.000MHz_1Msps.cs8 --decoder pagers --chan 153.15M --screenshot out/x.png --after 10   # pagers (make the file with `decpagers sim`)
cargo run -p declab-gui --example make_icon   # redraws apps/declab-gui/assets/icon.ico and icon-256.png (preview out/icon-preview.png)
cargo run --release -p declab-pmr --example dmr_make -- out/sim_dmr.cf32 15   # simulated DMR repeater: call, Tier III CSBKs, LRRP
cargo run --release -p declab-gui -- out/sim_dmr.cf32 --rate 48000 --decoder voice --screenshot out/x.png --after 8   # Digital voice: DMR, NXDN48, NXDN96, P25, TETRA detected by itself
cargo run --release -p declab-pmr --example dmr_sweep -- 4 800   # DMR sensitivity vs C/N (and CPU)
cargo run --release -p declab-pmr --example nxdn_make -- out/sim_nxdn.cf32 12 [96]   # simulated NXDN48 (or NXDN96) calls
cargo run --release -p declab-gui -- out/sim_nxdn.cf32 --rate 48000 --decoder voice --screenshot out/x.png --after 6
cargo run --release -p declab-pmr --example nxdn_sweep -- 4 800 - [96]   # NXDN48 (or 96) sensitivity, SNR gate on noise, CPU
cargo run --release -p declab-pmr --example p25_make -- out/sim_p25.cf32 14   # simulated P25 call and control channel (then --decoder voice)
cargo run --release -p declab-pmr --example p25_sweep -- 4 300   # P25 sensitivity, SNR gate on noise, CPU
cargo run --release -p declab-pmr --example tetra_make -- out/sim_tetra.cf32 12   # simulated TETRA cell and call (then --rate 72000 --decoder voice)
cargo run --release -p declab-pmr --example tetra_sweep -- 4 500 [diff]   # TETRA sensitivity, gate on noise, CPU (diff: differential detection)
cargo run --release -p declab-pmr --example tetra_file -- IN.cf32   # a 72 kS/s TETRA carrier's fit slot by slot (lock losses, re-alignments)
cargo run --release -p declab-cli -- run voice --radio rtlsdr --freq 439.5M --gain 40.2 --card
cargo run --release -p declab-cli -- list
cargo run --release -p declab-cli -- selftest level
cargo run --release -p declab-cli -- run wfm --file x_98.000MHz_10Msps.cs8 --offset 1.2M --realtime
cargo run --release -p declab-cli -- run nfm --radio airspy --freq 446.05M --offset 0
cargo run --release -p declab-cli -- run usb --radio hackrf --freq 14.2M --gain 1,32,20
```

Only an Airspy R2 is plugged in at the moment (the HackRF is not). All
three drivers (HackRF, Airspy, RTL-SDR) are pure Rust over nusb and need the
WinUSB driver. The user's own
DecDVB-T GUI may be holding the Airspy ("Access is denied"): never kill it.

The GUI keeps its settings in `%APPDATA%\DecLAB\prefs.txt`; screenshot runs
neither read nor write it.

## Adding a decoder

Copy `crates/declab-decoders/src/level.rs`, implement `Decoder` and
`DecoderFactory` (see `crates/declab-api/src/lib.rs`), add `mod` and the
factory to `ALL` in that crate's `lib.rs`.

## House rules

- **Licence:** GPL-3.0-or-later. Credit borrowed algorithms at the function or
  module that borrows them, as the source projects do.
- **CPU efficiency is the main requirement.** Measure each change.
- **Never commit** IQ captures, reference clones, built exes, the user's
  location or personal email address.
- **Specs are the authority:** cite the ETSI clause or table next to
  constants and bit layouts.
- The user has a strong DSP background and is newer to Rust: explain Rust
  idioms in comments, not DSP basics.
