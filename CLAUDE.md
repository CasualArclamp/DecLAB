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
cargo run --release -p declab-cli -- list
cargo run --release -p declab-cli -- selftest level
cargo run --release -p declab-cli -- run wfm --file x_98.000MHz_10Msps.cs8 --offset 1.2M --realtime
cargo run --release -p declab-cli -- run nfm --radio airspy --freq 446.05M --offset 0
cargo run --release -p declab-cli -- run usb --radio hackrf --freq 14.2M --gain 1,32,20
```

Only an Airspy R2 is plugged in at the moment (the HackRF is not). Both
drivers are pure Rust over nusb and need the WinUSB driver. The user's own
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
