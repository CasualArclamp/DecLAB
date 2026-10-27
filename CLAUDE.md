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
cargo run --release -p declab-cli -- list
cargo run --release -p declab-cli -- selftest level
```

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
