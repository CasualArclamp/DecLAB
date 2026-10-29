<!-- made by CasualArclamp -->
# DecLAB — Design

One receiver for the DVB family and DAB: **DVB-T / DVB-T2** (from DecDVB-T),
**DVB-S, DVB-S2/S2X (TS and GSE/IP) and DVB-RCS2 return bursts** (the DVB
modes of DecSAT, in `..\DecDVB`), **DAB / DAB+** (from DecDAB), and
**analog AM, DSB, USB, LSB, CW, NFM and WFM** (stereo + RDS, from DecSAT).
HackRF One, Airspy R2 and IQ-file replay, egui GUI + CLI, same look as the
other Dec apps.

Status date: 2026-10-27 (kickoff). This document is the source of truth for
scope and milestones; update it when a decision changes.

## 1. Kickoff decisions (2026-10-27)

- **Decoders are plug-in modules.** Each decoder is one file implementing a
  small trait (`declab-api`): the host sends it IQ down-converted to what it
  asked for, plus tuning and stream position; it hands back TS packets, IP
  packets, audio, pictures, text, a services list, measurements and scope
  points. Other people can write more decoders the same way.
- **Only the DVB modes of DecSAT** come across: DVB-S, DVB-S2/S2X TS and GSE,
  DVB-RCS2/TDMA return. Not the Comtech/Intelsat/Q-Flex modems, analog,
  radar, beacons or the PSK toolbox.
- **One GUI with a mode picker** (not DecSAT's many-VFO waterfall, not tabs
  of the old apps). Shared front end, spectrum/waterfall, card, scopes,
  services list, player.
- **Repository:** private GitHub repo CasualArclamp/DecLAB, local git.
- CPU efficiency stays the main requirement, as in DecDVB-T.
- (2026-10-27, during M1) **Airspy R2 as well as the HackRF**: the pure-Rust
  driver from DecDVB-T's Airspy branch (`..\DecDVB-T-airspy`), 10 MS/s IQ.
  DecDAB's Airspy branch has a variant that goes straight to DAB's rate; the
  host's down-converter makes that unnecessary here.
- (2026-10-27) **Analog modes** too: AM, DSB, USB, LSB, CW, NFM, WFM.
- (2026-10-28) **Icon** (user chose from three mockups): a spectrum with a
  TV multiplex (blue), a DAB block (green) and an FM carrier (orange), in
  the Dec apps' dark rounded square; drawn by `declab-gui`'s `make_icon`
  example, in the exe (build.rs, winresource) and the window.
- (2026-10-28) **Airspy gain is one slider** (user): step 0-21 on libairspy's
  Sensitivity or Linearity table, chosen in the gain menu (prefs
  `airspy_mode`, `airspy_step`); the menu shows the LNA/mixer/VGA indices it
  sets. After the optimiser, which can stop between steps, the button shows
  the three indices instead.
- (2026-10-28) **Pagers**: POCSAG 512/1200/2400 and FLEX 1600–6400 from
  DecPagers (`..\DecPagers`), as the `pagers` plug-in
  (`declab-decoders/src/pagers.rs`) over a copy of its protocol crate
  (`crates/declab-pagers`). Its scopes are "Symbols" (soft symbols in time
  order: 2 or 4 bands) and "Eye"; the GUI opens on "Symbols"
  (`DecoderFactory::default_scope`, new). Low-SNR receiver: tone
  correlators, per-tone AFC, Chase BCH (see DecPagers' DESIGN §3a).
- (2026-10-27) **GUI interactions** asked for: middle-drag pans the spectrum
  and waterfall; a middle click (no drag) opens a radial decoder picker like
  a game's weapon wheel — scroll to step round it, or point at a slice, and
  click to choose. UI mockups come before the GUI is built.

## 2. Shape

```
IqSource (HackRF / file)            ── declab-io (one copy, from DecDVB-T's)
  │
  ├─► spectrum + waterfall          ── host, shared
  │
  └─► DDC to the decoder's InputSpec (or untouched: sample_rate = None)
        │
        ▼ Block { iq, tuning, first_sample, dropped_before }
      Box<dyn Decoder>::process  on its own worker thread
        │
        ▼ Sink { lock, ts, ip, audio, pictures, text, services,
        │        measurements, constellation, log, retune_by_hz, custom }
  host: TS hub → player / EPG / Red Button,  IP → PCAP,  audio → speakers,
        card + history plots, services list; Panel (GUI thread) ↔ Commands
```

### The plug-in API (`crates/declab-api`)

| Piece | Role |
|---|---|
| `DecoderFactory` | static description (`DecoderInfo`: id, name, family, standard), the `InputSpec` it wants, `create()`, optional `panel()` |
| `Decoder` | `process(&Block, &mut Sink)`, `command(Command)`, `reset()`, `wants_threads()` |
| `Sink` | everything a call produces; the host drains it after each call, capacity kept so the loop does not allocate |
| `Panel` | the decoder's own settings UI on the GUI thread; talks to the decoder only through `Command` and `Sink::custom` |

Registration is explicit: `crates/declab-decoders/src/lib.rs` holds `ALL`, a
list of factories. A new decoder = one file + `mod` + one list entry
(`level.rs` is the template). Big decoders keep their DSP in crates of their
own and are wrapped by such a file.

**Later (open):** loading third-party decoders from a DLL at run time. Rust has
no stable ABI, so that needs a C-ABI shim (or `abi_stable`) over the same
trait; deferred until the in-tree API has settled on the real decoders.

### Crates

Shared crates are merged once; standard-specific DSP is copied in with a
`declab-` prefix and its history noted in the module docs.

| Crate | From |
|---|---|
| `declab-api` | new |
| `declab-decoders` | new: the one-file adapters + `ALL`; `analog.rs` (7 modes) + `analog_rds.rs` from DecSAT, `level.rs` |
| `declab-engine` | new `host.rs` (source thread, DDC + resampler, worker per decoder), `audio.rs` (cpal, from DecSAT's player); DecDVB-T's waterfall/spectrum/front end/meters |
| `declab-core` | DecDVB-T's core less its DVB tables: Iq, sample formats, errors, CRC, UHF plan |
| `declab-io` | DecDVB-T-airspy's `decdvbt-io` (HackRF, Airspy R2, files, WAV); receive only, no `hackrf_tx` yet; DecSAT's RTL/Pluto/UHD later if wanted |
| `declab-dsp` | DecDVB-T's `decdvbt-dsp` (DDC, resampler, FIR, AGC, DC); DecSAT's timing/carrier/RRC join with DVB-S |
| `declab-ts` | DecDVB-T's `decdvbt-ts` (PSI/SI, EPG with Huffman) — DecSAT's `decsat-ts` compared and merged |
| `declab-mheg` | DecDVB-T's Red Button engine |
| `declab-audio`, `declab-video` | DecDVB-T's player side; DAB audio decoders from DecDAB |
| `decdvbt-core`, `-fec`, `-ofdm`, `-rx`, `-ts`, `-mod` | copied from DecDVB-T (`5e9b249`) under their own names (see §11); `decdvbt-dsp` is `declab-dsp` (identical) |
| `declab-dvbs` | `decsat-dsp`/`-frame`/`-fec`/`-modem` DVB parts, `decsat-gse`, `decsat-ip` |
| `declab-dab` | `decdab-ofdm`, `-fic`, `-msc`, `-fec`, `-codecs`, `-data`, `decdab-fdk-sys` |
| `apps/declab-gui`, `apps/declab-cli` | new, built from the three GUIs' parts |

**Licence clash to settle in M3:** DecDAB uses FDK-AAC for DAB+, DecSAT
avoided FDK because its licence does not mix with the GPL (it uses libxaac).
DecLAB is GPL-3.0-or-later, so DAB+ should move to libxaac.

## 3. Milestones

| M | Content | Done when |
|---|---|---|
| M0 | Workspace, `declab-api`, decoder list, example decoder, CLI `list`/`selftest`, GUI mode picker | ✅ 2026-10-27 |
| M1a | Shared front end and host: `declab-core`/`-io`/`-dsp`/`-engine`, DDC + resampler + worker per decoder, waterfall rows, sound out, analog decoders, CLI `run` on files and radios | ✅ 2026-10-27 |
| M1b | GUI from the mockups: source controls, spectrum/waterfall (middle-drag moves the band, scroll zooms), decoder wheel with icons, card, scope, decoder panels, services, status bar, prefs | ✅ 2026-10-27 on files; live in the GUI not yet tried (the Airspy was in use) |
| M2a | DVB-T/T2 as a plug-in: decode, card, constellation, services, now/next, `.ts` out | ✅ 2026-10-28: C23 and C30 decode to DecDVB-T's very bytes at its speed |
| M2b | TS hub, player (video, audio, subtitles), EPG, Red Button, PLP choice, HTTP/UDP out | ✅ 2026-10-28 on captures; not yet live |
| M3 | DAB/DAB+ as a plug-in: audio, DLS, slideshow, SPI; libxaac question | 11D/12B captures play |
| M4 | DVB-S and DVB-S2/S2X TS + GSE/IP (PCAP) | DecSAT's test captures decode |
| M5 | DVB-RCS2/TDMA return burst analyser | DecSAT's inroute captures decode |
| M6 | Release: CI, portable exe, README | v0.1.0 |
| V1 | RTL-SDR source; DMR Tier I/II/III + packet data + LRRP (`declab-pmr`, `dmr` plug-in), weak-signal receiver | ✅ 2026-10-29 in simulation only: no real DMR capture yet; no AMBE+2 synthesiser, so no sound |
| V2–V4 | NXDN, then P25, then TETRA, on the same front end where they can | |
| later | DLL plug-ins; DVB-C (port of `Documents\DecSAT\dvbc_decode.py`); modulators |

## 4. Notes from M1a (2026-10-27)

- **Host shape.** The source thread never waits on a decoder when live
  (blocks dropped and counted); from a file it does, so the CLI decodes a
  file completely. Blocks are pooled `Arc<Vec<Iq>>`s: shared, not copied.
  `Host::finished()` = source ended and every block decoded.
- **Channel chain.** `Ddc` (NCO + one or two decimating FIR stages) to at
  least 2.5x the bandwidth, then `Resampler` (rational polyphase) to the
  exact rate the decoder asked for: e.g. WFM from 10 MS/s is ÷20 then
  ×96/125 to 384 kS/s. `InputSpec::sample_rate = None` with a centred,
  full-width channel passes the source's samples untouched.
- **CPU.** Live WFM from the Airspy at 10 MS/s: 15–18 % of one core for the
  decoder worker, nearly all of it the down-converter. Worth a pass later
  (a cheaper first stage, e.g. half-bands).
- **Fixed against DecSAT's analog code:** SSB/CW shifted once and never
  back (a 1 kHz tone came out at 700 Hz and both sidebands folded); now a
  Weaver demodulator with a 401-tap FIR, opposite sideband ≥ 40 dB down.
  The stereo pilot was judged on its level alone and FM hiss read above the
  threshold (stereo on noise); now judged against a 17.5 kHz guard band.
- **Not verified live:** analog audio on a real station. The aerial here is
  UHF band-pass filtered and shows no FM broadcast; the Airspy path itself
  (streaming, DDC, worker, sound device) was run live.

## 5. Notes from M1b (2026-10-27)

- **Mockups:** the interactive canvas the GUI follows is a private claude.ai
  artifact (main window + decoder wheel).
- **The view and the radio.** The view has its own centre and zoom. Scroll
  zooms about the pointer inside what the source covers. Middle-drag moves
  the view; with a radio the radio follows (a retune at most every 90 ms
  while dragging, one more on release); with a file the view is held inside
  the capture. Rows are drawn where their own frequencies fall, so a drag
  shows at once; when retuned rows arrive the held waterfall slides
  (`History::push_at`).
- **The channel stays on its station** while the band moves:
  `Host::set_channel_at` keeps the decoder's channel at an RF frequency and
  the worker recomputes the offset from each block's centre.
- **The wheel** is an egui Area over everything that takes every click, so
  nothing under it reacts. The pointer changes the slice only when it moves
  (one resting in the centre must not undo a scroll). Icons are drawn with
  the painter (no image assets). Only registered decoders appear.
- **Gestures are unit-tested** headless (`apps/declab-gui/src/sim.rs` feeds
  egui made-up mouse events): middle click opens the wheel, middle-drag pans
  by the distance dragged and opens nothing, left click tunes, scroll zooms,
  wheel point/scroll/click/cancel.
- **Decoder panels:** WFM (de-emphasis 50/75 µs/off, stereo auto/mono), NFM
  (de-emphasis, squelch), CW (note pitch), through `Command::Set`.
- **CPU** on the synthetic FM file (10 MS/s, WFM): about 40 % of one core
  for the whole app, 16 of it the decoder worker. Stopped, the window only
  redraws on input.
- `out/sim_fm_98.000MHz_10Msps.cs8` is a made-up FM band (a stereo station
  at 98.8 MHz with a 440 Hz left / 1 kHz right tone, three mono ones),
  made by `python tools/make_fm.py out/sim_fm_98.000MHz_10Msps.cs8 4`
  (numpy); the capture itself is not committed.

## 6. Live test and RDS (2026-10-27)

- **Live GUI on the Airspy** (with DecDVB-T closed): the FM band shows
  stations here after all (96.1, 97.3, 100.1 MHz among them; the earlier
  CLI probes had missed them). WFM on 96.1 MHz: constant-envelope ring,
  pilot 20-49 dB, stereo, about 17 % of a core for the decoder, 31-50 % for
  the whole app.
- **Fixed from the live run:** ~170 blocks were dropped at every start
  while the decoder's worker waited for the sound device to open; sound now
  has its own thread (`AudioOut::spawn`) and none are dropped.
- **RDS** was slow live (17 groups in 15 s, part of the RadioText, no PS).
  DecSAT's RDS used only the in-phase part of the 57 kHz subcarrier, which
  EN 62106 lets a station put in phase or in quadrature with the pilot's
  third harmonic. Now: I and Q, the BPSK phase followed from the symbols
  squared, chips decided on the derotated signal. Unit-tested with
  synthesized group 0A at 0, 90, 180 and 270 degrees (PS read every time);
  `tools/make_fm.py` puts RDS (PS "DECLAB") on its stereo station, and the
  GUI decodes it at ~10.7 groups/s. **Not yet tried live** (DecDVB-T had
  the Airspy again).
- **Decoder scopes:** `Sink::scopes` carries named scopes beside the
  channel IQ; WFM sends "RDS" (one derotated point per symbol) and an
  "RDS phase" reading. The GUI offers them above the scope
  (`--scope RDS` for screenshots).

### Live RDS check (2026-10-27, later)

- The 96.1 MHz signal that gave RDS earlier had gone. The broadcasts now
  heard, 97.3 and 101.1 MHz, are stereo (pilot 23-32 dB) but give no RDS.
  `tools/rds_look.py` on a 20 s recording (`out/fm_99.200MHz_10Msps.cs8`,
  `declab run … --record`) shows why: the 57 kHz band is at the FM noise
  floor (-65 dB beside -65 dB), the pilot 37 dB above its own. The aerial
  is UHF band-pass filtered; the stations arrive too weak for RDS.
- More Airspy gain makes it worse: at LNA 14 / mixer 12-15 / VGA 10-12 the
  pilot falls to the noise (input -9 to -16 dBFS): the tuner is overloaded,
  most likely by the UHF TV the filter passes. 12/10/8 is the best setting.
- The RDS I/Q rework is therefore verified on synthesized RDS only; it
  needs a station with RDS above the noise (a proper FM aerial).
- A 96-100 % decoder CPU reading seen once live at +3.1 MHz did not
  reproduce: both offsets decode the recording at the same speed (~6x
  real time).

## 7. Dragging, the card, 60 fps (2026-10-27)

- **Smooth band drags.** The engine's waterfall thread used to drop its
  held rows and wait a new quarter-second lead at every retune; a drag
  retunes every 90 ms, so the waterfall froze. Now a retune only drops the
  half-made row. The GUI keeps its rows about one anchor centre: a retuned
  row is shifted by whole bins as it goes in (one row's work), and only a
  move past a quarter of the span re-anchors (one full redraw). A file's
  band can be dragged until its edge reaches the middle of the view.
- **Channel by mouse.** Left-drag an edge of the bracket: bandwidth (both
  edges, centred). Left-drag inside: the channel moves (100 Hz steps, onto
  the raster on release). A move within half the bandwidth, or a new
  bandwidth, does not reset the decoder (it would mute during a drag).
- **The card** (`declab_api::Card` + `Gauge`): DecDVB-T's layout (title bar
  in the family colour with the frequency, a big headline with the margin
  beside it, a bar ticked at what is needed, label/value pairs two a row,
  labelled text). WFM's holds all of RDS: PS, PI, PTY name, TP/TA,
  music/speech, groups, groups/s, block error rate, subcarrier phase, clock
  time (4A), RadioText (A/B flag clears it), AF list, group-type counts; its
  gauge is the pilot against the 15.6 dB of full stereo. A decoder without
  a card gets one made from its measurements.
- **60 fps.** Decoders report every 1/60 s. egui's repaint timer does not
  keep the interval asked (a 16 ms request drew hundreds of frames: 65 %
  of a core idle, 133 % running); the wait is steered from the frame gaps
  seen (DecDVB-T's `pace`). 56-60 fps, whole app ~31-39 % on the FM file.

## 8. RDS for weak signals (2026-10-27)

The demodulator was rewritten (`analog_rds.rs`): the subcarrier to ±2.4 kHz
at ~12 kS/s by the shared `Ddc` (it had a single-pole filter), a biphase
matched filter, Gardner timing, BPSK phase from the squared symbols, soft
differential decisions; block sync on two good blocks in order, held until
30 of the last 50 fail (it was lost on any bad chip pair); bursts of one or
two bit errors corrected; partial groups used; PS and RadioText segments
shown only when the same value came twice.

| band SNR (±2.4 kHz) | old: groups of ~68 in 6 s | new | new BLER |
|---|---|---|---|
| +7.0 dB | 67 | 68 | 0 % |
| +1.0 dB | 0 | 68 | 0 % |
| −1.0 dB | 0 | 64 | 2.6 % |
| −2.6 dB | 0 | 50 | 17 % |
| −3.9 dB | 0 | 34 | 42 % |
| −5.1 dB | 0 | 3 | 65 % |

(`cargo test -p declab-decoders noise_sweep -- --nocapture`.) On the 20 s
recording `out/fm_99.200MHz_10Msps.cs8`, 101.1 MHz went from no groups to
~149 (PS, PI, PTY, TP/TA, RadioText filling in), symbol SNR ~5 dB, BLER
~36 % with ~270 blocks corrected; 97.3 MHz (symbol SNR 1-2 dB) still gives
none. The card shows RDS sync, RDS SNR and corrected blocks.

- **RDS carrier waterfall** (later the same day): `Sink::waterfalls` carries
  rows for a decoder's own small waterfalls (`WaterfallRow`: dB over a span
  about the decoder's 0 Hz); the host queues them, the GUI draws the one
  named like the scope shown, under it (ring texture, ~3 s). The RDS
  decoder makes 128-point Hann FFTs of the filtered subcarrier, three a
  row (~31 rows/s), cropped to ±3 kHz so the colours follow the band and
  not the filter's stopband: the two biphase sidebands about ±1 kHz and
  the null at 0 Hz show on a clean signal.

## 9. Multiplex waterfall, Es/N0, clock time (2026-10-27)

- `WaterfallRow` gained `low_hz` and `marks`. A decoder waterfall named
  like a scope is drawn small under it; any other is drawn large in the
  right-hand column, which held the services. The services (and text, when
  the card has none) now sit under the card.
- WFM sends "FM multiplex": 1024-point Hann FFTs of the demodulated
  multiplex at 384 kS/s, 12 to a row (~31 rows/s), 0-100 kHz, marked at the
  pilot (19), the stereo subcarrier (38) and RDS (57 kHz).
- RDS Es/N0 is now taken along the decision axis (A = mean |re|, σ² = mean
  re² − A², Es/N0 = A²/2σ²). The first figure (re² − im²)/im² was 3 dB
  optimistic; dividing by 2im² read 0.4 dB on air because the pilot's
  phase jitter spreads the cloud across the axis, which the scope shows
  but decisions never see. Now the recording's 101.1 MHz reads 2.0 dB at
  35 % BLER, the synthetic test 1.9 dB at 42 %.
- Clock time (4A) only from blocks that passed their check uncorrected,
  with a year 2000-2099 and an offset within ±13 h (a wrong correction had
  shown 1888-11-24 UTC−11:30).

## 10. Settings window (2026-10-27)

- The decoder-settings box under the band is gone: a **Settings** button
  in the top bar opens one window with General (volume, the running
  decoder's bandwidth) and every decoder's own panel, listed by family,
  the running one marked. The FM multiplex waterfall takes the freed half
  beside the card.
- `Panel` gained `settings()` (its settings as `Command::Set`s) and
  `restore(key, value)`. The GUI keeps a panel for every decoder, sends a
  new decoder its panel's settings when it starts, and keeps them between
  runs as `set.<id>.<key>=<value>` in prefs.txt. So a mode can be set up
  before it runs.

## 11. M2a: DVB-T/T2 (2026-10-28)

- **The receiver came over as it is.** DecDVB-T's `decdvbt-core`, `-fec`,
  `-ofdm`, `-rx`, `-ts` and `-mod` (≈40 000 lines, at DecDVB-T `5e9b249`)
  are copied under their own names rather than renamed `declab-*`: no
  churn in 40 000 lines, and a fix can be compared with DecDVB-T directly.
  Their DSP dependency is `declab-dsp` under the old name (the files are
  identical bar names). DecDVB-T's own tests run here and pass.
- **The plug-in** (`declab-decoders/src/dvbt.rs`) wraps `AutoReceiver`:
  one wheel entry, DVB-T or T2 found blind; the radio's own samples
  (`sample_rate: None`); TS to `Sink::ts` and a `TsAnalyser` for services
  and now/next; DecDVB-T's card (mode, effective SNR against the mode's
  QEF C/N, details); the equalised cells on the scope.
- **Same bytes, same speed.** C23 (DVB-T 64-QAM 2/3): 62 370 packets, byte
  for byte DecDVB-T's, 1.31 s against its 1.38 s for 4 s of signal. C30
  (T2 256-QAM 2/3 rotated): DecDVB-T's whole stream is an exact prefix of
  DecLAB's (its CLI stops before the FEC drains), no continuity jumps,
  ~56 000 packets/s against its ~58 000.
- **Two host fixes found on the way.** A decoder asking for the source's
  own rate with its channel centred now gets the samples untouched (a
  pointless full-rate filter had cost DVB-T 30 %). `Block::live` says
  whether the samples come from a radio: the T2 FEC drops a frame when
  behind only then (from a file, running flat out, it had been dropping).
- `declab run … --ts OUT.ts --card`; the host's `write_ts`.
- The decoder waterfalls are drawn a row to a physical pixel on a whole
  pixel (the RDS one shimmered: 96 rows in 64 points); the multiplex
  waterfall uses 4096-point FFTs (94 Hz bins).

## 12. M2b: everything after the transport stream (2026-10-28)

- **The TS hub is the host's**, not the DVB-T plug-in's: DecDVB-T's
  `hub.rs` is `declab_engine::hub`, and the decoder worker feeds it
  whatever `Sink::ts` a decoder makes (reset on a retune or a new
  decoder). So DVB-S/S2 TS later gets the same services, guide, outputs,
  player and Red Button. `Host::set_outputs / ts_view / epg`.
- **Copied in as they are:** `decdvbt-video` (the player, ffmpeg at run
  time), `decdvbt-audio`, `decdvbt-mheg` (Red Button), and DecDVB-T's GUI
  windows `player.rs` (with `redbutton.rs`), `epg_view.rs`, `ts_viewer.rs`,
  `services.rs`, `external.rs` (VLC). `tv.rs` is the glue (DecDVB-T's
  play/watch/VLC/outputs logic), so `main.rs` only calls it; when the
  decoder makes TS, the right-hand column is DecDVB-T's services list with
  Play / VLC / TS analyser / Guide and Outputs (record, UDP, HTTP).
- **Verified on the captures:** BBC One HD plays from C30 (1080i H.264,
  AAC, DVB subtitles found); the BBC Red Button Text app opens over it on
  `sim_rb_c30` (`DECLAB_REDBUTTON=wait,wait`); the guide shows 1438 events
  across the network; HTTP (curl: 81 211 packets in 3 s, all in sync) and
  UDP (7 packets a datagram, none bad) out.
- **PLP choice:** the DVB-T/T2 decoder sends L1-post's PLPs to its
  Settings panel; the pick goes back as `plp` (`auto` = the first data
  PLP) and is kept between runs.
- **Scopes drawn as fine points:** a texture, a physical pixel to a dot,
  brighter where dots pile up (DecDVB-T's way); DVB sends 6000 cells.
- GUI: `--play SID`, `--window ts|epg`, `--settings` opens on the running
  decoder's page. CLI: `--udp`, `--http`.
- Open: not yet tried live; the whole app reads 130-220 % of a core on
  C30 with the player or the TS analyser open (T2 FEC threads, ffmpeg's
  HD decode and the analyser's report): worth a CPU pass.

## 13. Airspy gain optimiser (2026-10-28)

- `apps/declab-gui/src/gain_opt.rs`, after DecDVB-T's and DecDAB's HackRF
  optimiser. The Airspy has three stages with the AGCs off (LNA 0-14, mixer
  0-15, VGA 0-15), so it climbs libairspy's *sensitivity* ladder (rungs
  0, 2 .. 20, 21: one knob moving all three), then up to three rounds of
  hill climbing, each stage stepped by 2 about the best. 0.8 s to settle and
  1.3 s to measure per setting; about 12 for the sweep and 6 a round.
- **Figure:** the decoder's MER while it is locked (DVB-T/T2), else the
  channel's C/N from the spectrum row (channel mean power over the 10th
  percentile of the bins outside it, the radio's outer 5 % and a guard left
  out), so analog and FM have one too. A C/N reading needs room outside the
  channel: for DVB-T's 8 MHz in the 10 MHz row there is none, and only the
  MER works.
- A setting that clips (> 0.05 %) or runs hotter than -12 dBFS is out; ties
  within 0.3 dB go to the coolest. If the best C/N of the sweep is under
  3 dB (no signal in the channel) or nothing could be measured, the old
  gains are put back. Cancelling puts them back too.
- GUI: "Optimise gain" in the gain menu (sliders locked while it
  runs); `--optimise` starts it 3 s after the radio is up and logs every
  setting to stderr.
- **One optimiser, two searches** (`Plan` in `gain_opt.rs`): the timing,
  figure, limits, past-peak test and "coolest within 0.3 dB" are DecDVB-T's
  HackRF optimiser's; the **HackRF uses its search as it is** (total gain in
  4 dB steps from 20 dB under the present one, LNA first, up to 90 dB or 24
  points, then the LNA/VGA splits 16/24/32/40 of the best total), the Airspy
  the ladder and hill climb. Both have a button in their gain menu. Not in
  DecDVB-T's: the no-signal guard (best C/N under 3 dB: nothing changes).
  The HackRF side is tested on a synthetic front end only (not plugged in).
- Tested on a synthetic front end (unit tests), then live on the Airspy
  (2026-10-28, a different site from the earlier captures: C39, C42 and C45
  are the strong muxes, C32/34/35 weaker). On C42 the default gains (LNA 12 /
  mixer 10 / VGA 8) ran at about -10 dBFS and the sweep found MER rising
  1.5 -> 30.4 dB from the lowest rung, clipping at LNA 14 / MIX 10 / VGA 8
  (MER 17), and chose **LNA 13 / MIX 8 / VGA 5: MER 30.3 dB at -20 dBFS** in
  14 settings (about 35 s), the cooler of two settings within 0.3 dB.

## 14. RTL-SDR and DMR (2026-10-29)

The user asked for RTL-SDR support and digital speech decoding (DMR Tier I,
II and III, then NXDN, P25 and TETRA), optimised for low SNR. Then, mid-task,
for LRRP on DMR.

- **RTL-SDR:** DecPagers' pure-Rust RTL2832U + R820T/R828D driver (from
  DecDAB), copied into `declab-io` behind the `rtlsdr` feature (on by default
  in both apps). It runs at 2.048 MS/s only. CLI: `--radio rtlsdr --gain
  40.2|agc --ppm N`. GUI: "RTL-SDR" in the source list, with a gain menu
  (AGC, the R820T's 29 gain steps, ppm). Not yet run with a dongle in
  DecLAB.
- **`crates/declab-pmr`**: `fsk4` (the 4FSK front end NXDN and P25 will
  share), `fec` (the codes of ETSI TS 102 361-1 annex B, with generator
  matrices copied from the spec tables), and `dmr/` (receiver, BPTC, AMBE
  frame FEC, PDUs, packet data, LRRP, simulator). Constants are cited to
  TS 102 361-1/-2/-3/-4. The AMBE interleave and Golay come from DSD/mbelib
  (ISC licence, notice kept in `ambe.rs`). The LRRP token layouts come from
  SDRTrunk (GPL-3), because LRRP has no public spec.
- **Plug-in `dmr`** (new family "Digital voice"): calls per slot (start,
  end, duration, voice BER), Tier III CSBKs (grants with LPCN and slot,
  Aloha, broadcasts, clears) and the CACH Short LC system ID. Data packets
  are reassembled, and LRRP locations are logged with latitude and
  longitude.
- **Voice is AMBE+2 parameter frames only.** There is no synthesiser yet.
  mbelib (ISC) has the codebooks, but bringing them in is a download, and
  AMBE+2 may still be patent-encumbered in places, so that is the user's
  decision.

### Weak-signal receiver: what the sweep measured

`cargo run --release -p declab-pmr --example dmr_sweep -- SEEDS OFFSET_HZ
[CUTOFFS]` runs a simulated Tier III site (a group call on TS1, CSBKs on
TS2) at falling C/N, with C/N measured in 12.5 kHz.

- **The discriminator beats the complex-product detector** (matched filter
  on z·z*[n−1], then arg) by 2–3 dB. The product version distorts at large
  phase steps. The plug-in uses the discriminator; the sweep keeps both.
- **Pre-detection low-pass at 4 kHz** (not 5.5 kHz) gains about 1 dB, but
  only with **AFC**. The receiver turns its measured offset into an NCO
  ahead of the filter, and it demodulates in 20 ms slices: run on whole host
  blocks, the AFC ran away, because each burst re-measured an offset already
  corrected.
- **Soft decoding everywhere:** ML for the slot type, EMB and TACT; Chase-II
  for the Hamming rows and the AMBE Golay words; a Pyndiah-style soft
  iteration for BPTC (196,96), worth about 0.5–1 dB on CSBKs; soft Viterbi
  for rate ¾ data.
- **Flywheel:** after a SYNC, bursts are predicted every 30 ms. Voice
  bursts B–F (no SYNC) and a voice A whose SYNC was lost are decoded on
  timing from the eye's energy.
- **Validation:** Hamming (7,4) is perfect, and Golay (20,8) and QR (16,7)
  have small covering radii, so noise passes their distance tests. A burst
  only counts if its BPTC decodes, its CRC passes, or its AMBE Golay
  corrections are few. Before this fix the receiver held "lock" on noise
  and the AFC wandered. The test `does_not_lock_on_noise` guards it.
- **Results** (discriminator, 800 Hz offset, 4 runs): voice frames (the
  protected bits) 100 % to 7 dB, 96 % at 5 dB, 91 % at 4 dB; CSBKs 98 % at
  7 dB, 88 % at 5 dB. The first version managed 41 % voice at 5 dB.
- **CPU:** about 0.4 % of one core decoding and 1.5 % hunting.

### Not yet / to check on real signals

- **CRC-32 byte order:** clause B.3.9 is ambiguous about the octet order
  of the message CRC-32. The assembler accepts either order.
- **MBC last-block CRC:** the scope of the last block's CRC is read from
  the spec text; it is untested against real traffic.
- **Rate 1 data** is not decoded.
- **Header compression:** the network number is configured in the radios,
  not sent, so compressed headers show 12.x.x.x addresses.
- **Simulation only:** everything is tested against the built-in modulator,
  which was written from the same reading of the spec. A real capture (the
  RTL-SDR or the Airspy on a DMR repeater or hotspot) is the next check.

