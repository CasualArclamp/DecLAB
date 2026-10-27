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
| `declab-dvbt` | `decdvbt-ofdm`, `-rx`, `-fec` (Viterbi/RS, T2 LDPC/BCH) |
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
| M2 | DVB-T/T2 as a plug-in: TS hub, services, player, EPG, Red Button | C30 and C23 captures decode as in DecDVB-T at the same CPU |
| M3 | DAB/DAB+ as a plug-in: audio, DLS, slideshow, SPI; libxaac question | 11D/12B captures play |
| M4 | DVB-S and DVB-S2/S2X TS + GSE/IP (PCAP) | DecSAT's test captures decode |
| M5 | DVB-RCS2/TDMA return burst analyser | DecSAT's inroute captures decode |
| M6 | Release: CI, portable exe, README | v0.1.0 |
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
