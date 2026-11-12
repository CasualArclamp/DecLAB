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
cargo run --release -p declab-ism --example ism_make -- out/sim_ism_433.920MHz_2500ksps.cs8 40   # simulated ISM band: every device sample, own frequency and level, some overlapping
cargo run --release -p declab-gui -- out/sim_ism_433.920MHz_2500ksps.cs8 --decoder ism --chan 433.92M --tab devices --screenshot out/x.png --after 38   # ISM devices (rtl_433's protocols); tabs devices, log, flex
cargo run --release -p declab-cli -- run ism --radio airspy --freq 433.92M --offset 0   # ISM devices live (868.3M for 868 MHz sensors)
cargo run --release -p declab-ism --example filetest -- out/sim_ism_433.920MHz_2500ksps.cs8   # messages, and the receiver's CPU (and on noise alone)
ISM_MODEL=Nexus-TH cargo test --release -p declab-ism debug_sample -- --ignored --nocapture   # one sample's pulses on the air against the ideal (ISM_AMP, ISM_OFF, ISM_SEED)
cargo run --release -p declab-lora --example lora_make -- out/sim_lora_869.525MHz_1000ksps.cs8 30   # simulated 869.525 MHz: Meshtastic and MeshCore packets, each its own SNR, offset and ppm
cargo run --release -p declab-gui -- out/sim_lora_869.525MHz_1000ksps.cs8 --decoder lora --chan 869.525M --set tab=packets --screenshot out/x.png --after 25   # LoRa mesh; tabs nodes, messages, packets, keys
cargo run --release -p declab-cli -- run lora --radio airspy --freq 869.525M --gain 10,7,11 --offset 0   # LoRa mesh live (Meshtastic EU LongFast, MeshCore 250 kHz)
cargo run --release -p declab-lora --example filetest -- out/sim_lora_869.525MHz_1000ksps.cs8   # every frame and what the meshes made of it, receiver CPU (and on noise)
cargo run --release -p declab-lora --example lora_sweep -- 20 [LongFast]   # sensitivity per preset, and the SNR it reports
LORA_BW=125000 LORA_SF=12 LORA_PPM=10 LORA_FREQ=8695 LORA_SNR=-15 LORA_DEBUG=1 cargo test --release -p declab-lora symbol_errors -- --ignored --nocapture   # one frame's symbols against those sent, with the receiver's trace
cargo run -p declab-gui --example make_icon   # redraws apps/declab-gui/assets/icon.ico and icon-256.png (preview out/icon-preview.png)
cargo run --release -p declab-pmr --example dmr_make -- out/sim_dmr.cf32 15   # simulated DMR repeater: call, Tier III CSBKs, LRRP
cargo run --release -p declab-gui -- out/sim_dmr.cf32 --rate 48000 --decoder voice --screenshot out/x.png --after 8   # Digital voice: DMR, NXDN48, NXDN96, P25, TETRA detected by itself
cargo run --release -p declab-gui -- out/sim_dmr.cf32 --rate 48000 --decoder voice --tab hex --screenshot out/x.png --after 9   # Digital voice's data tabs: heard, hex, lrrp, ip
cargo run --release -p declab-cli -- run nfm --radio airspy --freq 446.05M --set squelch=noise --set squelch_noise=10   # squelch: off, level, snr or noise, each with its threshold
cargo test -p declab-decoders --release squelch_sweep -- --ignored --nocapture   # each squelch method's reading against C/N
cargo run --release -p declab-pagers --example pocsag_make -- out/sim_pocsag_m.cf32 20   # simulated on-site pager: POCSAG 512 Manchester; then declab-gui -- out/sim_pocsag_m.cf32 --rate 250000 --decoder pagers --chan 40k
cargo run --release -p declab-pagers --example filetest -- IN.cf32   # pages and receiver CPU from a 50 kS/s pager channel, with and without the POCSAG Manchester chain
cargo run --release -p declab-pmr --example dmr_sweep -- 4 800   # DMR sensitivity vs C/N (and CPU)
cargo run --release -p declab-pmr --example dmr_file -- IN.cf32   # every DMR event of a real 24 kS/s channel, and the counters
cargo run --release -p declab-pmr --example nxdn_make -- out/sim_nxdn.cf32 12 [96]   # simulated NXDN48 (or NXDN96) calls
cargo run --release -p declab-gui -- out/sim_nxdn.cf32 --rate 48000 --decoder voice --screenshot out/x.png --after 6
cargo run --release -p declab-cli -- run voice --radio airspy --freq FREQ --offset OFFSET --record-channel out/x.cf32 --seconds 1800   # long recording of just the decoder's channel (cf32 at its rate, 72 kS/s for voice); replay with --file out/x.cf32 --rate 72000
cargo run --release -p declab-pmr --example nxdn_file -- IN.cf32 [96] [bits]   # every NXDN frame of a real 24 kS/s channel: LICH, pass or not, eye SNR (bits: the frame's bits)
cargo run --release -p declab-pmr --example nxdn_sweep -- 4 800 - [96]   # NXDN48 (or 96) sensitivity, SNR gate on noise, CPU
cargo run --release -p declab-pmr --example p25_make -- out/sim_p25.cf32 14   # simulated P25 call and control channel (then --decoder voice)
cargo run --release -p declab-pmr --example p25_sweep -- 4 300   # P25 sensitivity, SNR gate on noise, CPU
cargo run --release -p declab-pmr --example p25p2_make -- out/sim_p25p2.cf32 15   # simulated P25 phase 2 traffic channel, a call on each slot (then --rate 48000 --decoder voice)
cargo run --release -p declab-pmr --example p25p2_sweep -- 4 300   # P25 phase 2 sensitivity, seed recovery time, gate on noise, CPU
cargo run --release -p declab-pmr --example p25_trunk_make -- out/sim_p25_trunk.cf32 18   # trunked P25 site: phase 1 control channel granting a phase 2 call
cargo run --release -p declab-gui -- out/sim_p25_trunk.cf32 --rate 240000 --freq 851M --decoder voice --chan 850.95M --screenshot out/x.png --after 9   # call following: a VFO opens on the grant, closes after
cargo run --release -p declab-pmr --example tetra_make -- out/sim_tetra.cf32 12   # simulated TETRA cell and call (then --rate 72000 --decoder voice)
cargo run --release -p declab-pmr --example tetra_sweep -- 4 500 [diff]   # TETRA sensitivity, gate on noise, CPU (diff: differential detection)
cargo run --release -p declab-pmr --example tetra_file -- IN.cf32   # a 72 kS/s TETRA carrier's fit slot by slot (lock losses, re-alignments)
cargo run --release -p declab-cli -- run voice --radio rtlsdr --freq 439.5M --gain 40.2 --card
cargo run --release -p declab-cli -- run dab --file ../DecDAB/captures/11d_long_222.064MHz_4.096Msps_amp.cs8 --service "classic fm" --wav out/x.wav --card   # DAB/DAB+ (DecDAB's captures); AAC by libxaac
cargo run --release -p declab-gui -- ../DecDAB/captures/11d_long_222.064MHz_4.096Msps_amp.cs8 --decoder dab --screenshot out/x.png --after 12
cargo run --release -p declab-cli --example dab_slides -- FILE OFFSET_HZ OUT_DIR [SECONDS]   # every DAB+ service of a block at once: X-PAD apps, MOT groups and errors per service, slides written to OUT_DIR (DAB_DUMP=SID,SID writes those services' raw PAD fields, in hex)
cargo run --release -p declab-cli -- run wfm --radio airspy --freq 98M --bias-tee   # antenna-port power for an LNA (off unless given; also --bias-tee on declab-gui)
cargo run --release -p declab-decoders --example sat_make -- out/sim_sat_gse.cf32 gse 10 6 -120000   # simulated satellite carrier (s2ts, s2tv, s2x, gse, dvbs), 8 MS/s
cargo run --release -p declab-cli -- run sat --file out/sim_sat_gse.cf32 --rate 8M --bandwidth 3M --pcap out/x.pcap --ts out/x.ts --set symbol_rate=2000000 --card   # DVB-S/S2/S2X; GSE to PCAP, TS to a file
cargo run --release -p declab-gui -- out/sim_sat_s2x.cf32 --rate 8M --decoder sat --chan 0 --screenshot out/x.png --after 7
cargo run --release -p declab-gui -- out/sim_sat_s2x.cf32 --rate 8M --freq 1176M --decoder sat --chan 1176M --set lo_hz=9750000000 --screenshot out/x.png --after 7   # behind a Universal LNB, low band: shown as 10926 MHz
ffmpeg -ss 20 -t 40 -i ../DecDVB-T/out/c23_120.ts -map 0:p:4164:v -map 0:p:4164:a:0 -map 0:p:4164:s -c copy -f mpegts -muxrate 4355051 -mpegts_service_id 4164 -y out/sat_tv.ts   # one service, constant rate at s2tv's TS rate
cargo run --release -p declab-decoders --example sat_make -- out/sim_sat_tv.cs8 s2tv 14 30 -150000 2e6 out/sat_tv.ts   # DVB-S2 8PSK 3/4 carrying it (cs8 to keep it small)
cargo run --release -p declab-gui -- out/sim_sat_tv.cs8 --rate 8M --decoder sat --chan 0 --play 4164 --screenshot out/x.png --after 15   # the player on a satellite service
cargo run --release -p declab-gui -- out/sim_sat_tv.cs8 --rate 8M --decoder sat --chan 0 --in-app --screenshot out/x.png --after 15   # the card's "▶ In app" pressed for you
cargo run --release -p declab-gui -- ../DecDVB-T/captures/c30_546.000MHz_10Msps_amp.cs8 --decoder dvbt --chan 546M --play 17536 --screenshot out/x.png --after 15   # the player on DVB-T2 HD (C23: c23_490.000MHz_10Msps_amp120.cs8, 490M, --play 4164)
cargo run --release -p declab-cli --example retune_probe -- FILE voice HZ_A HZ_B 3   # retune a running decoder from one carrier to another mid-file, as a click does; prints the view each 0.25 s
cargo run --release -p declab-cli --example vfo_probe -- FILE pagers HZ_1 voice HZ_2   # two VFOs on a recording, as the GUI runs them; both views each 0.5 s
cargo test -p declab-decoders --release relock_after_retune -- --ignored --nocapture   # Digital voice: time to relock after a retune or a drag, each mode pair
cargo run --release -p declab-cli -- run wfm --net tcp://127.0.0.1:7777 --iq int8 --rate 10M --freq 98M --offset 0.8M   # raw IQ over the network: SDR++ IQ Exporter (Baseband, TCP (Server)), GNU Radio sinks; also listen://ADDR:PORT, udp://ADDR:PORT; rtl_tcp recognised and tuned
cargo run --release -p declab-gui -- --net tcp://127.0.0.1:7777 --iq int8 --rate 10M --freq 98M --decoder wfm --chan 98.8M
cargo run --release -p declab-io --example net_send -- out/sim_fm_98.000MHz_10Msps.cs8 10M tcp://127.0.0.1:7777 int8   # stand-in for SDR++'s IQ Exporter: plays a file out in real time (tcp:// listens, connect://, udp://)
cargo run --release -p declab-cli -- list
cargo run --release -p declab-cli -- selftest level
cargo run --release -p declab-cli -- run wfm --file x_98.000MHz_10Msps.cs8 --offset 1.2M --realtime
cargo run --release -p declab-cli -- run nfm --radio airspy --freq 446.05M --offset 0
cargo run --release -p declab-cli -- run usb --radio hackrf --freq 14.2M --gain 1,32,20
cargo run --release -p declab-cli -- run wfm --radio airspy --freq 98M --rate 6M   # an Airspy Mini's rates (6M, 3M; an R2 has 10M and 2.5M): the board takes the nearest to --rate
cargo run --release -p declab-gui -- out/sim_sat_gse.cf32 --rate 8M --freq 1023M --decoder sat --chan 1021.8M --set lo_hz=5150000000 --set lo_above=1 --screenshot out/x.png --after 7   # behind a C-band LNB (L.O. above): shown as 4128.2 MHz, the band drawn turned over
cargo run --release -p declab-cli -- run wfm --radio pluto --freq 98M --rate 2.5M --gain 50   # ADALM-PLUTO / Pluto+, receive only (--pluto-uri auto|usb|ip:HOST, --gain DB|agc, --pluto-16bit)
cargo test -p declab-io --features pluto live_pluto -- --ignored --nocapture   # a Pluto attached (env PLUTO_URI, default auto)
cargo run --release -p declab-cli -- run wfm --radio fobos --freq 98M --rate 8M --gain 1,8   # Fobos SDR (--gain LNA,VGA; --fobos-ext-clock); also --radio sdrplay (--gain LNASTATE,IFGR, --sdrplay-tuner-b) and --radio uhd (--gain DB, --uhd-args, --uhd-antenna)
cargo run --release -p declab-cli -- run wfm --radio remote --remote 127.0.0.1:5262 --remote-radio file --freq 98M --rate 10M --offset 0.8M   # a radio behind DecSAT's decsat-server (--remote-password-file, --remote-bits, --remote-zstd, --gain NAME=V,…)
cargo test -p declab-io --features remote remote   # the server link against a stand-in server on loopback
DECLAB_FOBOS_LIB=/path/to/lib cargo test -p declab-io --features fobos live_fobos -- --ignored --nocapture   # a Fobos attached; live_sdrplay likewise for an SDRplay
```

Only an Airspy R2 is plugged in at the moment (the HackRF is not). All
the drivers (HackRF, Airspy, RTL-SDR, Pluto) are pure Rust over nusb and need the
WinUSB driver (the Pluto also works over TCP). The Pluto driver
(`declab-io/src/pluto*`, from DecDVB-T) is receive only: it never opens the
transmit side. The user's own
DecDVB-T GUI may be holding the Airspy ("Access is denied"): never kill it.

**The radios from DecSAT** (`declab-io/src/fobos.rs`, `sdrplay.rs`, `uhd.rs`,
`remote*`; features `fobos`, `sdrplay`, `uhd`, `remote`, all on by default):
the Fobos SDR, the SDRplay RSPs and a USRP load their maker's library at run
time (`fobos_sdr.dll`/`fobos.dll`, `sdrplay_api.dll`, `uhd.dll`; UHD on
Windows only), so nothing is needed to build them; `DECLAB_FOBOS_LIB` and
`DECLAB_SDRPLAY_LIB` name a library file to use instead. `remote` is the
client of DecSAT's `decsat-server` in its IQ mode (the wire types in
`remote/msg.rs` must keep DecSAT's layout). All receive only. The GUI's
settings and controls for the four are in `apps/declab-gui/src/more_radios.rs`.
None has been run on its hardware here: the remote link is tested against a
stand-in server, the others only compile.

**Network IQ** (`declab-io/src/net.rs`) takes headerless IQ from another
program, so any radio SDR++ drives (the SDRplay RSPduo, say) works on
Windows and Linux. The stream says neither rate nor frequency: give both.

The GUI keeps its settings in `%APPDATA%\DecLAB\prefs.txt`; screenshot runs
neither read nor write it.

libxaac (DAB+ audio) builds from the `third_party/libxaac` submodule
(`git submodule update --init`); the first build takes a minute.

## Adding a decoder

Copy `crates/declab-decoders/src/level.rs`, implement `Decoder` and
`DecoderFactory` (see `crates/declab-api/src/lib.rs`), add `mod` and the
factory to `ALL` in that crate's `lib.rs`.

## ISM devices (rtl_433 ports)

`crates/declab-ism/src/devices/*.rs` are ports of rtl_433's device files;
the reference clone is `..\rtl_433-ref` (never commit it). A new device:
a module with `DEVICES` and `SAMPLES` (rows as its slicer outputs them,
valid checks, no real IDs), listed in `devices/mod.rs`; the tests then
check it through its decoder, its slicer and the whole receiver.

## LoRa mesh (Meshtastic, MeshCore)

`crates/declab-lora`: the coding chain is ported from gr-lora_sdr, the
Meshtastic field numbers from meshtastic/protobufs (`mt_names.rs` is
generated from them), MeshCore's layouts from its docs. Reference clones
are in `..\lora-ref` (gr-lora_sdr, protobufs, MeshCore; never commit them).
The receiver works at one sample a chip: read the module docs of
`receiver.rs` (the wrap's phase step) before changing it, and check with
`lora_sweep` and `filetest` (CPU) after. Live captures and screenshots
of real meshes carry node IDs, names and positions: never commit them.

## P25 phase 2

`crates/declab-pmr/src/p25p2`: layouts ported from OP25, MAC fields from
DSD-FME; reference clones in `..\op25-ref` and `..\dsd-fme-ref` (never
commit them). The scrambling seed is solved for from the channel's own
ACCHs (`scramble.rs` module docs) or set with `p25p2_seed`.

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
