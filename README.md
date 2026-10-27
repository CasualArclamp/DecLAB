<!-- made by CasualArclamp -->
# DecLAB

A software-defined receiver for the DVB family and DAB in one app: DVB-T and
DVB-T2, DVB-S, DVB-S2/S2X (transport stream and GSE/IP), DVB-RCS2 return
bursts, and DAB/DAB+. HackRF One or IQ recordings.

Every decoder is a plug-in module: it is handed IQ samples and hands back
transport streams, IP packets, audio, pictures and measurements, so new
decoders can be added in a single file. See [docs/DESIGN.md](docs/DESIGN.md).

Status: M1 — the shared front end (HackRF One, Airspy R2, IQ files), the
decoder host, analog AM/DSB/USB/LSB/CW/NFM/WFM decoders, the CLI, and the
GUI: spectrum and waterfall (left click tunes, middle-drag moves the band,
scroll zooms) and a decoder wheel on a middle click. DVB and DAB decoders
come next.

Licence: GPL-3.0-or-later.
