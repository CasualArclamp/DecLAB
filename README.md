<!-- made by CasualArclamp -->
# DecLAB

A software-defined receiver for the DVB family and DAB in one app: DVB-T and
DVB-T2, DVB-S, DVB-S2/S2X (transport stream and GSE/IP), DVB-RCS2 return
bursts, and DAB/DAB+. HackRF One or IQ recordings.

Every decoder is a plug-in module: it is handed IQ samples and hands back
transport streams, IP packets, audio, pictures and measurements, so new
decoders can be added in a single file. See [docs/DESIGN.md](docs/DESIGN.md).

Status: kickoff — the plug-in API, an example decoder, the CLI and a GUI
mode picker.

Licence: GPL-3.0-or-later.
