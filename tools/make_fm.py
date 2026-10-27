# made by CasualArclamp
"""A synthetic broadcast-FM band at 10 MS/s around 98.0 MHz, as .cs8.

Stations (offset Hz, level, stereo?, left tone, right tone):
the one at +0.8 MHz (98.8 MHz) is stereo: 440 Hz left, 1 kHz right,
pilot at 10 %. Usage: python make_fm.py OUT.cs8 [seconds]
"""
import sys

import numpy as np

FS = 10e6
DEV = 75e3
STATIONS = [
    (-3.5e6, 0.10, False, 600.0, 600.0),
    (-1.2e6, 0.05, False, 800.0, 800.0),
    (0.8e6, 0.20, True, 440.0, 1000.0),
    (2.3e6, 0.07, False, 300.0, 300.0),
]


def main():
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    total = int(FS * seconds)
    chunk = 1_000_000
    rng = np.random.default_rng(1)
    phases = [0.0] * len(STATIONS)
    with open(out, 'wb') as f:
        for start in range(0, total, chunk):
            n = min(chunk, total - start)
            t = (start + np.arange(n)) / FS
            x = np.zeros(n, dtype=np.complex128)
            for k, (off, amp, stereo, fl, fr) in enumerate(STATIONS):
                left = 0.5 * np.sin(2 * np.pi * fl * t)
                right = 0.5 * np.sin(2 * np.pi * fr * t)
                if stereo:
                    pilot = np.sin(2 * np.pi * 19e3 * t)
                    sub = np.sin(2 * np.pi * 38e3 * t)
                    m = 0.45 * (left + right) / 2 * 2 * 0.9 + 0.45 * (left - right) * sub * 0.9 + 0.1 * pilot
                else:
                    m = 0.9 * left
                inst = 2 * np.pi * (off + DEV * m) / FS
                ph = phases[k] + np.cumsum(inst)
                phases[k] = ph[-1] % (2 * np.pi)
                x += amp * np.exp(1j * ph)
            x += (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * 0.004
            iq = np.empty(2 * n, dtype=np.int8)
            iq[0::2] = np.clip(np.round(x.real * 127), -127, 127)
            iq[1::2] = np.clip(np.round(x.imag * 127), -127, 127)
            f.write(iq.tobytes())


if __name__ == '__main__':
    main()
