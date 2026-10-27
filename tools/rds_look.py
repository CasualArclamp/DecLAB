# made by CasualArclamp
"""Look at a station's FM multiplex in a .cs8 capture: pilot, RDS level.
Usage: python rds_look.py FILE OFFSET_HZ [seconds]"""
import sys

import numpy as np
from scipy import signal

FS = 10e6
path, off = sys.argv[1], float(sys.argv[2])
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
n = int(FS * secs)
raw = np.fromfile(path, dtype=np.int8, count=2 * n).astype(np.float32)
x = (raw[0::2] + 1j * raw[1::2]) / 128.0
t = np.arange(len(x)) / FS
x = x * np.exp(-2j * np.pi * off * t)
# To 400 kS/s: 10 MHz / 25.
h = signal.firwin(401, 120e3, fs=FS)
y = signal.lfilter(h, 1.0, x)[::25]
fs = FS / 25
mpx = np.angle(y[1:] * np.conj(y[:-1])) * fs / (2 * np.pi * 75e3)
f, p = signal.welch(mpx, fs=fs, nperseg=1 << 15)
db = 10 * np.log10(p + 1e-20)


def band(lo, hi):
    m = (f >= lo) & (f <= hi)
    return db[m].max(), f[m][db[m].argmax()]


print("pilot peak %.1f dB at %.1f Hz" % band(18.5e3, 19.5e3))
print("noise 16-17 kHz %.1f dB" % db[(f > 16e3) & (f < 17e3)].mean())
print("57 kHz +-2.4 kHz mean %.1f dB, max %.1f" % (db[(f > 54.6e3) & (f < 59.4e3)].mean(), db[(f > 54.6e3) & (f < 59.4e3)].max()))
print("noise 60-62 kHz %.1f dB" % db[(f > 60e3) & (f < 62e3)].mean())
print("noise 66-70 kHz %.1f dB" % db[(f > 66e3) & (f < 70e3)].mean())
# RDS amplitude as a fraction of 75 kHz deviation: power in 54.6-59.4 kHz.
m = (f > 54.6e3) & (f < 59.4e3)
pw = np.trapezoid(p[m], f[m])
mn = (f > 60e3) & (f < 64.8e3)
pn = np.trapezoid(p[mn], f[mn])
print("RDS rms %.4f of full deviation (noise in same width %.4f)" % (np.sqrt(pw), np.sqrt(pn)))
mp = (f > 18.9e3) & (f < 19.1e3)
print("pilot rms %.4f" % np.sqrt(np.trapezoid(p[mp], f[mp])))
