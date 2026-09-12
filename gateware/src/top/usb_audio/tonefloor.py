"""
Noise floor and near-carrier sideband power of a looped-back pure tone,
for comparing MCLK jitter between builds under identical analog conditions.
Usage: tonefloor.py rec.wav f0
"""
import sys, numpy as np, wave
fn = sys.argv[1]; f0 = float(sys.argv[2])
with wave.open(fn) as w:
    nch, fs, n = w.getnchannels(), w.getframerate(), w.getnframes()
    x = np.frombuffer(w.readframes(n), dtype='<i4').reshape(-1, nch)
c = x[:, 1].astype(np.float64) / 2**31
idx = np.flatnonzero(np.abs(c) > 0.01); c = c[idx[0] + 2*fs: idx[-1] - 2*fs]
N = 2**16; hop = N
win = np.hanning(N); wsum = np.sum(win**2)
psd = np.zeros(N//2 + 1)
frames = 0
for s0 in range(0, len(c) - N, hop):
    X = np.fft.rfft(c[s0:s0+N] * win)
    psd += np.abs(X)**2 / (wsum * fs); frames += 1
psd /= frames
f = np.fft.rfftfreq(N, 1/fs)
df = f[1] - f[0]
def band_power(lo, hi):
    m = (f >= lo) & (f < hi)
    return np.sum(psd[m]) * df
tone = band_power(f0 - 50, f0 + 50)
def db(p): return 10*np.log10(p + 1e-30)
print(f"{fn}: {len(c)/fs:.0f} s, tone {f0:.0f} Hz at {db(tone):.1f} dBFS")
for lo, hi, label in ((20, f0-200, "below tone (20 Hz..f0-200)"), (f0+200, 20000, "above tone (f0+200..20k)"),
                      (f0-2000, f0-100, "close-in sidebands -2k..-100"), (f0+100, f0+2000, "close-in sidebands +100..+2k")):
    if lo < hi:
        print(f"  {label:34s}: {db(band_power(lo, hi)):7.1f} dBFS  ({db(band_power(lo,hi)) - db(tone):+.1f} dBc)")
h2, h3 = band_power(2*f0-50, 2*f0+50) if 2*f0 < 24000 else 0, band_power(3*f0-50, 3*f0+50) if 3*f0 < 24000 else 0
print(f"  harmonics 2f/3f: {db(h2)-db(tone):+.1f} / {db(h3)-db(tone):+.1f} dBc")
