"""
Decode the device telemetry embedded in capture channels 2/3 by
`pdm usb_audio build --adaptive --telemetry`, from a 4-channel S32 WAV
recorded with e.g.:

    pw-record --target <tiliqua pro-input node> --rate 48000 --channels 4 --format s32 rec.wav

Field layout is documented in top.py. Prints a per-interval summary of
DAC FIFO level (as seen every sample), underrun events, PLL error and FCW
deviation, and capture fill/skip events, plus the glitch scan of the
audio on the loudest of channels 0/1 (expects a pure sine of `--f0` Hz).

Usage: python telemetry_decode.py rec.wav [--interval 1.0] [--f0 1000]
"""
import argparse
import wave
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("wav")
ap.add_argument("--interval", type=float, default=1.0)
ap.add_argument("--f0", type=float, default=1000.0)
ap.add_argument("--events", type=int, default=30, help="max glitch events to list")
args = ap.parse_args()

with wave.open(args.wav) as w:
    nch, fs, n = w.getnchannels(), w.getframerate(), w.getnframes()
    x = np.frombuffer(w.readframes(n), dtype='<i4').reshape(-1, nch)
assert nch == 4, "need the 4-channel capture"

# Telemetry words live in the top 16 bits of the 32-bit samples.
t2 = (x[:, 2].astype(np.int64) >> 16) & 0xffff
t3 = (x[:, 3].astype(np.int64) >> 16) & 0xffff
dac_level = t2 & 0x1f
k = (t2 >> 5) & 3
primed = (t2 >> 7) & 1
out_active = (t2 >> 8) & 1
locked = (t2 >> 9) & 1
adc_level = (t2 >> 10) & 0x1f

def s16(v):
    v = v.astype(np.int64)
    return np.where(v >= 0x8000, v - 0x10000, v)

underruns = t3[k == 0]
err = s16(t3[k == 1])
fcw_dev = s16(t3[k == 2]) * 64
fills = t3[k == 3] & 0xff
skips = (t3[k == 3] >> 8) & 0xff

nominal_fcw = round((12_288_000 / 60_000_000) * 2**32)
ppm = fcw_dev / nominal_fcw * 1e6

print(f"{args.wav}: {n/fs:.1f} s at {fs} Hz")
print(f"locked {locked.mean()*100:.1f}% of samples, out_active {out_active.mean()*100:.1f}%, primed {primed.mean()*100:.1f}%")
print(f"underrun counter: start {underruns[0]} end {underruns[-1]} (+{(underruns[-1]-underruns[0]) & 0xffff})")
print(f"capture fill events: +{(fills[-1]-fills[0]) & 0xff}, skip events: +{(skips[-1]-skips[0]) & 0xff}")

# Audio glitch scan on loudest of ch0/ch1.
a = x[:, :2].astype(np.float64) / 2**31
ch = int(np.argmax(np.sqrt(np.mean(a**2, axis=0))))
c = a[:, ch]
active = np.abs(c) > 0.02
idx = np.flatnonzero(active)
events = []
if len(idx) > fs:
    lo, hi = idx[0] + fs // 2, idx[-1] - fs // 2
    seg = c[lo:hi]
    amp = np.sqrt(2) * np.sqrt(np.mean(seg**2))
    w0 = 2 * np.pi * args.f0 / fs
    r = seg[2:] - 2 * np.cos(w0) * seg[1:-1] + seg[:-2]
    noise = np.sqrt(np.mean(r**2))
    thr = max(0.02 * amp, 8 * noise)
    bad = np.flatnonzero(np.abs(r) > thr)
    for i in bad:
        if events and i - events[-1][-1] <= 4:
            events[-1].append(i)
        else:
            events.append([i])
    print(f"audio ch{ch}: {len(seg)/fs:.1f} s analysed, amplitude {amp:.3f}, residual floor {20*np.log10(noise+1e-12):.1f} dBFS, glitch events {len(events)}")
    for e in events[:args.events]:
        i = lo + e[0] + 1
        pk = np.max(np.abs(r[e[0]:e[-1]+1]))
        # phase jump estimate in samples from residual magnitude: |1-e^{jw d}| * amp
        d = np.arccos(np.clip(1 - (pk/amp)**2/2, -1, 1)) / w0
        print(f"  t={i/fs:9.4f}s residual {pk:.4f} (~{d:4.1f} samples) dac_level={dac_level[i]:2d} primed={primed[i]} "
              f"underruns={underruns[min(i//4, len(underruns)-1)]} adc_level={adc_level[i]}")
else:
    print("no audio signal on ch0/ch1")

# Per-interval summary.
step = int(args.interval * fs)
print(f"\n{'t[s]':>7} {'dac min/mean/max':>17} {'adc max':>7} {'underruns':>9} {'err':>5} {'fcw ppm':>8} {'fill':>4} {'skip':>4} {'act':>3} {'lk':>2}")
for s0 in range(0, n, step):
    s1 = min(n, s0 + step)
    sl = slice(s0, s1)
    kk = k[sl]
    u = t3[sl][kk == 0]; e = s16(t3[sl][kk == 1]); f = s16(t3[sl][kk == 2]) * 64
    fl = t3[sl][kk == 3] & 0xff; sk = (t3[sl][kk == 3] >> 8) & 0xff
    dl = dac_level[sl]
    print(f"{s0/fs:7.1f} {dl.min():5d}/{dl.mean():5.1f}/{dl.max():3d}   {adc_level[sl].max():5d}   {u[-1] if len(u) else 0:7d} {int(np.median(e)) if len(e) else 0:5d} "
          f"{np.median(f)/nominal_fcw*1e6 if len(f) else 0:8.1f} {fl[-1] if len(fl) else 0:4d} {sk[-1] if len(sk) else 0:4d} "
          f"{out_active[sl].mean():3.1f} {locked[sl].mean():2.0f}")
