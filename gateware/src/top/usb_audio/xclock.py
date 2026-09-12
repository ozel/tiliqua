"""
Cross-clock analysis: ch0 carries a 1 kHz tone from another USB sound card,
ch1 the Tiliqua loopback, ch2/3 device telemetry. For each tone channel:
frequency offset (ppm) and phase drift over time from block-wise sine fits,
plus a discontinuity scan. Memory-lean (float32, chunked fits).
"""
import sys, numpy as np, wave
fn = sys.argv[1]; f0 = 1000.0; block = 1.0
with wave.open(fn) as w:
    nch, fs, n = w.getnchannels(), w.getframerate(), w.getnframes()
    raw = np.frombuffer(w.readframes(n), dtype='<i4').reshape(-1, nch)
t2 = raw[:,2]; t3 = raw[:,3]
dac_level = ((t2 >> 16) & 0x1f).astype(np.int8); k = ((t2 >> 21) & 3).astype(np.int8)
primed = ((t2 >> 23) & 1).astype(np.int8)
u = (t3[k==0] >> 16) & 0xffff; fills = (t3[k==3] >> 16) & 0xff
fcw = ((t3[k==2] >> 16) & 0xffff).astype(np.int64); fcw = np.where(fcw >= 0x8000, fcw-0x10000, fcw)*64
nominal = round(12_288_000/60_000_000*2**32)
print(f"{fn}: {n/fs:.1f} s")
if len(u):
    print(f"telemetry: underruns {u[0]}->{u[-1]}, capture fills {fills[0]}->{fills[-1]}, "
          f"PLL fcw {np.median(fcw)/nominal*1e6:+.2f} ppm (min {fcw.min()/nominal*1e6:+.2f} max {fcw.max()/nominal*1e6:+.2f}), "
          f"dac level min/max {dac_level.min()}/{dac_level.max()}, per-minute min {[int(dac_level[i:i+60*fs].min()) for i in range(0,n,60*fs)]}")
w0 = 2*np.pi*f0/fs
for ch, name in ((0, "ch0 other-card tone"), (1, "ch1 loopback")):
    c = raw[:, ch].astype(np.float32)/np.float32(2**31)
    idx = np.flatnonzero(np.abs(c) > 0.005)
    if len(idx) < fs: print(f"{name}: no signal"); continue
    lo, hi = idx[0]+fs, idx[-1]-fs
    amp = np.sqrt(2)*np.sqrt(np.mean(c[lo:hi].astype(np.float64)**2))
    # block-wise phase of the 1 kHz component relative to a fixed 1 kHz reference
    bl = int(block*fs); phases=[]; times=[]
    for s0 in range(lo, hi-bl, bl):
        nn = np.arange(s0, s0+bl, dtype=np.float64)
        seg = c[s0:s0+bl].astype(np.float64)
        I = np.mean(seg*np.cos(w0*nn)); Q = np.mean(seg*np.sin(w0*nn))
        phases.append(np.arctan2(-Q, I)); times.append(s0/fs)
    ph = np.unwrap(np.array(phases)); tt = np.array(times)
    slope = np.polyfit(tt, ph, 1)[0]            # rad/s
    ppm = slope/(2*np.pi*f0)*1e6
    resid = ph - np.polyval(np.polyfit(tt, ph, 1), tt)
    drift_samples = (ph[-1]-ph[0])/w0
    print(f"{name}: amp {amp:.4f}, {hi-lo:.0f} samples analysed; frequency offset {ppm:+.3f} ppm "
          f"(= {f0*(1+ppm*1e-6):.5f} Hz); total phase drift {drift_samples:+.2f} samples over {tt[-1]-tt[0]:.0f} s; "
          f"wander around linear fit ({resid.std()/w0:.3f} samples rms)")
    # glitch scan
    seg = c[lo:hi]
    r = seg[2:] - np.float32(2*np.cos(w0))*seg[1:-1] + seg[:-2]
    noise = np.sqrt(np.mean(r.astype(np.float64)**2)); thr = max(0.02*amp, 8*noise)
    bad = np.flatnonzero(np.abs(r) > thr); ev=[]
    for i in bad:
        if ev and i-ev[-1][-1] <= 40: ev[-1].append(i)
        else: ev.append([i])
    print(f"   residual floor {20*np.log10(noise+1e-12):.1f} dBFS, glitch events {len(ev)}: " +
          ", ".join(f"{(lo+e[0]+1)/fs:.2f}s" for e in ev[:12]))
