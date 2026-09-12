# Tiliqua USB Audio — Adaptive sync mode

`pdm usb_audio build --adaptive` builds a UAC2 device whose isochronous
endpoints are declared *adaptive* instead of *asynchronous*. There is no
feedback endpoint; the host sends exactly 6 samples per microframe at its
own USB clock, and the device recovers an audio clock from that stream.

## How the clock is recovered

All of this lives in `usb_audio/sw_pll.py` and runs in the 60 MHz `usb`
domain.

Two 32-bit phase accumulators (NCOs) run side by side:

| NCO | driven by | purpose |
|---|---|---|
| reference | `fcw_base` | measured by the SOF counter; locked to the host's SOF rate |
| output | `fcw_base + level_term` | its MSB is MCLK for the whole `audio` domain |

**Frequency loop.** `sof_counter.py` counts reference-NCO wraps (carry
outs, so no edge detector and no lost ticks) per window of 32 SOFs and
compares against the nominal 49152. The correction is incremental,
`fcw_base += error * Kp`, with Kp equal to half the deadbeat gain, so the
error halves every 4 ms window and settles to the ±1 tick quantisation
(one tick is 20 ppm). `fcw_base` is clamped to ±2000 ppm of nominal.

**Level loop.** At every SOF, just before that microframe's packet lands,
the DAC FIFO level is sampled and compared to the setpoint (6 of 16). The
difference times 20 ppm per sample is added to the output NCO only. A
FIFO above setpoint therefore drains slightly faster and vice versa, with
a first-order time constant of about 1 s. The loop is disabled (term
forced to zero) while the host is not streaming.

Because the frequency loop measures the *reference* NCO and never the
output NCO, the two loops do not fight: the level term is invisible to
the SOF measurement.

**Prefill.** In adaptive mode `AudioToChannels` holds the DAC FIFO read
side until 12 samples (setpoint plus one packet) have arrived, then
streams. It re-arms whenever the FIFO runs dry, so an underrun costs one
250 µs resync rather than continuous glitching.

**Clock path.** The output NCO's MSB drives the `audio` domain through a
DCCA global clock buffer for the whole life of the bitstream. There is no
runtime clock mux (a fabric mux would glitch the audio domain and its
async FIFO pointers at the moment of switching) and the SI5351 `clk0` is
simply unused. Before lock the free-running NCO is already within crystal
tolerance of 12.288 MHz.

Known limitation: an NCO clock has ±8 ns period jitter (the MSB toggles
after 4 or 5 reference cycles). This is inherent to the approach and
bounds converter SNR at high audio frequencies. Steering the SI5351's
fractional divider over I2C from the same loops would remove it.

## What went wrong in the first version

For reference, the closed-loop simulation in `tests/test_sw_pll.py`
reproduces all three failure modes of the original implementation:

1. **No FIFO level control.** Only frequency was locked, so the DAC FIFO
   sat wherever the stream start left it, typically near empty, and any
   microframe of host packet jitter underran.
2. **One tick lost per five windows.** A reference tick landing on the
   same cycle as the window-closing SOF was overwritten by the counter
   reset. The loop compensated by running MCLK ~4 ppm fast, draining the
   FIFO at ~0.2 samples/s: a glitch every ~5 s once the FIFO emptied.
3. **Proportional term recomputed from scratch.** `fcw = nominal + P + I`
   meant only the tiny integrator held a persistent offset, giving a ~2 s
   lock time constant and several ticks of residual error.

## Testing

### Simulation

`pdm run pytest tests/test_sw_pll.py` runs the loop closed at a scaled
clock (600 kHz reference, 8-SOF windows) with a host clock offset and a
FIFO model, asserting lock, zero lost ticks, and level convergence.

### Device telemetry (recommended first step on hardware)

`pdm usb_audio build --adaptive --telemetry` replaces capture channels 2
and 3 with device state, recorded sample-aligned with the audio:

| word | contents |
|---|---|
| ch2 | `[4:0]` DAC FIFO level, `[6:5]` field index k, `[7]` prefill released, `[8]` host streaming, `[9]` PLL locked, `[14:10]` ADC FIFO level |
| ch3, k=0 | prefill re-arm (underrun) count |
| ch3, k=1 | PLL error, ticks per window (signed) |
| ch3, k=2 | (FCW − nominal) / 64 (signed) |
| ch3, k=3 | `[7:0]` capture zero-fill events, `[15:8]` capture discard events |

Record with the AUX channel map, otherwise PipeWire zeroes the unmapped
channels 2/3:

```
pw-record --target alsa_input.usb-apf.audio_Tiliqua_beta-0000-00.pro-input-0 \
          --rate 48000 --channels 4 --channel-map AUX0,AUX1,AUX2,AUX3 --format s32 rec.wav
python src/top/usb_audio/telemetry_decode.py rec.wav --interval 5
```

With a cable from an output to an input and a 1 kHz sine playing, the
decoder also scans the audio for discontinuities and prints the DAC level
and counters at each one. A pure sine is used because it passes any linear
analog path unchanged, and a dropped or repeated sample breaks the exact
second-difference identity `x[n+1] - 2cos(w) x[n] + x[n-1] = 0`. Do not
use a ramp: its edges ring through the codec filters.

### Results (2026-09-11, Tiliqua R5, Arch host, xHCI)

Raw ALSA (`aplay`/`arecord -D hw:Tiliqua`, PipeWire profile set to off),
1 kHz sine at −20 dBFS, output 1 cabled to input 1:

| bitstream | run | mid-stream glitches | DAC FIFO level | notes |
|---|---|---|---|---|
| async (feedback endpoint) | 120 s | 10, one every ~10 s, each a 1-sample zero pad | not instrumented | capture packetiser padding when the ADC clock runs behind the packet rate mirrored from the host |
| original adaptive (branch head) | 120 s | 18, one every 5–10 s, each a 1-sample insert | not instrumented | matches the "never worked" report |
| new adaptive | 120 s | 0 | 4–10, never below 4 | one re-prime at stream start, one at stream end |
| new adaptive | 626 s, 3 file restarts | 0 between restarts | 4–10 in every 5-minute stretch | per restart: one re-prime plus three single-sample zero pads from the capture packetiser within 1.2 s |

The frequency loop locks within 20 ms and sits at ±1 tick (20 ppm) of
quantisation; the host's USB clock measured −6.2 ppm relative to the
Tiliqua 60 MHz crystal.

**PipeWire caveat.** Through PipeWire (pro-audio profile, either node as
driver, any quantum, zero xruns reported) the same recording shows clean
±16-sample phase jumps every 15–30 s. Device telemetry across those events
is undisturbed, and they vanish with raw ALSA, so they are PipeWire
resyncs: it marks both Tiliqua nodes as sharing `clock.name api.alsa.1`
and corrects position drift between the driver and follower node by
jumping rather than resampling. Judge the device with raw ALSA, or watch
the telemetry counters, not the PipeWire recording alone.

### Cross-clock sync test against another USB sound card (2026-09-12)

Setup: a C-Media full-speed USB card (adaptive OUT endpoint, so its DAC
clock also follows the host's SOF timing) plays a 1 kHz sine into Tiliqua
input 0; Tiliqua output 0 loops back to input 1; raw ALSA on both cards,
10.6 minutes, telemetry on channels 2/3. `xclock.py` fits the 1 kHz
component per second and tracks its phase.

| | adaptive | async (SI5351) |
|---|---:|---:|
| other card's tone, frequency offset vs Tiliqua clock | +0.007 ppm | +15.2 ppm |
| accumulated phase drift over 636 s | −0.01 samples | +435 samples |
| tone wander around the fit (rms) | 0.50 samples | 6.3 samples |
| capture zero-pad events | 10 (9 of them at stream start) | 117 (one every ~1.3 s while full duplex) |
| DAC FIFO level | 4–10, 0 only at start/end | 0–13 |

Two adaptive devices on one host therefore stay sample-locked
indefinitely; two async devices walk apart at 15 ppm, about 0.7 samples
per second. The async capture packetiser also pads a zero frame whenever
the ADC (SI5351 clock) falls behind the IN packet size, which is
mirrored from the host's OUT packets, giving the regular zero-pads above.

### Further hardware checks

1. **Stress.** Start/stop the stream a hundred times, change the quantum
   mid-stream, go through a hub, use a second host with a different
   crystal.
2. **Frequency.** Play 1 kHz from Tiliqua into a second sound card and
   measure it. Adaptive and async bitstreams should differ by exactly the
   SI5351 versus host-USB-clock offset, and adaptive must not wander.
3. **Two Tiliquas** in adaptive mode on one host, both playing the same
   sine, captured together: they must stay sample-aligned indefinitely
   (the C-Media test above is the single-Tiliqua version of this).
