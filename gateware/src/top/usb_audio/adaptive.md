# Tiliqua USB Audio — Adaptive sync mode

`pdm usb_audio build --adaptive` builds a UAC2 device whose isochronous
endpoints are declared *adaptive* instead of *asynchronous*. There is no
feedback endpoint; the host sends exactly 6 samples per microframe at its
own USB clock, and the device recovers an audio clock from that stream.

## How the clock is recovered

`usb_audio/clock_recovery.py` runs in the 60 MHz `usb` domain and is
independent of what generates the audio clock. It counts audio-clock ticks
per window of 32 SOFs and produces a control value in 1/256 ppm:

* **Frequency loop.** `sof_counter.py` counts ticks per window (4 ms) and
  compares against the nominal 49152. The correction is incremental,
  `ctrl_freq += error * Kp`, with Kp half the deadbeat gain, so the error
  halves every window and settles to the ±1 tick quantisation (one tick is
  20 ppm). Clamped to ±2000 ppm.
* **Level loop.** At the window-closing SOF, just before that microframe's
  packet lands, the DAC FIFO level is compared to the setpoint (6 of 16).
  The difference times 20 ppm per sample is `ctrl_level`. A FIFO above
  setpoint drains slightly faster and vice versa, with a first-order time
  constant of about 1 s. Forced to zero while the host is not streaming.
* The actuator applies `ctrl_total = ctrl_freq + ctrl_level`. Because the
  measured clock carries the level term too, its known contribution
  (`ctrl_level * 49152 * 1e-6` ticks per window) is subtracted from the
  measurement, so the frequency loop never fights the level loop.

Two actuators, selected at build time with `--adaptive-clock`:

| | `si5351` (default) | `nco` |
|---|---|---|
| audio clock source | SI5351A clk0, as in async mode | 32-bit phase accumulator in the FPGA, MSB through a DCCA |
| how it is tuned | `usb_audio/si5351_tuner.py` rewrites PLLA's 20-bit fractional numerator over the mobo I2C bus (0x60, 100 kHz) once per window; one step is 0.027 ppm | frequency control word |
| jitter on MCLK | the SI5351's own, tens of ps | ±8 ns period jitter (MSB toggles after 4 or 5 reference cycles) |
| extra hardware use | mobo I2C bus, shared with LED driver, USB-C controller and EDID | none |
| clock path | unchanged, `expll_clk0` | audio domain clocked from fabric for the life of the bitstream |

SI5351 details: the tuner recomputes the bootloader's PLLA settings (a=35,
b=408357, c=2^20−1, VCO 884.736 MHz, MultiSynth 72) and only ever changes
`b`. Updating the fractional numerator is glitch-free and needs no PLL
reset; the chip's spread-spectrum engine modulates that same divider. With
`c = 2^20−1` the register math needs no divider (`floor(128b/c) = b >> 13`).
At start-up the tuner clears the spread-spectrum enable bit, since the
bootloader turns on a 1 % spread by default and that would add periodic
jitter to MCLK; the bitstream manifest for this mode also requests no
spread. Each update writes registers 26–33 in one 10-byte transaction,
about 1 ms at 100 kHz, a quarter of the 4 ms window.

**Prefill.** In adaptive mode `AudioToChannels` holds the DAC FIFO read
side until 12 samples (setpoint plus one packet) have arrived, then
streams. It re-arms whenever the FIFO runs dry, so an underrun costs one
250 µs resync rather than continuous glitching.

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
clock with a host clock offset and a FIFO model, for both actuators:
the NCO directly, and the SI5351 path through the tuner, a bus model that
ACKs and decodes the I2C bytes, and a behavioural SI5351 whose frequency
follows the written numerator. Asserted: lock, zero lost ticks, clamping,
level convergence from above and below, the prefill handshake, and the
exact register bytes of the start-up and update transactions.

### Device telemetry (recommended first step on hardware)

`pdm usb_audio build --adaptive --telemetry` replaces capture channels 2
and 3 with device state, recorded sample-aligned with the audio:

| word | contents |
|---|---|
| ch2 | `[4:0]` DAC FIFO level, `[6:5]` field index k, `[7]` prefill released, `[8]` host streaming, `[9]` PLL locked, `[14:10]` ADC FIFO level |
| ch3, k=0 | prefill re-arm (underrun) count |
| ch3, k=1 | PLL error, ticks per window (signed) |
| ch3, k=2 | loop frequency control in 1/16 ppm (signed) |
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
| new adaptive, NCO | 626 s, 3 file restarts | 0 between restarts | 4–10 in every 5-minute stretch | per restart: one re-prime plus three single-sample zero pads from the capture packetiser within 1.2 s |
| new adaptive, SI5351 | 120 s | 0 | 4–10, never below 4 | loop control settled at +18.5 ppm; one re-prime at stream start, one at stream end |

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

| | adaptive, NCO | adaptive, SI5351 | async (SI5351 free-running) |
|---|---:|---:|---:|
| other card's tone, frequency offset vs Tiliqua clock | +0.007 ppm | −0.004 ppm | +15.2 ppm |
| accumulated phase drift over 636 s | −0.01 samples | −0.06 samples | +435 samples |
| tone wander around the fit (rms) | 0.50 samples | 0.22 samples | 6.3 samples |
| loopback wander around the fit (rms) | 2.4 samples | 0.55 samples | 5.2 samples |
| capture zero-pad events | 10 (9 of them at stream start) | 6 (all at stream start) | 117 (one every ~1.3 s while full duplex) |
| DAC FIFO level | 4–10, 0 only at start/end | 4–10, 0 only at start/end | 0–13 |
| loop frequency control over the run | −6.2 ppm, steady | +7 to +26 ppm, tracking the SI5351 crystal's drift | n/a |

Two adaptive devices on one host therefore stay sample-locked
indefinitely; two async devices walk apart at 15 ppm, about 0.7 samples
per second. The async capture packetiser also pads a zero frame whenever
the ADC (SI5351 clock) falls behind the IN packet size, which is
mirrored from the host's OUT packets, giving the regular zero-pads above.

### MCLK jitter: 10 kHz tone noise floor (2026-09-12)

Same loopback, a 10 kHz sine at −20 dBFS for 90 s per build, raw ALSA,
`tonefloor.py`. Jitter noise scales with signal frequency, so 10 kHz is
where clock quality shows above the analog floor. Tone lands at −26.4 dBFS
on the input in all three cases.

| build | audio clock | floor 20 Hz..9.8 kHz | close-in ±0.1..2 kHz | 2nd harmonic |
|---|---|---:|---:|---:|
| adaptive, SI5351 | SI5351 retuned, spread spectrum off | −81.6 dBFS | −79.3 dBFS | −76.4 dBc |
| adaptive, NCO | 60 MHz accumulator MSB | −79.7 dBFS | −77.7 dBFS | −76.3 dBc |
| async | SI5351 as programmed by the bootloader, 1 % spread spectrum on | −73.0 dBFS | −70.0 dBFS | −75.4 dBc |

The tuned SI5351 is 2 dB cleaner than the NCO and 9 dB cleaner than the
stock async clock. Most of that 9 dB is the bootloader's default 1 %
spread spectrum on PLLA, which also feeds the codec in async mode; that
default is worth revisiting independently of adaptive mode.

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
