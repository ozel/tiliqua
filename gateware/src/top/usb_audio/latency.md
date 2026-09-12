# Tiliqua USB Audio — Round-Trip Latency Measurement

*Measured in the default asynchronous (feedback endpoint) mode. For the
adaptive mode, its clock recovery loop and the device telemetry / sine
loopback test that also catches single-sample dropouts (which
`jack_iodelay` cannot see), read [adaptive.md](adaptive.md).*

Measurement of end-to-end round-trip latency (RTL) through the Tiliqua
`usb_audio` core, from USB host → FPGA → AK4619 DAC → analog loopback
cable → AK4619 ADC → FPGA → USB host.

## Setup

### Host system

| Item | Value |
|---|---|
| OS | Arch Linux, GNOME |
| Kernel | 6.18.22-1-lts |
| Clocksource | tsc |
| Audio stack | PipeWire with `pipewire-jack` compatibility |
| USB connection | High-speed (480 Mbit/s), USB 2.0 |
| Sample rate | 48 000 Hz (forced via `clock.force-rate`) |

### Gateware configuration

| Item | Value | Source |
|---|---|---|
| `fs` | 48 000 Hz | [gateware/src/tiliqua/usb_audio/__init__.py:60](gateware/src/tiliqua/usb_audio/__init__.py#L60) |
| `nr_channels` | 4 | same |
| `max_packet_size` | 128 B | [gateware/src/tiliqua/usb_audio/__init__.py:62](gateware/src/tiliqua/usb_audio/__init__.py#L62) |
| Audio-class sync mode | ASYNC isochronous w/ feedback EP | [gateware/src/tiliqua/usb_audio/__init__.py:188-207](gateware/src/tiliqua/usb_audio/__init__.py#L188-L207) |
| Audio microframe interval | 125 µs | |
| `dac_fifo` / `adc_fifo` depth | 16 samples each | [gateware/src/tiliqua/usb_audio/__init__.py:420](gateware/src/tiliqua/usb_audio/__init__.py#L420) |
| DAC FIFO nominal target | ~half (8 samples); observed ~12 | [gateware/src/tiliqua/usb_audio/__init__.py:437](gateware/src/tiliqua/usb_audio/__init__.py#L437) |
| ADC FIFO steady state | ~0–1 samples (no feedback on this side) | observed via LED bar |
| Feedback update interval | every 32 SOFs (≈32 ms) | [gateware/src/tiliqua/usb_audio/__init__.py:434](gateware/src/tiliqua/usb_audio/__init__.py#L434) |
| Codec | AK4619 on eurorack-pmod | — |

## Method

Physical loopback cable connecting `output_0` → `input_0` on the
eurorack-pmod. `jack_iodelay` sends an impulse and cross-correlates
the received signal to derive the total loopback delay in frames.

```
pactl set-card-profile alsa_card.usb-apf.audio_Tiliqua_beta-0000-00 pro-audio
pw-metadata -n settings 0 clock.force-rate 48000
pw-metadata -n settings 0 clock.force-quantum <Q>
pw-jack jack_iodelay &
pw-jack jack_connect jack_delay:out  "Tiliqua Pro:playback_AUX0"
pw-jack jack_connect "Tiliqua Pro:capture_AUX0" jack_delay:in
```

`jack_iodelay` reports:
- **total roundtrip latency** — measured end-to-end delay.
- **extra loopback latency** — total minus what the PipeWire graph
  claims, i.e. everything below the host software layer (USB driver
  buffering + FPGA FIFOs + codec group delay).

## Results

### Sweep across buffer sizes

| quantum | total (fr) | total (ms) | PW graph (2×q) | extra (fr) | stability |
|---:|---:|---:|---:|---:|---|
|  32 |  115 |  2.40 |  64 | 51 | feedback-loop hunting visible (bimodal extra 39↔51) |
|  64 |  168 |  3.50 | 128 | 40 | stable — **recommended for low-latency work** |
| 128 |  293 |  6.11 | 256 | 37 | very stable |
| 256 |  537 | 11.19 | 512 | 25 | very stable |
| 512 | 1049 | 21.86 | 1024 | 25 | very stable |

Model: `total = 2 × quantum + extra`. Slope exactly 2 frames per
quantum-frame confirms PipeWire is running the Tiliqua as a driver
node (no resampling, no extra PW graph buffering).

### ASCII plot — total RTL vs quantum

```
 total round-trip latency (ms)

 22 ┤                                                        ●  q=512 (21.86)
 20 ┤
 18 ┤
 16 ┤                                       slope ≈ 2 × q / fs
 14 ┤                                   ⋯⋯⋯⋯
 12 ┤                               ●  q=256 (11.19)
 10 ┤                           ⋯⋯⋯
  8 ┤
  6 ┤                  ●  q=128 (6.11)
  4 ┤            ●  q=64 (3.50)
  2 ┤      ●  q=32 (2.40)  ← feedback hunting
  0 ┼──────┬────────┬────────┬──────────────┬──────────────────────────●──
         32        64      128             256                         512
                          quantum (frames)
```

### ASCII plot — device-only "extra loopback" vs quantum

```
 extra loopback (frames @ 48 kHz)

 55 ┤  ●  q=32: 51 (DAC FIFO overshoots
 50 ┤     |       feedback setpoint)
 45 ┤     |
 40 ┤     |     ●  q=64: 40
 35 ┤     ╲     |     ●  q=128: 37
 30 ┤      ╲    |     |
 25 ┤       ●⋯⋯⋯●⋯⋯⋯⋯●⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯●⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯●
 20 ┤                    q=256: 25             q=512: 25
 15 ┤                    ← floor reached (FPGA FIFOs + codec GD only)
 10 ┤
  5 ┤
  0 ┼──────┬────────┬────────┬──────────────┬──────────────────────────●──
         32        64      128             256                         512
                          quantum (frames)
```

At `q ≥ 256` the hardware contribution plateaus at **~25 frames ≈
520 µs** — the irreducible floor of the flashed design.

## USB frame buffering — why it doesn't appear as a separate term

USB 2.0 high-speed delivers one audio microframe every **125 µs** — at
48 kHz that's **6 samples per channel per microframe**. So samples
never arrive as a smooth flow; they arrive in burst-per-microframe on
both the OUT (host → device) and IN (device → host) paths.

### Host side

| Layer | Where | Typical size |
|---|---|---|
| PipeWire ring (software) | userspace | 2 × quantum frames (reported as PW graph latency) |
| ALSA PCM ring + snd-usb-audio URB queue | kernel | ≥ 2 URBs of ~1 ms each, sized by the driver |
| USB host-controller DMA | xHCI/EHCI hardware | ≤ 1 microframe (~6 frames) ahead |

`jack_iodelay`'s "extra loopback latency" is
`total_measured − PipeWire_graph_claim`. The PipeWire graph claim
already tracks the software ring size the driver is asked to maintain,
so the kernel URB queue piggy-backs onto that same reservoir rather
than stacking on top of it.

### Gateware side (LUNA + usb_audio)

On the device end there are **two microframe-scale buffers** sitting
either side of the `audio_to_channels` FIFOs that are usually discussed
as "the" FIFOs:

| Path | Buffer | Depth | Source | Steady state |
|---|---|---:|---|---:|
| Playback (IN from host) | LUNA `USBIsochronousStreamOutEndpoint.fifo` | 2 × `max_packet_size` = **256 B = 16 fr** | [isochronous_stream_out.py:53,100](https://github.com/greatscottgadgets/luna) | ~0–3 fr (backpressured empty by `dac_fifo.w_rdy`) |
| Playback (→ codec) | `audio_to_channels.dac_fifo` | **16 fr** | [audio_to_channels.py:105](gateware/src/tiliqua/usb_audio/audio_to_channels.py#L105) | ~12 fr (feedback setpoint) |
| Capture (codec →) | `audio_to_channels.adc_fifo` | **16 fr** | [audio_to_channels.py:52](gateware/src/tiliqua/usb_audio/audio_to_channels.py#L52) | ~0–1 fr |
| Capture (OUT to host) | `ChannelsToUSBStream.out_fifo` | 2 × `max_packet_size` = **256 B = 16 fr** | [channels_to_usb_stream.py:50](gateware/src/tiliqua/usb_audio/channels_to_usb_stream.py#L50) | ~6 fr (prefetch kept above `no_channels × 4` bytes by the `FILL` state at [channels_to_usb_stream.py:192-193](gateware/src/tiliqua/usb_audio/channels_to_usb_stream.py#L192-L193)) |
| Capture (→ host) | LUNA `USBIsochronousStreamInEndpoint` | **0** (pure passthrough FSM) | [isochronous_stream_in.py](https://github.com/greatscottgadgets/luna) | 0 |

So **LUNA/Tiliqua buffers up to 2 microframes (~250 µs / 16 frames)
in each direction**, but the capture-side 2-microframe buffer lives in
`ChannelsToUSBStream.out_fifo`, not in LUNA itself. Its level is
**not** currently exposed in `USB2AudioInterface.DebugInterface` — the
LED bar can't see it.

### Why the measured `extra` looks flat

What remains visible as "extra" at `q ≥ 256` is:

- **HCI DMA head-of-queue** (≤ 1 microframe, absorbed into FIFO level
  fluctuation on both paths).
- **Four gateware FIFOs** in steady state: ~12 (dac) + ~1 (adc) + ~6
  (`out_fifo` prefetch) + ~0 (LUNA RX, drained) ≈ **19 frames**.
- **AK4619 codec group delay** (~14 frames combined, short-delay FIR).
- A handful of frames of I²S/SOF alignment.

Total matches the measured ~25–27 frames. Adding more host-side
quantum does **not** increase the gateware overhead — the USB frame
mechanism is a rate-matched pipeline, not a serial buffer stack.

## Decomposition of the hardware floor (~25 frames @ 48 kHz)

The gateware FIFO levels were visualised on an 8-LED PMOD (see the
LED bar visualiser in [gateware/src/top/usb_audio/top.py](gateware/src/top/usb_audio/top.py))
and directly confirm the steady-state occupancies used here:

| Contributor | Frames | Time | How measured / known |
|---|---:|---:|---|
| AK4619 ADC group delay (short-delay FIR, ~6/fs) | ~6 | ~125 µs | Datasheet (short-delay mode) |
| AK4619 DAC group delay (short-delay FIR, ~8/fs) | ~8 | ~167 µs | Datasheet (short-delay mode) |
| `dac_fifo` steady-state occupancy | ~12 | ~250 µs | Observed: feedback loop settles at ~12/16 — see [__init__.py:437](gateware/src/tiliqua/usb_audio/__init__.py#L437), the `>> 3` correction term equilibrates wherever host/device XO drift cancels |
| LUNA `USBIsochronousStreamOutEndpoint` RX FIFO | ~0–3 | ~0–63 µs | 2-µframe capacity, backpressured nearly empty by `dac_fifo.w_rdy` |
| `adc_fifo` steady-state occupancy | ~0–1 | ~0–21 µs | Observed: host drains it as fast as samples arrive — LED4 solid, LED5–7 dark |
| `ChannelsToUSBStream.out_fifo` prefetch | ~4–6 | ~83–125 µs | FSM's `FILL` state holds level above `no_channels × 4` bytes = 4 frames; average lands just above this floor unless USB drain jitters |
| I²S TDM serialization / SOF alignment | ~1 | ~21 µs | Structural |
| **Total (range)** | **~27–34** | **~560–710 µs** | Measured: 25 at q≥256. Codec GD numbers are upper-bound estimates; actual AK4619 short-delay FIR may be a few frames lower. |

The lion's share of the floor is the **two 2-microframe buffers** on
either side of the audio path (one in LUNA, one in `ChannelsToUSBStream`)
plus the **~12-frame DAC FIFO occupancy** from the feedback loop's
equilibrium. Together those three account for ~22 of the ~25 observed
floor frames; the codec contributes only a handful.

Note the DAC FIFO is the single largest contributor to the fixed
floor — and it's there deliberately, to give the async feedback loop
room to correct for host/device clock drift without underrunning. The
ADC FIFO, lacking a feedback path, runs essentially empty.

At small quantum the feedback loop at
[gateware/src/tiliqua/usb_audio/__init__.py:437](gateware/src/tiliqua/usb_audio/__init__.py#L437)
updates only every 32 ms, slower than the host wake-up period. The
DAC FIFO transiently overshoots/undershoots its setpoint during each
loop update, adding the ~12–26 frames of extra overhead seen at
`q ≤ 128`.

## Findings

- **`jack_iodelay` measures delay, not continuity.** A later sine-loopback
  scan (see [adaptive.md](adaptive.md)) showed that async mode pads one
  zero sample into the capture stream roughly every 10 s, invisible to
  this measurement.

- **Fixed hardware contribution: ~25 frames / 520 µs.** This is
  excellent for a class-compliant USB audio interface.
- **The design scales ideally with buffer size** — a simple
  `total = 2 × quantum + floor` line fits all stable points.
- **q = 64 is the practical low-latency sweet spot.** 3.5 ms RTL,
  fully stable, no dropouts.
- **q = 32 shows the feedback loop hunting.** Extra loopback is
  bimodal between 39 and 51 frames — the loop update period (32 ms)
  is longer than the host wake-up, so the DAC FIFO swings around its
  setpoint. Audibly usable if the host CPU can keep up, but no longer
  deterministic.
- Best-case hardware floor only appears at `q ≥ 256`, where the
  feedback loop has time to settle. This is a property of the
  loop period, not of the FIFO depth.

## Paths to lower RTL

In order of increasing invasiveness:

1. **Stay at q = 64** — already near optimal for stock firmware.
2. **Tighten the feedback loop** at
   [gateware/src/tiliqua/usb_audio/__init__.py:434](gateware/src/tiliqua/usb_audio/__init__.py#L434)
   — update every 8 or 16 SOFs instead of 32. Should cut the extra
   loopback at small quantum from ~50 frames back to the ~25-frame
   floor and may allow stable operation at q = 32 (~2 ms RTL).
3. **Shift the DAC FIFO setpoint down** (same file, line 437) — the
   `>> 3` proportional term currently equilibrates at ~12/16. Changing
   the shift / adding an offset to target ~6/16 would reclaim ~6 frames
   directly. Risk: less margin for XO drift; must go hand-in-hand with
   (2) to keep underrun probability unchanged.
4. **Reduce FIFO depth** at
   [gateware/src/tiliqua/usb_audio/__init__.py:420](gateware/src/tiliqua/usb_audio/__init__.py#L420)
   from `16 → 8`. **Careful**: the DAC FIFO currently runs at ~12/16
   (75% full), so a depth of 8 would require (2)+(3) first or the FIFO
   will underrun on the first feedback cycle. Safer stepping stone: 12.
5. **AK4619 filter mode** — the observed ~520 µs floor is already
   consistent with short-delay FIR being active. If a future firmware
   switches to normal FIR (sharp roll-off), expect an additional
   ~20 frames of codec group delay.

## Raw `jack_iodelay` output snippets

```
q = 32  (feedback hunting)
   115.117 frames      2.398 ms total roundtrip latency
       extra loopback latency: 51 frames
   (with transient modes: 39, 43, 52–54, 22/63 outliers
    as the feedback loop settles)

q = 64
   168.117 frames      3.502 ms total roundtrip latency
       extra loopback latency: 40 frames

q = 128
   293.117 frames      6.107 ms total roundtrip latency
       extra loopback latency: 37 frames

q = 256
   537.117 frames     11.190 ms total roundtrip latency
       extra loopback latency: 25 frames

q = 512
  1049.117 frames     21.857 ms total roundtrip latency
       extra loopback latency: 25 frames
```
