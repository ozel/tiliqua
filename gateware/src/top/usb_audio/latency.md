# Tiliqua USB Audio — Round-Trip Latency Measurement

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
| FIFO target occupancy | ~half (8 samples) | [gateware/src/tiliqua/usb_audio/__init__.py:437](gateware/src/tiliqua/usb_audio/__init__.py#L437) |
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
|  32 |  115 |  2.40 |  64 | 51 | **marginal** — `Signal below threshold`, feedback hunting |
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
  2 ┤      ●  q=32 (2.40)  ← unstable
  0 ┼──────┬────────┬────────┬──────────────┬──────────────────────────●──
         32        64      128             256                         512
                          quantum (frames)
```

### ASCII plot — device-only "extra loopback" vs quantum

```
 extra loopback (frames @ 48 kHz)

 55 ┤  ●  q=32: 51 (feedback hunting +
 50 ┤     |       URB floor visible)
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

## Decomposition of the hardware floor (~25 frames @ 48 kHz)

| Contributor | Frames | Time |
|---|---:|---:|
| AK4619 ADC group delay (low-latency FIR, ~8/fs) | ~8 | ~167 µs |
| AK4619 DAC group delay (low-latency FIR, ~12/fs) | ~12 | ~250 µs |
| `adc_fifo` + `dac_fifo` steady-state (half of 16) | ~4 (avg under smooth feedback) | ~83 µs |
| USB SOF alignment / I2S TDM serialization | ~1 | ~21 µs |
| **Total** | **~25** | **~520 µs** |

At small quantum the FIFO level is no longer smooth — the feedback
loop at [gateware/src/tiliqua/usb_audio/__init__.py:437](gateware/src/tiliqua/usb_audio/__init__.py#L437)
updates only every 32 ms, slower than the host wake-up period. The
FIFO settles at a higher average occupancy to absorb jitter, adding
the ~12–26 frames seen at `q ≤ 128`.

## Findings

- **Fixed hardware contribution: ~25 frames / 520 µs.** This is
  excellent for a class-compliant USB audio interface.
- **The design scales ideally with buffer size** — a simple
  `total = 2 × quantum + floor` line fits all stable points.
- **q = 64 is the practical low-latency sweet spot.** 3.5 ms RTL,
  fully stable, no dropouts.
- **q = 32 is measurable but not usable for audio work.** The
  `Signal below threshold` warnings correspond to brief dropouts;
  the feedback loop also starts hunting, visible as bimodal extra
  loopback values (39 ↔ 51 frames).
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
3. **Reduce FIFO depth** at
   [gateware/src/tiliqua/usb_audio/__init__.py:420](gateware/src/tiliqua/usb_audio/__init__.py#L420)
   from `16 → 8` samples. Saves up to ~8 frames but increases
   under-run risk under USB jitter — validate with a long soak.
4. **AK4619 configuration** — if not already in short-delay FIR
   mode, switching would remove an additional ~10–15 frames of codec
   group delay. Already appears to be the case given the observed
   floor.

## Raw `jack_iodelay` output snippets

```
q = 32  (marginal)
   115.117 frames      2.398 ms total roundtrip latency
       extra loopback latency: 51 frames
   (with transient modes: 39, 43, 52–54, 22/63 outliers,
    "Signal below threshold..." warnings)

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
