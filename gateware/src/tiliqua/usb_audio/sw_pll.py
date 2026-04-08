# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
Software PLL (NCO + PI control loop) for adaptive USB audio clock recovery.

Generates an audio master clock (MCLK) from a phase accumulator NCO,
disciplined by SOF timing measurements. When the host sends audio at
its own clock rate, this module adjusts the local audio clock to match.

The NCO runs in the default (sync) clock domain (expected 60 MHz).
Output `mclk_out` is the recovered audio clock signal.
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out
from amaranth.lib.cdc import FFSynchronizer

from .sof_counter import SOFCounter
from .util import EdgeToPulse


class SwPLL(wiring.Component):
    """
    Software PLL: NCO disciplined by USB SOF timing.

    Parameters
    ----------
    ref_freq : int
        Reference clock frequency (sync domain), typically 60 MHz.
    target_freq : int
        Target audio clock frequency, e.g. 12_288_000 for 48 kHz audio.
    sof_accumulation : int
        Number of SOF frames per measurement window (power of 2).
    accum_bits : int
        Width of the phase accumulator. More bits = finer frequency resolution.
    kp_shift_left : int
        Proportional gain as left-shift amount. Gain = 2^kp_shift_left.
        One tick of measurement error multiplies by this to give the FCW
        nudge. The measurement window is `sof_accumulation` SOFs, so
        1 tick ≈ 1/(target_freq/8000 * sof_accumulation) of MCLK — e.g.
        1/49152 ≈ 20 ppm at default settings. A kp_shift_left of 14
        gives ~19 ppm FCW correction per tick-of-error, roughly matched
        to the resolution of the measurement.
    ki_shift_left : int
        Integral gain as left-shift amount. Gain = 2^ki_shift_left. Each
        measurement window adds `error << ki_shift_left` to the integrator.
    """

    # Inputs
    sof_detected: In(1)        # SOF pulse (sync domain)
    audio_clock_tick: In(1)    # Audio clock tick pulse (sync domain, from FFSync+EdgeToPulse)

    # Outputs
    mclk_out: Out(1)           # Recovered audio master clock
    locked: Out(1)             # PLL has locked (at least one measurement cycle)

    # Debug outputs
    dbg_fcw: Out(32)           # Current frequency control word
    dbg_error: Out(signed(24)) # Last measured error
    dbg_measured: Out(24)      # Last measured audio clock count

    def __init__(self, *,
                 ref_freq=60_000_000,
                 target_freq=12_288_000,
                 sof_accumulation=32,
                 accum_bits=32,
                 kp_shift_left=14,
                 ki_shift_left=6):
        super().__init__()
        self.ref_freq = ref_freq
        self.target_freq = target_freq
        self.sof_accumulation = sof_accumulation
        self.accum_bits = accum_bits
        self.kp_shift_left = kp_shift_left
        self.ki_shift_left = ki_shift_left

        # Calculate nominal frequency control word:
        # FCW = (target_freq / ref_freq) * 2^accum_bits
        self._nominal_fcw = int((target_freq / ref_freq) * (2 ** accum_bits))

        # Expected audio clock count per measurement window:
        # At 48kHz (MCLK = 12.288 MHz), SOF rate = 8 kHz (USB HS microframe)
        # ticks_per_sof = 12_288_000 / 8_000 = 1536
        # expected_count = ticks_per_sof * sof_accumulation
        self._expected_count = int(target_freq / 8000) * sof_accumulation

    def elaborate(self, platform):
        m = Module()

        # --- SOF Counter (measures audio clock ticks per N SOFs) ---
        m.submodules.sof_counter = sof_counter = SOFCounter(
            sof_accumulation=self.sof_accumulation)
        m.d.comb += [
            sof_counter.sof_detected.eq(self.sof_detected),
            sof_counter.audio_clock_tick.eq(self.audio_clock_tick),
        ]
        # |error| routed back to SOFCounter's lock-detection FSM.
        error_abs = Signal(24)

        # --- Phase Accumulator (NCO) ---
        accum = Signal(self.accum_bits)
        fcw = Signal(32, reset=self._nominal_fcw)

        # NCO: increment phase accumulator by FCW every clock cycle
        m.d.sync += accum.eq(accum + fcw)

        # Output clock: MSB of accumulator (50% duty cycle)
        m.d.comb += self.mclk_out.eq(accum[-1])

        # --- PI Control Loop ---
        # When a new measurement arrives, compute error and adjust FCW.
        #
        # Error = expected_count - measured_count
        #   Positive error: audio clock is too slow, increase FCW
        #   Negative error: audio clock is too fast, decrease FCW
        #
        # PI update (multiply-based, so sub-integer tick errors also drive
        # a correction):
        #   p_term  = error << kp_shift_left
        #   i_update = error << ki_shift_left
        #   integrator += i_update
        #   fcw = nominal + p_term + integrator

        expected = Signal(24, reset=self._expected_count)
        error = Signal(signed(24))
        # Integrator holds accumulated shifted errors; size for many seconds
        # of sustained drift without saturation.
        integrator = Signal(signed(40))

        # Latch error one cycle after measurement_valid; valid_d1 stays aligned
        # with the now-fresh error register. (The old code used a second delay
        # on `error` which misaligned with valid_d1 and effectively threw away
        # each window's correction.)
        valid_d1 = Signal()
        with m.If(sof_counter.measurement_valid):
            m.d.sync += error.eq(expected - sof_counter.measured_count)
        m.d.sync += valid_d1.eq(sof_counter.measurement_valid)

        # Shifted error terms; widened to absorb the left-shift without truncation.
        p_term = Signal(signed(40))
        i_update = Signal(signed(40))
        m.d.comb += [
            p_term.eq(error << self.kp_shift_left),
            i_update.eq(error << self.ki_shift_left),
        ]

        with m.If(valid_d1):
            m.d.sync += [
                integrator.eq(integrator + i_update),
                fcw.eq(self._nominal_fcw + p_term + integrator + i_update),
            ]

        # Feed |error| back to the SOF counter's lock-detection FSM.
        m.d.comb += [
            error_abs.eq(Mux(error < 0, -error, error)),
            sof_counter.error_abs.eq(error_abs),
        ]

        # --- Outputs ---
        m.d.comb += [
            self.locked.eq(sof_counter.locked),
            self.dbg_fcw.eq(fcw),
            self.dbg_error.eq(error),
            self.dbg_measured.eq(sof_counter.measured_count),
        ]

        return m
