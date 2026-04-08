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
    kp_shift : int
        Proportional gain as right-shift amount. Gain = 1 / 2^kp_shift.
    ki_shift : int
        Integral gain as right-shift amount. Gain = 1 / 2^ki_shift.
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
                 kp_shift=4,
                 ki_shift=10):
        super().__init__()
        self.ref_freq = ref_freq
        self.target_freq = target_freq
        self.sof_accumulation = sof_accumulation
        self.accum_bits = accum_bits
        self.kp_shift = kp_shift
        self.ki_shift = ki_shift

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
        # PI update:
        #   fcw += (error >> kp_shift) + integrator_update
        #   integrator += error >> ki_shift

        expected = Signal(24, reset=self._expected_count)
        error = Signal(signed(24))
        integrator = Signal(signed(32))

        with m.If(sof_counter.measurement_valid):
            m.d.sync += error.eq(expected - sof_counter.measured_count)

        # Apply correction one cycle after error is computed
        # (to avoid long combinational path through subtraction + shift + add)
        error_d1 = Signal(signed(24))
        valid_d1 = Signal()
        m.d.sync += [
            error_d1.eq(error),
            valid_d1.eq(sof_counter.measurement_valid),
        ]

        p_term = Signal(signed(32))
        i_update = Signal(signed(32))
        m.d.comb += [
            p_term.eq(error_d1 >> self.kp_shift),
            i_update.eq(error_d1 >> self.ki_shift),
        ]

        with m.If(valid_d1):
            m.d.sync += [
                integrator.eq(integrator + i_update),
                fcw.eq(self._nominal_fcw + p_term + integrator + i_update),
            ]

        # --- Outputs ---
        m.d.comb += [
            self.locked.eq(sof_counter.locked),
            self.dbg_fcw.eq(fcw),
            self.dbg_error.eq(error),
            self.dbg_measured.eq(sof_counter.measured_count),
        ]

        return m
