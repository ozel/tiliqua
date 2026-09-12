# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
SOF timing measurement module for adaptive USB audio clock recovery.

Counts audio clock ticks between USB SOF (Start of Frame) events,
accumulating over N SOF frames for noise averaging. This measurement
is used by the software PLL to discipline an NCO to track the host's
USB clock.
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out
from amaranth.lib.cdc import FFSynchronizer

from .util import EdgeToPulse


class SOFCounter(wiring.Component):
    """
    Measures audio clock ticks per N USB SOF frames.

    Operates in the default (sync) clock domain. The audio clock
    is synchronized in via FFSynchronizer + EdgeToPulse.

    Parameters
    ----------
    sof_accumulation : int
        Number of SOF frames to accumulate before producing a measurement.
        Must be a power of 2. Default 32 (matching USB2 spec chapter 5.12.4.2).
    lock_threshold : int
        Measurements must report |error| ≤ `lock_threshold` ticks before the
        `locked` output is asserted. An `error` input signal must be driven
        externally for this to work; if left at 0 the locked signal asserts
        on the first measurement. Default: 2 ticks.
    lock_hold : int
        Number of consecutive in-threshold measurements required before
        asserting `locked`. Default 16 ≈ 64 ms at 32-SOF accumulation.
    """

    # Inputs
    sof_detected: In(1)       # Pulse when USB SOF is detected (sync domain)
    audio_clock_tick: In(1)   # Pulse per audio clock edge (sync domain)
    error_abs: In(24)         # |loop error| from SwPLL; used to gate `locked`

    # Outputs
    measured_count: Out(24)   # Audio clock ticks in last measurement window
    measurement_valid: Out(1) # Strobes when a new measurement is available
    locked: Out(1)            # High once the loop has held error within threshold

    def __init__(self, *, sof_accumulation=32, lock_threshold=2, lock_hold=16):
        super().__init__()
        self.sof_accumulation = sof_accumulation
        self.lock_threshold = lock_threshold
        self.lock_hold = lock_hold
        # Number of bits needed for the SOF sub-counter
        self._sof_bits = (sof_accumulation - 1).bit_length()

    def elaborate(self, platform):
        m = Module()

        audio_counter = Signal(24)
        sof_sub = Signal(self._sof_bits)

        # Count consecutive in-threshold measurements before declaring lock.
        lock_counter = Signal(range(self.lock_hold + 1))

        with m.If(self.audio_clock_tick):
            m.d.sync += audio_counter.eq(audio_counter + 1)

        with m.If(self.sof_detected):
            m.d.sync += sof_sub.eq(sof_sub + 1)

            # When the SOF sub-counter wraps (every N SOFs),
            # latch the audio counter as a measurement and reset.
            with m.If(sof_sub == self.sof_accumulation - 1):
                m.d.sync += [
                    sof_sub.eq(0),
                    # A tick landing on this exact cycle belongs to the window
                    # being closed. It must be folded into the measurement and
                    # not simply overwritten by the reset below, otherwise the
                    # loop sees a ~0.2 tick/window undercount and runs the NCO
                    # fast by ~4 ppm forever (slow, permanent FIFO drain).
                    self.measured_count.eq(audio_counter + self.audio_clock_tick),
                    self.measurement_valid.eq(1),
                    audio_counter.eq(0),
                ]
                # Lock-state machine: count consecutive in-threshold measurements.
                with m.If(self.error_abs <= self.lock_threshold):
                    with m.If(lock_counter == self.lock_hold):
                        m.d.sync += self.locked.eq(1)
                    with m.Else():
                        m.d.sync += lock_counter.eq(lock_counter + 1)
                with m.Else():
                    # Out of threshold: reset counter. Don't un-latch `locked`
                    # once achieved, to avoid dropping the audio clock mux.
                    m.d.sync += lock_counter.eq(0)
            with m.Else():
                m.d.sync += self.measurement_valid.eq(0)
        with m.Else():
            m.d.sync += self.measurement_valid.eq(0)

        return m
