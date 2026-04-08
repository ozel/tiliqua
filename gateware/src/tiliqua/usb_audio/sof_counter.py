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
    """

    # Inputs
    sof_detected: In(1)       # Pulse when USB SOF is detected (sync domain)
    audio_clock_tick: In(1)   # Pulse per audio clock edge (sync domain)

    # Outputs
    measured_count: Out(24)   # Audio clock ticks in last measurement window
    measurement_valid: Out(1) # Strobes when a new measurement is available
    locked: Out(1)            # High once at least one measurement has been taken

    def __init__(self, *, sof_accumulation=32):
        super().__init__()
        self.sof_accumulation = sof_accumulation
        # Number of bits needed for the SOF sub-counter
        self._sof_bits = (sof_accumulation - 1).bit_length()

    def elaborate(self, platform):
        m = Module()

        audio_counter = Signal(24)
        sof_sub = Signal(self._sof_bits)

        with m.If(self.audio_clock_tick):
            m.d.sync += audio_counter.eq(audio_counter + 1)

        with m.If(self.sof_detected):
            m.d.sync += sof_sub.eq(sof_sub + 1)

            # When the SOF sub-counter wraps (every N SOFs),
            # latch the audio counter as a measurement and reset.
            with m.If(sof_sub == self.sof_accumulation - 1):
                m.d.sync += [
                    sof_sub.eq(0),
                    self.measured_count.eq(audio_counter + 1),
                    self.measurement_valid.eq(1),
                    self.locked.eq(1),
                    audio_counter.eq(0),
                ]
            with m.Else():
                m.d.sync += self.measurement_valid.eq(0)
        with m.Else():
            m.d.sync += self.measurement_valid.eq(0)

        return m
