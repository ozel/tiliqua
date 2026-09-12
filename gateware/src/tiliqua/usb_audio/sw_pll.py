# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
NCO actuator for adaptive USB audio clock recovery.

A 32-bit phase accumulator in the reference (usb, 60 MHz) domain generates
MCLK; its wrap events are the ticks measured by :class:`ClockRecovery`, and
its frequency control word is nominal plus the loop's ``ctrl_total``.

The MSB of a 60 MHz accumulator toggles after 4 or 5 reference cycles, so
this clock carries about +-8 ns of period jitter. :class:`Si5351Tuner` is
the low-jitter alternative.
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out

from .clock_recovery import ClockRecovery, CTRL_FRAC_BITS


class SwPLL(wiring.Component):
    """NCO disciplined by :class:`ClockRecovery`. See there for loop parameters."""

    sof_detected: In(1)
    fifo_level: In(8)
    stream_active: In(1)

    mclk_out: Out(1)
    locked: Out(1)

    dbg_fcw: Out(32)
    dbg_error: Out(signed(24))
    dbg_measured: Out(24)
    dbg_ctrl_freq: Out(signed(32))

    def __init__(self, *, ref_freq=60_000_000, target_freq=12_288_000,
                 sof_accumulation=32, accum_bits=32, **loop_kwargs):
        super().__init__()
        self.accum_bits = accum_bits
        self.loop = ClockRecovery(target_freq=target_freq,
                                  sof_accumulation=sof_accumulation, **loop_kwargs)
        self._nominal_fcw = round((target_freq / ref_freq) * (2 ** accum_bits))
        # FCW change per ctrl unit (1/256 ppm), as a 2**16 fixed-point constant.
        self._fcw_k = round(self._nominal_fcw * 1e-6 / (2 ** CTRL_FRAC_BITS) * 2 ** 16)
        # Conveniences for tests.
        self._expected_count = self.loop._expected_count
        self._tick_fcw = self._nominal_fcw / self._expected_count
        self._fcw_limit = round(self._nominal_fcw * loop_kwargs.get("max_ppm", 2000.0) * 1e-6)

    def elaborate(self, platform):
        m = Module()
        m.submodules.loop = loop = self.loop

        fcw = Signal(32, init=self._nominal_fcw)
        delta = Signal(signed(48))
        m.d.comb += delta.eq((loop.ctrl_total * self._fcw_k) >> 16)
        m.d.sync += fcw.eq(self._nominal_fcw + delta)

        accum = Signal(self.accum_bits)
        acc_sum = Signal(self.accum_bits + 1)
        m.d.comb += acc_sum.eq(accum + fcw)
        m.d.sync += accum.eq(acc_sum[:-1])

        m.d.comb += [
            loop.sof_detected.eq(self.sof_detected),
            loop.clk_tick.eq(acc_sum[-1]),      # carry out: one wrap = one tick
            loop.fifo_level.eq(self.fifo_level),
            loop.stream_active.eq(self.stream_active),
            self.mclk_out.eq(accum[-1]),
            self.locked.eq(loop.locked),
            self.dbg_fcw.eq(fcw),
            self.dbg_error.eq(loop.dbg_error),
            self.dbg_measured.eq(loop.dbg_measured),
            self.dbg_ctrl_freq.eq(loop.ctrl_freq),
        ]
        return m
