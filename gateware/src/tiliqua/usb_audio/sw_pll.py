# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
Software PLL (NCO + control loop) for adaptive USB audio clock recovery.

Generates an audio master clock (MCLK) from a phase accumulator NCO,
disciplined by USB SOF timing and by the DAC FIFO fill level. When the
host sends audio at its own clock rate, this module adjusts the local
audio clock so that the codec consumes samples at exactly the rate the
host delivers them, and so that the DAC FIFO sits at a chosen setpoint.

Structure
---------

Two phase accumulators run on the same reference clock (60 MHz usb domain):

* A *reference* NCO driven by ``fcw_base`` only. Its wrap (carry) events are
  counted per window of ``sof_accumulation`` SOFs. The frequency loop steers
  ``fcw_base`` so that this count equals the nominal ticks-per-window, i.e.
  the reference NCO is locked to the host's SOF clock. The correction is
  incremental (``fcw_base += Kp * error``) so the steady-state error is zero
  and there is no reliance on a slow integrator.

* An *output* NCO driven by ``fcw_base + level_term``. Its MSB is
  ``mclk_out``. ``level_term`` is proportional to the DAC FIFO level error
  measured at each SOF, so the audio clock runs marginally fast when the
  FIFO is above the setpoint and marginally slow below it.

Measuring a separate reference NCO (rather than the output NCO) is what
lets the two loops coexist: the frequency loop never sees, and therefore
never fights, the level correction.

Ticks are counted as accumulator carries in the same clock domain, so no
edge detector is needed and no tick can be lost.
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out

from .sof_counter import SOFCounter


class SwPLL(wiring.Component):
    """
    Software PLL: NCO disciplined by USB SOF timing and DAC FIFO level.

    Parameters
    ----------
    ref_freq : int
        Reference clock frequency (sync domain), typically 60 MHz.
    target_freq : int
        Target audio clock frequency, e.g. 12_288_000 for 48 kHz audio.
    sof_accumulation : int
        Number of SOF frames per measurement window (power of 2).
    accum_bits : int
        Width of the phase accumulators.
    kp_num, kp_den : int
        Proportional gain of the incremental frequency loop, as a fraction
        of the "deadbeat" gain (one tick of error corrects exactly one tick
        per window). 1/2 halves the error every window and is robust to
        gain mismatch. Values >= 2 are unstable.
    level_setpoint : int
        DAC FIFO level (in samples, as sampled at SOF time, i.e. just before
        the microframe's packet arrives) that the level loop aims for.
    level_gain_ppm : float
        Frequency correction per sample of level error, in ppm. The level
        loop is first order with time constant 1/(gain * fs); 20 ppm at
        48 kHz gives about 1 s.
    max_ppm : float
        Clamp for ``fcw_base`` around nominal. Protects the codec from a
        wildly wrong clock if SOFs are missing or garbage.
    """

    # Inputs
    sof_detected: In(1)        # SOF pulse (sync domain)
    fifo_level: In(8)          # DAC FIFO level (sync domain, write side)
    stream_active: In(1)       # Host is streaming to us; enables the level loop

    # Outputs
    mclk_out: Out(1)           # Recovered audio master clock
    locked: Out(1)             # Frequency loop has settled

    # Debug outputs
    dbg_fcw: Out(32)           # Current base frequency control word
    dbg_error: Out(signed(24)) # Last measured frequency error (ticks/window)
    dbg_measured: Out(24)      # Last measured reference tick count
    dbg_level_term: Out(signed(32))  # Current level correction (FCW units)

    def __init__(self, *,
                 ref_freq=60_000_000,
                 target_freq=12_288_000,
                 sof_accumulation=32,
                 accum_bits=32,
                 kp_num=1, kp_den=2,
                 level_setpoint=6,
                 level_gain_ppm=20.0,
                 max_ppm=2000.0,
                 lock_threshold=1,
                 lock_hold=16):
        super().__init__()
        self.ref_freq = ref_freq
        self.target_freq = target_freq
        self.sof_accumulation = sof_accumulation
        self.accum_bits = accum_bits
        self.level_setpoint = level_setpoint
        self.lock_threshold = lock_threshold
        self.lock_hold = lock_hold

        # Nominal frequency control word: FCW = (target_freq / ref_freq) * 2^accum_bits
        self._nominal_fcw = round((target_freq / ref_freq) * (2 ** accum_bits))

        # Expected reference ticks per window. SOF rate is 8 kHz (USB HS microframes).
        assert (target_freq * sof_accumulation) % 8000 == 0, \
            "target_freq * sof_accumulation must be a whole number of ticks per window"
        self._expected_count = (target_freq * sof_accumulation) // 8000

        # FCW change that moves the per-window count by exactly one tick.
        self._tick_fcw = self._nominal_fcw / self._expected_count
        self._kp_gain = round(self._tick_fcw * kp_num / kp_den)
        self._level_gain = round(self._nominal_fcw * level_gain_ppm * 1e-6)
        self._fcw_limit = round(self._nominal_fcw * max_ppm * 1e-6)

    def elaborate(self, platform):
        m = Module()

        # --- Reference NCO: measured by the SOF counter ---
        fcw_base = Signal(32, init=self._nominal_fcw)
        accum_ref = Signal(self.accum_bits)
        ref_tick = Signal()
        ref_sum = Signal(self.accum_bits + 1)
        m.d.comb += [
            ref_sum.eq(accum_ref + fcw_base),
            ref_tick.eq(ref_sum[-1]),        # carry out = one wrap = one tick
        ]
        m.d.sync += accum_ref.eq(ref_sum[:-1])

        m.submodules.sof_counter = sof_counter = SOFCounter(
            sof_accumulation=self.sof_accumulation,
            lock_threshold=self.lock_threshold,
            lock_hold=self.lock_hold)
        m.d.comb += [
            sof_counter.sof_detected.eq(self.sof_detected),
            sof_counter.audio_clock_tick.eq(ref_tick),
        ]

        # --- Frequency loop (incremental proportional on tick error) ---
        # error = expected - measured. Positive: reference NCO too slow.
        error = Signal(signed(24))
        valid_d1 = Signal()
        with m.If(sof_counter.measurement_valid):
            m.d.sync += error.eq(self._expected_count - sof_counter.measured_count)
        m.d.sync += valid_d1.eq(sof_counter.measurement_valid)

        correction = Signal(signed(40))
        m.d.comb += correction.eq(error * self._kp_gain)

        fcw_next = Signal(signed(40))
        lo = self._nominal_fcw - self._fcw_limit
        hi = self._nominal_fcw + self._fcw_limit
        m.d.comb += fcw_next.eq(fcw_base + correction)
        with m.If(valid_d1):
            with m.If(fcw_next < lo):
                m.d.sync += fcw_base.eq(lo)
            with m.Elif(fcw_next > hi):
                m.d.sync += fcw_base.eq(hi)
            with m.Else():
                m.d.sync += fcw_base.eq(fcw_next)

        error_abs = Signal(24)
        m.d.comb += [
            error_abs.eq(Mux(error < 0, -error, error)),
            sof_counter.error_abs.eq(error_abs),
        ]

        # --- Level loop (proportional on DAC FIFO level, sampled at SOF) ---
        # Level above setpoint -> positive term -> output NCO faster -> FIFO drains.
        level_err = Signal(signed(9))
        level_term = Signal(signed(32))
        with m.If(self.sof_detected):
            with m.If(self.stream_active):
                m.d.sync += level_err.eq(self.fifo_level - self.level_setpoint)
            with m.Else():
                m.d.sync += level_err.eq(0)
        m.d.comb += level_term.eq(level_err * self._level_gain)

        # --- Output NCO ---
        accum_out = Signal(self.accum_bits)
        fcw_out = Signal(32)
        m.d.comb += fcw_out.eq(fcw_base + level_term)
        m.d.sync += accum_out.eq(accum_out + fcw_out)
        m.d.comb += self.mclk_out.eq(accum_out[-1])

        # --- Outputs ---
        m.d.comb += [
            self.locked.eq(sof_counter.locked),
            self.dbg_fcw.eq(fcw_base),
            self.dbg_error.eq(error),
            self.dbg_measured.eq(sof_counter.measured_count),
            self.dbg_level_term.eq(level_term),
        ]

        return m
