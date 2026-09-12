# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
Clock recovery loop for adaptive USB audio, independent of the actuator.

Measures ticks of the audio clock (whatever generates it) per window of
``sof_accumulation`` USB SOFs and produces a frequency control value in
units of 1/256 ppm relative to the actuator's nominal frequency:

* ``ctrl_freq``  - integrated frequency correction (host clock vs local),
* ``ctrl_level`` - proportional DAC FIFO level correction,
* ``ctrl_total`` - their sum, what the actuator must apply.

Both corrections are updated once per window. Because the measured clock
carries the level correction too, its expected contribution
(``ctrl_level * expected_count * 1e-6``) is subtracted from the measurement
before the frequency loop sees it, so the two loops do not fight.

Actuators: :class:`SwPLL` (NCO) and :class:`Si5351Tuner` (external PLL).
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out

from .sof_counter import SOFCounter


CTRL_FRAC_BITS = 8   # ctrl values are ppm * 2**8


class ClockRecovery(wiring.Component):
    """
    Parameters
    ----------
    target_freq : int
        Nominal audio clock frequency being measured (12_288_000 for 48 kHz).
    sof_accumulation : int
        SOFs per measurement window (power of two).
    kp_num, kp_den : int
        Frequency loop gain as a fraction of deadbeat (one tick of error
        corrects one tick per window). 1/2 halves the error every window.
    level_setpoint : int
        DAC FIFO level (sampled at the window-closing SOF) to aim for.
    level_gain_ppm : float
        Frequency correction per sample of level error. Loop time constant
        is 1/(gain * fs): 20 ppm at 48 kHz gives about 1 s.
    max_ppm : float
        Clamp on ``ctrl_freq``.
    """

    sof_detected: In(1)
    clk_tick: In(1)            # one pulse per measured audio clock edge (sync domain)
    fifo_level: In(8)
    stream_active: In(1)

    ctrl_freq: Out(signed(32))
    ctrl_level: Out(signed(32))
    ctrl_total: Out(signed(32))
    update: Out(1)             # strobes when ctrl_* changed (once per window)
    locked: Out(1)

    dbg_error: Out(signed(24)) # last error in whole ticks
    dbg_measured: Out(24)

    def __init__(self, *, target_freq=12_288_000, sof_accumulation=32,
                 kp_num=1, kp_den=2, level_setpoint=6, level_gain_ppm=20.0,
                 max_ppm=2000.0, lock_threshold=1, lock_hold=16):
        super().__init__()
        self.sof_accumulation = sof_accumulation
        self.level_setpoint = level_setpoint
        self.lock_threshold = lock_threshold
        self.lock_hold = lock_hold
        assert (target_freq * sof_accumulation) % 8000 == 0
        self._expected_count = (target_freq * sof_accumulation) // 8000
        F = 2 ** CTRL_FRAC_BITS
        self._ppm_per_tick = 1e6 / self._expected_count
        self._kp = round(self._ppm_per_tick * kp_num / kp_den * F)       # ctrl per tick
        self._level_gain = round(level_gain_ppm * F)                      # ctrl per sample
        self._ctrl_limit = round(max_ppm * F)
        # ticks (in 1/256) contributed per window by one ctrl unit of level term:
        # expected * 1e-6, as a 2**24 fixed-point constant.
        self._comp_k = round(self._expected_count * 1e-6 * 2 ** 24)

    def elaborate(self, platform):
        m = Module()

        m.submodules.sof_counter = sof_counter = SOFCounter(
            sof_accumulation=self.sof_accumulation,
            lock_threshold=self.lock_threshold, lock_hold=self.lock_hold)
        m.d.comb += [
            sof_counter.sof_detected.eq(self.sof_detected),
            sof_counter.audio_clock_tick.eq(self.clk_tick),
        ]

        # Level at the most recent SOF (just before that microframe's packet).
        level_at_sof = Signal(signed(9))
        with m.If(self.sof_detected):
            with m.If(self.stream_active):
                m.d.sync += level_at_sof.eq(self.fifo_level - self.level_setpoint)
            with m.Else():
                m.d.sync += level_at_sof.eq(0)

        # Everything below changes once per window, so the arithmetic is
        # pipelined freely: valid_d[n] is measurement_valid delayed n cycles.
        valid_d = [sof_counter.measurement_valid]
        for n in range(6):
            d = Signal(name=f"valid_d{n+1}")
            m.d.sync += d.eq(valid_d[-1])
            valid_d.append(d)

        # Compensation for the level term (registered; ctrl_level is static
        # for a whole window before the next measurement).
        comp = Signal(signed(48))
        m.d.sync += comp.eq((self.ctrl_level * self._comp_k) >> 24)

        # error (1/256 ticks) = expected + level compensation - measured
        error_q = Signal(signed(32))
        with m.If(valid_d[0]):
            m.d.sync += error_q.eq((self._expected_count << CTRL_FRAC_BITS) + comp
                                   - (sof_counter.measured_count << CTRL_FRAC_BITS))

        # Frequency loop: incremental proportional, clamped.
        step = Signal(signed(48))
        freq_next = Signal(signed(48))
        m.d.sync += [
            step.eq((error_q * self._kp) >> CTRL_FRAC_BITS),   # valid at d2
            freq_next.eq(self.ctrl_freq + step),               # valid at d3
        ]
        with m.If(valid_d[3]):
            with m.If(freq_next < -self._ctrl_limit):
                m.d.sync += self.ctrl_freq.eq(-self._ctrl_limit)
            with m.Elif(freq_next > self._ctrl_limit):
                m.d.sync += self.ctrl_freq.eq(self._ctrl_limit)
            with m.Else():
                m.d.sync += self.ctrl_freq.eq(freq_next)
            # Level loop: proportional on the level seen at the closing SOF.
            m.d.sync += self.ctrl_level.eq(level_at_sof * self._level_gain)

        m.d.sync += self.ctrl_total.eq(self.ctrl_freq + self.ctrl_level)   # valid at d5
        m.d.comb += self.update.eq(valid_d[6])

        # Lock detection on whole-tick error magnitude.
        error_ticks = Signal(signed(24))
        m.d.comb += error_ticks.eq(error_q >> CTRL_FRAC_BITS)
        m.d.comb += [
            sof_counter.error_abs.eq(Mux(error_ticks < 0, -error_ticks, error_ticks)),
            self.locked.eq(sof_counter.locked),
            self.dbg_error.eq(error_ticks),
            self.dbg_measured.eq(sof_counter.measured_count),
        ]
        return m
