# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
Tests for the adaptive USB audio clock recovery (SOFCounter, SwPLL, prefill).

The SwPLL tests run the loop *closed*: the DUT's own reference NCO is what
the SOF counter measures, SOFs come from a fractional-period generator with
a programmable host clock offset, and a behavioural FIFO model converts
output NCO ticks into codec sample consumption. Everything is scaled down
(600 kHz reference, 120 kHz target, 8 SOFs/window) so a few hundred windows
simulate in seconds; the loop gains are defined relative to ticks/window so
the dynamics are representative of the 60 MHz / 12.288 MHz / 32-SOF build.
"""

import unittest

from amaranth import *
from amaranth.sim import Simulator

from tiliqua.usb_audio.sof_counter import SOFCounter
from tiliqua.usb_audio.sw_pll import SwPLL
from tiliqua.usb_audio.audio_to_channels import AudioToChannels


# Scaled configuration: 75 reference cycles per SOF, 15 ticks per SOF.
REF_FREQ    = 600_000
TARGET_FREQ = 120_000
SOF_ACC     = 8
CYCLES_PER_SOF = REF_FREQ / 8000          # 75
TICKS_PER_SOF  = TARGET_FREQ // 8000      # 15
TICKS_PER_SAMPLE = 5                      # -> 3 samples per SOF
SAMPLES_PER_SOF  = TICKS_PER_SOF // TICKS_PER_SAMPLE


class SOFCounterTests(unittest.TestCase):

    def test_tick_coincident_with_window_end_is_counted(self):
        """A tick on the same cycle as the window-closing SOF belongs to that window."""
        m = Module()
        dut = SOFCounter(sof_accumulation=2)
        m.submodules.dut = dut
        measured = []

        async def process(ctx):
            for window in range(3):
                for sof in range(2):
                    for _ in range(4):
                        ctx.set(dut.audio_clock_tick, 1)
                        await ctx.tick()
                        ctx.set(dut.audio_clock_tick, 0)
                        await ctx.tick()
                    # 5th tick lands on the SOF cycle itself.
                    ctx.set(dut.audio_clock_tick, 1)
                    ctx.set(dut.sof_detected, 1)
                    await ctx.tick()
                    ctx.set(dut.audio_clock_tick, 0)
                    ctx.set(dut.sof_detected, 0)
                    await ctx.tick()
                measured.append(ctx.get(dut.measured_count))

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)
        sim.run()
        # 5 ticks per SOF, 2 SOFs per window, none lost.
        self.assertEqual(measured, [10, 10, 10])

    def test_locked_after_hold(self):
        """`locked` asserts only after lock_hold consecutive in-threshold windows."""
        m = Module()
        dut = SOFCounter(sof_accumulation=2, lock_threshold=2, lock_hold=1)
        m.submodules.dut = dut
        locked_values = []

        async def process(ctx):
            ctx.set(dut.error_abs, 0)
            for window in range(2):
                for sof in range(2):
                    for _ in range(5):
                        ctx.set(dut.audio_clock_tick, 1)
                        await ctx.tick()
                        ctx.set(dut.audio_clock_tick, 0)
                        await ctx.tick()
                    ctx.set(dut.sof_detected, 1)
                    await ctx.tick()
                    ctx.set(dut.sof_detected, 0)
                    await ctx.tick()
                locked_values.append(ctx.get(dut.locked))

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)
        sim.run()
        self.assertEqual(locked_values, [0, 1])


class ClosedLoopHarness(Elaboratable):
    """
    SwPLL + SOF generator with host clock offset + DAC FIFO model.

    FIFO model: host writes SAMPLES_PER_SOF entries at every SOF, codec reads
    one entry every TICKS_PER_SAMPLE output-NCO rising edges.
    """

    def __init__(self, host_ppm, stream_active, initial_level=0, **pll_kwargs):
        self.pll = SwPLL(ref_freq=REF_FREQ, target_freq=TARGET_FREQ,
                         sof_accumulation=SOF_ACC, **pll_kwargs)
        self.host_ppm = host_ppm
        self.stream_active = stream_active
        self.initial_level = initial_level
        self.window_done = Signal()
        self.out_ticks = Signal(32)              # cumulative output NCO rising edges
        self.level = Signal(signed(16), init=initial_level)
        self.level_at_sof = Signal(signed(16))
        self.level_min = Signal(signed(16), init=initial_level)

    def elaborate(self, platform):
        m = Module()
        m.submodules.pll = pll = self.pll

        # SOF generator, period CYCLES_PER_SOF * (1 + ppm) with 16 fractional bits.
        period_q16 = round(CYCLES_PER_SOF * (1 + self.host_ppm * 1e-6) * 65536)
        acc = Signal(32)
        sof = Signal()
        m.d.sync += sof.eq(0)
        with m.If(acc + 65536 >= period_q16):
            m.d.sync += [acc.eq(acc + 65536 - period_q16), sof.eq(1)]
        with m.Else():
            m.d.sync += acc.eq(acc + 65536)

        # Output NCO edge detect.
        mclk_d = Signal()
        out_edge = Signal()
        m.d.sync += mclk_d.eq(pll.mclk_out)
        m.d.comb += out_edge.eq(pll.mclk_out & ~mclk_d)
        with m.If(out_edge):
            m.d.sync += self.out_ticks.eq(self.out_ticks + 1)

        # Window bookkeeping.
        sub = Signal(range(SOF_ACC))
        m.d.sync += self.window_done.eq(0)
        with m.If(sof):
            with m.If(sub == SOF_ACC - 1):
                m.d.sync += [sub.eq(0), self.window_done.eq(1)]
            with m.Else():
                m.d.sync += sub.eq(sub + 1)

        # FIFO model.
        div = Signal(range(TICKS_PER_SAMPLE))
        dec = Signal()
        m.d.comb += dec.eq(out_edge & (div == TICKS_PER_SAMPLE - 1))
        with m.If(out_edge):
            m.d.sync += div.eq(Mux(dec, 0, div + 1))
        m.d.sync += self.level.eq(self.level + Mux(sof, SAMPLES_PER_SOF, 0) - Mux(dec, 1, 0))
        with m.If(sof):
            m.d.sync += self.level_at_sof.eq(self.level)
        with m.If(self.level < self.level_min):
            m.d.sync += self.level_min.eq(self.level)

        m.d.comb += [
            pll.sof_detected.eq(sof),
            pll.stream_active.eq(self.stream_active),
            pll.fifo_level.eq(Mux(self.level < 0, 0, self.level)),
        ]
        return m


def run_closed_loop(h, windows):
    """Run the harness for `windows` measurement windows; return per-window rows."""
    rows = []

    async def tb(ctx):
        for w in range(windows):
            await ctx.tick().until(h.window_done)
            await ctx.tick().repeat(3)
            rows.append(dict(
                measured=ctx.get(h.pll.dbg_measured),
                error=ctx.get(h.pll.dbg_error),
                fcw=ctx.get(h.pll.dbg_fcw),
                out_ticks=ctx.get(h.out_ticks),
                level_at_sof=ctx.get(h.level_at_sof),
                level_min=ctx.get(h.level_min),
                locked=ctx.get(h.pll.locked),
            ))

    sim = Simulator(h)
    sim.add_clock(1 / REF_FREQ)
    sim.add_testbench(tb)
    sim.run()
    return rows


class SwPLLTests(unittest.TestCase):

    def test_nominal_frequency(self):
        """With a perfect host clock the loop stays put at the nominal FCW."""
        h = ClosedLoopHarness(host_ppm=0, stream_active=0)
        rows = run_closed_loop(h, 20)
        expected = TICKS_PER_SOF * SOF_ACC
        self.assertTrue(all(r["measured"] == expected for r in rows), rows[-1])
        self.assertEqual(rows[-1]["fcw"], h.pll._nominal_fcw)
        self.assertEqual(rows[-1]["locked"], 1)

    def _check_frequency_lock(self, ppm):
        h = ClosedLoopHarness(host_ppm=ppm, stream_active=0)
        rows = run_closed_loop(h, 60)
        expected = TICKS_PER_SOF * SOF_ACC
        # Converged: error within quantization, lock declared.
        tail = rows[-10:]
        self.assertTrue(all(abs(r["error"]) <= 1 for r in tail), [r["error"] for r in rows])
        self.assertEqual(rows[-1]["locked"], 1)
        # FCW ended near nominal / (1 + ppm): within one tick-equivalent.
        target_fcw = h.pll._nominal_fcw / (1 + ppm * 1e-6)
        self.assertLess(abs(rows[-1]["fcw"] - target_fcw), 1.5 * h.pll._tick_fcw)
        # No ticks lost: the sum of measurements tracks the output NCO,
        # which with the level loop idle is identical to the reference NCO.
        meas_sum = sum(r["measured"] for r in rows)
        self.assertLessEqual(abs(meas_sum - rows[-1]["out_ticks"]), 2,
                             f"measured {meas_sum} vs output ticks {rows[-1]['out_ticks']}")
        # Output NCO frequency matches the host: cumulative ticks over the
        # run equal expected*windows within the acquisition transient.
        self.assertLess(abs(rows[-1]["out_ticks"] - expected * len(rows)), 3 * expected / 100)

    def test_frequency_lock_host_fast(self):
        self._check_frequency_lock(+300)

    def test_frequency_lock_host_slow(self):
        self._check_frequency_lock(-300)

    def test_fcw_clamped_without_sofs_garbage(self):
        """A wildly wrong SOF rate can only push FCW to the configured clamp."""
        h = ClosedLoopHarness(host_ppm=+50_000, stream_active=0, max_ppm=1000)
        rows = run_closed_loop(h, 40)
        lo = h.pll._nominal_fcw - h.pll._fcw_limit
        hi = h.pll._nominal_fcw + h.pll._fcw_limit
        self.assertTrue(all(lo <= r["fcw"] <= hi for r in rows))
        self.assertEqual(rows[-1]["fcw"], lo)

    def test_level_regulation(self):
        """FIFO level (sampled at SOF) converges to the setpoint and stays there."""
        setpoint = 6
        h = ClosedLoopHarness(host_ppm=+200, stream_active=1, initial_level=12,
                              level_setpoint=setpoint, level_gain_ppm=2000)
        rows = run_closed_loop(h, 200)
        tail = rows[-40:]
        levels = [r["level_at_sof"] for r in tail]
        self.assertTrue(all(abs(l - setpoint) <= 1 for l in levels),
                        f"level at SOF did not settle: {[r['level_at_sof'] for r in rows]}")
        self.assertGreaterEqual(rows[-1]["level_min"], 0, "FIFO model underran")
        # Frequency loop unaffected by the level loop.
        self.assertTrue(all(abs(r["error"]) <= 1 for r in tail))
        self.assertEqual(rows[-1]["locked"], 1)

    def test_level_regulation_from_low(self):
        """Starting below the setpoint, the clock slows until the FIFO fills up."""
        setpoint = 6
        h = ClosedLoopHarness(host_ppm=-200, stream_active=1, initial_level=1,
                              level_setpoint=setpoint, level_gain_ppm=2000)
        rows = run_closed_loop(h, 200)
        levels = [r["level_at_sof"] for r in rows[-40:]]
        self.assertTrue(all(abs(l - setpoint) <= 1 for l in levels), levels)


class _FakeChannelStream:
    """Minimal stand-in for the legacy channel stream interfaces."""
    def __init__(self, width, nr_channels):
        self.payload = Signal(width)
        self.channel_nr = Signal(range(nr_channels))
        self.valid = Signal()
        self.ready = Signal()
        self.first = Signal()
        self.last = Signal()


class PrefillTests(unittest.TestCase):

    def test_prefill_holds_until_level_then_streams(self):
        nr = 4
        to_usb = _FakeChannelStream(24, nr)
        from_usb = _FakeChannelStream(24, nr)
        m = Module()
        dut = AudioToChannels(nr_channels=nr, to_usb_stream=to_usb, from_usb_stream=from_usb,
                              fifo_depth=16, prefill_level=8)
        m.submodules.dut = dut
        seen_valid_at = []

        async def writer(ctx):
            # Write 12 samples (4 channels each) into the DAC FIFO from the usb side.
            for n in range(12):
                for ch in range(nr):
                    ctx.set(from_usb.channel_nr, ch)
                    ctx.set(from_usb.payload, n)
                    ctx.set(from_usb.valid, 1)
                    await ctx.tick("usb")
                ctx.set(from_usb.valid, 0)
                await ctx.tick("usb").repeat(2)
            await ctx.tick("usb").repeat(200)

        async def reader(ctx):
            ctx.set(dut.o.ready, 0)
            for cyc in range(400):
                await ctx.tick("sync")
                if ctx.get(dut.o.valid):
                    seen_valid_at.append(cyc)
                    break
            # Let the writer finish, then drain everything.
            await ctx.tick("sync").repeat(100)
            ctx.set(dut.o.ready, 1)
            got = 0
            for _ in range(200):
                # Count transfers: valid & ready sampled before each edge.
                if ctx.get(dut.o.valid):
                    got += 1
                await ctx.tick("sync")
            seen_valid_at.append(got)

        sim = Simulator(m)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_clock(1 / 61e6, domain="sync")
        sim.add_testbench(writer)
        sim.add_testbench(reader)
        sim.run()
        self.assertEqual(len(seen_valid_at), 2, "output never became valid")
        # valid must appear only after >= 8 samples were written (each sample
        # takes nr+2 usb cycles), never on the first few.
        self.assertGreater(seen_valid_at[0], 8 * (nr + 2) * 60 / 61 * 0.9)
        self.assertEqual(seen_valid_at[1], 12)


if __name__ == "__main__":
    unittest.main()


# ---------------------------------------------------------------------------
# SI5351 actuator
# ---------------------------------------------------------------------------

from tiliqua.usb_audio.clock_recovery import ClockRecovery, CTRL_FRAC_BITS
from tiliqua.usb_audio.si5351_tuner import (Si5351Tuner, si5351_pll_params,
                                            si5351_msn_registers, REG_MSNA, REG_SSC,
                                            SI5351_ADDRESS)


class I2CAckSlave(Elaboratable):
    """
    Bus model for the tuner's open-drain pads: pull-ups, an ACK on every
    9th clock, and a byte decoder (`byte`/`byte_valid`, `start` pulses).
    """
    def __init__(self, pins):
        self.pins = pins
        self.byte = Signal(8)
        self.byte_valid = Signal()
        self.start = Signal()

    def elaborate(self, platform):
        m = Module()
        scl = Signal()
        sda_m = Signal()
        m.d.comb += [scl.eq(~self.pins.scl.oe), sda_m.eq(~self.pins.sda.oe)]
        scl_r = Signal(init=1)
        sda_r = Signal(init=1)
        m.d.sync += [scl_r.eq(scl), sda_r.eq(sda_m)]
        bit = Signal(4)
        shreg = Signal(8)
        ack = Signal()
        m.d.sync += [self.byte_valid.eq(0), self.start.eq(0)]
        with m.If(scl & sda_r & ~sda_m):            # START
            m.d.sync += [bit.eq(0), self.start.eq(1), ack.eq(0)]
        with m.Elif(~scl_r & scl):                  # SCL rising: master's bit is valid
            with m.If(bit < 8):
                m.d.sync += [shreg.eq(Cat(sda_m, shreg[:7])), bit.eq(bit + 1)]
            with m.Elif(bit == 8):                  # 9th clock: master samples our ACK
                m.d.sync += bit.eq(9)
        with m.Elif(scl_r & ~scl):                  # SCL falling
            with m.If(bit == 8):                    # 8 bits in: drive ACK for the 9th clock
                m.d.sync += [ack.eq(1), self.byte.eq(shreg), self.byte_valid.eq(1)]
            with m.Elif(bit == 9):
                m.d.sync += [ack.eq(0), bit.eq(0)]
        m.d.comb += [
            self.pins.scl.i.eq(scl),
            self.pins.sda.i.eq(sda_m & ~ack),
        ]
        return m


class Si5351TunerTests(unittest.TestCase):

    def test_register_math_matches_driver(self):
        """Gateware shortcut (b>>13) equals the driver's floor(128b/c) formula for all b < c."""
        p = si5351_pll_params()
        self.assertEqual((p["a"], p["b"], p["c"], p["ms"]), (35, 408357, 0xfffff, 72))
        for b in [0, 1, 8191, 8192, 8193, p["b"] - 1, p["b"], p["b"] + 1, 0xffffe]:
            ratio = (128 * b) // p["c"]
            self.assertEqual(ratio, b >> 13, b)
            self.assertEqual(128 * b - p["c"] * ratio, (b >> 13) + 128 * (b & 8191), b)

    def test_i2c_transactions(self):
        """Start-up clears spread spectrum, writes nominal b, then follows ctrl updates."""
        m = Module()
        dut = Si5351Tuner(period_cyc=16)
        m.submodules.dut = dut
        m.submodules.slave = slave = I2CAckSlave(dut.pins)
        p = dut.params
        transactions = []

        async def monitor(ctx):
            cur = None
            for _ in range(60000):
                await ctx.tick()
                if ctx.get(slave.start):
                    cur = []
                    transactions.append(cur)
                if ctx.get(slave.byte_valid) and cur is not None:
                    cur.append(ctx.get(slave.byte))

        async def stim(ctx):
            await ctx.tick().until(dut.ready)
            await ctx.tick().until(~dut.busy)
            # +100 ppm -> b_nom + 100/step
            ctx.set(dut.ctrl, round(100 * 2 ** CTRL_FRAC_BITS))
            ctx.set(dut.update, 1); await ctx.tick(); ctx.set(dut.update, 0)
            await ctx.tick().until(dut.busy)
            await ctx.tick().until(~dut.busy)
            # -2500 ppm -> clamped
            ctx.set(dut.ctrl, -round(2500 * 2 ** CTRL_FRAC_BITS))
            ctx.set(dut.update, 1); await ctx.tick(); ctx.set(dut.update, 0)
            await ctx.tick().until(dut.busy)
            await ctx.tick().until(~dut.busy)
            await ctx.tick().repeat(50)

        sim = Simulator(m)
        sim.add_clock(1 / 60e6)
        sim.add_testbench(stim)
        sim.add_testbench(monitor, background=True)
        sim.run()

        addr = SI5351_ADDRESS << 1
        self.assertEqual(len(transactions), 4, transactions)
        self.assertEqual(transactions[0], [addr, REG_SSC, 0x00])
        self.assertEqual(transactions[1], [addr, REG_MSNA] + si5351_msn_registers(p["a"], p["b"], p["c"]),
                         transactions)
        b100 = p["b"] + round(100 / p["step_ppm"])
        self.assertEqual(transactions[2][:2], [addr, REG_MSNA])
        got = transactions[2][2:]
        self.assertIn(got, [si5351_msn_registers(p["a"], b100 + d, p["c"]) for d in (-1, 0, 1)])
        self.assertEqual(transactions[3], [addr, REG_MSNA] +
                         si5351_msn_registers(p["a"], p["b"] - dut._b_lim, p["c"]))
        self.assertEqual(len(transactions), 4)


# Scaled configuration for the SI5351 closed loop: windows long enough for
# the I2C write (about 1.5k cycles at period 16) to be a quarter of a window,
# as on hardware (1 ms of 4 ms).
S_REF = 6_000_000
S_TARGET = 1_200_000
S_TICKS_PER_SOF = S_TARGET // 8000      # 150
S_TICKS_PER_SAMPLE = 5                  # 30 samples per SOF, 240 per window


class Si5351ClosedLoopHarness(Elaboratable):
    """
    ClockRecovery + Si5351Tuner + bus model + behavioural SI5351 (an NCO whose
    frequency follows the written numerator) + DAC FIFO model.
    """
    def __init__(self, host_ppm, initial_level, **loop_kwargs):
        self.loop = ClockRecovery(target_freq=S_TARGET, sof_accumulation=SOF_ACC, **loop_kwargs)
        self.tuner = Si5351Tuner(period_cyc=16)
        self.host_ppm = host_ppm
        self.window_done = Signal()
        self.level = Signal(signed(16), init=initial_level)
        self.level_at_sof = Signal(signed(16))
        self.level_min = Signal(signed(16), init=initial_level)

    def elaborate(self, platform):
        m = Module()
        m.submodules.loop = loop = self.loop
        m.submodules.tuner = tuner = self.tuner
        m.submodules.slave = I2CAckSlave(tuner.pins)

        # SOF generator.
        cps = S_REF / 8000
        period_q16 = round(cps * (1 + self.host_ppm * 1e-6) * 65536)
        acc = Signal(32); sof = Signal()
        m.d.sync += sof.eq(0)
        with m.If(acc + 65536 >= period_q16):
            m.d.sync += [acc.eq(acc + 65536 - period_q16), sof.eq(1)]
        with m.Else():
            m.d.sync += acc.eq(acc + 65536)

        # Behavioural SI5351: frequency = nominal * (1 + (b - b_nom) * step_ppm).
        p = tuner.params
        nominal = round(S_TARGET / S_REF * 2 ** 32)
        km = round(nominal * p["step_ppm"] * 1e-6 * 2 ** 16)
        b_delta = Signal(signed(24))
        fcw = Signal(32)
        m.d.comb += b_delta.eq(tuner.numerator - p["b"])
        m.d.sync += fcw.eq(nominal + ((b_delta * km) >> 16))
        nco = Signal(32); nco_sum = Signal(33); tick = Signal()
        m.d.comb += nco_sum.eq(nco + fcw)
        m.d.sync += [nco.eq(nco_sum[:-1]), tick.eq(nco_sum[-1])]

        sub = Signal(range(SOF_ACC))
        m.d.sync += self.window_done.eq(0)
        with m.If(sof):
            with m.If(sub == SOF_ACC - 1):
                m.d.sync += [sub.eq(0), self.window_done.eq(1)]
            with m.Else():
                m.d.sync += sub.eq(sub + 1)

        div = Signal(range(S_TICKS_PER_SAMPLE)); dec = Signal()
        m.d.comb += dec.eq(tick & (div == S_TICKS_PER_SAMPLE - 1))
        with m.If(tick):
            m.d.sync += div.eq(Mux(dec, 0, div + 1))
        m.d.sync += self.level.eq(self.level + Mux(sof, S_TICKS_PER_SOF // S_TICKS_PER_SAMPLE, 0)
                                  - Mux(dec, 1, 0))
        with m.If(sof):
            m.d.sync += self.level_at_sof.eq(self.level)
        with m.If(self.level < self.level_min):
            m.d.sync += self.level_min.eq(self.level)

        m.d.comb += [
            loop.sof_detected.eq(sof),
            loop.clk_tick.eq(tick),
            loop.stream_active.eq(1),
            loop.fifo_level.eq(Mux(self.level < 0, 0, self.level)),
            tuner.ctrl.eq(loop.ctrl_total),
            tuner.update.eq(loop.update),
        ]
        return m


class Si5351ClosedLoopTests(unittest.TestCase):

    def _run(self, h, windows):
        rows = []

        async def tb(ctx):
            for w in range(windows):
                await ctx.tick().until(h.window_done)
                await ctx.tick().repeat(4)
                rows.append(dict(
                    error=ctx.get(h.loop.dbg_error), locked=ctx.get(h.loop.locked),
                    ctrl_freq=ctx.get(h.loop.ctrl_freq), numerator=ctx.get(h.tuner.numerator),
                    level_at_sof=ctx.get(h.level_at_sof), level_min=ctx.get(h.level_min),
                    writes=ctx.get(h.tuner.writes)))

        sim = Simulator(h)
        sim.add_clock(1 / S_REF)
        sim.add_testbench(tb)
        sim.run()
        return rows

    def test_lock_and_level_via_si5351(self):
        setpoint = 6
        # Level gain such that the largest level error stays well inside the
        # tuner's +-2000 ppm clamp (as on hardware: 20 ppm/sample, <=10 samples).
        # Start one packet (30 samples) plus a margin above the setpoint, as the
        # prefill does on hardware, so the model never underflows.
        h = Si5351ClosedLoopHarness(host_ppm=+200, initial_level=42,
                                    level_setpoint=setpoint, level_gain_ppm=100)
        rows = self._run(h, 200)
        tail = rows[-40:]
        self.assertTrue(all(abs(r["error"]) <= 1 for r in tail), [r["error"] for r in rows])
        self.assertEqual(rows[-1]["locked"], 1)
        self.assertTrue(all(abs(r["level_at_sof"] - setpoint) <= 1 for r in tail),
                        [r["level_at_sof"] for r in rows])
        self.assertGreaterEqual(rows[-1]["level_min"], 0)
        # SOF period is 200 ppm long (host slow), so the local clock must end up
        # 200 ppm slow: numerator below nominal by 200/step. In this scaled
        # configuration one tick is 833 ppm, so individual windows jitter by a
        # few hundred ppm; judge the mean over the tail.
        p = h.tuner.params
        expect = p["b"] - round(200 / p["step_ppm"])
        mean_num = sum(r["numerator"] for r in tail) / len(tail)
        self.assertLess(abs(mean_num - expect), round(100 / p["step_ppm"]),
                        f"mean numerator {mean_num:.0f} vs expected {expect}")
        self.assertGreater(rows[-1]["writes"], 20)
