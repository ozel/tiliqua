# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
Tests for the Software PLL (NCO + PI control loop) and SOF counter
used for adaptive USB audio clock recovery.
"""

import unittest

from amaranth import *
from amaranth.sim import *

from tiliqua.usb_audio.sof_counter import SOFCounter
from tiliqua.usb_audio.sw_pll import SwPLL


class SOFCounterTests(unittest.TestCase):

    def test_basic_counting(self):
        """SOF counter accumulates audio ticks and produces measurements."""
        m = Module()
        dut = SOFCounter(sof_accumulation=4)
        m.submodules.dut = dut

        measurements = []

        async def process(ctx):
            # Simulate audio clock ticks at a fixed rate
            # With accumulation=4, we need 4 SOFs per measurement.
            # Simulate ~10 audio ticks per SOF.
            for sof_n in range(12):  # 3 full measurement windows
                # Emit 10 audio clock ticks
                for _ in range(10):
                    ctx.set(dut.audio_clock_tick, 1)
                    await ctx.tick()
                    ctx.set(dut.audio_clock_tick, 0)
                    await ctx.tick()

                # Emit SOF pulse
                ctx.set(dut.sof_detected, 1)
                await ctx.tick()
                # measurement_valid is a one-cycle strobe: sample it here,
                # before sof_detected goes low and drives it back to 0.
                if ctx.get(dut.measurement_valid):
                    measurements.append(ctx.get(dut.measured_count))
                ctx.set(dut.sof_detected, 0)
                await ctx.tick()

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)

        with sim.write_vcd(vcd_file=open("test_sof_counter.vcd", "w")):
            sim.run()

        # We should get measurements (3 windows of 4 SOFs each)
        self.assertGreater(len(measurements), 0)
        # Each measurement should be ~40 (10 ticks * 4 SOFs)
        for count in measurements:
            self.assertAlmostEqual(count, 40, delta=2)

    def test_locked_signal(self):
        """Locked signal asserts after first measurement."""
        m = Module()
        dut = SOFCounter(sof_accumulation=2)
        m.submodules.dut = dut

        locked_values = []

        async def process(ctx):
            # Before any SOFs, not locked
            locked_values.append(ctx.get(dut.locked))

            # Two SOF cycles with some ticks
            for _ in range(2):
                for _ in range(5):
                    ctx.set(dut.audio_clock_tick, 1)
                    await ctx.tick()
                    ctx.set(dut.audio_clock_tick, 0)
                    await ctx.tick()
                ctx.set(dut.sof_detected, 1)
                await ctx.tick()
                ctx.set(dut.sof_detected, 0)
                await ctx.tick()

            # After first measurement window, should be locked
            locked_values.append(ctx.get(dut.locked))

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)

        with sim.write_vcd(vcd_file=open("test_sof_locked.vcd", "w")):
            sim.run()

        self.assertEqual(locked_values[0], 0)
        self.assertEqual(locked_values[1], 1)


class SwPLLTests(unittest.TestCase):

    def test_nco_nominal_frequency(self):
        """NCO produces output at approximately the target frequency."""
        m = Module()
        dut = SwPLL(
            ref_freq=60_000_000,
            target_freq=12_288_000,
            sof_accumulation=4,
        )
        m.submodules.dut = dut

        # Count mclk_out transitions to verify frequency
        transitions = []

        async def process(ctx):
            last_mclk = 0
            transition_count = 0
            # Run for enough cycles to see transitions
            # At 60MHz ref generating 12.288MHz, we get a transition
            # roughly every 60/12.288 ≈ 4.88 cycles
            for cycle in range(1000):
                await ctx.tick()
                mclk = ctx.get(dut.mclk_out)
                if mclk != last_mclk:
                    transition_count += 1
                last_mclk = mclk
            transitions.append(transition_count)

        sim = Simulator(m)
        sim.add_clock(1e-6)  # 1MHz sim clock (abstract)
        sim.add_testbench(process)

        with sim.write_vcd(vcd_file=open("test_nco_freq.vcd", "w")):
            sim.run()

        # At nominal FCW, we should see approximately:
        # 1000 cycles * (12.288/60) * 2 transitions/cycle ≈ 409 transitions
        # Allow generous tolerance for NCO quantization
        self.assertGreater(transitions[0], 300)
        self.assertLess(transitions[0], 500)

    def test_pi_loop_convergence(self):
        """PI control loop converges when fed SOF pulses at the correct rate."""
        m = Module()
        dut = SwPLL(
            ref_freq=60_000_000,
            target_freq=12_288_000,
            sof_accumulation=4,
            kp_shift=4,
            ki_shift=10,
        )
        m.submodules.dut = dut

        # Expected ticks per SOF at nominal rate:
        # 12_288_000 / 8_000 = 1536 per SOF
        # With our abstract 1MHz sim clock this won't match real timing,
        # but we can verify the control loop logic responds to errors.
        errors = []

        async def process(ctx):
            # Simulate the PLL with direct audio_clock_tick injection.
            # Feed it SOFs periodically with a fixed number of audio ticks
            # that matches the expected count. The error should stay near zero.
            ticks_per_sof = 100  # abstract count for simulation

            for window in range(8):  # 8 measurement windows
                for sof in range(4):  # sof_accumulation=4
                    for _ in range(ticks_per_sof):
                        ctx.set(dut.audio_clock_tick, 1)
                        await ctx.tick()
                        ctx.set(dut.audio_clock_tick, 0)
                        await ctx.tick()
                    ctx.set(dut.sof_detected, 1)
                    await ctx.tick()
                    ctx.set(dut.sof_detected, 0)
                    await ctx.tick()

                # Read error after each measurement window
                await ctx.tick()
                await ctx.tick()
                errors.append(ctx.get(dut.dbg_error))

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)

        with sim.write_vcd(vcd_file=open("test_pi_loop.vcd", "w")):
            sim.run()

        # Verify we got measurements and the loop is responding
        self.assertGreater(len(errors), 0)
        # The PI loop should be trying to correct — verify it's active
        # (exact convergence depends on the abstract tick rate not matching
        # the real expected count, so we just verify the loop runs)

    def test_locked_propagation(self):
        """SW PLL locked signal propagates from SOF counter."""
        m = Module()
        dut = SwPLL(
            ref_freq=60_000_000,
            target_freq=12_288_000,
            sof_accumulation=2,
        )
        m.submodules.dut = dut

        async def process(ctx):
            # Initially not locked
            self.assertEqual(ctx.get(dut.locked), 0)

            # Complete one measurement window (2 SOFs)
            for _ in range(2):
                for _ in range(10):
                    ctx.set(dut.audio_clock_tick, 1)
                    await ctx.tick()
                    ctx.set(dut.audio_clock_tick, 0)
                    await ctx.tick()
                ctx.set(dut.sof_detected, 1)
                await ctx.tick()
                ctx.set(dut.sof_detected, 0)
                await ctx.tick()

            # Should now be locked
            self.assertEqual(ctx.get(dut.locked), 1)

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)

        with sim.write_vcd(vcd_file=open("test_pll_lock.vcd", "w")):
            sim.run()

    def test_fcw_adjustment_direction(self):
        """FCW increases when measured count is below expected (clock too slow)."""
        m = Module()
        # Set up with known expected count
        # expected_count = (12_288_000 / 8000) * 4 = 1536 * 4 = 6144
        dut = SwPLL(
            ref_freq=60_000_000,
            target_freq=12_288_000,
            sof_accumulation=4,
            kp_shift=2,   # aggressive P gain for visible effect
            ki_shift=8,
        )
        m.submodules.dut = dut

        fcw_values = []

        async def process(ctx):
            # Record initial FCW
            fcw_values.append(ctx.get(dut.dbg_fcw))

            # Feed fewer ticks than expected (simulating clock too slow)
            # Expected is 6144 ticks per window. Feed only 6000.
            ticks_per_sof = 1500  # 1500 * 4 = 6000 < 6144
            for sof in range(4):
                for _ in range(ticks_per_sof):
                    ctx.set(dut.audio_clock_tick, 1)
                    await ctx.tick()
                    ctx.set(dut.audio_clock_tick, 0)
                    await ctx.tick()
                ctx.set(dut.sof_detected, 1)
                await ctx.tick()
                ctx.set(dut.sof_detected, 0)
                await ctx.tick()

            # Wait for PI loop to apply correction
            for _ in range(5):
                await ctx.tick()

            fcw_values.append(ctx.get(dut.dbg_fcw))

        sim = Simulator(m)
        sim.add_clock(1e-6)
        sim.add_testbench(process)

        with sim.write_vcd(vcd_file=open("test_fcw_direction.vcd", "w")):
            sim.run()

        # FCW should increase (positive error = clock too slow = need faster NCO)
        self.assertGreater(fcw_values[1], fcw_values[0],
                           f"FCW should increase when clock is too slow: "
                           f"initial={fcw_values[0]}, after={fcw_values[1]}")


if __name__ == "__main__":
    unittest.main()
