# Copyright (c) 2024 ozel & Claude (Anthropic)
#
# SPDX-License-Identifier: BSD--3-Clause

"""
SI5351A actuator for adaptive USB audio clock recovery.

The bootloader programs the SI5351A (mobo I2C bus, address 0x60) so that
PLLA runs at ``target * ms`` from the 25 MHz crystal in fractional mode and
clk0 (the audio MCLK) is PLLA / ms in integer MultiSynth mode. This module
recomputes those settings with the same algorithm as the Rust driver and
then steers only the 20-bit fractional numerator ``b`` of PLLA's feedback
multiplier ``a + b/c`` from the loop's control value. One numerator step is
``xtal / (c * f_pll)``, about 0.027 ppm.

Updating the fractional numerator is glitch-free and needs no PLL reset;
the chip's own spread-spectrum engine modulates this very divider. The
denominator ``c = 2**20 - 1`` makes the register math exact without a
divider: ``floor(128 b / c) == b >> 13`` for every ``b < c``.

At start-up the spread-spectrum enable bit is cleared (register 149), since
the bootloader enables a 1 % spread on PLLA by default and that would add
periodic jitter to MCLK.
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out

from vendor.i2c import I2CRegisterInterface, I2CPinSignature

from .clock_recovery import CTRL_FRAC_BITS

SI5351_ADDRESS = 0x60
REG_MSNA = 26
REG_SSC = 149


def si5351_pll_params(*, xtal=25_000_000, target=12_288_000, max_pll=900_000_000):
    """PLLA parameters as chosen by tiliqua_hal::si5351 set_frequencies()."""
    total_div = max_pll // target
    r_div = 1                                       # OutputDivider::min_divider(total_div // 900)
    assert total_div // 900 == 0
    ms = max((total_div // (2 * r_div)) * 2, 6)
    pll = target * ms
    c = 0xfffff
    a = pll // xtal
    b = ((pll % xtal) * c) // xtal
    step_ppm = xtal / c / pll * 1e6
    return dict(ms=ms, pll=pll, a=a, b=b, c=c, step_ppm=step_ppm)


def si5351_msn_registers(a, b, c):
    """Register 26..33 contents for feedback multiplier a + b/c (fractional mode)."""
    ratio = (128 * b) // c
    p1 = 128 * a + ratio - 512
    p2 = 128 * b - c * ratio
    p3 = c
    return [
        (p3 >> 8) & 0xff, p3 & 0xff,
        (p1 >> 16) & 0x03, (p1 >> 8) & 0xff, p1 & 0xff,
        ((p3 >> 12) & 0xf0) | ((p2 >> 16) & 0x0f),
        (p2 >> 8) & 0xff, p2 & 0xff,
    ]


class Si5351Tuner(wiring.Component):
    """
    Turns ``ctrl`` (1/256 ppm) into SI5351A PLLA numerator writes.

    A write is issued whenever ``update`` strobes and the resulting numerator
    differs from the one last written. Writes are serialised on the I2C bus;
    if another ``update`` arrives while busy, the newest value is written next.
    """

    def __init__(self, *, xtal=25_000_000, target=12_288_000, period_cyc=600, max_ppm=2000.0):
        p = si5351_pll_params(xtal=xtal, target=target)
        super().__init__({
            "ctrl": In(signed(32)),         # 1/256 ppm
            "update": In(1),
            "pins": Out(I2CPinSignature()),
            "numerator": Out(20, init=p["b"]),  # numerator the chip is running on
            "busy": Out(1),
            "ready": Out(1),                # start-up (spread-spectrum off) done
            "writes": Out(16),
        })
        self.params = p
        assert p["c"] == 0xfffff
        self._a = p["a"]
        self._b_nom = p["b"]
        self._step_ppm = p["step_ppm"]
        # numerator steps per ctrl unit, 2**16 fixed point
        self._k = round(1.0 / self._step_ppm / (2 ** CTRL_FRAC_BITS) * 2 ** 16)
        self._b_lim = round(max_ppm / self._step_ppm)
        assert 0 < self._b_nom - self._b_lim and self._b_nom + self._b_lim < 0xfffff
        self.period_cyc = period_cyc

    def elaborate(self, platform):
        m = Module()
        m.submodules.i2c = i2c = I2CRegisterInterface(period_cyc=self.period_cyc,
                                                      clk_stretch=False, max_data_bytes=8)
        wiring.connect(m, wiring.flipped(self.pins), i2c.pins)
        m.d.comb += i2c.dev_address.eq(SI5351_ADDRESS)

        # Target numerator from ctrl, clamped. Pipelined (3 cycles); ctrl only
        # changes once per window and `pending` is delayed to match.
        ctrl_r = Signal(signed(32))
        delta = Signal(signed(48))
        b_unclamped = Signal(signed(32))
        b_target = Signal(20)
        m.d.sync += [
            ctrl_r.eq(self.ctrl),
            delta.eq((ctrl_r * self._k) >> 16),
            b_unclamped.eq(self._b_nom + delta),
        ]
        with m.If(b_unclamped < self._b_nom - self._b_lim):
            m.d.sync += b_target.eq(self._b_nom - self._b_lim)
        with m.Elif(b_unclamped > self._b_nom + self._b_lim):
            m.d.sync += b_target.eq(self._b_nom + self._b_lim)
        with m.Else():
            m.d.sync += b_target.eq(b_unclamped)

        # Register image for numerator b (see si5351_msn_registers).
        b = Signal(20)
        q = Signal(7)
        r = Signal(13)
        p1 = Signal(18)
        p2 = Signal(20)
        p3 = 0xfffff
        m.d.comb += [
            q.eq(b[13:20]),
            r.eq(b[0:13]),
            p1.eq(128 * self._a - 512 + q),
            p2.eq(q + (r << 7)),
        ]
        regs = [
            Const((p3 >> 8) & 0xff, 8), Const(p3 & 0xff, 8),
            p1[16:18], p1[8:16], p1[0:8],
            Cat(p2[16:20], Const((p3 >> 16) & 0xf, 4)),
            p2[8:16], p2[0:8],
        ]
        # First byte sent is the most significant byte of write_data.
        msn_image = Cat(*[Cat(x, Const(0, 8 - len(x))) if len(x) < 8 else x for x in reversed(regs)])

        pending = Signal()
        update_d = Signal(5)
        m.d.sync += update_d.eq(Cat(self.update, update_d[:-1]))
        with m.If(update_d[-1]):
            m.d.sync += pending.eq(1)

        idle = Signal()
        m.d.comb += self.busy.eq(~idle | pending | update_d.any() | i2c.busy)

        with m.FSM() as fsm:
            m.d.comb += idle.eq(fsm.ongoing("IDLE"))
            with m.State("SSC_OFF"):
                with m.If(~i2c.busy):
                    m.d.comb += [
                        i2c.reg_address.eq(REG_SSC),
                        i2c.size.eq(1),
                        i2c.write_data.eq(Cat(Const(0, 56), Const(0, 8))),
                        i2c.write_request.eq(1),
                    ]
                    m.next = "SSC_WAIT"
            with m.State("SSC_WAIT"):
                with m.If(i2c.busy):
                    m.next = "SSC_DONE"
            with m.State("SSC_DONE"):
                with m.If(~i2c.busy):
                    m.d.sync += [self.ready.eq(1), pending.eq(1)]
                    m.next = "IDLE"
            with m.State("IDLE"):
                with m.If(pending & ~i2c.busy):
                    m.d.sync += [pending.eq(0), b.eq(b_target)]
                    m.next = "WRITE"
            with m.State("WRITE"):
                m.d.comb += [
                    i2c.reg_address.eq(REG_MSNA),
                    i2c.size.eq(8),
                    i2c.write_data.eq(msn_image),
                    i2c.write_request.eq(1),
                ]
                m.d.sync += [self.writes.eq(self.writes + 1), self.numerator.eq(b)]
                m.next = "WAIT_BUSY"
            with m.State("WAIT_BUSY"):
                with m.If(i2c.busy):
                    m.next = "WAIT_DONE"
            with m.State("WAIT_DONE"):
                with m.If(~i2c.busy):
                    m.next = "IDLE"
        return m
