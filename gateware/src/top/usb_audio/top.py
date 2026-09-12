"""
4-channel USB2 audio interface, no video, no SoC.

Enumerates as a 4-in, 4-out 48kHz sound card.

By default the device is an asynchronous UAC2 sink/source with a feedback
endpoint (the SI5351 provides the audio clock). With ``--adaptive`` it is an
adaptive sink/source instead: a loop recovers the audio clock from USB SOF
timing and the DAC FIFO level (see ``adaptive.md``). ``--adaptive-clock``
picks the actuator: ``si5351`` (default) steers the external PLL's fractional
numerator over I2C and keeps its low-jitter output as MCLK; ``nco`` generates
MCLK from a phase accumulator in the FPGA. ``--telemetry`` replaces capture
channels 2/3 with FIFO/loop state for bench testing (decode with
``telemetry_decode.py``).
"""

from amaranth import *
from amaranth.build import Attrs, PinsN, Resource, Subsignal
from amaranth.lib import cdc, wiring

from tiliqua import pll, usb_audio
from tiliqua.build.cli import top_level_cli
from tiliqua.periph import eurorack_pmod, i2c
from tiliqua.usb_audio.si5351_tuner import Si5351Tuner
from tiliqua.build.types import BitstreamHelp
from tiliqua.platform import RebootProvider
from vendor.ila import AsyncSerialILA


class USBAudioTop(Elaboratable):

    bitstream_help = BitstreamHelp(
        brief="USB soundcard, 4in + 4out.",
        io_left=['in0', 'in1', 'in2', 'in3', 'out0', 'out1', 'out2', 'out3'],
        io_right=['', 'USB audio device', '', 'FIFO bar (pmod0)', '', '']
    )

    def __init__(self, clock_settings, sync_mode="async", adaptive_clock="si5351", telemetry=False):
        super().__init__()
        self.clock_settings = clock_settings
        self.sync_mode = sync_mode
        self.adaptive_clock = adaptive_clock
        self.telemetry = telemetry
        self.use_nco = sync_mode == "adaptive" and adaptive_clock == "nco"
        self.use_si5351 = sync_mode == "adaptive" and adaptive_clock == "si5351"
        # Spread spectrum on the SI5351 audio PLL would add periodic jitter to
        # MCLK and is pointless on a clock we retune anyway (the tuner also
        # switches it off at start-up). Bitstream archiver picks this up.
        self.external_pll_spread_spectrum = None if self.use_si5351 else 0.01

    def elaborate(self, platform):
        m = Module()

        if self.sync_mode == "adaptive" and \
                platform.clock_domain_generator is not pll.TiliquaDomainGeneratorPLLExternal:
            raise ValueError("adaptive USB audio requires a platform with the external-PLL clock generator")
        if self.use_nco:
            # The audio domain is clocked by the SW PLL NCO output (fabric clock
            # via DCCA) for the whole life of the bitstream.
            nco_mclk = Signal()
            m.submodules.car = car = platform.clock_domain_generator(
                    self.clock_settings, audio_clock_override_clk=nco_mclk)
        else:
            m.submodules.car = car = platform.clock_domain_generator(self.clock_settings)
        m.submodules.reboot = reboot = RebootProvider(car.settings.frequencies.sync)
        m.submodules.btn = cdc.FFSynchronizer(
                platform.request("encoder").s.i, reboot.button)

        m.submodules.pmod0_provider = pmod0_provider = eurorack_pmod.FFCProvider()
        m.submodules.pmod0 = pmod0 = eurorack_pmod.EurorackPmod(self.clock_settings.audio_clock)
        wiring.connect(m, pmod0.pins, pmod0_provider.pins)
        m.d.comb += pmod0.codec_mute.eq(reboot.mute)

        m.submodules.usbif = usbif = usb_audio.USB2AudioInterface(
                audio_clock=self.clock_settings.audio_clock, nr_channels=4,
                sync_mode=self.sync_mode, adaptive_clock=self.adaptive_clock)

        if self.use_si5351:
            # Steer the external PLL over the mobo I2C bus (100 kHz max).
            m.submodules.i2c_provider = i2c_provider = i2c.Provider()
            m.submodules.si5351_tuner = tuner = DomainRenamer("usb")(Si5351Tuner(period_cyc=600))
            wiring.connect(m, tuner.pins, i2c_provider.pins)
            m.d.comb += [
                tuner.ctrl.eq(usbif.clk_ctrl),
                tuner.update.eq(usbif.clk_update),
            ]

        if self.telemetry:
            # Replace capture channels 2 and 3 with device-side telemetry so
            # that FIFO/loop state is recorded sample-aligned with the audio
            # on channels 0/1 (see telemetry_decode.py next to this file).
            #
            # ch2 (16 bit): [4:0] DAC FIFO level  [6:5] field index k
            #               [7] dac_primed [8] out_active [9] sw_pll_locked
            #               [14:10] ADC FIFO level
            # ch3 (16 bit): k=0 underrun count (prefill re-arms)
            #               k=1 SW PLL error (signed, ticks/window)
            #               k=2 loop frequency control in 1/16 ppm (signed)
            #               k=3 [7:0] capture fill events, [15:8] capture skip events
            d = usbif.dbg
            def rising(sig):
                prev = Signal()
                m.d.usb += prev.eq(sig)
                return sig & ~prev
            underruns = Signal(16)
            fills = Signal(8)
            skips = Signal(8)
            with m.If(rising(~d.dac_primed)):
                m.d.usb += underruns.eq(underruns + 1)
            with m.If(rising(d.capture_filling)):
                m.d.usb += fills.eq(fills + 1)
            with m.If(rising(d.capture_skipping)):
                m.d.usb += skips.eq(skips + 1)
            ctrl_ppm16 = Signal(signed(32))
            m.d.comb += ctrl_ppm16.eq(d.sw_pll_ctrl_freq >> 4)

            k = Signal(2)
            ch2 = Signal(16)
            ch3 = Signal(16)
            m.d.comb += ch2.eq(Cat(d.dac_fifo_level[:5], k, d.dac_primed, d.out_active,
                                   d.sw_pll_locked, d.adc_fifo_level[:5]))
            with m.Switch(k):
                with m.Case(0):
                    m.d.comb += ch3.eq(underruns)
                with m.Case(1):
                    m.d.comb += ch3.eq(d.sw_pll_error[:16])
                with m.Case(2):
                    m.d.comb += ch3.eq(ctrl_ppm16[:16])
                with m.Case(3):
                    m.d.comb += ch3.eq(Cat(fills, skips))

            m.d.comb += [
                usbif.i.valid.eq(pmod0.o_cal.valid),
                pmod0.o_cal.ready.eq(usbif.i.ready),
                usbif.i.payload[0].eq(pmod0.o_cal.payload[0]),
                usbif.i.payload[1].eq(pmod0.o_cal.payload[1]),
                usbif.i.payload[2].as_value().eq(ch2),
                usbif.i.payload[3].as_value().eq(ch3),
            ]
            with m.If(usbif.i.valid & usbif.i.ready):
                m.d.sync += k.eq(k + 1)
        else:
            wiring.connect(m, pmod0.o_cal, usbif.i)
        wiring.connect(m, usbif.o, pmod0.i_cal)

        # 8-LED PMOD on expansion port 0: thermometer-style FIFO level display.
        # LEDs 0-3 = DAC FIFO bar, LEDs 4-7 = ADC FIFO bar. PinsN handles the
        # low-active polarity at the pad, so we drive 1 to turn an LED on.
        # FIFO depth is 16 samples at 48 kHz. Async mode's feedback settles the
        # DAC FIFO around 12; adaptive mode's level loop holds it between ~4 and 10.
        platform.add_resources([
            Resource("usb_audio_leds", 0,
                Subsignal("led0", PinsN("7",  conn=("pmod", 0), dir="o")),
                Subsignal("led1", PinsN("8",  conn=("pmod", 0), dir="o")),
                Subsignal("led2", PinsN("9",  conn=("pmod", 0), dir="o")),
                Subsignal("led3", PinsN("10", conn=("pmod", 0), dir="o")),
                Subsignal("led4", PinsN("1",  conn=("pmod", 0), dir="o")),
                Subsignal("led5", PinsN("2",  conn=("pmod", 0), dir="o")),
                Subsignal("led6", PinsN("3",  conn=("pmod", 0), dir="o")),
                Subsignal("led7", PinsN("4",  conn=("pmod", 0), dir="o")),
                Attrs(IO_TYPE="LVCMOS33"),
            )
        ])
        leds = platform.request("usb_audio_leds", 0)

        # DAC FIFO has explicit USB feedback targeting ~half-full, so the raw
        # level is already slow and readable. Display the instantaneous level.
        dac_thresholds = (4, 8, 12, 15)

        # ADC FIFO has no feedback: the host drains samples as fast as they arrive,
        # so instantaneous level is 0 almost all the time with sub-microsecond
        # excursions — invisible to the eye. Flip ADC_PEAK_HOLD to choose between
        # a peak-hold with slow decay (rare backpressure bursts become visible on
        # LED5-7) and a straight thermometer of the instantaneous level (LED4
        # solid, others rarely fire).
        ADC_PEAK_HOLD = True

        if ADC_PEAK_HOLD:
            adc_peak  = Signal(5)
            adc_decay = Signal(17)
            # Decay rate is one step per wrap of adc_decay in the usb domain (60 MHz).
            # What matters isn't full decay time, but whether the peak can drop
            # below an LED's threshold between successive events AT THAT LEVEL:
            #
            #   LED4 (peak ≥ 1): events every 21 µs (every audio sample) —
            #       no practical decay is fast enough + visible; LED4 is a
            #       solid-on "ADC is alive" indicator in all regimes.
            #   LED5 (peak ≥ 2): USB-drain jitter events every few ms —
            #       width 17 (≈2.2 ms/step) is the sweet spot; width 18+
            #       (≥4 ms/step) is too slow → peak pins at 2, LED5 solid.
            #   LED6/7 (peak ≥ 4/8): only fire on real backpressure; widths
            #       20+ (≥17 ms/step) keep those events visible longest.
            m.d.usb += adc_decay.eq(adc_decay + 1)
            with m.If(usbif.dbg.adc_fifo_level > adc_peak):
                m.d.usb += adc_peak.eq(usbif.dbg.adc_fifo_level)
            with m.Elif((adc_decay == 0) & (adc_peak != 0)):
                m.d.usb += adc_peak.eq(adc_peak - 1)
            adc_display = adc_peak
        else:
            adc_display = usbif.dbg.adc_fifo_level

        adc_thresholds = (1, 2, 4, 8)
        for n, (dac_thr, adc_thr) in enumerate(zip(dac_thresholds, adc_thresholds)):
            m.d.comb += getattr(leds, f"led{n}").o.eq(usbif.dbg.dac_fifo_level >= dac_thr)
            m.d.comb += getattr(leds, f"led{n+4}").o.eq(adc_display >= adc_thr)

        if self.use_nco:
            m.d.comb += nco_mclk.eq(usbif.mclk_out)

        if platform.ila:

            # TODO: unbitrot ILA flag
            # https://github.com/apfaudio/tiliqua/issues/113

            test_signal = Signal(16, reset=0xFEED)

            pmod_sample_o0 = Signal(16)
            m.d.comb += pmod_sample_o0.eq(pmod0.i_cal.payload[0])

            ila_signals = [
                test_signal,
                pmod_sample_o0,
                pmod0.i_cal.valid,
                usbif.dbg.dac_fifo_level,
                usbif.dbg.adc_fifo_level,
                usbif.dbg.sof_detected,
                usbif.dbg.channel_stream_out_valid,
                usbif.dbg.channel_stream_out_first,
                usbif.dbg.usb_stream_in_valid,
                usbif.dbg.usb_stream_in_payload,
                usbif.dbg.usb_stream_in_ready,
            ]

            self.ila = AsyncSerialILA(signals=ila_signals,
                                      sample_depth=8192, divisor=521,
                                      domain='usb', sample_rate=60e6) # ~115200 baud on USB clock
            m.submodules += self.ila

            m.d.comb += [
                self.ila.trigger.eq((pmod_sample_o0 > Const(1000)) & pmod0.i_cal.valid),
                platform.request("uart").tx.o.eq(self.ila.tx), # needs FFSync?
            ]


        return m


def add_adaptive_args(parser):
    parser.add_argument('--adaptive', action='store_true',
                        help="Use adaptive USB audio sync mode (device recovers its clock from the host).")
    parser.add_argument('--adaptive-clock', choices=['si5351', 'nco'], default='si5351',
                        help="Adaptive mode actuator: retune the external SI5351 PLL over I2C "
                             "(default, low jitter) or generate MCLK from an NCO in the FPGA.")
    parser.add_argument('--telemetry', action='store_true',
                        help="Replace capture channels 2/3 with FIFO/PLL telemetry (see telemetry_decode.py).")


def parse_adaptive_args(args):
    if args.adaptive and args.fs_192khz:
        raise SystemExit("error: --adaptive is not supported with --fs-192khz (192kHz mode)")
    return {"sync_mode": "adaptive" if args.adaptive else "async",
            "adaptive_clock": args.adaptive_clock,
            "telemetry": args.telemetry}


if __name__ == "__main__":
    top_level_cli(USBAudioTop, video_core=False, ila_supported=True,
                  argparse_callback=add_adaptive_args,
                  argparse_fragment=parse_adaptive_args)
