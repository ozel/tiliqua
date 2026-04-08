"""
4-channel USB2 audio interface, no video, no SoC.

Enumerates as a 4-in, 4-out 48kHz sound card.
"""

from amaranth import *
from amaranth.lib import cdc, wiring

from tiliqua import pll, usb_audio
from tiliqua.build.cli import top_level_cli
from tiliqua.periph import eurorack_pmod
from tiliqua.build.types import BitstreamHelp
from tiliqua.platform import RebootProvider
from vendor.ila import AsyncSerialILA


class USBAudioTop(Elaboratable):

    bitstream_help = BitstreamHelp(
        brief="USB soundcard, 4in + 4out.",
        io_left=['in0', 'in1', 'in2', 'in3', 'out0', 'out1', 'out2', 'out3'],
        io_right=['', 'USB audio device', '', '', '', '']
    )

    def __init__(self, clock_settings, sync_mode="async"):
        super().__init__()
        self.clock_settings = clock_settings
        self.sync_mode = sync_mode

    def elaborate(self, platform):
        m = Module()

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
                sync_mode=self.sync_mode)

        wiring.connect(m, pmod0.o_cal, usbif.i)
        wiring.connect(m, usbif.o, pmod0.i_cal)

        # In adaptive mode, wire up the SW PLL recovered clock
        # to override the audio domain clock source.
        if self.sync_mode == "adaptive":
            if hasattr(car, 'audio_clock_override'):
                m.d.comb += [
                    car.audio_clock_override.eq(usbif.sw_pll_locked),
                    car.audio_clock_override_clk.eq(usbif.mclk_out),
                ]

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
                        help="Use adaptive USB audio sync mode (device tracks host clock via SW PLL).")


def parse_adaptive_args(args):
    if args.adaptive and args.fs_192khz:
        raise SystemExit("error: --adaptive is not supported with --fs-192khz (192kHz mode)")
    return {"sync_mode": "adaptive" if args.adaptive else "async"}


if __name__ == "__main__":
    top_level_cli(USBAudioTop, video_core=False, ila_supported=True,
                  argparse_callback=add_adaptive_args,
                  argparse_fragment=parse_adaptive_args)
