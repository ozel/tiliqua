# Copyright (c) 2024 ozel
#
# SPDX-License-Identifier: CERN-OHL-S-2.0
#

"""USB MIDI *device* (host computer -> Tiliqua) and a demo MIDI-to-CV core using it."""

from amaranth import *
from amaranth.lib import data, stream, wiring
from amaranth.lib.memory import Memory
from amaranth.lib.wiring import In, Out

from amaranth_future import fixed

from luna.gateware.stream.future import Packet
from luna.usb2 import USBDevice, USBStreamOutEndpoint
from luna.gateware.usb.usb2.request import StallOnlyRequestHandler

from usb_protocol.types import USBRequestType, USBDirection
from usb_protocol.emitters import DeviceDescriptorCollection
from usb_protocol.emitters.descriptors import midi1

from ..dsp import ASQ
from .types import *

__all__ = ["USBMIDIDevice", "USBMonoMidiCV"]

class USBMIDIDevice(Elaboratable):
    """
    USB MIDI device interface.

    This creates a USB device that receives MIDI data from a host computer.
    The MIDI data is output as a stream of 4-byte USB MIDI packets.

    Adapted from: https://github.com/hansfbaier/jt51-synth
    """

    MAX_PACKET_SIZE = 512

    def __init__(self):
        # Output stream of USB MIDI packet bytes (4-byte packets)
        self.o = stream.Signature(Packet(unsigned(8))).create()

        # USB status signals
        self.usb_tx_active = Signal()
        self.usb_rx_active = Signal()
        self.usb_suspended = Signal()
        self.usb_reset_detected = Signal()

    def create_descriptors(self):
        """Creates USB descriptors for a MIDI device."""

        descriptors = DeviceDescriptorCollection()

        with descriptors.DeviceDescriptor() as d:
            d.bcdUSB             = 2.00
            d.bDeviceClass       = 0xEF
            d.bDeviceSubclass    = 0x02
            d.bDeviceProtocol    = 0x01
            d.idVendor           = 0x1209  # pid.codes VID
            d.idProduct          = 0xAA63  # Tiliqua MIDI device PID

            d.iManufacturer      = "apf.audio"
            d.iProduct           = "Tiliqua MIDI"
            d.iSerialNumber      = "0001"
            d.bcdDevice          = 0.01

            d.bNumConfigurations = 1

        with descriptors.ConfigurationDescriptor() as configDescr:
            interface = midi1.StandardMidiStreamingInterfaceDescriptorEmitter()
            interface.bInterfaceNumber = 0
            interface.bNumEndpoints = 1  # OUT endpoint only (receive MIDI from host)
            configDescr.add_subordinate_descriptor(interface)

            streamingInterface = midi1.ClassSpecificMidiStreamingInterfaceDescriptorEmitter()

            # MIDI IN Jack (embedded) - receives from host
            inFromHostJack = midi1.MidiInJackDescriptorEmitter()
            inFromHostJack.bJackID = 1
            inFromHostJack.bJackType = midi1.MidiStreamingJackTypes.EMBEDDED
            streamingInterface.add_subordinate_descriptor(inFromHostJack)

            # MIDI OUT Jack (external) - outputs to device
            outFromDeviceJack = midi1.MidiOutJackDescriptorEmitter()
            outFromDeviceJack.bJackID = 2
            outFromDeviceJack.bJackType = midi1.MidiStreamingJackTypes.EXTERNAL
            outFromDeviceJack.add_source(1)
            streamingInterface.add_subordinate_descriptor(outFromDeviceJack)

            # Bulk OUT endpoint for receiving MIDI
            outEndpoint = midi1.StandardMidiStreamingBulkDataEndpointDescriptorEmitter()
            outEndpoint.bEndpointAddress = USBDirection.OUT.to_endpoint_address(1)
            outEndpoint.wMaxPacketSize = self.MAX_PACKET_SIZE
            streamingInterface.add_subordinate_descriptor(outEndpoint)

            outMidiEndpoint = midi1.ClassSpecificMidiStreamingBulkDataEndpointDescriptorEmitter()
            outMidiEndpoint.add_associated_jack(1)
            streamingInterface.add_subordinate_descriptor(outMidiEndpoint)

            configDescr.add_subordinate_descriptor(streamingInterface)

        return descriptors

    def elaborate(self, platform):
        m = Module()

        ulpi = platform.request(platform.default_usb_connection)
        m.submodules.usb = usb = USBDevice(bus=ulpi)

        # Add standard control endpoint with descriptors
        descriptors = self.create_descriptors()
        control_ep = usb.add_standard_control_endpoint(descriptors)

        # Stall vendor/reserved requests
        stall_condition = lambda setup : \
            (setup.type == USBRequestType.VENDOR) | \
            (setup.type == USBRequestType.RESERVED)
        control_ep.add_request_handler(StallOnlyRequestHandler(stall_condition))

        # Bulk OUT endpoint for receiving MIDI data
        ep1_out = USBStreamOutEndpoint(
            endpoint_number=1,
            max_packet_size=self.MAX_PACKET_SIZE)
        usb.add_endpoint(ep1_out)

        # 2-bit counter to frame 4-byte USB MIDI event packets.
        # USB MIDI events are always 4 bytes: [CIN+Cable, Status, Data0, Data1].
        # We generate 'first' on byte 0 so MidiDecodeUSB can sync.
        byte_count = Signal(2)

        m.d.comb += [
            usb.connect.eq(1),
            usb.full_speed_only.eq(0),
            # Connect endpoint stream to our output with proper Packet framing
            self.o.valid.eq(ep1_out.stream.valid),
            self.o.payload.data.eq(ep1_out.stream.payload),
            self.o.payload.first.eq(byte_count == 0),
            self.o.payload.last.eq(byte_count == 3),
            ep1_out.stream.ready.eq(self.o.ready),
            # Status signals
            self.usb_tx_active.eq(usb.tx_activity_led),
            self.usb_rx_active.eq(usb.rx_activity_led),
            self.usb_suspended.eq(usb.suspended),
            self.usb_reset_detected.eq(usb.reset_detected),
        ]

        # Increment byte counter on each transferred byte.
        # Resync on USB packet start (first from endpoint) for robustness.
        with m.If(ep1_out.stream.valid & self.o.ready):
            with m.If(ep1_out.stream.first):
                m.d.sync += byte_count.eq(1)
            with m.Else():
                m.d.sync += byte_count.eq(byte_count + 1)

        return m


class USBMonoMidiCV(wiring.Component):

    """
    Simple monophonic USB MIDI to CV conversion.

    USB MIDI data from a host computer is converted to CV outputs.

    in (usb midi stream): usb midi data for conversion
    in (audio): not used
    out0: Gate
    out1: V/oct CV
    out2: Velocity
    out3: Mod Wheel (CC1)
    """

    from tiliqua.build.types import BitstreamHelp

    bitstream_help = BitstreamHelp(
        brief="USB MIDI device to CV conversion.",
        io_left=['', '', '', '', 'gate', 'V/oct', 'velocity', 'mod wheel'],
        io_right=['', 'USB MIDI device', '', '', '', '']
    )

    i: In(stream.Signature(data.ArrayLayout(ASQ, 4)))
    o: Out(stream.Signature(data.ArrayLayout(ASQ, 4)))

    # Note: USB MIDI input - this triggers USB device mode connection in CoreTop
    i_usb_midi: In(stream.Signature(MidiMessage))
                        
    def elaborate(self, platform):
        m = Module()

        m.d.comb += [
            # Always forward our audio payload
            self.i.ready.eq(1),
            self.o.valid.eq(1),

            # Always ready for MIDI messages
            self.i_usb_midi.ready.eq(1),
        ]

        # Create a LUT from midi note to voltage (output ASQ).
        lut = []
        for i in range(128):
            volts_per_note = 1.0/12.0
            volts = i*volts_per_note - 5
            # convert volts to audio sample
            x = volts/(2**15/4000)
            lut.append(fixed.Const(x, shape=ASQ)._value)

        # Store it in a memory where the address is the midi note,
        # and the data coming out is directly routed to V/Oct out.
        m.submodules.mem = mem = Memory(
            shape=signed(ASQ.as_shape().width), depth=len(lut), init=lut)
        rport = mem.read_port()
        m.d.comb += [
            rport.en.eq(1),
        ]

        # Route memory straight out to our note payload.
        m.d.sync += self.o.payload[1].as_value().eq(rport.data),

        # DEBUG: latch high on mod wheel output (ch3) when any MIDI message arrives.
        # If this LED turns on when sending MIDI, MidiDecode is working.
        midi_debug = Signal()
        with m.If(self.i_usb_midi.valid):
            m.d.sync += midi_debug.eq(1)
        m.d.sync += self.o.payload[3].eq(Mux(midi_debug,
            fixed.Const(0.5, shape=ASQ), 0))

        with m.If(self.i_usb_midi.valid):
            msg = self.i_usb_midi.payload
            with m.Switch(msg.status.kind):
                with m.Case(Status.Kind.NOTE_ON):
                    m.d.sync += [
                        # Gate output on
                        self.o.payload[0].eq(fixed.Const(0.5, shape=ASQ)),
                        # Set velocity output
                        self.o.payload[2].as_value().eq(
                            msg.midi_payload.note_on.velocity << 8),
                        # Set note index in LUT
                        rport.addr.eq(msg.midi_payload.note_on.note),
                    ]
                with m.Case(Status.Kind.NOTE_OFF):
                    # Zero gate and velocity on NOTE_OFF
                    m.d.sync += [
                        self.o.payload[0].eq(0),
                        self.o.payload[2].eq(0),
                    ]
                with m.Case(Status.Kind.CONTROL_CHANGE):
                    # mod wheel is CC 1
                    with m.If(msg.midi_payload.control_change.controller_number == 1):
                        m.d.sync += [
                            self.o.payload[3].as_value().eq(
                                msg.midi_payload.control_change.data << 8),
                        ]

        return m

