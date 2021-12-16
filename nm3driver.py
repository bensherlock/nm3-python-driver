#! /usr/bin/env python
#
# Python Driver for NM3
#
# This file is part of NM3 Python Driver. https://github.com/bensherlock/nm3-python-driver
#
#
# MIT License
#
# Copyright (c) 2019 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""Python driver for the NM3 over serial port."""

from collections import deque
import struct
from typing import Tuple, Union, List
import time


class MessagePacket:
    """NM3 Message Packet Structure."""

    # Packet Type "Enums"
    PACKETTYPE_BROADCAST, PACKETTYPE_UNICAST = 'B', 'U'

    PACKETTYPE_NAMES = {
        PACKETTYPE_BROADCAST: 'Broadcast',
        PACKETTYPE_UNICAST: 'Unicast',
    }

    NAMES_PACKETTYPE = {
        'Broadcast': PACKETTYPE_BROADCAST,
        'Unicast': PACKETTYPE_UNICAST,
    }

    PACKETTYPES = (PACKETTYPE_BROADCAST, PACKETTYPE_UNICAST)

    def __init__(self, source_address=None, destination_address=None, packet_type=None, packet_payload=None,
                 packet_lqi=None, packet_doppler=None,
                 packet_timestamp_count=None):
        self._source_address = source_address
        self._destination_address = destination_address
        self._packet_type = packet_type
        self._packet_payload = packet_payload
        self._packet_lqi = packet_lqi  # Optional link quality indicator (LQI)
        self._packet_doppler = packet_doppler  # Optional Doppler tracking
        self._packet_timestamp_count = packet_timestamp_count  # Optional timestamp

        self._serial_string = None # The processed received uart string

    def __call__(self):
        return self

    @property
    def source_address(self) -> int:
        """Gets the source address."""
        return self._source_address

    @source_address.setter
    def source_address(self,
                       source_address: int):
        """Sets the the source address (0-255)."""
        if source_address and (source_address < 0 or source_address > 255):
            raise ValueError('Invalid Address Value (0-255): {!r}'.format(source_address))
        self._source_address = source_address

    @property
    def destination_address(self) -> int:
        """Gets the destination address."""
        return self._destination_address

    @destination_address.setter
    def destination_address(self,
                            destination_address: int):
        """Sets the the destination address (0-255)."""
        if destination_address and (destination_address < 0 or destination_address > 255):
            raise ValueError('Invalid Address Value (0-255): {!r}'.format(destination_address))
        self._destination_address = destination_address

    @property
    def packet_type(self):
        """Gets the packet type (unicast, broadcast)."""
        return self._packet_type

    @packet_type.setter
    def packet_type(self,
                    packet_type):
        """Sets the the packet type (unicast, broadcast)."""
        if packet_type not in self.PACKETTYPES:
            raise ValueError('Invalid Packet Type: {!r}'.format(packet_type))
        self._packet_type = packet_type

    @property
    def packet_payload(self):
        """Gets the packet payload bytes."""
        return self._packet_payload

    @packet_payload.setter
    def packet_payload(self,
                       packet_payload):
        """Sets the the packet payload bytes ."""
        self._packet_payload = packet_payload

    @property
    def packet_lqi(self):
        """Gets the packet LQI - a number between 00 and 99 inclusive."""
        return self._packet_lqi

    @packet_lqi.setter
    def packet_lqi(self, packet_lqi):
        self._packet_lqi = packet_lqi


    @property
    def packet_doppler(self):
        """Gets the packet Doppler - a number between +/-999 and 999 inclusive."""
        return self._packet_doppler

    @packet_doppler.setter
    def packet_doppler(self, packet_doppler):
        self._packet_doppler = packet_doppler

    @property
    def packet_timestamp_count(self):
        """Gets the packet timestamp count - an overflowing 32-bit counter at 24MHz."""
        return self._packet_timestamp_count

    @packet_timestamp_count.setter
    def packet_timestamp_count(self,
                       packet_timestamp_count):
        """Sets the packet timestamp count - an overflowing 32-bit counter at 24MHz."""
        self._packet_timestamp_count = packet_timestamp_count


    @property
    def serial_string(self) -> str:
        return self._serial_string
    
    @serial_string.setter
    def serial_string(self, serial_string: str):
        self._serial_string = serial_string
        


    def json(self):
        """Returns a json dictionary representation."""
        jason = {"SourceAddress": self._source_address,
                 "DestinationAddress": self._destination_address,
                 "PacketType": MessagePacket.PACKETTYPE_NAMES[self._packet_type],
                 "PayloadLength": len(self._packet_payload),
                 "PayloadBytes": self._packet_payload,
                 "PacketLqi": self._packet_lqi,
                 "PacketDoppler": self._packet_doppler,
                 "PacketTimestampCount": self._packet_timestamp_count
                 }

        return jason

    @classmethod
    def from_json(cls, jason):
        """Constructs a MessagePacket from a json dictionary representation."""

        # convert packet type name to packet Type
        packet_type_name = jason.get("PacketType")
        packet_type = None
        if packet_type_name:
            packet_type = cls.NAMES_PACKETTYPE.get(packet_type_name)

        message_packet = cls(source_address=jason.get("SourceAddress"),
                             destination_address=jason.get("DestinationAddress"),
                             packet_type=packet_type,
                             packet_payload=jason.get("PayloadBytes"),
                             packet_lqi=jason.get("PacketLqi"),
                             packet_doppler=jason.get("PacketDoppler"),
                             packet_timestamp_count=jason.get("PacketTimestampCount")
                             )

        return message_packet


class MessagePacketParser:
    """Message Packet Parser takes bytes and uses a state machine to construct
       MessagePacket structures"""

    PARSERSTATE_IDLE, PARSERSTATE_TYPE, \
    PARSERSTATE_ADDRESS, PARSERSTATE_LENGTH, \
    PARSERSTATE_PAYLOAD, PARSERSTATE_ADDENDUMFLAG, PARSERSTATE_LQI, PARSERSTATE_DOPPLER, PARSERSTATE_TIMESTAMP = range(9)

    PARSERSTATE_NAMES = {
        PARSERSTATE_IDLE: 'Idle',
        PARSERSTATE_TYPE: 'Type',
        PARSERSTATE_ADDRESS: 'Address',
        PARSERSTATE_LENGTH: 'Length',
        PARSERSTATE_PAYLOAD: 'Payload',
        PARSERSTATE_ADDENDUMFLAG: 'AddendumFlag',
        PARSERSTATE_LQI: 'Lqi',
        PARSERSTATE_DOPPLER: 'Doppler',
        PARSERSTATE_TIMESTAMP: 'Timestamp'

    }

    PARSERSTATES = (PARSERSTATE_IDLE, PARSERSTATE_TYPE,
                    PARSERSTATE_ADDRESS, PARSERSTATE_LENGTH,
                    PARSERSTATE_PAYLOAD, PARSERSTATE_ADDENDUMFLAG, PARSERSTATE_LQI, PARSERSTATE_DOPPLER, PARSERSTATE_TIMESTAMP)

    def __init__(self):
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_message_packet = None
        self._current_byte_counter = 0
        self._current_integer = 0
        self._current_integer_sign = 1
        self._current_serial_string = None
        self._packet_queue = deque()

    def __call__(self):
        return self

    def reset(self):
        """Resets the parser state machine."""

        self._parser_state = self.PARSERSTATE_IDLE
        self._current_message_packet = None
        self._current_byte_counter = 0
        self._current_integer = 0
        self._current_integer_sign = 1
        self._current_serial_string = None

    def process(self, next_byte) -> bool:
        """Process the next byte. Returns True if a packet completes on this byte."""

        # Received Message Structures:
        # '#B25500' + payload bytes + '\r\n'
        # '#U00' + payload bytes + '\r\n'
        # Or for some NM3 firmware versions:
        # '#B25500' + payload bytes + 'T' + timestamp + '\r\n'
        # '#U00' + payload bytes + 'T' + timestamp + '\r\n'
        # Where timestamp is a 10 digit (fixed width) number representing a 32-bit counter value 
        # on a 24 MHz clock which is latched when the synch waveform arrives
        # Or for future release firmware versions (v1.1.0+)
        # '#B25500' + payload bytes + 'Q' + lqi + 'D' + doppler + '\r\n'
        # '#U00' + payload bytes + 'Q' + lqi + 'D' + doppler + '\r\n'
        # And if we end up with mix and match addendums/addenda
        # '#B25500' + payload bytes + 'Q' + lqi + 'D' + doppler + 'T' + timestamp + '\r\n'
        # '#U00' + payload bytes + 'T' + timestamp + 'Q' + lqi + 'D' + doppler + '\r\n'

        return_flag = False

        # print('next_byte: ' + bytes([next_byte]).decode('utf-8'))

        if self._parser_state != self.PARSERSTATE_IDLE:
            # Store bytes
            self._current_serial_string = self._current_serial_string +  bytes([next_byte]).decode('utf-8')


        if self._parser_state == self.PARSERSTATE_IDLE:

            if bytes([next_byte]).decode('utf-8') == '#':
                self._current_serial_string = '#'
                # Next state
                self._parser_state = self.PARSERSTATE_TYPE

        elif self._parser_state == self.PARSERSTATE_TYPE:

            if bytes([next_byte]).decode('utf-8') == 'B':
                self._current_message_packet = MessagePacket()
                self._current_message_packet.source_address = 0
                self._current_message_packet.destination_address = None
                self._current_message_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
                self._current_message_packet.packet_payload = []
                self._current_message_packet.packet_timestamp_count = 0

                self._current_byte_counter = 3
                self._current_integer = 0
                self._parser_state = self.PARSERSTATE_ADDRESS

            elif bytes([next_byte]).decode('utf-8') == 'U':
                self._current_message_packet = MessagePacket()
                self._current_message_packet.source_address = None
                self._current_message_packet.destination_address = None
                self._current_message_packet.packet_type = MessagePacket.PACKETTYPE_UNICAST
                self._current_message_packet.packet_payload = []
                self._current_message_packet.packet_timestamp_count = 0

                self._current_byte_counter = 2
                self._current_integer = 0
                self._parser_state = self.PARSERSTATE_LENGTH

            else:
                # Unknown packet type
                self._parser_state = self.PARSERSTATE_IDLE

        elif self._parser_state == self.PARSERSTATE_ADDRESS:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                self._current_message_packet.source_address = self._current_integer
                self._current_integer = 0
                self._current_byte_counter = 2
                self._parser_state = self.PARSERSTATE_LENGTH

        elif self._parser_state == self.PARSERSTATE_LENGTH:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                self._current_byte_counter = self._current_integer
                self._parser_state = self.PARSERSTATE_PAYLOAD

        elif self._parser_state == self.PARSERSTATE_PAYLOAD:
            self._current_byte_counter = self._current_byte_counter - 1

            self._current_message_packet.packet_payload.append(next_byte)

            if self._current_byte_counter == 0:
                # Completed this packet
                self._parser_state = self.PARSERSTATE_ADDENDUMFLAG

        elif self._parser_state == self.PARSERSTATE_ADDENDUMFLAG:

            # Timestamp Addendum
            if bytes([next_byte]).decode('utf-8') == 'T':
                self._current_byte_counter = 14
                self._current_integer = 0
                self._current_integer_sign = 1
                self._parser_state = self.PARSERSTATE_TIMESTAMP

            # LQI Addendum
            elif bytes([next_byte]).decode('utf-8') == 'Q':
                self._current_byte_counter = 2
                self._current_integer = 0
                self._current_integer_sign = 1
                self._parser_state = self.PARSERSTATE_LQI

            # Doppler Addendum
            elif bytes([next_byte]).decode('utf-8') == 'D':
                self._current_byte_counter = 4
                self._current_integer = 0
                self._current_integer_sign = 1
                self._parser_state = self.PARSERSTATE_DOPPLER

            # Unrecognised or no addendum
            else:
                # No recognised addendum on this message. Completed Packet
                self._current_message_packet.serial_string = self._current_serial_string
                self._current_serial_string = None
                self._packet_queue.append(self._current_message_packet)
                self._current_message_packet = None
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

        elif self._parser_state == self.PARSERSTATE_TIMESTAMP:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                # Completed this addendum
                self._current_message_packet.packet_timestamp_count = self._current_integer
                # Back to checking for further addendums/addenda
                self._parser_state = self.PARSERSTATE_ADDENDUMFLAG

        elif self._parser_state == self.PARSERSTATE_LQI:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                # Completed this addendum
                self._current_message_packet.packet_lqi = self._current_integer
                # Back to checking for further addendums/addenda
                self._parser_state = self.PARSERSTATE_ADDENDUMFLAG

        elif self._parser_state == self.PARSERSTATE_DOPPLER:
            self._current_byte_counter = self._current_byte_counter - 1

            # Check for + or -
            if bytes([next_byte]).decode('utf-8') == '+':
                self._current_integer_sign = 1
            elif bytes([next_byte]).decode('utf-8') == '-':
                self._current_integer_sign = -1
            else:
                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + (self._current_integer_sign * int(bytes([next_byte]).decode('utf-8')))

            if self._current_byte_counter == 0:
                # Completed this addendum
                self._current_message_packet.packet_doppler = self._current_integer
                # Back to checking for further addendums/addenda
                self._parser_state = self.PARSERSTATE_ADDENDUMFLAG

        else:
            # Unknown state
            self._parser_state = self.PARSERSTATE_IDLE

        return return_flag

    def has_packet(self) -> bool:
        """Has packets in the queue."""

        if self._packet_queue:
            return True

        return False

    def get_packet(self) -> Union[MessagePacket, None]:
        """Gets the next received packet or None if the queue is empty.
        """
        if not self._packet_queue:
            return None

        # Pop the packet from the queue
        packet = self._packet_queue.popleft()

        return packet


class Nm3ResponseParser:
    """Parser for responses to commands."""

    PARSERSTATE_IDLE, PARSERSTATE_STRING = range(2)

    PARSERSTATE_NAMES = {
        PARSERSTATE_IDLE: 'Idle',
        PARSERSTATE_STRING: 'String',
    }

    PARSERSTATES = (PARSERSTATE_IDLE, PARSERSTATE_STRING)

    def __init__(self):
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_bytes = []
        self._current_byte_counter = 0
        self._delimiter_byte = ord('\n')
        self._has_response_flag = False

    def reset(self):
        """Resets the parser state machine."""
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_bytes = []
        self._current_byte_counter = 0
        self._has_response_flag = False

    def set_delimiter_byte(self, delimiter_byte):
        self._delimiter_byte = delimiter_byte

    def process(self, next_byte) -> bool:
        """Process the next byte. Returns True if a response completes on this byte."""

        return_flag = False

        # print('next_byte: ' + bytes([next_byte]).decode('utf-8'))

        if self._parser_state == self.PARSERSTATE_IDLE:

            if (bytes([next_byte]).decode('utf-8') == '#') or (bytes([next_byte]).decode('utf-8') == '$'):
                # Next state
                self._current_bytes = [next_byte]  # Include the '#' or '$' character.
                self._current_byte_counter = 1
                self._parser_state = self.PARSERSTATE_STRING

        elif self._parser_state == self.PARSERSTATE_STRING:
            self._current_bytes.append(next_byte)
            self._current_byte_counter = self._current_byte_counter + 1

            # Check delimiter
            if next_byte == self._delimiter_byte:
                self._has_response_flag = True
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

        else:
            # Unknown
            self._parser_state = self.PARSERSTATE_IDLE

        return return_flag

    def has_response(self):
        return self._has_response_flag

    def get_last_response_string(self):
        return bytes(self._current_bytes).decode('utf-8')


class Nm3:
    """NM3 Driver over Serial Port."""

    RESPONSE_TIMEOUT = 0.5

    def __init__(self, input_stream, output_stream):
        """Constructor. input_stream and output_stream need to be (bytes) IO with
        non-blocking Read() and Write() binary functions."""

        self._input_stream = input_stream
        self._output_stream = output_stream
        self._incoming_bytes_buffer = deque()  # List/Deque of integers
        self._received_packet_parser = MessagePacketParser()

    def __call__(self):
        return self

    def query_status(self):
        """Query the modem status ($? command).
        Returns the address and battery voltage.
        In newer firmware versions (v1.1.0+) it also returns the semantic version number and the build date."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1
        #            0123456789012        012345678901234567890123456789012345678901234
        # Expecting '#A255V21941\r\n' or '#A255V21941R001.001.000B2021-05-26T09:23:46\r\n'
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11 or resp_string[0:2] != '#A':
            return -1

        addr_string = resp_string[2:5]
        addr_int = int(addr_string)

        adc_string = resp_string[6:11]
        adc_int = int(adc_string)

        # Convert the ADC value to a float voltage. V = adc_int * 15 / 65536.
        voltage = float(adc_int) * 15.0 / 65536.0

        version_string = ''
        build_date_string = ''

        if len(resp_string) >= 23:
            version_string = resp_string[12:23]

        if len(resp_string) >= 43:
            build_date_string = resp_string[24:43]

        return addr_int, voltage, version_string, build_date_string

    def get_address(self) -> int:
        """Gets the NM3 Address (000-255)."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#A255V21941\r\n'
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5 or resp_string[0:2] != '#A':
            return -1

        addr_string = resp_string[2:5]
        addr_int = int(addr_string)

        return addr_int

    def set_address(self,
                    address: int) -> int:
        """Sets the NM3 Address (000-255)."""

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$A' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#A255\r\n' 7 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5 or resp_string[0:2] != '#A':
            return -1

        addr_string = resp_string[2:5]
        addr_int = int(addr_string)

        return addr_int

    def get_battery_voltage(self) -> float:
        """Gets the NM3 Battery Voltage."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#A255V21941\r\n'
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11 or resp_string[0:2] != '#A':
            return -1

        adc_string = resp_string[6:11]
        adc_int = int(adc_string)

        # Convert the ADC value to a float voltage. V = adc_int * 15 / 65536.
        voltage = float(adc_int) * 15.0 / 65536.0

        return voltage

    def measure_local_ambient_noise(self):
        """Measure the local ambient noise for a duration of around 1 second.
        Returns RMS, P2P (peak-to-peak) and MM (mean magnitude) values in ADC units.
        rms_int, p2p_int, mm_int"""

        timeout = 2.0  # 2 second fixed timeout

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$N'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        resp_bytes = deque()  # Create the queue object for first response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '$N\r\n' 4 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 4 or resp_string[0:2] != '$N':  # E
            return -1

        # Now await the measurement after around 1 second
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        #            0123456789012345678901234
        # Expecting '#NR123456P123456M123456\r\n' 25 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 25 or resp_string[0:2] != '#N':  #
            return -1

        rms_string = resp_string[3:9]
        rms_int = int(rms_string)

        p2p_string = resp_string[10:16]
        p2p_int = int(p2p_string)

        mm_string = resp_string[17:23]
        mm_int = int(mm_string)

        return rms_int, p2p_int, mm_int

    def measure_local_noise_spectrum(self):
        """Measure the local noise spectrum for a duration of around 2 seconds.
        Returns bin count and FFT values in ADC units.
        Returned bin count is equivalent to NFFT/2 + 1, the returned values occupy the positive frequency bins
        from DC to Nyquist/2 inclusive.
        Sampling frequency on the device is 160kHz, for NFFT=512 the bin width = 312.5Hz."""

        timeout = 6.0  # 6 second fixed timeout

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$S'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        resp_bytes = deque()  # Create the queue object for first response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '$S\r\n' 4 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 4 or resp_string[0:2] != '$S':  # E
            return -1

        # Now await the measurement after around 2 seconds
        # '#Snnnnnbbbbbbbbbbbbb\r\n Expected response is ascii count then binary data.
        #
        # Await the response
        PARSERSTATE_IDLE, PARSERSTATE_HASH, PARSERSTATE_COUNT, PARSERSTATE_DATA = range(4)

        parser_state = PARSERSTATE_IDLE
        current_byte_counter = 5  # For ascii integer
        current_integer = 0  # For ascii integer
        current_data_bytes = []

        data_count = 0
        data_value_size = 2
        data_values = []

        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()

                    if parser_state == PARSERSTATE_IDLE:
                        if bytes([b]).decode('utf-8') == '#':
                            parser_state = PARSERSTATE_HASH
                        pass
                    elif parser_state == PARSERSTATE_HASH:
                        if bytes([b]).decode('utf-8') == 'S':
                            current_byte_counter = 5  # For ascii integer
                            current_integer = 0  # For ascii integer

                            parser_state = PARSERSTATE_COUNT
                        pass
                    elif parser_state == PARSERSTATE_COUNT:
                        current_byte_counter = current_byte_counter - 1

                        # Append the next ascii string integer digit
                        current_integer = (current_integer * 10) + int(bytes([b]).decode('utf-8'))

                        if current_byte_counter == 0:
                            data_count = current_integer
                            current_integer = 0
                            current_byte_counter = data_count * data_value_size
                            current_data_bytes = []
                            parser_state = PARSERSTATE_DATA

                        pass
                    elif parser_state == PARSERSTATE_DATA:
                        current_byte_counter = current_byte_counter - 1

                        current_data_bytes.append(b)

                        if current_byte_counter == 0:
                            # Convert the bytes into integers - https://docs.python.org/3/library/struct.html
                            data_values = struct.unpack('<' + str(data_count) + 'H', bytes(bytearray(current_data_bytes)))

                            parser_state = PARSERSTATE_IDLE
                            # Got a response
                            awaiting_response = False

                        pass
                    else:
                        # Unknown state
                        parser_state = PARSERSTATE_IDLE
                        pass

        if awaiting_response:
            return -1

        return data_count, data_values

    def send_ping(self,
                  address: int,
                  timeout: float = 5.0) -> float:
        """Sends a ping to the addressed node and returns the one way time of flight in seconds
           from this device to the node address provided.
        """

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$P' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        resp_bytes = deque()  # Create the queue object for first response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '$P255\r\n' 7 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5 or resp_string[0:2] != '$P':  # E
            return -1

        # Now await the range or TO after 4 seconds
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11 or resp_string[0:2] != '#R':  # TO
            return -1

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        timeofarrival = float(time_int) * 31.25E-6

        return timeofarrival

    def send_ping_for_channel_impulse_response(self,
                  address: int,
                  mode_magnitude_or_complex = 'M',
                  timeout: float = 7.0):  # -> Tuple[float, int, List[int]]:
        """Sends a ping to the addressed node and returns a normalised channel impulse response where 0-1 == 0->50000
        and the one way time of flight in seconds from this device to the node address provided.
        Returns, time of flight, data count, data values."""

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$C' + mode_magnitude_or_complex + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        resp_bytes = deque()  # Create the queue object for first response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '$CM255\r\n' or '$CC255\r\n' 8 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 6 or resp_string[0:2] != '$C':  # E
            return -1

        # Now await the range or TO after 4 seconds of propagation (processing takes longer)
        # Now await the measurement after around 2 seconds
        # #CM255T12345Lnnnnnbbbbbbbbb\r\n' or '#TO\r\n' ? or 5 bytes
        # Expected response is ascii count then binary data.
        #
        # Await the response
        PARSERSTATE_IDLE, PARSERSTATE_HASH, PARSERSTATE_MODE, PARSERSTATE_ADDRESS, PARSERSTATE_TIMEFLAG, PARSERSTATE_TIME, \
        PARSERSTATE_COUNTFLAG, PARSERSTATE_COUNT, PARSERSTATE_DATA = range(9)

        parser_state = PARSERSTATE_IDLE
        current_byte_counter = 3  # For ascii integer
        current_integer = 0  # For ascii integer
        current_data_bytes = []

        data_count = 0
        data_value_size = 2
        data_values = []

        timeofflight_count = 0
        address = 0

        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()

                    if parser_state == PARSERSTATE_IDLE:
                        if bytes([b]).decode('utf-8') == '#':
                            parser_state = PARSERSTATE_HASH
                        pass

                    elif parser_state == PARSERSTATE_HASH:
                        if bytes([b]).decode('utf-8') == 'C':
                            parser_state = PARSERSTATE_MODE

                        else:
                            # Error or timeout
                            return -1
                        pass

                    elif parser_state == PARSERSTATE_MODE:
                        if bytes([b]).decode('utf-8') == 'M':

                            current_byte_counter = 3  # For ascii integer
                            current_integer = 0  # For ascii integer
                            parser_state = PARSERSTATE_ADDRESS

                        elif bytes([b]).decode('utf-8') == 'C':

                            current_byte_counter = 3  # For ascii integer
                            current_integer = 0  # For ascii integer
                            parser_state = PARSERSTATE_ADDRESS

                        else:
                            # Error or timeout
                            return -1
                        pass

                    elif parser_state == PARSERSTATE_ADDRESS:
                        current_byte_counter = current_byte_counter - 1

                        # Append the next ascii string integer digit
                        current_integer = (current_integer * 10) + int(bytes([b]).decode('utf-8'))

                        if current_byte_counter == 0:
                            address = current_integer

                            current_integer = 0
                            current_byte_counter = data_count * data_value_size
                            current_data_bytes = []
                            parser_state = PARSERSTATE_TIMEFLAG

                        pass

                    elif parser_state == PARSERSTATE_TIMEFLAG:
                        if bytes([b]).decode('utf-8') == 'T':

                            current_byte_counter = 5  # For ascii integer
                            current_integer = 0  # For ascii integer
                            parser_state = PARSERSTATE_TIME

                        else:
                            # Error
                            return -1
                        pass

                    elif parser_state == PARSERSTATE_TIME:
                        current_byte_counter = current_byte_counter - 1

                        # Append the next ascii string integer digit
                        current_integer = (current_integer * 10) + int(bytes([b]).decode('utf-8'))

                        if current_byte_counter == 0:
                            timeofflight_count = current_integer

                            current_integer = 0
                            current_byte_counter = data_count * data_value_size
                            current_data_bytes = []
                            parser_state = PARSERSTATE_COUNTFLAG

                        pass

                    elif parser_state == PARSERSTATE_COUNTFLAG:
                        if bytes([b]).decode('utf-8') == 'L':

                            current_byte_counter = 5  # For ascii integer
                            current_integer = 0  # For ascii integer
                            parser_state = PARSERSTATE_COUNT

                        else:
                            # Error
                            return -1
                        pass

                    elif parser_state == PARSERSTATE_COUNT:
                        current_byte_counter = current_byte_counter - 1

                        # Append the next ascii string integer digit
                        current_integer = (current_integer * 10) + int(bytes([b]).decode('utf-8'))

                        if current_byte_counter == 0:
                            data_count = current_integer
                            current_integer = 0
                            current_byte_counter = data_count * data_value_size
                            current_data_bytes = []
                            parser_state = PARSERSTATE_DATA

                        pass

                    elif parser_state == PARSERSTATE_DATA:
                        current_byte_counter = current_byte_counter - 1

                        current_data_bytes.append(b)

                        if current_byte_counter == 0:
                            # Convert the bytes into integers - https://docs.python.org/3/library/struct.html
                            if mode_magnitude_or_complex == 'M':
                                # Magnitudes so unsigned int (uint16_t)

                                data_values = struct.unpack('<' + str(data_count) + 'H',
                                                            bytes(bytearray(current_data_bytes)))

                            elif mode_magnitude_or_complex == 'C':
                                # Complex so signed int (int16_t)

                                data_values = struct.unpack('<' + str(data_count) + 'h',
                                                            bytes(bytearray(current_data_bytes)))

                            parser_state = PARSERSTATE_IDLE
                            # Got a response
                            awaiting_response = False

                        pass
                    else:
                        # Unknown state
                        parser_state = PARSERSTATE_IDLE
                        pass

        if awaiting_response:
            return -1

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        timeofarrival = float(timeofflight_count) * 31.25E-6

        return timeofarrival, data_count, data_values

    def send_broadcast_message(self,
                               message_bytes: bytes,
                               transmit_timestamp: int = None) -> int:
        """Sends a broadcast message of message_bytes. Maximum of 64 bytes.
        """

        # Checks on parameters
        if len(message_bytes) < 2 or len(message_bytes) > 64:
            print('Invalid length of message_bytes (2-64): ' + str(len(message_bytes)))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$B' + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        if transmit_timestamp:
            timestamp_string = 'T{:014d}'.format(transmit_timestamp)
            cmd_bytes = cmd_bytes + timestamp_string.encode('utf-8')

        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '$B00\r\n' 6 bytes

        # Check that it has received all the expected bytes. Return error if not.
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 4 or resp_string[0:2] != '$B':
            return -1

        return len(message_bytes)

    def send_unicast_message(self,
                             address: int,
                             message_bytes: bytes,
                             transmit_timestamp: int = None) -> int:
        """Sends a unicast message of message_bytes to address. Maximum of 64 bytes.
        """

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        if len(message_bytes) < 2 or len(message_bytes) > 64:
            print('Invalid length of message_bytes (2-64): ' + str(len(message_bytes)))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$U' + '{:03d}'.format(address) + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        if transmit_timestamp:
            timestamp_string = 'T{:014d}'.format(transmit_timestamp)
            cmd_bytes = cmd_bytes + timestamp_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '$U12300\r\n' 9 bytes

        # Check that it has received all the expected bytes. Return error if not.
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 7 or resp_string[0:2] != '$U':
            return -1

        # If the address is invalid then returns 'E\r\n'

        return len(message_bytes)

    def send_unicast_message_with_ack(self,
                                      address: int,
                                      message_bytes: bytes,
                                      timeout: float = 5.0,
                                      transmit_timestamp: int = None) -> float:
        """Sends a unicast message of message_bytes to address. Maximum of 64 bytes.
           Waits for Ack from the remote node and returns the time of flight in seconds.
        """

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        if len(message_bytes) < 2 or len(message_bytes) > 64:
            print('Invalid length of message_bytes (2-64): ' + str(len(message_bytes)))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        cmd_string = '$M' + '{:03d}'.format(address) + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        if transmit_timestamp:
            timestamp_string = 'T{:014d}'.format(transmit_timestamp)
            cmd_bytes = cmd_bytes + timestamp_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        resp_bytes = deque()  # Create the queue object for first response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '$M12300\r\n' 9 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 7 or resp_string[0:2] != '$M':  # E
            return -1

        # Now await the range or TO after 4 seconds
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp = self._input_stream.read()

            if resp:
                for b in resp:
                    resp_bytes.append(b)

            if resp_bytes:
                while resp_bytes:
                    b = resp_bytes.popleft()
                    if response_parser.process(b):
                        # Got a response
                        awaiting_response = False
                        break

        if not response_parser.has_response():
            return -1

        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11 or resp_string[0:2] != '#R':  # TO
            return -1

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        timeofarrival= float(time_int) * 31.25E-6

        return timeofarrival


    def system_time_command(self, cmd_string):
        """Internal function for system timer commands. Returns is_enabled and counter value."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()

        # Write the command to the serial port
        #cmd_string = '$XTG'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + Nm3.RESPONSE_TIMEOUT
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1
        #            01234567890123456789
        # Expecting '#XTetttttttttttttt\r\n'
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 18 or resp_string[0:3] != '#XT':
            return -1

        enabled_string = resp_string[3:4]
        time_count_string = resp_string[4:18]

        enabled_flag = False
        if enabled_string == 'E':
            enabled_flag = True

        time_count_int = int(time_count_string)

        return enabled_flag, time_count_int


    def get_system_timer_status(self):
        """Get the status of the system timer. Returns is_enabled and counter value."""

        return self.system_time_command(cmd_string='$XTG')


    def enable_system_timer(self):
        """Enable the system timer. Returns is_enabled and counter value."""

        return self.system_time_command(cmd_string='$XTE')

    def disable_system_timer(self):
        """Disable the system timer. Returns is_enabled and counter value."""

        return self.system_time_command(cmd_string='$XTD')

    def clear_system_timer(self):
        """Clears the system timer. Returns is_enabled and counter value."""

        return self.system_time_command(cmd_string='$XTC')



    def poll_receiver(self):
        """Check the input_stream (non-blocking) and place bytes into incoming buffer for processing.
        """

        resp_bytes = self._input_stream.read()
        while resp_bytes:
            for a_byte in resp_bytes:
                self._incoming_bytes_buffer.append(a_byte)

            resp_bytes = self._input_stream.read()

    def process_incoming_buffer(self,
                                max_bytes_count: int = 0) -> int:
        """Process the bytes stored in the incoming buffer up to the max_bytes_count (0=ignore).
           Returns the number of bytes still awaiting processing.
        """

        byte_count = 0
        while self._incoming_bytes_buffer:
            self._received_packet_parser.process(self._incoming_bytes_buffer.popleft())
            byte_count = byte_count + 1

            # Check for max_bytes_count
            if 0 < max_bytes_count <= byte_count:
                return len(self._incoming_bytes_buffer)

        return len(self._incoming_bytes_buffer)

    def has_received_packet(self) -> bool:
        """Has received packet in the queue."""

        return self._received_packet_parser.has_packet()

    def get_received_packet(self) -> MessagePacket:
        """Gets the next received packet or None if the queue is empty.
        """

        return self._received_packet_parser.get_packet()
