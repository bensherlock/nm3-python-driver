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
from typing import Tuple, Union
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
                 packet_timestamp_count=None):
        self._source_address = source_address
        self._destination_address = destination_address
        self._packet_type = packet_type
        self._packet_payload = packet_payload
        self._packet_timestamp_count = packet_timestamp_count


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
    def packet_timestamp_count(self):
        """Gets the packet timestamp count - an overflowing 32-bit counter at 24MHz."""
        return self._packet_timestamp_count

    @packet_timestamp_count.setter
    def packet_timestamp_count(self,
                       packet_timestamp_count):
        """Sets the packet timestamp count - an overflowing 32-bit counter at 24MHz."""
        self._packet_timestamp_count = packet_timestamp_count


    def json(self):
        """Returns a json dictionary representation."""
        jason = {"SourceAddress": self._source_address,
                 "DestinationAddress": self._destination_address,
                 "PacketType": MessagePacket.PACKETTYPE_NAMES[self._packet_type],
                 "PayloadLength": len(self._packet_payload),
                 "PayloadBytes": self._packet_payload,
                 "PacketTimestampCount": self._packet_timestamp_count}

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
                             packet_timestamp_count=jason.get("PacketTimestampCount"))

        return message_packet



class MessagePacketParser:
    """Message Packet Parser takes bytes and uses a state machine to construct
       MessagePacket structures"""

    PARSERSTATE_IDLE, PARSERSTATE_TYPE, \
    PARSERSTATE_ADDRESS, PARSERSTATE_LENGTH, \
    PARSERSTATE_PAYLOAD, PARSERSTATE_TIMESTAMPFLAG, PARSERSTATE_TIMESTAMP = range(7)

    PARSERSTATE_NAMES = {
        PARSERSTATE_IDLE: 'Idle',
        PARSERSTATE_TYPE: 'Type',
        PARSERSTATE_ADDRESS: 'Address',
        PARSERSTATE_LENGTH: 'Length',
        PARSERSTATE_PAYLOAD: 'Payload',
        PARSERSTATE_TIMESTAMPFLAG: 'TimestampFlag',
        PARSERSTATE_TIMESTAMP: 'Timestamp',
    }

    PARSERSTATES = (PARSERSTATE_IDLE, PARSERSTATE_TYPE,
                    PARSERSTATE_ADDRESS, PARSERSTATE_LENGTH,
                    PARSERSTATE_PAYLOAD, PARSERSTATE_TIMESTAMPFLAG, PARSERSTATE_TIMESTAMP)

    def __init__(self):
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_message_packet = None
        self._current_byte_counter = 0
        self._current_integer = 0
        self._packet_queue = deque()

    def __call__(self):
        return self


    def reset(self):
        """Resets the parser state machine."""

        self._parser_state = self.PARSERSTATE_IDLE
        self._current_message_packet = None
        self._current_byte_counter = 0
        self._current_integer = 0


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

        return_flag = False

        #print('next_byte: ' + bytes([next_byte]).decode('utf-8'))

        if self._parser_state == self.PARSERSTATE_IDLE:

            if bytes([next_byte]).decode('utf-8') == '#':
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
                #self._packet_queue.append(self._current_message_packet)
                #self._current_message_packet = None
                #return_flag = True
                self._parser_state = self.PARSERSTATE_TIMESTAMPFLAG

        elif self._parser_state == self.PARSERSTATE_TIMESTAMPFLAG:

            if bytes([next_byte]).decode('utf-8') == 'T':
                self._current_byte_counter = 10
                self._current_integer = 0
                self._parser_state = self.PARSERSTATE_TIMESTAMP
            else:
                # No timestamp on this message. Completed Packet
                self._packet_queue.append(self._current_message_packet)
                self._current_message_packet = None
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

        elif self._parser_state == self.PARSERSTATE_TIMESTAMP:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                # Completed this packet
                self._current_message_packet.packet_timestamp_count = self._current_integer
                self._packet_queue.append(self._current_message_packet)
                self._current_message_packet = None
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

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

        #print('next_byte: ' + bytes([next_byte]).decode('utf-8'))

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
        self._incoming_bytes_buffer = deque() # List/Deque of integers
        self._received_packet_parser = MessagePacketParser()
        #self._received_packets = deque()

    def __call__(self):
        return self


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
        if not resp_string or len(resp_string) < 5:
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
        if not resp_string or len(resp_string) < 5:
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
        if not resp_string or len(resp_string) < 11:
            return -1

        adc_string = resp_string[6:11]
        adc_int = int(adc_string)

        # Convert the ADC value to a float voltage. V = adc_int * 15 / 65536.
        voltage = float(adc_int) * 15.0 / 65536.0

        return voltage


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

        # Expecting '$P255\r\n' 7 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5:  # E
            return -1


        # Now await the range or TO after 4 seconds
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11: # TO
            return -1

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        timeofarrival = float(time_int) * 31.25E-6

        return timeofarrival


    def send_broadcast_message(self,
                               message_bytes: bytes) -> int:
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
        if not resp_string or len(resp_string) < 4:
            return -1


        return len(message_bytes)


    def send_unicast_message(self,
                             address: int,
                             message_bytes: bytes) -> int:
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
        if not resp_string or len(resp_string) < 7:
            return -1

        # If the address is invalid then returns 'E\r\n'

        return len(message_bytes)


    def send_unicast_message_with_ack(self,
                                      address: int,
                                      message_bytes: bytes,
                                      timeout: float = 5.0) -> float:
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


        # Expecting '$M12300\r\n' 9 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 7:  # E
            return -1


        # Now await the range or TO after 4 seconds
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = time.time() + timeout
        while awaiting_response and (time.time() < timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11: # TO
            return -1

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        timeofarrival= float(time_int) * 31.25E-6

        return timeofarrival


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
            if max_bytes_count > 0 and byte_count >= max_bytes_count:
                return len(self._incoming_bytes_buffer)


        return len(self._incoming_bytes_buffer)


    def has_received_packet(self) -> bool:
        """Has received packet in the queue."""

        return self._received_packet_parser.has_packet()


    def get_received_packet(self) -> MessagePacket:
        """Gets the next received packet or None if the queue is empty.
        """

        return self._received_packet_parser.get_packet()
