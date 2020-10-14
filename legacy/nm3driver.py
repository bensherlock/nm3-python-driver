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
import serial



class MessagePacket:
    """NM3 Message Packet Structure."""

    # Packet Type "Enums"
    PACKETTYPE_BROADCAST, PACKETTYPE_UNICAST = 'B', 'U'

    PACKETTYPE_NAMES = {
        PACKETTYPE_BROADCAST: 'Broadcast',
        PACKETTYPE_UNICAST: 'Unicast',
    }

    PACKETTYPES = (PACKETTYPE_BROADCAST, PACKETTYPE_UNICAST)

    def __init__(self):
        self._source_address = None
        self._destination_address = None
        self._packet_type = None
        self._packet_payload = None
        self._packet_timestamp_count = None


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


    def get_packet(self) -> MessagePacket:
        """Gets the next received packet or None if the queue is empty.
        """
        if not self._packet_queue:
            return None

        # Pop the packet from the queue
        packet = self._packet_queue.popleft()

        return packet



class Nm3:
    """NM3 Driver over Serial Port."""

    def __init__(self,
                 serial_port: serial.Serial):
        """Constructor. 
           Recommend that the serial port is created with a reasonable read timeout
           e.g. 100ms has been used in testing."""
        self._serial_port = serial_port
        self._incoming_bytes_buffer = deque() # List/Deque of integers
        self._received_packet_parser = MessagePacketParser()
        #self._received_packets = deque()

    def __call__(self):
        return self


    def get_address(self) -> int:
        """Gets the NM3 Address (000-255)."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response from the serial port
        # Expecting '#A255V21941\r\n' 13 bytes
        resp_string = '#A255V21941\r\n'
        resp_bytes = self._serial_port.read(13)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            return -1

        resp_string = resp_bytes.decode('utf-8')

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

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$A' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response from the serial port
        # Expecting '#A255\r\n' 7 bytes
        resp_string = '#A255\r\n'
        resp_bytes = self._serial_port.read(7)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            return -1

        resp_string = resp_bytes.decode('utf-8')

        addr_string = resp_string[2:5]
        addr_int = int(addr_string)

        return addr_int


    def get_battery_voltage(self) -> float:
        """Gets the NM3 Battery Voltage."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response from the serial port
        # Expecting '#A255V21941\r\n' 13 bytes
        resp_string = '#A255V21941\r\n'
        resp_bytes = self._serial_port.read(13)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            return -1

        resp_string = resp_bytes.decode('utf-8')

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

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Store the default timeout
        default_read_timeout = self._serial_port.timeout


        # Write the command to the serial port
        cmd_string = '$P' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the first response from the serial port
        # Expecting '$P255\r\n' 7 bytes
        resp_string = '$P255\r\n'
        resp_bytes = self._serial_port.read(7)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            return -1


        # Set the temporary read timeout for the propagation delay
        self._serial_port.timeout = timeout

        # Now await the range or TO after 4 seconds
        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = '#R255T12345\r\n'
        resp_bytes = self._serial_port.read(13)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            # Set the default read timeout
            self._serial_port.timeout = default_read_timeout
            return -1

        resp_string = resp_bytes.decode('utf-8')

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        time = float(time_int) * 31.25E-6

        # Set the default read timeout
        self._serial_port.timeout = default_read_timeout

        return time


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

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$B' + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the first response from the serial port
        # Expecting '$B00\r\n' 6 bytes
        resp_string = '$B00\r\n'
        resp_bytes = self._serial_port.read(6)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
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

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$U' + '{:03d}'.format(address) + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the first response from the serial port
        # Expecting '$U12300\r\n' 9 bytes
        resp_string = '$U12300\r\n'
        resp_bytes = self._serial_port.read(9)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
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

        # Clear the input buffer
        self._serial_port.reset_input_buffer()

        # Store the default timeout
        default_read_timeout = self._serial_port.timeout

        # Write the command to the serial port
        cmd_string = '$M' + '{:03d}'.format(address) + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        # Check that it has written all the bytes. Return error if not.
        if self._serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the first response from the serial port
        # Expecting '$M12300\r\n' 9 bytes
        resp_string = '$M12300\r\n'
        resp_bytes = self._serial_port.read(9)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            return -1

        # If the address is invalid then returns 'E\r\n'

        # Set the temporary read timeout for the propagation delay
        self._serial_port.timeout = timeout

        # Now await the range or TO after 4 seconds
        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = '#R255T12345\r\n'
        resp_bytes = self._serial_port.read(13)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            # Set the default read timeout
            self._serial_port.timeout = default_read_timeout
            return -1

        resp_string = resp_bytes.decode('utf-8')

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        time = float(time_int) * 31.25E-6

        # Set the default read timeout
        self._serial_port.timeout = default_read_timeout

        return time


    def poll_receiver(self):
        """Check the serial port and place bytes into incoming buffer for processing.
        """
        # TODO - Replace with generic IO input

        # Absorb any incoming bytes into the receive buffers to process later
        while self._serial_port.in_waiting:
            a_byte = self._serial_port.read()[0] # as individual integer
            self._incoming_bytes_buffer.append(a_byte)

        #return
		
    def poll_receiver_blocking(self):
        """Check the serial port and place bytes into incoming buffer for processing.
		   Blocking on serial port read until bytes received or timeout. 
        """
        # TODO - Replace with generic IO input

        # First byte is blocking
        some_bytes = self._serial_port.read() # Read one byte
        if some_bytes:
            a_byte = some_bytes[0]
            self._incoming_bytes_buffer.append(a_byte)

        # Absorb any incoming bytes into the receive buffers to process later
        while self._serial_port.in_waiting:
            a_byte = self._serial_port.read()[0] # as individual integer
            self._incoming_bytes_buffer.append(a_byte)

        #return


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
