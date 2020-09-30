#! /usr/bin/env python
#
# NM V3 Simulator
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
"""NM V3 Simulator to test code using the Nm3 driver. """


import argparse
import random
import serial
import time

class Nm3SimulatedNode:
    """A NM3 Simulated Node."""

    def __init__(self, address=255, distance_m=1000.0, probability=1.0, speed_of_sound=1500.0):
        self._address = address
        self._distance_m = distance_m
        self._probability = probability
        self._speed_of_sound = speed_of_sound

    @property
    def address(self):
        return self._address

    @address.setter
    def address(self, address):
        self._address = address

    @property
    def distance_m(self):
        return self._distance_m

    @distance_m.setter
    def distance_m(self, distance_m):
        self._distance_m = distance_m

    @property
    def probability(self):
        return self._probability

    @probability.setter
    def probability(self, probability):
        if probability > 1.0:
            probability = 1.0
        elif probability < 0.0:
            probability = 0.0
        self._probability = probability

    @property
    def speed_of_sound(self):
        return self._speed_of_sound

    @speed_of_sound.setter
    def speed_of_sound(self, speed_of_sound):
        self._speed_of_sound = speed_of_sound



class Nm3Simulator:
    """NM3 Simulator Class.
    Currently supports: Query ($?), Set Local Address ($Axxx), Ping ($Pxxx),
    Broadcast Message ($Byydd...dd), Unicast Message ($Uxxxyydd..dd),
    Unicast with Ack Message ($Mxxxyydd..dd)."""

    SIMULATOR_STATE_IDLE, SIMULATOR_STATE_COMMAND, \
    SIMULATOR_STATE_SET_ADDRESS, SIMULATOR_STATE_PING, \
    SIMULATOR_STATE_MESSAGE_ADDRESS, SIMULATOR_STATE_MESSAGE_LENGTH, \
    SIMULATOR_STATE_MESSAGE_DATA = range(7)

    SIMULATOR_STATE_NAMES = {
        SIMULATOR_STATE_IDLE: 'Idle',
        SIMULATOR_STATE_COMMAND: 'Command',
        SIMULATOR_STATE_SET_ADDRESS: 'SetAddress',
        SIMULATOR_STATE_PING: 'Ping',
        SIMULATOR_STATE_MESSAGE_ADDRESS: 'MessageAddress',
        SIMULATOR_STATE_MESSAGE_LENGTH: 'MessageLength',
        SIMULATOR_STATE_MESSAGE_DATA: 'MessageData',
    }

    SIMULATOR_STATES = (SIMULATOR_STATE_IDLE, SIMULATOR_STATE_COMMAND,
                        SIMULATOR_STATE_SET_ADDRESS, SIMULATOR_STATE_PING,
                        SIMULATOR_STATE_MESSAGE_ADDRESS, SIMULATOR_STATE_MESSAGE_LENGTH,
                        SIMULATOR_STATE_MESSAGE_DATA)



    def __init__(self, input_stream, output_stream, local_address=255):
        """input_stream and output_stream implement the Bytes IO interface.
        Namely: readable()->bool, writeable()->bool, read(bytes) and write(bytes)."""
        self._input_stream = input_stream
        self._output_stream = output_stream
        self._simulator_state = self.SIMULATOR_STATE_IDLE

        self._local_address = local_address

        # Parser variables
        self._current_byte_counter = 0
        self._current_integer = 0

        # Sending message
        self._message_type = None
        self._message_address = None
        self._message_length = None
        self._message_bytes = None

        # Simulated Nodes
        self._nm3_simulated_nodes = {}

    def __call__(self):
        return self

    def add_nm3_simulated_node(self, address, distance_m, probability, speed_of_sound=1500.0):
        """Add a simulated node"""
        node = Nm3SimulatedNode(address, distance_m, probability, speed_of_sound)
        self._nm3_simulated_nodes[address] = node


    def run(self):
        """Run the simulator. Never returns."""
        while True:
            if self._input_stream.readable():
                some_bytes = self._input_stream.read()  # Read

                if some_bytes:
                    #print("Bytes received len(" + str(len(some_bytes)) + ")")
                    self.process_bytes(some_bytes)

    def process_bytes(self, some_bytes: bytes):
        """Process bytes in the state machine and act accordingly."""
        for b in some_bytes:
            if self._simulator_state == self.SIMULATOR_STATE_IDLE:
                if bytes([b]).decode('utf-8') == '$':
                    self._simulator_state = self.SIMULATOR_STATE_COMMAND

            elif self._simulator_state == self.SIMULATOR_STATE_COMMAND:
                if bytes([b]).decode('utf-8') == '?':
                    # Query Status - Send back #AxxxVyyyy<CR><LF>
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        response_str = "#A" + "{:03d}".format(self._local_address) + "V0000" + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE
                elif bytes([b]).decode('utf-8') == 'A':
                    # Set Address
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_SET_ADDRESS

                elif bytes([b]).decode('utf-8') == 'P':
                    # Ping Address
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_PING

                elif bytes([b]).decode('utf-8') == 'B':
                    # Broadcast Message
                    print("MessageType: B. Broadcast")
                    self._message_type = 'B'
                    self._current_byte_counter = 2
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_LENGTH

                elif bytes([b]).decode('utf-8') == 'U':
                    # Unicast Message
                    print("MessageType: U. Unicast")
                    self._message_type = 'U'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS

                elif bytes([b]).decode('utf-8') == 'M':
                    # Unicast with Ack Message
                    print("MessageType: M. Unicast with Ack")
                    self._message_type = 'M'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS
                else:
                    # Unhandled
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_SET_ADDRESS:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._local_address = self._current_integer
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        response_str = "#A" + "{:03d}".format(self._local_address) + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_PING:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    address_to_ping = self._current_integer
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        if address_to_ping == self._local_address:
                            # Error - cannot ping self
                            response_str = "E" + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                        else:
                            # Immediate response
                            response_str = "$P" + "{:03d}".format(address_to_ping) + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)

                            # Then delay or timeout response
                            # Check the simulated nodes
                            if address_to_ping in self._nm3_simulated_nodes:
                                node = self._nm3_simulated_nodes[address_to_ping]
                                # Probability of arrival
                                if random.random() < node.probability:
                                    delay_time = 2.0 * (node.distance_m / node.speed_of_sound)
                                    time.sleep(delay_time)
                                    timeval = int(delay_time * 16000.0)
                                    response_str = "#R" + "{:03d}".format(address_to_ping) + "T" + "{:05d}".format(timeval) + "\r\n"
                                else:
                                    # Timeout = "#TO\r\n"
                                    delay_time = 4.0
                                    time.sleep(delay_time)
                                    response_str = "#T0" + "\r\n"
                            else:
                                # Timeout = "#TO\r\n"
                                delay_time = 4.0
                                time.sleep(delay_time)
                                response_str = "#T0" + "\r\n"

                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_ADDRESS:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._message_address = self._current_integer
                    print("MessageAddress: " + str(self._message_address))

                    # Now the message length
                    self._current_byte_counter = 2
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_LENGTH

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_LENGTH:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._message_length = self._current_integer
                    print("MessageLength: " + str(self._message_length))

                    # Now the Message Data
                    self._current_byte_counter = self._message_length
                    self._current_integer = 0
                    self._message_bytes = []
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_DATA

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_DATA:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next data byte
                self._message_bytes.append(b)

                if self._current_byte_counter == 0:
                    print("MessageData: " + str(self._message_bytes))

                    if self._output_stream and self._output_stream.writable():
                        # Immediate Response
                        response_str = ""
                        if self._message_type == 'B':
                            response_str = "$B" + "{:02d}".format(self._message_length)  + "\r\n"
                        else:
                            response_str = "$" + self._message_type + "{:03d}".format(self._message_address) + "{:02d}".format(self._message_length) +  "\r\n"

                        print("Sending Response: " + response_str)
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)

                        # If Ack
                        if self._message_type == 'M':
                            # Then delay or timeout response
                            # Check the simulated nodes
                            if self._message_address in self._nm3_simulated_nodes:
                                node = self._nm3_simulated_nodes[self._message_address]
                                # Probability of arrival
                                if random.random() < node.probability:
                                    delay_time = 2.0 * (node.distance_m / node.speed_of_sound)
                                    time.sleep(delay_time)
                                    timeval = int(delay_time * 16000.0)
                                    response_str = "#R" + "{:03d}".format(self._message_address) + "T" + "{:05d}".format(timeval) + "\r\n"
                                else:
                                    # Timeout = "#TO\r\n"
                                    delay_time = 4.0
                                    time.sleep(delay_time)
                                    response_str = "#T0" + "\r\n"
                            else:
                                # Timeout = "#TO\r\n"
                                delay_time = 4.0
                                time.sleep(delay_time)
                                response_str = "#T0" + "\r\n"

                            print("Sending Response: " + response_str)
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)


                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            else:
                # Unknown state
                self._simulator_state = self.SIMULATOR_STATE_IDLE



def node_parameter_parser(s):
    """Expects arguments as (address,range,probability)"""
    try:
        vals = s.split(",")
        address = int(vals[0])
        range = float(vals[1])
        probability = float(vals[2])
        return address, range, probability
    except:
        raise argparse.ArgumentTypeError("Node parameters must be address,range,probability")

def main():
    """Main Program Entry.
    Example usage python3 nm3simulator.py /dev/ttyS4 --nodes 007,1000.0 008,600.0"""
    cmdline_parser = argparse.ArgumentParser(description='NM V3 Simulator. '
                                                         'Example usage python3 nm3simulator.py /dev/ttyS4 '
                                                         '--nodes 007,1000.0,0.8 008,600.0,0.3')

    # Add Command Line Arguments
    # Serial Port
    cmdline_parser.add_argument('port', help='The serial port to connect to.')

    # Local Address
    cmdline_parser.add_argument('--address', help='The local node address on start.')

    # Simulated Nodes
    cmdline_parser.add_argument('--nodes', help="Simulated Nodes as address,range,probability triples.", dest="nodes",
                                type=node_parameter_parser, nargs='*')

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    port = cmdline_args.port

    address = cmdline_args.address

    nodes = cmdline_args.nodes


    # Serial Port is opened with a 100ms timeout for reading.
    with serial.Serial(port, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:
        nm3_simulator = Nm3Simulator(serial_port, serial_port, local_address=address)

        if nodes:
            for n in nodes:
                nm3_simulator.add_nm3_simulated_node(n[0], n[1], n[2])

        nm3_simulator.run()


if __name__ == '__main__':
    main()
