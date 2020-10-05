#! /usr/bin/env python
#
# NM V3 Network Simulator
#
# This file is part of NM3 Python Driver.
# https://github.com/bensherlock/nm3-python-driver
#
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
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
"""NM V3 Network Simulator to test code using the Nm3 driver across multiple
nodes either on the local machine or across the network/internet. Nodes can be
software on PC or embedded hardware platforms attached to Serial Ports with
Virtual NM3 Modems on the PC connected across the network. This utility is for
testing out network protocols prior to deploying hardware at sea.
Note: the channel models are simplistic - open water without interaction from
bottom layers. Basic attenuation calculations based on range, depths, bandwidth,
transmit source level, and packet length produce probabilities of successful
packet reception at the far node. Other tools are available to produce fancier
propagation modelling if that's what you're looking for. """



# Nm3 from nm3driver - revise to take io streams (not rely just on serial port)
# MessagePacket from nm3driver
# Revise Nm3Simulator from nm3simulator - Nm3VirtualModem
# Controller takes connections from Nm3Simulator over zmq
# Virtual Channels single direction between all node pairs. Uses JN Calculations.

import copy
from typing import Tuple, Union
import zmq

class Nm3SimulatorNode:
    """NM3 Simulator Node Information held by the Nm3SimulatorController."""

    def __init__(self, unique_id):
        self._unique_id = unique_id

        self._node_address = 255
        self._node_depth = 100.0
        self._node_position_xy = (0.0, 0.0)


    def __call__(self):
        return self

    @property
    def unique_id(self) -> int:
        return self._unique_id

    @unique_id.setter
    def unique_id(self, unique_id: int):
        self._unique_id = unique_id

    @property
    def node_address(self) -> int:
        return self._node_address

    @node_address.setter
    def node_address(self, node_address: int):
        self._node_address = node_address

    @property
    def node_depth(self) -> float:
        return self._node_depth

    @node_depth.setter
    def node_depth(self, node_depth: float):
        self._node_depth = node_depth

    @property
    def node_position_xy(self) -> Tuple[float, float]:
        return self._node_position_xy

    @node_position_xy.setter
    def node_position_xy(self, node_position_xy: Tuple[float, float]):
        self._node_position_xy = node_position_xy




class Nm3SimulatorController:
    """NM3 Simulator Controller. """

    def __init__(self, network_address=None, network_port=None):
        self._network_address = network_address
        self._network_port = network_port
        self._socket = None

        self._unique_id_counter = 0
        self._nm3_simulator_nodes = {} # Map unique_id to node

    def __call__(self):
        return self

    def add_nm3_simulator_node(self) -> int:
        """Add a new Nm3SimulatorNode to the controller.
        Return the unique id for this node."""
        unique_id  = self._unique_id_counter
        self._unique_id_counter = self._unique_id_counter + 1

        self._nm3_simulator_nodes[unique_id] = Nm3SimulatorNode(unique_id)

        return unique_id

    def delete_nm3_simulator_node(self, unique_id: int):
        """Delete the Nm3SimulatorNode from the controller using the id."""
        self._nm3_simulator_nodes.pop(unique_id, None)


    def get_nm3_simulator_node(self, unique_id: int) \
            -> Union[Nm3SimulatorNode, None]:
        """Get a copy of the Nm3SimulatorNode from the controller by id.
        Returns a copy of the Nm3SimulatorNode or None."""
        if unique_id in self._nm3_simulator_nodes:
            return copy.deepcopy(self._nm3_simulator_nodes[unique_id])
        else:
            return None

    def update_nm3_simulator_node(self, nm3_simulator_node: Nm3SimulatorNode):
        unique_id = nm3_simulator_node.unique_id
        if unique_id in self._nm3_simulator_nodes:
            self._nm3_simulator_nodes[unique_id] = nm3_simulator_node




    def start(self):
        """Start the simulation. Bind to the address and port ready for
        virtual nodes."""
        if not self._network_address or not self._network_port:
            raise TypeError("Network Address/Port not set. Address("
                                    + self._network_address
                                    + ") Port( "
                                    + str(self._network_port) + ")" )
        if self._socket:
            # Already created?
            pass
        else:
            # https://stackoverflow.com/questions/38978804/zeromq-master-slave-two-way-communication
            # https://stackoverflow.com/questions/34242316/pyzmq-recv-json-cant-decode-message-sent-by-send-json
            context = zmq.Context()
            self._socket = context.socket(zmq.ROUTER)
            self._socket.bind("tcp://" + self._network_address + ":"
                              + str(self._network_port))

        while True:
            # Poll the socket
            # Router socket so first frame of multi part message is an identifier for the client.







class Nm3VirtualModem:
    """NM3 Virtual Modem Class.
    Connects over network to Nm3SimulatorController.
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



    def __init__(self, input_stream, output_stream,
                 network_address=None, network_port=None, local_address=255):
        """input_stream and output_stream implement the Bytes IO interface.
        Namely: readable()->bool, writeable()->bool, read(bytes) and write(bytes)."""
        self._input_stream = input_stream
        self._output_stream = output_stream
        self._simulator_state = self.SIMULATOR_STATE_IDLE

        self._network_address = network_address
        self._network_port = network_port
        self._socket = None

        self._local_address = local_address

        # Parser variables
        self._current_byte_counter = 0
        self._current_integer = 0

        # Sending message
        self._message_type = None
        self._message_address = None
        self._message_length = None
        self._message_bytes = None


    def __call__(self):
        return self

    def run(self):
        """Run the simulator. Never returns."""
        # Connect to the controller
        if not self._network_address or not self._network_port:
            raise TypeError("Network Address/Port not set. Address("
                                    + self._network_address
                                    + ") Port( "
                                    + str(self._network_port) + ")" )
        if self._socket:
            # Already created?
            pass
        else:
            context = zmq.Context()
            self._socket = context.socket(zmq.DEALER)
            self._socket.connect("tcp://" + self._network_address + ":"
                              + str(self._network_port))

        while True:
            if self._input_stream.readable():
                some_bytes = self._input_stream.read()  # Read

                if some_bytes:
                    #print("Bytes received len(" + str(len(some_bytes)) + ")")
                    self.process_bytes(some_bytes)

            # Poll the socket for incoming "acoustic" messages


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

                            # Send to the Controller
                            if self._socket:
                                acoustic_msg = {"MessageType": "P",
                                                "Address": address_to_ping}
                                self._socket.send_json(acoustic_msg)

                                # Have to wait for timeout or an Ack response via the server

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






def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator. '
                    'Example usage python3 nm3networksimulator.py')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.')

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')


    # Serial Port
    #cmdline_parser.add_argument('port', help='The serial port to connect to.')

    # Local Address
    #cmdline_parser.add_argument('--address', help='The local node address on start.')

    # Simulated Nodes
    #cmdline_parser.add_argument('--nodes', help="Simulated Nodes as address,range,probability triples.", dest="nodes",
    #                            type=node_parameter_parser, nargs='*')

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address

    #port = cmdline_args.port

    #address = cmdline_args.address

    #nodes = cmdline_args.nodes

    #
    # Controller Mode
    #

    nm3_simulator_controller = Nm3SimulatorController(
        network_address=network_address, network_port=network_port)

    nm3_simulator_controller.start()

    #
    # Virtual NM3 Modem Mode (With stdin/stdout as interface)
    #

    #
    # Headless Virtual NM3 Modems Mode - A list of headless modems.
    #


    # Serial Port is opened with a 100ms timeout for reading.
    #with serial.Serial(port, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:
    #    nm3_simulator = Nm3Simulator(serial_port, serial_port, local_address=address)
    #    if nodes:
    #        for n in nodes:
    #            nm3_simulator.add_nm3_simulated_node(n[0], n[1], n[2])
    #    nm3_simulator.run()


if __name__ == '__main__':
    main()