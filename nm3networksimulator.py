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

import argparse
import copy
import json
import math
import serial
import time
from typing import Tuple, Union
import sys
import tornado
import zmq
from zmq.eventloop.zmqstream import ZMQStream
#from zmq.eventloop.ioloop import IOLoop
from tornado.ioloop import IOLoop

def _debug_print(*args, **kwargs):
    """File local debug printing"""
    #print(*args, **kwargs)
    pass

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
    def unique_id(self):
        return self._unique_id


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


class TimePacket:
    """Time Packet Class"""

    def __init__(self):
        # There...
        self._client_transmit_time = 0.0
        self._server_arrival_time = 0.0
        # ...and back again
        self._server_transmit_time = 0.0
        self._client_arrival_time = 0.0

    def __call__(self):
        return self

    @property
    def client_transmit_time(self):
        return self._client_transmit_time
    
    @client_transmit_time.setter
    def client_transmit_time(self, client_transmit_time):
        self._client_transmit_time = client_transmit_time

    @property
    def server_arrival_time(self):
        return self._server_arrival_time
    
    @server_arrival_time.setter
    def server_arrival_time(self, server_arrival_time):
        self._server_arrival_time = server_arrival_time

    @property
    def server_transmit_time(self):
        return self._server_transmit_time
    
    @server_transmit_time.setter
    def server_transmit_time(self, server_transmit_time):
        self._server_transmit_time = server_transmit_time

    @property
    def client_arrival_time(self):
        return self._client_arrival_time
    
    @client_arrival_time.setter
    def client_arrival_time(self, client_arrival_time):
        self._client_arrival_time = client_arrival_time


    @property
    def offset(self):
        time_offset = (self._server_arrival_time - self._client_transmit_time) \
                      + (self._server_transmit_time - self._client_arrival_time) / 2.0
        return time_offset
    
    @property
    def delay(self):
        round_trip_delay = (self._client_arrival_time - self._client_transmit_time) \
                           - (self._server_transmit_time - self._server_arrival_time)
        return round_trip_delay


    def calculate_offset(self):
        """Calculate the offset."""
        # Clock synchronization algorithm from
        # https://en.wikipedia.org/wiki/Network_Time_Protocol
        time_offset = (self._server_arrival_time - self._client_transmit_time) \
                      + (self._server_transmit_time - self._client_arrival_time) / 2.0
        round_trip_delay = (self._client_arrival_time - self._client_transmit_time) \
                           - (self._server_transmit_time - self._server_arrival_time)
        return time_offset

    def json(self):
        """Returns a json dictionary representation."""
        jason = {"ClientTransmitTime": self._client_transmit_time,
                 "ServerArrivalTime": self._server_arrival_time,
                 "ServerTransmitTime": self._server_transmit_time,
                 "ClientArrivalTime": self._client_arrival_time }
        return jason

    @staticmethod
    def from_json(jason): # -> Union[TimePacket, None]:
        time_packet = TimePacket()
        time_packet.client_transmit_time = jason["ClientTransmitTime"]
        time_packet.server_arrival_time = jason["ServerArrivalTime"]
        time_packet.server_transmit_time = jason["ServerTransmitTime"]
        time_packet.client_arrival_time = jason["ClientArrivalTime"]
        return time_packet

    def to_string(self):
        return "TimePacket:" \
               + "\n  client_transmit_time = " + str(self.client_transmit_time) \
               + "\n  server_arrival_time  = " + str(self.server_arrival_time) \
               + "\n  server_transmit_time = " + str(self.server_transmit_time) \
               + "\n  client_arrival_time  = " + str(self.client_arrival_time) \
               + "\n  offset               = " + str(self.offset) \
               + "\n  delay                = " + str(self.delay)




class NodePacket:
    """Node Packet class."""


    def __init__(self, position_xy=(0.0,0.0), depth=10.0):
        self._position_xy = position_xy
        self._depth = depth
        
    @property
    def position_xy(self):
        return self._position_xy
    
    @position_xy.setter
    def position_xy(self, position_xy):
        self._position_xy = position_xy
        
    @property
    def depth(self):
        return self._depth
    
    @depth.setter
    def depth(self, depth):
        self._depth = depth

    def json(self):
        """Returns a json dictionary representation."""
        jason = {"PositionXY": { "x": self._position_xy[0], "y": self._position_xy[1] },
                 "Depth": self._depth}
        return jason

    @staticmethod
    def from_json(jason): # -> Union[NodePacket, None]:
        node_packet = NodePacket(position_xy=(jason["PositionXY"]["x"], jason["PositionXY"]["y"]),
                                 depth=jason["Depth"])

        return node_packet


class AcousticPacket:
    """Acoustic Packet class.
    PACKET MESSAGE HEADER STRUCTURE
    LFM (UP / DN)   Payload Length  CMD     Address (Src / Dest)    Packet Type
    ===============================================================================================
    UP              0               0       Dest                    Ping Request
    DN                              1       Src                     Ack / Ping Response
    UP                              2       Dest                    Test Message Request
    UP                              3       Dest                    VBatt Request
    UP              1-63            0       Dest                    Unicast Message
    UP                              1       Src                     Broadcast Message
    UP                              2       Dest                    Unicast Message With Ack Request
    UP                              3       Dest                    Echo Message Request
    """

    FRAMESYNCH_UP, FRAMESYNCH_DN = range(2)
    CMD_PING_REQ, CMD_PING_REP, CMD_TEST_REQ, CMD_VBATT_REQ = range(4)
    CMD_UNICAST_MSG, CMD_BROADCAST_MSG, CMD_UNICAST_ACK_MSG, CMD_ECHO_MSG = range(4)

    def __init__(self, frame_synch=FRAMESYNCH_UP, address=255, command=CMD_PING_REQ, payload_length=0, payload_bytes=None, hamr_timestamp=0.0):
        #AcousticPacket: { FrameSynch: Up/Dn, Address: 0-255, Command: 0-3, PayloadLength: 0-64, PayloadBytes: bytes(0-64) }
        self._frame_synch = frame_synch
        self._address = address
        self._command = command
        self._payload_length = payload_length
        self._payload_bytes = payload_bytes
        self._hamr_timestamp = hamr_timestamp

        
    @property
    def frame_synch(self):
        return self._frame_synch
    
    @frame_synch.setter
    def frame_synch(self, frame_synch):
        self._frame_synch = frame_synch
        
    @property
    def address(self):
        return self._address
    
    @address.setter
    def address(self, address):
        self._address = address
        
    @property
    def command(self):
        return self._command
    
    @command.setter
    def command(self, command):
        self._command = command
    
    @property
    def payload_length(self):
        return self._payload_length
    
    @payload_length.setter
    def payload_length(self, payload_length):
        self._payload_length = payload_length
        
    @property
    def payload_bytes(self):
        return self._payload_bytes
    
    @payload_bytes.setter
    def payload_bytes(self, payload_bytes):
        self._payload_bytes = payload_bytes

    @property
    def hamr_timestamp(self):
        return self._hamr_timestamp
    
    @hamr_timestamp.setter
    def hamr_timestamp(self, hamr_timestamp):
        self._hamr_timestamp = hamr_timestamp


    def json(self):
        """Returns a json dictionary representation."""
        jason = {"FrameSynch": self._frame_synch,
                 "Address": self._address,
                 "Command": self._command,
                 "PayloadLength": self._payload_length,
                 "PayloadBytes": self._payload_bytes,
                 "HamrTimestamp": self._hamr_timestamp}

        return jason

    @staticmethod
    def from_json(jason): # -> Union[AcousticPacket, None]:
        acoustic_packet = AcousticPacket(frame_synch=jason["FrameSynch"],
                                         address=jason["Address"],
                                         command=jason["Command"],
                                         payload_length=jason["PayloadLength"],
                                         payload_bytes=jason["PayloadBytes"],
                                         hamr_timestamp=jason["HamrTimestamp"])
        return acoustic_packet



class Nm3SimulatorController:
    """NM3 Simulator Controller. """

    def __init__(self, network_address=None, network_port=None):
        self._network_address = network_address
        self._network_port = network_port
        self._socket = None

        # NTP Offset to synchronise timestamps
        self._ntp_offset = 0.0

        # Polling version
        #self._socket_poller = None

        # Async version
        self._socket_stream = None
        #self._socket_loop = None

        self._nm3_simulator_nodes = {} # Map unique_id to node

        self._scheduled_network_packets = []

        self._startup_time = 1602256464 #time.time()

    def __call__(self):
        return self


    def get_hamr_time(self, local_time=None):
        """Get Homogenous Acoustic Medium Relative time from either local_time or time.time()."""
        if local_time:
            hamr_time = self._ntp_offset + local_time
            return hamr_time
        else:
            hamr_time = self._ntp_offset + time.time()
            return hamr_time

    def get_local_time(self, hamr_time=None):
        """Get local time from Homogenous Acoustic Medium Relative time or time.time()."""
        if hamr_time:
            local_time = hamr_time - self._ntp_offset
            return local_time
        else:
            return time.time()


    def add_nm3_simulator_node(self, unique_id):
        """Add a new Nm3SimulatorNode to the controller.
        Return the unique id for this node."""
        self._nm3_simulator_nodes[unique_id] = Nm3SimulatorNode(unique_id)

        return unique_id

    def has_nm3_simulator_node(self, unique_id):
        """Check if Nm3SimulatorNode exists for this unique_id."""
        return unique_id in self._nm3_simulator_nodes

    def delete_nm3_simulator_node(self, unique_id):
        """Delete the Nm3SimulatorNode from the controller using the id."""
        self._nm3_simulator_nodes.pop(unique_id, None)


    def get_nm3_simulator_node(self, unique_id) \
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

    def calculate_propagation(self, nm3_simulator_source_node: Nm3SimulatorNode,
                              nm3_simulator_destination_node: Nm3SimulatorNode):
        """Calculate the propagation delay of acoustic packet from source to destination node.
        Returns propagation_delay and probability. (probability will always be 1.0)."""
        x0 = nm3_simulator_source_node.node_position_xy[0]
        y0 = nm3_simulator_source_node.node_position_xy[1]
        z0  = nm3_simulator_source_node.node_depth

        x1 = nm3_simulator_destination_node.node_position_xy[0]
        y1 = nm3_simulator_destination_node.node_position_xy[1]
        z1 = nm3_simulator_destination_node.node_depth

        # Please note: This is a **very** simplistic model to just get us started.
        # Assuming no losses. And isovelocity. And no obstructions. And no multipath. And no noise.
        # The joy of simulation.

        straight_line_range = math.sqrt(((x1-x0)*(x1-x0)) +  ((y1-y0)*(y1-y0)) + ((z1-z0)*(z1-z0)))
        speed_of_sound = 1500.0
        propagation_delay = straight_line_range / speed_of_sound

        return propagation_delay, 1.0



    def schedule_network_packet(self, transmit_time, unique_id, network_packet_json_string):
        """Schedule a network packet transmission."""
        self._scheduled_network_packets.append( (transmit_time, unique_id, network_packet_json_string) )
        self._scheduled_network_packets.sort(key=lambda tup: tup[0]) # sort by time


    def next_scheduled_network_packet(self, current_time):
        """Get the next scheduled network packet to be transmitted at the current time."""
        if self._scheduled_network_packets and self._scheduled_network_packets[0][0] <= current_time:
            scheduled_network_packet = self._scheduled_network_packets.pop(0)
            unique_id = scheduled_network_packet[1]
            network_packet_json_string = scheduled_network_packet[2]
            return unique_id, network_packet_json_string

        return None, None


    def on_recv(self, msg):
        """Callback handler for on_recv."""
        #unique_id, network_message_json_bytes = self._socket.recv_multipart(zmq.DONTWAIT)  # blocking
        unique_id = msg[0]
        network_message_json_bytes = msg[1]
        zmq_timestamp = float(msg[2].decode('utf-8'))

        local_received_time = time.time()
        _debug_print("NetworkPacket transmitted at: " + str(zmq_timestamp) + " received at: " + str(local_received_time - self._startup_time))
        network_message_json_str = network_message_json_bytes.decode('utf-8')
        network_message_jason = json.loads(network_message_json_str)

        # _debug_print("Network Packet received from: " + str(unique_id) + " -- " + network_message_json_str)


        if "TimePacket" in network_message_jason:
            # Quick Turnaround
            time_packet = TimePacket.from_json(network_message_jason["TimePacket"])
            time_packet.server_arrival_time = local_received_time
            #time_packet.client_transmit_time = zmq_timestamp

            time_packet.server_transmit_time = time.time()
            new_network_message_jason = {"TimePacket": time_packet.json()}
            new_network_message_json_str = json.dumps(new_network_message_jason)

            self._socket.send_multipart([unique_id, new_network_message_json_str.encode('utf-8'), str(time.time()).encode('utf-8')])


        if not self.has_nm3_simulator_node(unique_id):
            self.add_nm3_simulator_node(unique_id)

        if "NodePacket" in network_message_jason:
            nm3_simulator_node = self.get_nm3_simulator_node(unique_id)
            if nm3_simulator_node:
                node_packet = NodePacket.from_json(network_message_jason["NodePacket"])
                nm3_simulator_node.node_position_xy = node_packet.position_xy
                nm3_simulator_node.node_depth = node_packet.depth
                self.update_nm3_simulator_node(nm3_simulator_node)

        if "AcousticPacket" in network_message_jason:
            # Send to all nodes, except this one
            # For now this is instant: no propagation, or probability, or filtering.
            acoustic_packet = AcousticPacket.from_json(network_message_jason["AcousticPacket"])
            # Process Channels and Scheduling of acoustic_packet
            # This would need updating if the receiving node changes its position - work this out later.

            # Get source position Information
            nm3_simulator_source_node = self.get_nm3_simulator_node(unique_id)

            hamr_received_time = acoustic_packet.hamr_timestamp

            for socket_id in self._nm3_simulator_nodes:
                # Don't send to itself
                if socket_id != unique_id:
                    # Get destination position Information
                    nm3_simulator_destination_node = self.get_nm3_simulator_node(socket_id)
                    propagation_delay, probability = self.calculate_propagation(
                        nm3_simulator_source_node, nm3_simulator_destination_node)

                    # Calculate a transmit_time
                    hamr_transmit_time = hamr_received_time + propagation_delay
                    local_transmit_time = self.get_local_time(hamr_transmit_time)

                    acoustic_packet.hamr_timestamp = hamr_transmit_time
                    new_network_message_jason = {"AcousticPacket": acoustic_packet.json()}
                    new_network_message_json_str = json.dumps(new_network_message_jason)
                    self.schedule_network_packet(local_transmit_time, socket_id,
                                                 new_network_message_json_str)
                    # IOLoop set the callback
                    IOLoop.instance().call_at(local_transmit_time, self.check_for_packets_to_send)


    def check_for_packets_to_send(self):
        """Check for packets to send."""
        socket_id, network_packet_json_str = self.next_scheduled_network_packet(time.time())
        while socket_id:
            # _debug_print("Sending scheduled network packet: " + str(socket_id) + " - " + new_json_str)
            self._socket.send_multipart([socket_id, network_packet_json_str.encode('utf-8'), str(time.time()).encode('utf-8')])
            sent_time = time.time()
            _debug_print("NetworkPacket sent at: " + str(sent_time - self._startup_time))
            # Get next scheduled network Packet
            socket_id, network_packet = self.next_scheduled_network_packet(time.time())

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
            socket_string = "tcp://" + self._network_address + ":"+ str(self._network_port)
            print("Binding to: " + socket_string)
            self._socket.bind(socket_string)

            # Polling version
            #self._socket_poller = zmq.Poller()
            #self._socket_poller.register(self._socket, zmq.POLLIN)

            # Async version
            #self._socket_loop = IOLoop()
            self._socket_stream = ZMQStream(self._socket)
            self._socket_stream.on_recv(self.on_recv)

            #self._socket_loop.start()
            IOLoop.instance().start() # Stays here



        while True:
            # Poll the socket
            # Router socket so first frame of multi part message is an identifier for the client.
            # Incoming Network Messages
            # NetworkMessage {
            # 1. AcousticPacket: { FrameSynch: Up/Dn, Address: 0-255, Command: 0-3, PayloadLength: 0-64, PayloadBytes: bytes(0-64) }
            # 2. NodePacket: { PositionXY: {x: float, y: float}, Depth: float }
            # }
            # Poll the socket for incoming messages
            # _debug_print("Checking socket poller")
            #sockets = dict(self._socket_poller.poll(0))
            #if self._socket in sockets:
                #unique_id, network_message_json_bytes = self._socket.recv_multipart(zmq.DONTWAIT) #  blocking
            #    msg = self._socket.recv_multipart(zmq.DONTWAIT)  # blocking
            #    self.on_recv(msg)

            # Get next scheduled network Packet
            self.check_for_packets_to_send()








class Nm3VirtualModem:
    """NM3 Virtual Modem Class.
    Connects over network to Nm3SimulatorController.
    Currently supports: Query ($?), Set Local Address ($Axxx), Ping ($Pxxx),
    Broadcast Message ($Byydd...dd), Unicast Message ($Uxxxyydd..dd),
    Unicast with Ack Message ($Mxxxyydd..dd)."""

    SIMULATOR_STATE_IDLE, SIMULATOR_STATE_COMMAND, \
    SIMULATOR_STATE_SET_ADDRESS, SIMULATOR_STATE_PING, SIMULATOR_STATE_TEST, \
    SIMULATOR_STATE_MESSAGE_ADDRESS, SIMULATOR_STATE_MESSAGE_LENGTH, \
    SIMULATOR_STATE_MESSAGE_DATA = range(8)

    SIMULATOR_STATE_NAMES = {
        SIMULATOR_STATE_IDLE: 'Idle',
        SIMULATOR_STATE_COMMAND: 'Command',
        SIMULATOR_STATE_SET_ADDRESS: 'SetAddress',
        SIMULATOR_STATE_PING: 'Ping',
        SIMULATOR_STATE_TEST: 'Test',
        SIMULATOR_STATE_MESSAGE_ADDRESS: 'MessageAddress',
        SIMULATOR_STATE_MESSAGE_LENGTH: 'MessageLength',
        SIMULATOR_STATE_MESSAGE_DATA: 'MessageData',
    }

    SIMULATOR_STATES = (SIMULATOR_STATE_IDLE, SIMULATOR_STATE_COMMAND,
                        SIMULATOR_STATE_SET_ADDRESS, SIMULATOR_STATE_PING, SIMULATOR_STATE_TEST,
                        SIMULATOR_STATE_MESSAGE_ADDRESS, SIMULATOR_STATE_MESSAGE_LENGTH,
                        SIMULATOR_STATE_MESSAGE_DATA)

    ACOUSTIC_STATE_IDLE, ACOUSTIC_STATE_WAIT_ACK = range(2)
    ACOUSTIC_STATE_NAMES = {
        ACOUSTIC_STATE_IDLE: 'Idle',
        ACOUSTIC_STATE_WAIT_ACK: 'WaitAck',
    }

    ACOUSTIC_STATES = (ACOUSTIC_STATE_IDLE, ACOUSTIC_STATE_WAIT_ACK)


    def __init__(self, input_stream, output_stream,
                 network_address=None, network_port=None, local_address: int =255, position_xy=(0.0,0.0), depth=10.0):
        """input_stream and output_stream implement the Bytes IO interface.
        Namely: readable()->bool, writeable()->bool, read(bytes) and write(bytes)."""
        self._input_stream = input_stream
        self._output_stream = output_stream
        self._simulator_state = self.SIMULATOR_STATE_IDLE
        self._acoustic_state = self.ACOUSTIC_STATE_IDLE
        self._acoustic_ack_wait_address = None
        self._acoustic_ack_wait_time = None

        self._acoustic_ack_fixed_offset_time = 0.040 # 40ms

        self._network_address = network_address
        self._network_port = network_port
        self._socket = None
        self._socket_poller = None

        # Offset to synchronise times
        self._hamr_time_offset = 0.0

        self._local_address = local_address

        # Parser variables
        self._current_byte_counter = 0
        self._current_integer = 0

        # Sending message
        self._message_type = None
        self._message_address = None
        self._message_length = None
        self._message_bytes = None

        # Positional information
        self._position_xy = position_xy
        self._depth = depth
        self._position_information_updated = True

        self._startup_time = 1602256464 #time.time()
        self._local_received_time = None
        self._local_sent_time = None
        self._last_packet_received_time = None
        self._last_packet_sent_time = None

    def __call__(self):
        return self


    @property
    def position_xy(self):
        return self._position_xy
    
    @position_xy.setter
    def position_xy(self, position_xy):
        self._position_xy = position_xy
        self._position_information_updated = True
        
    @property
    def depth(self):
        return self._depth
    
    @depth.setter
    def depth(self, depth):
        self._depth = depth
        self._position_information_updated = True


    def get_hamr_time(self, local_time=None):
        """Get Homogenous Acoustic Medium Relative time from either local_time or time.time()."""
        if local_time:
            hamr_time = self._hamr_time_offset + local_time
            return hamr_time
        else:
            hamr_time = self._hamr_time_offset + time.time()
            return hamr_time

    def get_local_time(self, hamr_time=None):
        """Get local time from Homogenous Acoustic Medium Relative time or time.time()."""
        if hamr_time:
            local_time = hamr_time - self._hamr_time_offset
            return local_time
        else:
            return time.time()

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
            self._socket_poller = zmq.Poller()
            self._socket_poller.register(self._socket, zmq.POLLIN)

            self.send_time_packet()

        while True:
            if self._position_information_updated:
                # Send positional information update
                self._position_information_updated = False
                node_packet = NodePacket(position_xy=self._position_xy, depth=self._depth)
                self.send_node_packet(node_packet)

            if self._input_stream and self._input_stream.readable():
                #_debug_print("Checking input_stream")
                some_bytes = self._input_stream.read()  # Read

                if some_bytes:
                    #_debug_print("some_bytes=" + str(some_bytes))
                    self.process_bytes(some_bytes)

            # Poll the socket for incoming "acoustic" messages
            #_debug_print("Checking socket poller")
            sockets = dict(self._socket_poller.poll(1))
            if self._socket in sockets:
                msg = self._socket.recv_multipart(zmq.DONTWAIT)
                network_message_json_bytes = msg[0]
                zmq_timestamp = float(msg[1].decode('utf-8'))

                self._local_received_time = time.time()
                _debug_print("NetworkPacket transmitted at: " + str(zmq_timestamp) + " received at: " + str(self._local_received_time-self._startup_time))
                network_message_json_str = network_message_json_bytes.decode('utf-8')
                network_message_jason = json.loads(network_message_json_str)

               # _debug_print("Network Packet received: " + network_message_json_str)

                if "TimePacket" in network_message_jason:
                    # Process the TimePacket
                    time_packet = TimePacket.from_json(network_message_jason["TimePacket"])
                    time_packet.client_arrival_time = self._local_received_time
                    #time_packet.server_transmit_time = zmq_timestamp
                    self._hamr_time_offset = time_packet.calculate_offset()

                    print(time_packet.to_string())

                    print("TimePacket offset: " + str(self._hamr_time_offset))

                if "AcousticPacket" in network_message_jason:
                    # Process the AcousticPacket
                    acoustic_packet = AcousticPacket.from_json(network_message_jason["AcousticPacket"])
                    self.process_acoustic_packet(acoustic_packet)

            # Check for timeout if awaiting an Ack
            #_debug_print("Checking ack status")
            if self._acoustic_state == self.ACOUSTIC_STATE_WAIT_ACK:
                delay_time = time.time() - self._acoustic_ack_wait_time
                if delay_time > 4.0:
                    # Cancel the wait for ack and indicate timeout
                    self._acoustic_state = self.ACOUSTIC_STATE_IDLE

                    if self._output_stream and self._output_stream.writable():
                        response_str = "#T0" + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()



    def send_time_packet(self):
        """Create and send a TimePacket to determine offset time."""
        time_packet = TimePacket()
        time_packet.client_transmit_time = time.time()
        jason = {"TimePacket": time_packet.json()}
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])

        return


    def send_acoustic_packet(self, acoustic_packet: AcousticPacket):
        """Send an AcousticPacket.
        Returns time sent"""
        jason = { "AcousticPacket": acoustic_packet.json() }
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])

        self._local_sent_time = time.time()
        _debug_print("NetworkPacket sent at: " + str(self._local_sent_time-self._startup_time))
        if self._local_received_time:
            _debug_print("-Turnaround: " + str(self._local_sent_time-self._local_received_time))

        return self._local_sent_time

    def send_node_packet(self, node_packet: NodePacket):
        """Send a NodePacket.
        Returns time sent"""
        jason = { "NodePacket": node_packet.json() }
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])

        self._local_sent_time = time.time()
        _debug_print("NetworkPacket sent at: " + str(self._local_sent_time-self._startup_time))

        return self._local_sent_time

    def process_acoustic_packet(self, acoustic_packet: AcousticPacket):
        """Process an AcousticPacket."""
        # State
        if self._acoustic_state == self.ACOUSTIC_STATE_WAIT_ACK:
            # Check for DownChirp on the acoustic_packet - ignore if not downchip.
            if acoustic_packet.frame_synch == AcousticPacket.FRAMESYNCH_DN:
                # Ack Received
                if acoustic_packet.address == self._acoustic_ack_wait_address:
                    # This is the Ack we are looking for.
                    if self._output_stream and self._output_stream.writable():
                        local_received_time = self.get_local_time(acoustic_packet.hamr_timestamp)
                        delay_time = local_received_time - self._acoustic_ack_wait_time
                        _debug_print("Ack delay_time: " + str(delay_time))
                        timeval = int(delay_time * 16000.0)
                        response_str = "#R" + "{:03d}".format(
                            acoustic_packet.address) + "T" + "{:05d}".format(timeval) + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()

                    self._acoustic_state = self.ACOUSTIC_STATE_IDLE
        else:
            # Check for UpChirp on the acoustic_packet - ignore if not upchirp.
            if acoustic_packet.frame_synch == AcousticPacket.FRAMESYNCH_UP:
                if acoustic_packet.payload_length == 0 and acoustic_packet.address == self._local_address:
                    # Control commands
                    # CMD_PING_REQ, CMD_PING_REP, CMD_TEST_REQ, CMD_VBATT_REQ = range(4)
                    if acoustic_packet.command == AcousticPacket.CMD_PING_REQ:
                        # Ping request so send a reply
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_DN,
                            address=self._local_address,
                            command=AcousticPacket.CMD_PING_REP,
                            hamr_timestamp=acoustic_packet.hamr_timestamp)
                        self.send_acoustic_packet(acoustic_packet_to_send)

                    elif acoustic_packet.command == AcousticPacket.CMD_TEST_REQ:
                        # Test message acoustic message as a broadcast
                        payload_str = "This is a test message from a Virtual NM3"
                        payload_bytes = list(payload_str.encode('utf-8'))

                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_UP,
                            address=self._local_address,
                            command=AcousticPacket.CMD_BROADCAST_MSG,
                            payload_length=len(payload_bytes),
                            payload_bytes=payload_bytes,
                            hamr_timestamp=self.get_hamr_time())
                        self.send_acoustic_packet(acoustic_packet_to_send)

                    # Other commands not supported at the moment.

                else:
                    # Message Packets
                    # CMD_UNICAST_MSG, CMD_BROADCAST_MSG, CMD_UNICAST_ACK_MSG, CMD_ECHO_MSG = range(4)
                    if acoustic_packet.command == AcousticPacket.CMD_UNICAST_MSG and acoustic_packet.address == self._local_address:
                        # Construct the bytes to be sent to the output_stream
                        # "#U..."
                        if self._output_stream and self._output_stream.writable():
                            response_bytes = b"#U" \
                                             + "{:02d}".format(
                                acoustic_packet.payload_length).encode('utf-8') \
                                             + bytes(acoustic_packet.payload_bytes) + b"\r\n"
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                    elif acoustic_packet.command == AcousticPacket.CMD_BROADCAST_MSG:
                        # Construct the bytes to be sent to the output_stream
                        # "#B..."
                        if self._output_stream and self._output_stream.writable():
                            response_bytes = b"#B" \
                                             + "{:03d}".format(
                                acoustic_packet.address).encode('utf-8') \
                                             + "{:02d}".format(
                                acoustic_packet.payload_length).encode('utf-8') \
                                             + bytes(acoustic_packet.payload_bytes) + b"\r\n"
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                    elif acoustic_packet.command == AcousticPacket.CMD_UNICAST_ACK_MSG and acoustic_packet.address == self._local_address:
                        # Ack request so send a reply
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_DN,
                            address=self._local_address,
                            command=AcousticPacket.CMD_PING_REP,
                            hamr_timestamp=acoustic_packet.hamr_timestamp)
                        self.send_acoustic_packet(acoustic_packet_to_send)

                        # Construct the bytes to be sent to the output_stream
                        # "#U..."
                        if self._output_stream and self._output_stream.writable():
                            response_bytes = b"#U" \
                                             + "{:02d}".format(
                                acoustic_packet.payload_length).encode('utf-8') \
                                             + bytes(acoustic_packet.payload_bytes) + b"\r\n"
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                    elif acoustic_packet.command == AcousticPacket.CMD_ECHO_MSG and acoustic_packet.address == self._local_address:
                        # Echo the acoustic message as a broadcast
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_UP,
                            address=self._local_address,
                            command=AcousticPacket.CMD_BROADCAST_MSG,
                            payload_length=acoustic_packet.payload_length,
                            payload_bytes=acoustic_packet.payload_bytes,
                            hamr_timestamp=self.get_hamr_time())
                        self.send_acoustic_packet(acoustic_packet_to_send)



    def process_bytes(self, some_bytes: bytes):
        """Process bytes in the state machine and act accordingly."""
        for b in some_bytes:
            if self._simulator_state == self.SIMULATOR_STATE_IDLE:
                if bytes([b]).decode('utf-8') == '$':
                    self._simulator_state = self.SIMULATOR_STATE_COMMAND

                    # Cancel any ongoing Ack wait state
                    self._acoustic_state = self.ACOUSTIC_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_COMMAND:
                if bytes([b]).decode('utf-8') == '?':
                    # Query Status - Send back #AxxxVyyyy<CR><LF>
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        response_str = "#A" + "{:03d}".format(self._local_address) + "V0000" + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()

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

                elif bytes([b]).decode('utf-8') == 'T':
                    # Test Address
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_TEST

                elif bytes([b]).decode('utf-8') == 'B':
                    # Broadcast Message
                    #_debug_print("MessageType: B. Broadcast")
                    self._message_type = 'B'
                    self._current_byte_counter = 2
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_LENGTH

                elif bytes([b]).decode('utf-8') == 'U':
                    # Unicast Message
                    #_debug_print("MessageType: U. Unicast")
                    self._message_type = 'U'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS

                elif bytes([b]).decode('utf-8') == 'M':
                    # Unicast with Ack Message
                    #_debug_print("MessageType: M. Unicast with Ack")
                    self._message_type = 'M'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS

                elif bytes([b]).decode('utf-8') == 'E':
                    # Echo Request Message
                    #_debug_print("MessageType: E. Echo")
                    self._message_type = 'E'
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
                        self._output_stream.flush()

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
                            self._output_stream.flush()
                        else:
                            # Immediate response
                            response_str = "$P" + "{:03d}".format(address_to_ping) + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                            # Send to the Controller
                            self._acoustic_ack_wait_time = self.get_local_time()

                            acoustic_packet_to_send = AcousticPacket(
                                frame_synch=AcousticPacket.FRAMESYNCH_UP,
                                address=address_to_ping,
                                command=AcousticPacket.CMD_PING_REQ,
                                hamr_timestamp=self.get_hamr_time(self._acoustic_ack_wait_time))
                            self.send_acoustic_packet(acoustic_packet_to_send)
                            self._acoustic_ack_wait_address = address_to_ping
                            self._acoustic_state = self.ACOUSTIC_STATE_WAIT_ACK

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_TEST:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    address_to_test = self._current_integer
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        if address_to_test == self._local_address:
                            # Error - cannot test self
                            response_str = "E" + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()
                        else:
                            # Immediate response
                            response_str = "$T" + "{:03d}".format(address_to_test) + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                            # Send to the Controller
                            self._acoustic_ack_wait_time = self.get_local_time()

                            acoustic_packet_to_send = AcousticPacket(
                                frame_synch=AcousticPacket.FRAMESYNCH_UP,
                                address=address_to_test,
                                command=AcousticPacket.CMD_TEST_REQ,
                                hamr_timestamp=self.get_hamr_time(self._acoustic_ack_wait_time))
                            self.send_acoustic_packet(acoustic_packet_to_send)

                            self._acoustic_state = self.ACOUSTIC_STATE_IDLE

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_ADDRESS:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._message_address = self._current_integer
                    #_debug_print("MessageAddress: " + str(self._message_address))

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
                    #_debug_print("MessageLength: " + str(self._message_length))

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
                    #_debug_print("MessageData: " + str(self._message_bytes))

                    if self._output_stream and self._output_stream.writable():
                        # Immediate Response
                        response_str = ""
                        if self._message_type == 'B':
                            response_str = "$B" + "{:02d}".format(self._message_length)  + "\r\n"
                            self._message_address = self._local_address
                        else:
                            response_str = "$" + self._message_type + "{:03d}".format(self._message_address) + "{:02d}".format(self._message_length) +  "\r\n"

                        #_debug_print("Sending Response: " + response_str)
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()

                        # Send to the Controller
                        acoustic_packet_command_to_send =  AcousticPacket.CMD_BROADCAST_MSG
                        if self._message_type == 'U':
                            acoustic_packet_command_to_send = AcousticPacket.CMD_UNICAST_MSG
                        elif self._message_type == 'M':
                            acoustic_packet_command_to_send = AcousticPacket.CMD_UNICAST_ACK_MSG
                        elif self._message_type == 'E':
                            acoustic_packet_command_to_send = AcousticPacket.CMD_ECHO_MSG

                        self._acoustic_ack_wait_time = self.get_local_time()
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_UP,
                            address=self._message_address,
                            command=acoustic_packet_command_to_send,
                            payload_length=len(self._message_bytes),
                            payload_bytes=self._message_bytes,
                            hamr_timestamp=self.get_hamr_time(self._acoustic_ack_wait_time))
                        self.send_acoustic_packet(acoustic_packet_to_send)

                        # If Ack
                        if self._message_type == 'M':
                            # Then delay or timeout response
                            self._acoustic_ack_wait_address = self._message_address
                            self._acoustic_state = self.ACOUSTIC_STATE_WAIT_ACK

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            else:
                # Unknown state
                self._simulator_state = self.SIMULATOR_STATE_IDLE





def node_position_parser(s):
    """Expects arguments as (x,y,depth)"""
    try:
        vals = s.split(",")
        x = float(vals[0])
        y = float(vals[1])
        depth = float(vals[2])
        return (x, y), depth
    except:
        raise argparse.ArgumentTypeError("Node parameters must be x,y,depth")


from threading import Thread
class TtyWrapper:

    def __init__(self, stdin):
        self._stdin = stdin
        self._line = None

        self._thread = Thread(target=self._poll_stdin)
        self._thread.start()

    def readable(self):
        return self._stdin.readable()

    def read(self):
        bytes_input = None
        if self._line:
            bytes_input = self._line.encode('utf-8')
            self._line = None
            # Fire off another poll on stdin
            self._thread = Thread(target=self._poll_stdin)
            self._thread.start()

        return bytes_input

    def _poll_stdin(self):
        if self._stdin:
            self._line = self._stdin.readline()



def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator. '
                    'Example usage python3 nm3networksimulator.py')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.', type=int)

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')


    # Mode
    cmdline_parser.add_argument('--mode',
                                help='The operating mode: controller/terminal/headless/serial.')

    # Serial Port
    cmdline_parser.add_argument('--serial_port', help='The serial port to connect to.')

    # Local Address
    cmdline_parser.add_argument('--address', help='The local node address on start.', type=int)

    # Virtual Modem Node Position
    cmdline_parser.add_argument('--position', help="Node position as x,y,depth.", dest="position",
                                type=node_position_parser)

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address

    mode = cmdline_args.mode

    position_xy = (0.0,0.0)
    depth = 10.0
    if cmdline_args.position:
        position_xy, depth = cmdline_args.position

    serial_port_name = cmdline_args.serial_port

    address = 255
    if cmdline_args.address:
        address = cmdline_args.address


    #
    # Controller Mode
    #

    if not mode or mode == "controller":
        print("Starting Controller")
        print("zmq version: " + zmq.zmq_version())
        print("pyzmq version: " + zmq.pyzmq_version())
        print("tornado version: " + tornado.version)

        nm3_simulator_controller = Nm3SimulatorController(
            network_address=network_address, network_port=network_port)

        nm3_simulator_controller.start()

    #
    # Virtual NM3 Modem Mode (With stdin/stdout as interface)
    #
    elif mode == "terminal":
        input_stream = sys.stdin.buffer # bytes from a piped input
        if sys.stdin.isatty():
            input_stream = TtyWrapper(sys.stdin) # wrapped to grab lines and convert to bytes

        print("Starting NM3 Virtual Terminal Modem")
        print("zmq version: " + zmq.zmq_version())
        print("pyzmq version: " + zmq.pyzmq_version())
        print("tornado version: " + tornado.version)

        # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
        nm3_modem = Nm3VirtualModem(input_stream=input_stream,
                                    output_stream=sys.stdout.buffer,
                                    network_address=network_address,
                                    network_port=network_port,
                                    local_address=address,
                                    position_xy=position_xy,
                                    depth=depth)

        nm3_modem.run()
    #
    # Headless Virtual NM3 Modems Mode
    #
    elif mode == "headless":
        print("Starting NM3 Virtual Headless Modem")
        print("zmq version: " + zmq.zmq_version())
        print("pyzmq version: " + zmq.pyzmq_version())
        print("tornado version: " + tornado.version)

        # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
        nm3_modem = Nm3VirtualModem(input_stream=None,
                                    output_stream=None,
                                    network_address=network_address,
                                    network_port=network_port,
                                    local_address=address,
                                    position_xy=position_xy,
                                    depth=depth)

        nm3_modem.run()

    #
    # Serial Port NM3 Virtual Modem Modem
    #
    elif mode == "serial":
        # Serial Port is opened with a 100ms timeout for reading.
        with serial.Serial(serial_port_name, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:

            # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
            nm3_modem = Nm3VirtualModem(input_stream=serial_port,
                                        output_stream=serial_port,
                                        network_address=network_address,
                                        network_port=network_port,
                                        local_address=address,
                                        position_xy=position_xy,
                                        depth=depth)

            nm3_modem.run()




if __name__ == '__main__':
    main()