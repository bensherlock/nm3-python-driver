#! /usr/bin/env python
#
# Example Usage of the Python Driver for NM V3
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
"""Example program for using the Nm3 driver. """

import time
import serial
from nm3driver import Nm3
from nm3driver import MessagePacket

def main():
    """Main Program Entry."""
    
    # Serial Port is opened with a 100ms timeout for reading.
    serial_port = serial.Serial('/dev/ttyS3', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
    nm3_modem = Nm3(serial_port)

    addr = nm3_modem.get_address()
    print('Get Address=' + '{:03d}'.format(addr))

    voltage = nm3_modem.get_battery_voltage()
    print('Battery Voltage=' + '{:.2f}'.format(voltage) + 'V')

    #addr = nm3_modem.set_address(3)
    #print('Set Address=' + '{:03d}'.format(addr))

    #addr = nm3_modem.get_address()
    #print('Get Address=' + '{:03d}'.format(addr))

    addr = 3
    # Speed of Sound.
    # In dry air @ 20C = 343m/s.
    # In water around 1500m/s.
    # Note for both, temperature, salinity, pressure, etc. all affect the speed of sound.
    # For more precise ranging please determine the correct local speed of sound.
    speed_of_sound = 343.0
    tof = nm3_modem.send_ping(addr)
    distance = tof * speed_of_sound
    print('Time of Flight to ' '{:03d}'.format(addr) + ' = ' + '{:.4f}'.format(tof) + 's' +
          ' distance = ' + '{:.4f}'.format(distance) + 'm')


    broadcast_message = 'Hello World.'
    sent_bytes_count = nm3_modem.send_broadcast_message(broadcast_message.encode('utf-8'))
    print('Sent Broadcast Message of ' + str(sent_bytes_count) + ' bytes')

    # Need a pause between transmissions for the modem to finish the last one
    time.sleep(2.0)

    # Send a test request so the remote node sends a broadcast message that we'll look at below.
    bytes_count = serial_port.write('$T003'.encode('utf-8'))
    # Expecting '$T002\r\n' 7 bytes
    resp = serial_port.read(7)


    # Receiving unicast and broadcast messages
    while True:
        # Periodically poll the serial port for bytes
        nm3_modem.poll_receiver()

        # Periodically process any bytes received
        nm3_modem.process_incoming_buffer()

        # Periodically check for received packets
        if nm3_modem.has_received_packet():
            message_packet = nm3_modem.get_received_packet()

            payload_as_string = bytes(message_packet.packet_payload).decode('utf-8')

            print('Received a message packet: ' +
                  MessagePacket.PACKETTYPE_NAMES[message_packet.packet_type] +
                  ' src: ' + str(message_packet.source_address) + ' data: ' +
                  payload_as_string + ' timestamp_count: ' 
                  + str(message_packet.packet_timestamp_count))



if __name__ == '__main__':
    main()
