#! /usr/bin/env python
#
# NM V3 Logger
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
"""NM3 Acoustic Modem Logger using the Nm3 driver. """

import argparse
from datetime import datetime
import serial
from nm3driver import Nm3
from nm3driver import MessagePacket


class Nm3Logger:

    def __init__(self):
        self._nm3_modem = None
        self._filename_root = None

    def start_logging(self, nm3_modem, filename_root):
        self._nm3_modem = nm3_modem
        self._filename_root = filename_root

        # Open a new logfile with current time
        file_datetime = datetime.utcnow()
        dt_str = file_datetime.strftime('%Y%m%dT%H%M%S')
        filename = self._filename_root + '-' + dt_str + '.csv'

        # 0=no buffer, 1=Line buffering, N=bytes buffering, None=system default
        bufsize = 1
        logfile = open(filename, 'w', buffering=bufsize)

        # Write the header row to the logfile
        self.write_header_line(logfile)

        # Debug Code
        # Send a test request so the remote node sends a broadcast message that we'll look at below.
        # bytes_count = serial_port.write('$T007'.encode('utf-8'))
        # Expecting '$T002\r\n' 7 bytes
        # resp = serial_port.read(7)

        packet_id = 0

        # Receiving unicast and broadcast messages
        while True:
            # Check current time. We want to start a new logfile once the day changes.
            current_datetime = datetime.utcnow()
            if current_datetime.day != file_datetime.day:
                # Start a new log file
                logfile.close()
                file_datetime = current_datetime
                dt_str = file_datetime.strftime('%Y%m%dT%H%M%S')
                filename = self._filename_root + '-' + dt_str + '.csv'

                # 0=no buffer, 1=Line buffering, N=bytes buffering, None=system default
                bufsize = 1
                logfile = open(filename, 'w', buffering=bufsize)

                # Write the header row to the logfile
                self.write_header_line(logfile)

                # Reset the packet id for the new file
                packet_id = 0

            # Periodically poll the serial port for bytes
            # nm3_modem.poll_receiver_blocking() # blocking on first byte (or timeout)
            nm3_modem.poll_receiver() # non-blocking returns immediately if no bytes ready to read.

            # Periodically process any bytes received
            nm3_modem.process_incoming_buffer()

            # Periodically check for received packets
            while nm3_modem.has_received_packet():
                message_packet = nm3_modem.get_received_packet()

                # Write to file
                self.write_packet_line(logfile, message_packet, packet_id)

                # Print to console
                # payload_as_string = bytes(message_packet.packet_payload).decode('utf-8')
                print('Received message packet: ' +
                      MessagePacket.PACKETTYPE_NAMES[message_packet.packet_type] +
                      ' src: ' + str(message_packet.source_address) +
                      ' dest: ' + str(message_packet.destination_address))

                packet_id = packet_id + 1



    def write_header_line(self, the_file):
        """Write the text header line to the given file."""
        # Write the header row to the logfile
        # PacketId, Timestamp, PacketType (Broadcast/Unicast), Source Address, Destination Address,
        # PayloadLength, PayloadBytes(hex encoded)
        # New Additions for V1.3 Firmware
        # LQI, Doppler, TimestampCount
        # \r\n
        the_file.write('PacketId,Timestamp,PacketType,SourceAddress,DestinationAddress,PayloadLength,')

        # Hex encoded payload bytes
        for i in range(0, 64):
            the_file.write('PayloadBytes[' + str(i) + ']')
            if i < 63:
                the_file.write(',')

        # V1.3 Additions
        the_file.write(',LQI,Doppler,TimestampCount')

        # Newline
        the_file.write('\n')


    def write_packet_line(self, the_file, the_packet, packet_id):
        """Write the text packet line to the given file."""
        # Write to file
        # PacketId, Timestamp, PacketType (Broadcast/Unicast), Source Address,
        # Destination Address, PayloadLength, PayloadBytes(hex encoded) \n
        the_file.write(str(packet_id) + ',')
        dt_str = datetime.utcnow().isoformat() + 'Z'
        the_file.write(dt_str + ',')
        the_file.write(MessagePacket.PACKETTYPE_NAMES[the_packet.packet_type] + ',')
        if the_packet.source_address:
            the_file.write('{:03d}'.format(the_packet.source_address))
        the_file.write(',')
        if the_packet.destination_address:
            the_file.write('{:03d}'.format(the_packet.destination_address))
        the_file.write(',')
        the_file.write('{:02d}'.format(len(the_packet.packet_payload)) + ',')

        # Hex encoded payload bytes
        for i in range(0, 64):
            if i < len(the_packet.packet_payload):
                the_file.write('0x' + '{:02x}'.format(the_packet.packet_payload[i]))
            if i < 63:
                the_file.write(',')

        # V1.3 Additions
        # the_file.write(',LQI,Doppler,TimestampCount')
        the_file.write(',')
        the_file.write('{:02d}'.format(the_packet.packet_lqi))

        the_file.write(',')
        the_file.write('{:03d}'.format(the_packet.packet_doppler))

        the_file.write(',')
        the_file.write('{:014d}'.format(the_packet.packet_timestamp_count))

        # Newline
        the_file.write('\n')
    


def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(description='NM3 Logger')

    # Add Command Line Arguments
    # Serial Port
    cmdline_parser.add_argument('port', help='The serial port to connect to the NM3.')

    # Filename root for logging - Rotates at start of each day (midnight UTC)
    cmdline_parser.add_argument('filenameroot', help='The filename root to log to. {filenameroot}-20191204T161500.csv')

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    port = cmdline_args.port
    filenameroot = cmdline_args.filenameroot

    # Serial Port is opened with a 100ms timeout for reading - non-blocking.
    #serial_port = serial.Serial(port, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
    with serial.Serial(port, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:
        nm3_modem = Nm3(input_stream=serial_port, output_stream=serial_port)

        # Enable System Timer
        nm3_modem.enable_system_timer()

        nm3_logger = Nm3Logger()

        nm3_logger.start_logging(nm3_modem=nm3_modem, filename_root=filenameroot)


if __name__ == '__main__':
    main()
