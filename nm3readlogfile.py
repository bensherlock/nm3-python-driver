#!/usr/bin/python3.7
#
#
# NM V3 Read Logfile
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
"""NM3 Read Log File created using the Nm3 driver. """

import argparse
import sys
import os
import csv
from datetime import datetime
import dateutil.parser
from typing import List


class NM3LogFileEntry:
    """NM3 Log File Entry."""

    def __init__(self):
        self._packet_id = None
        self._timestamp = None
        self._packet_type = None
        self._source_address = None
        self._destination_address = None
        self._payload_length = None
        self._payload_bytes = None

    def __call__(self):
        return self

    @property
    def packet_id(self) -> int:
        """Gets the packet id."""
        return self._packet_id

    @packet_id.setter
    def packet_id(self,
                  packet_id: int):
        """Sets the packet id."""
        if packet_id and packet_id < 0:
            raise ValueError('Invalid Packet Id Value (0+): {!r}'.format(packet_id))
        self._packet_id = packet_id

    @property
    def timestamp(self) -> datetime:
        """Gets the timestamp."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self,
                  timestamp: datetime):
        """Sets the timestamp."""
        self._timestamp = timestamp

    @property
    def packet_type(self):
        """Gets the packet type."""
        return self._packet_type

    @packet_type.setter
    def packet_type(self, packet_type):
        """Sets the packet type."""
        self._packet_type = packet_type

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
    def payload_length(self) -> int:
        """Gets the payload length"""
        return self._payload_length

    @property
    def payload_bytes(self) -> bytes:
        """Gets the payload bytes"""
        return self._payload_bytes

    @payload_bytes.setter
    def payload_bytes(self,
                      payload_bytes: bytes):
        self._payload_bytes = payload_bytes
        self._payload_length = 0
        if self._payload_bytes:
            self._payload_length = len(self._payload_bytes)


def read_nm3_logfile(filename) -> List[NM3LogFileEntry]:
    """Read the given log file and return a list of Nm3LogEntries."""
    entries = []

    with open(filename, 'rt') as csv_file:
        #csv_reader = csv.reader(csv_file, delimiter=",")
        # https://www.reddit.com/r/pythontips/comments/4md6p0/null_bytes_break_csv_reader/
        csv_reader = csv.reader( (line.replace('\0', '') for line in csv_file) , delimiter=",")

        # check if not empty from http://stackoverflow.com/a/15606960/209647
        sentinel = object()
        # get the header row
        hrow = next(csv_reader, sentinel)
        if hrow is sentinel:
            return []

        # Parse the rows to convert to Nm3LogEntry instances.
        # PacketId, Timestamp, PacketType (Broadcast/Unicast), Source Address, Destination Address,
        # PayloadLength, PayloadBytes(hex encoded) \r\n

        for row in csv_reader:
            if len(row) > 0:
                entry = NM3LogFileEntry()
                entry.packet_id = int(row[0])
                entry.timestamp = dateutil.parser.parse(row[1])
                entry.packet_type = row[2]
                entry.source_address = int(row[3]) if row[3] else None
                entry.destination_address = int(row[4]) if row[4] else None
                payload_length = int(row[5])
                entry.payload_bytes = [int(b, 0) for b in row[6:6 + payload_length]]
                entries.append(entry)

    return entries


def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(description='NM V3 Logfile Reader')

    # Add Command Line Arguments

    # Filename to load the logfile from.
    cmdline_parser.add_argument('filename', help='The logfile filename to read.')

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    filename = cmdline_args.filename

    # Read in the entries from the logfile
    nm3_entries = read_nm3_logfile(filename)

    # Now process as you wish...


if __name__ == '__main__':
    main()
