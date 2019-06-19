#! /usr/bin/env python
#
# Python Driver for NM3
#
#
"""Python driver for the NM3 over serial port."""


import serial


class Nm3:
    """NM3 Driver over Serial Port."""

    def __init__(self,
                 serial_port: serial.Serial):
        self.serial_port = serial_port


    def __call__(self):
        return self


    def get_address(self) -> int:
        """Gets the NM3 Address (000-255)."""
        # Clear the input buffer
        self.serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self.serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response from the serial port
        # Expecting '#A255V21941\r\n' 13 bytes
        resp_string = '#A255V21941\r\n'
        resp_bytes = self.serial_port.read(13)

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
        # Clear the input buffer
        self.serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$A' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self.serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response from the serial port
        # Expecting '#A255\r\n' 7 bytes
        resp_string = '#A255\r\n'
        resp_bytes = self.serial_port.read(7)

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
        """Gets the NM3 Address (000-255)."""
        # Clear the input buffer
        self.serial_port.reset_input_buffer()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self.serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response from the serial port
        # Expecting '#A255V21941\r\n' 13 bytes
        resp_string = '#A255V21941\r\n'
        resp_bytes = self.serial_port.read(13)

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

    def get_time_of_flight(self,
                           address: int,
                           timeout: float = 5.0) -> float:
        """Gets the one way time of flight in seconds from this device to the node address
           provided.
        """
        # Clear the input buffer
        self.serial_port.reset_input_buffer()

        default_read_timeout = self.serial_port.timeout
        # Set the temporary read timeout
        self.serial_port.timeout = timeout

        # Write the command to the serial port
        cmd_string = '$P' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self.serial_port.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            # Set the default read timeout
            self.serial_port.set_timeout(default_read_timeout)
            return -1

        # Await the first response from the serial port
        # Expecting '$P255\r\n' 7 bytes
        resp_string = '$P255\r\n'
        resp_bytes = self.serial_port.read(7)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            # Set the default read timeout
            self.serial_port.timeout = default_read_timeout
            return -1

        # Now await the range or TO after 4 seconds
        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = '#R255T12345\r\n'
        resp_bytes = self.serial_port.read(13)

        # Check that it has received all the expected bytes. Return error if not.
        if len(resp_bytes) != len(resp_string):
            print('Error receiving number of bytes=' + str(len(resp_bytes)) +
                  ' expected=' + str(len(resp_string)))
            # Set the default read timeout
            self.serial_port.timeout = default_read_timeout
            return -1

        resp_string = resp_bytes.decode('utf-8')

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        time = float(time_int) * 31.25E-6

        # Set the default read timeout
        self.serial_port.timeout = default_read_timeout

        return time
