#! /usr/bin/env python
#
# Example Usage of the Python Driver for NM V3
#
#
"""Example program for using the Nm3 driver. """

import serial
from nm3driver import Nm3

def main():
    """Main Program Entry."""
    serial_port = serial.Serial('/dev/ttyS4', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
    nm3_modem = Nm3(serial_port)

    addr = nm3_modem.get_address()
    print('Get Address=' + '{:03d}'.format(addr))
    
    voltage = nm3_modem.get_battery_voltage()
    print('Battery Voltage=' + '{:.2f}'.format(voltage) + 'V')

    #addr = nm3_modem.set_address(3)
    #print('Set Address=' + '{:03d}'.format(addr))

    #addr = nm3_modem.get_address()
    #print('Get Address=' + '{:03d}'.format(addr))

    tof = nm3_modem.get_time_of_flight(255)
    print('Time of Flight=' + '{:.2f}'.format(tof) + 's')
    
if __name__ == '__main__':
    main()
