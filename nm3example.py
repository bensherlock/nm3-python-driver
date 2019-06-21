#! /usr/bin/env python
#
# Example Usage of the Python Driver for NM V3
#
#
"""Example program for using the Nm3 driver. """

import time
import serial
from nm3driver import Nm3
from nm3driver import MessagePacket

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

    addr = 2
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
    bytes_count = serial_port.write('$T002'.encode('utf-8'))
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
                  payload_as_string)



if __name__ == '__main__':
    main()
