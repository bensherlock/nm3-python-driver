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


def example_query_status(nm3_modem):
    """Example: $? - Query Status."""
    print('Example: Query Status')
    addr_int, voltage, version_string, build_date_string = nm3_modem.query_status()

    print(' Modem Address={:03d}'.format(addr_int))
    print(' Battery Voltage={:.2f}V'.format(voltage))
    print(' Version=' + version_string)
    print(' Build Date=' + build_date_string)


def example_set_address(nm3_modem, new_address):
    """Example: $A - Set Address."""
    print('Example: Set Address')

    print('  Query Current Status')
    ret = nm3_modem.query_status()
    if ret == -1:
        print(' Error')
    else:
        addr_int, voltage, version_string, build_date_string = ret
        print(' Initial Modem Address={:03d}'.format(addr_int))

    addr_new_int = nm3_modem.set_address(new_address)
    print(' Set Modem Address={:03d}'.format(addr_new_int))

    print('  Query Current Status')
    ret = nm3_modem.query_status()
    if ret == -1:
        print(' Error')
    else:
        addr_int, voltage, version_string, build_date_string = ret
        print(' Current Modem Address={:03d}'.format(addr_int))


def example_broadcast_data(nm3_modem, message):
    """Example: $B - Broadcast Data."""
    print('Example: Broadcast Data')

    sent_bytes_count = nm3_modem.send_broadcast_message(message)
    if sent_bytes_count == -1:
        print(' Error')
    else:
        # Pause for the modem to finish the transmission
        time.sleep(4.0)
        print(' Bytes Transmitted={:02d}'.format(sent_bytes_count))


def example_channel_impulse_response(nm3_modem, remote_address, plot_results=False):
    """Example: $C - Channel Impulse Response."""
    print('Example: Channel Impulse Response')

    print('Magnitudes: Remote Address={:03d}'.format(remote_address))
    ret = nm3_modem.send_ping_for_channel_impulse_response(remote_address, 'M')
    if ret == -1:
        print(' Error')
    else:
        timeofarrival, data_count, data_values = ret
        print(' Time of Arrival={:.6f} seconds'.format(timeofarrival))
        print(' Data Count={:04d}'.format(data_count))
        if plot_results:
            import matplotlib.pyplot as plt
            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1)
            ax.set_title("Channel Impulse Response (Magnitude)")
            ax.set_xlabel("Delay (s)")
            ax.set_ylabel("Magnitude")
            ax.grid(True, which='both')

            time_values = [(float(b-100) / 16e3) for b in range(data_count)]

            ax.plot(time_values, data_values)
            plt.show()


    print('Complex: Remote Address={:03d}'.format(remote_address))
    ret = nm3_modem.send_ping_for_channel_impulse_response(remote_address, 'C')
    if ret == -1:
        print(' Error')
    else:
        timeofarrival, data_count, data_values = ret
        print(' Time of Arrival={:.6f} seconds'.format(timeofarrival))
        print(' Data Count={:04d}'.format(data_count))
        real_data_values = data_values[0::2]
        imaginary_data_values = data_values[1::2]

        if plot_results:
            import matplotlib.pyplot as plt
            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1)
            ax.set_title("Channel Impulse Response (Complex)")
            ax.set_xlabel("Delay (s)")
            ax.set_ylabel("Correlation Value")
            ax.grid(True, which='both')

            time_values = [(float(b-50) / 16e3) for b in range(int(data_count/2))]

            ax.plot(time_values, real_data_values, label='Real')
            ax.plot(time_values, imaginary_data_values, label='Imaginary')
            plt.show()



def example_unicast_data_with_ack(nm3_modem, remote_address, message):
    """Example: $M - Unicast Data with Ack."""
    print('Example: Unicast Data with Ack')

    ret = nm3_modem.send_unicast_message_with_ack(remote_address, message)
    if ret == -1:
        print(' Error')
    else:
        print(' Time of Arrival={:.6f} seconds'.format(ret))


def example_unicast_data(nm3_modem, remote_address, message):
    """Example: $U - Unicast Data."""
    print('Example: Unicast Data')

    sent_bytes_count = nm3_modem.send_unicast_message(remote_address, message)
    if sent_bytes_count == -1:
        print(' Error')
    else:
        # Pause for the modem to finish the transmission
        time.sleep(4.0)
        print(' Bytes Transmitted={:02d}'.format(sent_bytes_count))



def example_noise_measurement(nm3_modem):
    """Example: $N - Noise Measurement"""
    print('Example: Noise Measurement')

    ret = nm3_modem.measure_local_ambient_noise()
    if ret == -1:
        print(' Error')
    else:
        rms_int, p2p_int, mm_int = ret
        print(' RMS={:05d}'.format(rms_int))
        print(' P2P={:05d}'.format(p2p_int))
        print(' Mean Magnitude={:05d}'.format(mm_int))


def example_ping(nm3_modem, remote_address):
    """Example: $P - Ping"""
    print('Example: Ping')

    ret = nm3_modem.send_ping(remote_address)
    if ret == -1:
        print(' Error')
    else:
        print(' Time of Arrival={:.6f} seconds'.format(ret))



def example_spectrum_measurement(nm3_modem, plot_results=False):
    """Example: $S - Spectrum Measurement"""
    print('Example: Spectrum Measurement')

    ret = nm3_modem.measure_local_noise_spectrum()
    if ret == -1:
        print(' Error')
    else:
        data_count, data_values = ret
        print(' Data Count={:04d}'.format(data_count))
        if plot_results:
            import matplotlib.pyplot as plt
            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1)
            ax.set_title("FFT")
            ax.set_xlabel("Frequency (Hz)")
            ax.set_ylabel("Magnitude")
            ax.grid(True, which='both')

            bin_values = range(data_count)

            fs = 160.0e3
            nfft = (data_count-1)*2.0
            bin_step_frequency = fs/nfft
            bin_frequencies = [ (float(f)*bin_step_frequency) for f in bin_values ]

            ax.bar(bin_frequencies, data_values, width=bin_step_frequency, linewidth=0.0)
            #ax.plot(bin_frequencies, data_values)

            plt.show()


def example_system_timer(nm3_modem):
    """Example: $XT - System Timer"""
    print('Example: System Timer')

    print(' Get System Timer Status')
    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Enable System Timer')
    ret = nm3_modem.enable_system_timer()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Get System Timer Status')
    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Get System Timer Status')
    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Clear System Timer')
    ret = nm3_modem.clear_system_timer()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))

    print(' Get System Timer Status')
    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Disable System Timer')
    ret = nm3_modem.disable_system_timer()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Get System Timer Status')
    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


def example_delayed_transmission(nm3_modem, remote_address, message):
    """Example: Delayed Transmission"""
    print('Example: Delayed Transmission')

    time_count_int = 0

    # Enable System Timer
    print(' Enable System Timer')
    ret = nm3_modem.enable_system_timer()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Send Broadcast Message')
    transmit_time_count_int = time_count_int + 1000000  # 1 second
    sent_bytes_count = nm3_modem.send_broadcast_message(message, transmit_time_count_int)
    if sent_bytes_count == -1:
        print('  Error')
    else:
        # Pause for the modem to finish the transmission
        time.sleep(2.0)
        print('  Bytes Transmitted={:02d}'.format(sent_bytes_count))


    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Send Unicast Message')
    transmit_time_count_int = time_count_int + 1000000  # 1 second
    sent_bytes_count = nm3_modem.send_unicast_message(remote_address, message, transmit_time_count_int)
    if sent_bytes_count == -1:
        print('  Error')
    else:
        # Pause for the modem to finish the transmission
        time.sleep(2.0)
        print('  Bytes Transmitted={:02d}'.format(sent_bytes_count))


    ret = nm3_modem.get_system_timer_status()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))


    print(' Send Unicast with Ack Message')
    transmit_time_count_int = time_count_int + 1000000  # 1 second
    ret = nm3_modem.send_unicast_message_with_ack(remote_address, message, 5.0, transmit_time_count_int)
    if ret == -1:
        print('  Error')
    else:
        # Pause for the modem to finish the transmission
        time.sleep(2.0)
        print('  Time of Arrival={:.6f} seconds'.format(ret))



def example_receiver(nm3_modem, remote_address, timeout_seconds):
    """Example: Receiver"""
    print('Example: Receiver')

    # Enable System Timer
    print(' Enable System Timer')
    ret = nm3_modem.enable_system_timer()
    if ret == -1:
        print('  Error')
    else:
        enabled_flag, time_count_int = ret
        if enabled_flag:
            print('  Enabled=True')
        else:
            print('  Enabled=False')

        print('  Time Count={:014d}'.format(time_count_int))

    # Send Test Message Request
    now_time = time.time()
    end_time = now_time + timeout_seconds

    # Receiving unicast and broadcast messages
    while time.time() < end_time:
        # Periodically poll the serial port for bytes
        nm3_modem.poll_receiver()

        # Periodically process any bytes received
        nm3_modem.process_incoming_buffer()

        # Periodically check for received packets
        while nm3_modem.has_received_packet():
            message_packet = nm3_modem.get_received_packet()

            payload_as_string = bytes(message_packet.packet_payload).decode('utf-8')

            print('Received a message packet: ')
            print(' serial_string: ' + message_packet.serial_string)
            print(' type: ' + MessagePacket.PACKETTYPE_NAMES[message_packet.packet_type])
            print(' src: ' + str(message_packet.source_address))
            print(' data: ' + payload_as_string)
            print(' lqi: ' + str(message_packet.packet_lqi))
            print(' doppler: ' + str(message_packet.packet_doppler))
            print(' timestamp_count: ' + str(message_packet.packet_timestamp_count))


def main():
    """Main Program Entry."""

    ## Create the Modem
    # Serial Port is opened with a 100ms timeout for reading.
    serial_port = serial.Serial('COM8', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
    nm3_modem = Nm3(input_stream=serial_port, output_stream=serial_port)


    ## Parameters used for the examples
    local_address = 255
    remote_address = 122
    message = b'Hello'

    ## Run Through Examples

    # $? - Query Status
    example_query_status(nm3_modem=nm3_modem)

    # $A - Set Address
    example_set_address(nm3_modem=nm3_modem, new_address=local_address)

    # $B - Broadcast Data
    example_broadcast_data(nm3_modem=nm3_modem, message=message)

    # $C - Channel Impulse Response
    example_channel_impulse_response(nm3_modem=nm3_modem, remote_address=remote_address, plot_results=True)

    # $N - Ambient Noise Measurement
    example_noise_measurement(nm3_modem=nm3_modem)

    # $P - Ping
    example_ping(nm3_modem=nm3_modem, remote_address=remote_address)

    # $S - Spectrum Measurement
    example_spectrum_measurement(nm3_modem=nm3_modem, plot_results=True)

    # $XT - System Timer
    example_system_timer(nm3_modem=nm3_modem)

    # Delayed Transmission
    example_delayed_transmission(nm3_modem=nm3_modem, remote_address=remote_address, message=message)

    # Receiver
    example_receiver(nm3_modem=nm3_modem, remote_address=remote_address, timeout_seconds=20)


    return


if __name__ == '__main__':
    main()
