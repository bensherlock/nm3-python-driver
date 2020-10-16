# nm3-python-driver package

A collection of drivers and utilities for easily working with NM3 underwater acoustic modems. 
Intended to get researchers/engineers up and running quickly so you can spend more effort on your application/science. 

## NM3 Underwater Acoustic Modem

The NM3 underwater acoustic modem was developed by the SEA Lab team at Newcastle University, UK.

 - Project Homepage: https://github.com/bensherlock/nm3-python-driver/
 - USMART Homepage: https://research.ncl.ac.uk/usmart/
 

## The Utilities

* [NM3 Driver](#nm3driverpy) 
* [NM3 Logger](#nm3loggerpy)
* [NM3 Logs Reader](#nm3readlogfilepy)
* [NM3 Standalone Virtual Modem](#nm3simulatorpy)
* [NM3 Network Simulator with Virtual Modems](#nm3networksimulatorpy)

The nm3driver.py provides an interface to control the modem and to receive incoming messages without having to write 
your own parser. 

The nm3logger.py lets you simply store all incoming message packets in a csv file, which can then be read back in with 
the nm3readlogfile.py for analysis of your data in Python. 

The nm3networksimulator.py is a full virtual sea of virtual modems, each of which can be connected to exernal sensor 
node hardware via the serial port, or a human terminal interface, or else your own Python code for running network 
protocols. The Controller and virtual modems can be running on a single machine, or across the network for distributed 
development and testing of networks prior to deployment with physical modems. 

MIT license, (C) 2019 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>


## nm3driver.py

This driver provides a Python interface to the NM3 Underwater Acoustic Modem via a serial port (pySerial). 

Please Note: This is currently a work-in-progress, highly fluid, and the API/behaviour may change without warning. 
Although version tagging of the first useable alpha is expected quite soon.

The sourcefiles contain docstrings that can be viewed using pydocs for information about functions and classes. 
Further information about usage will be added to this README in due course.

### Usage in code

Download the nm3driver.py and include as part of your project. Then import the modules as below:
```python
from nm3driver import Nm3
from nm3driver import MessagePacket
```

If connecting to a physical NM3 modem via the serial port, using pyserial create a serial port instance based on 
9600 baud, no parity, 8 databits, one stop bit, and read timeout of say 0.1s. 
Then use this to construct the NM3 instance.

```python
from nm3driver import Nm3
from nm3driver import MessagePacket
import serial

serial_port = serial.Serial('/dev/ttyS4', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
nm3_modem = Nm3(input_stream=serial_port, output_stream=serial_port)

# Receiving unicast and broadcast messages
while True:
    # Periodically poll the serial port for bytes
    nm3_modem.poll_receiver()

    # Periodically process any bytes received
    nm3_modem.process_incoming_buffer()

    # Periodically check for received packets
    while nm3_modem.has_received_packet():
        message_packet = nm3_modem.get_received_packet()

        payload_as_string = bytes(message_packet.packet_payload).decode('utf-8')

        print('Received a message packet: ' +
              MessagePacket.PACKETTYPE_NAMES[message_packet.packet_type] +
              ' src: ' + str(message_packet.source_address) + ' data: ' +
              payload_as_string + ' timestamp_count: ' 
              + str(message_packet.packet_timestamp_count))
```

Example usage is shown in nm3example.py for modem commands for range pinging, getting and setting address, reading 
local battery level, and message transmission and receipt - broadcast, unicast, and unicast with ack.


## nm3logger.py

The Nm3Logger will listen for incoming MessagePackets and write these to a csv file. The csv filename is based on the 
filename_root and the datetime it was created. If the logger is left running it will automatically create a fresh file 
at midnight. 

### Commandline application

```shell
python3 nm3logger.py /dev/ttyS1 Nm3Log
```

### In code

```python
from nm3driver import Nm3
from nm3driver import MessagePacket
import serial

with serial.Serial(port, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:
    nm3_modem = Nm3(input_stream=serial_port, output_stream=serial_port)

    nm3_logger = Nm3Logger()

    nm3_logger.start_logging(nm3_modem=nm3_modem, filename_root=filenameroot) 
```

## nm3readlogfile.py

```python
import nm3readlogfile
from nm3readlogfile import NM3LogFileEntry

entries = nm3readlogfile.read_nm3_logfile(filename)

for e in entries:
    # Process a NM3LogFileEntry 
    print(e.PacketType)
```

## nm3simulator.py

Acts as a virtual modem without sending any acoustic packets. 

## nm3networksimulator.py

Virtual NM3 modems (Nm3VirtualModem) in a virtual sea simulator (Nm3SimulatorController). 
The Controller and virtual modems can be run on a single machine or on multiple machines across the network/internet.


### Commandline applications
From the command line a number of use cases are already provided for:

The Controller 
```shell
python3 nm3networksimulator.py --mode controller --network_address 127.0.0.1 --network_port 8080
```

The virtual modems can be connected to via the serial port for testing sensor hardware as if the hardware were 
connected to a real NM3 modem. 
```shell
python3 nm3networksimulator.py --mode serial --network_address 127.0.0.1 --network_port 8080 \
    --serial_port /dev/ttyS1 --position 0.0,0.0,10.0 --address 7
```

Virtual modems can be run as human usable terminal in the console window and respond to the typed commands as per the 
real NM3 hardware.
```shell
python3 nm3networksimulator.py --mode terminal --network_address 127.0.0.1 --network_port 8080 \
    --position 0.0,0.0,10.0 --address 7
```

Virtual modems can be run as headless transponders.
```shell
python3 nm3networksimulator.py --mode headless --network_address 127.0.0.1 --network_port 8080 \
    --position 0.0,0.0,10.0 --address 7
```

Virtual modems can act as Nm3Loggers.
```shell
python3 nm3networksimulator.py --mode logger --network_address 127.0.0.1 --network_port 8080 \
    --filename_root Nm3Logs --position 0.0,0.0,10.0 --address 7
```

### In code

Virtual modems can be incorporated into user code with user defined network stacks interacting across the virtual sea.
Virtual modems can have their position in the virtual sea modified at run time.

```python
from nm3driver import Nm3
from nm3driver import MessagePacket
from nm3networksimulator import Nm3VirtualModem
from queue import Queue
import serial
import time
from threading import Thread

class BufferedIOQueueWrapper:
    """Wraps a Queue as IO with Read and Write binary functions."""

    def __init__(self):
        self._the_queue = Queue()

    def readable(self):
        return True

    def read(self, n = -1):
        """Read up to n bytes"""
        the_bytes = []
        while not self._the_queue.empty() and (n == -1 or len(the_bytes) < n):
            the_bytes.append(self._the_queue.get())

        return bytes(the_bytes)

    def writable(self):
        return True

    def write(self, the_bytes: bytes):
        """Returns the number of bytes written."""
        for b in the_bytes:
            self._the_queue.put(b)

        return len(the_bytes)

    def flush(self):
        """Nothing to flush to."""
        pass


def main():
    # Pipes
    outgoing_stream = BufferedIOQueueWrapper()
    incoming_stream = BufferedIOQueueWrapper()

    nm3_driver = Nm3(input_stream=incoming_stream, output_stream=outgoing_stream)


    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    nm3_modem = Nm3VirtualModem(input_stream=outgoing_stream,
                                output_stream=incoming_stream,
                                network_address=network_address,
                                network_port=network_port,
                                local_address=address,
                                position_xy=position_xy,
                                depth=depth)
    a_thread = Thread(target=nm3_modem.run)
    a_thread.start()  #nm3_modem.run()


    # Now loop
    beacon_time = time.time()



    while True:
        if beacon_time < time.time():
            timestamp_str = "%d-%02d-%02d %02d:%02d:%02d" % time.localtime()[:6]
            message_string = "Beacon Message: " + timestamp_str
            message_bytes = message_string.encode('utf-8')

            nm3_driver.send_broadcast_message(message_bytes)
            beacon_time = beacon_time + 60.0  # Every 60 seconds

        nm3_driver.poll_receiver()
        nm3_driver.process_incoming_buffer()
        while nm3_driver.has_received_packet():
            packet = nm3_driver.get_received_packet()
            print("Packet received:" + str(packet.json()))
```

### Acoustic Packet Propagation
At the moment the Controller handles AcousticPacket propagation in a very overly simple way. 
Assuming no losses. And isovelocity. And no obstructions. And no multipath. And no noise. 
It simply calculates the expected arrival time based on a nominal speed of sound (1500 m/s) and 
the straight line distance between the transmitting node and the receiving node. 

This will be changed to add calculations to determine probability of successful packet arrival and demodulation, 
these will be specific to the frequency band used by the NM3 and the duration of the packet. 
However, this will still make assumptions of an open sea and no obstructions with no multipath arrivals. 

If you require more in-depth channel modelling then you are free to extend the Nm3SimulatorController class and alter the 
```calculate_propagation``` function accordingly. 



