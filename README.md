# nm3-python-driver package

A collection of drivers and utilities for easily working with NM3 underwater acoustic modems. Intended to get researchers/engineers up and running quickly so you can spend more effort on your application/science. 

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

The nm3driver.py provides an interface to control the modem and to receive incoming messages without having to write your own parser. 

The nm3logger.py lets you simply store all incoming message packets in a csv file, which can then be read back in with the nm3readlogfile.py for analysis of your data in Python. 

The nm3networksimulator.py is a full virtual sea of virtual modems, each of which can be connected to exernal sensor node hardware via the serial port, or a human terminal interface, or else your own Python code for running network protocols. The Controller and virtual modems can be running on a single machine, or across the network for distributed development and testing of networks prior to deployment with physical modems. 

MIT license, (C) 2019 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>


## nm3driver.py

This driver provides a Python interface to the NM3 Underwater Acoustic Modem via a serial port (pySerial). 

Please Note: This is currently a work-in-progress, highly fluid, and the API/behaviour may change without warning. Although version tagging of the first useable alpha is expected quite soon.

The sourcefiles contain docstrings that can be viewed using pydocs for information about functions and classes. Further information about usage will be added to this README in due course.

### Usage

Download the nm3driver.py and include as part of your project. Then import the modules as below:
~~~~
from nm3driver import Nm3
from nm3driver import MessagePacket
~~~~

Create a serial port instance based on 9600 baud, no parity, 8 databits, one stop bit, and read timeout of say 0.1s. Then use this to construct the NM3 instance.
~~~
    serial_port = serial.Serial('/dev/ttyS4', 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
    nm3_modem = Nm3(serial_port)
~~~

Example usage is shown in nm3example.py for modem commands for range pinging, getting and setting address, reading local battery level, and message transmission and receipt - broadcast, unicast, and unicast with ack.


## nm3logger.py


## nm3readlogfile.py

## nm3simulator.py

## nm3networksimulator.py


