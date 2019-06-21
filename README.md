# nm3-python-driver

## Overview

This driver provides a Python interface to the NM3 Underwater Acoustic Modem via a serial port (pySerial). The NM3 was developed by the SEA Lab team at Newcastle University, UK.

 - Project Homepage: https://github.com/bensherlock/nm3-python-driver/
 - USMART Homepage: https://research.ncl.ac.uk/usmart/

MIT license, (C) 2019 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>

Please Note: This is currently a work-in-progress, highly fluid, and the API/behaviour may change without warning. Although version tagging of the first useable alpha is expected quite soon.

## Documentation

The sourcefiles contain docstrings that can be viewed using pydocs for information about functions and classes. Further information about usage will be added to this README in due course.

## Usage

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

