# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$
#
# Modifed from the orginal comModbusTcp by Kelsey Hawkins @ Georgia Tech
import fcntl
import ready_logging
from math import ceil
from pymodbus.client.sync import ModbusSerialClient
from threading import Lock


@ready_logging.logged
class SingleCommClient:
    """
    Client for communicating with a single device ID on a shared channel.
    """
    def __init__(self, dev_id, comm_channel):
        self.dev_id = dev_id
        self.comms = comm_channel

    def send_command(self, data):
        return self.comms.send_command(self.dev_id, data)

    def get_status(self, num_bytes):
        return self.comms.get_status(self.dev_id, num_bytes)


@ready_logging.logged
class SingleCommunication:
    def __init__(self):
        self.client = None
        self.modbus_comms_lock = Lock()

    def connect(self, device):
        """
        Connect to the client using the system device at the provided path.
        """
        self.client = ModbusSerialClient(method='rtu', port=device, stopbits=1, bytesize=8, baudrate=115200,
                                         timeout=0.01)
        with self.modbus_comms_lock:
            connected = self.client.connect() and self.client.socket is not None
        if connected:
            fcntl.flock(self.client.socket, fcntl.LOCK_EX)
        else:
            self.__log.warn('Unable to connect to {}'.format(device))
        return connected

    def disconnect(self):
        """Close connection"""
        with self.modbus_comms_lock:
            if self.client.socket is not None:
                fcntl.flock(self.client.socket, fcntl.LOCK_UN)
            self.client.close()

    def send_command(self, dev_id, data):
        """
            Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of
            each variable depends on the Gripper model (see support.robotiq.com for more details)
        """
        # make sure data has an even number of elements
        if len(data) % 2 == 1:
            data.append(0)

        # Initiate message as an empty list
        message = []

        # Fill message by combining two bytes in one register
        for i in range(0, len(data) / 2):
            message.append((data[2 * i] << 8) + data[2 * i + 1])

        # To do!: Implement try/except
        with self.modbus_comms_lock:
            self.client.write_registers(0x03E8, message, unit=dev_id)
        return True

    def get_status(self, dev_id, num_bytes):
        """
            Sends a request to read, wait for the response and returns the Gripper status. The method gets the number
            of bytes to read as an argument
        """
        num_regs = int(ceil(num_bytes / 2.0))

        # To do!: Implement try/except
        # Get status from the device
        with self.modbus_comms_lock:
            response = self.client.read_holding_registers(0x07D0, num_regs, unit=dev_id)

        if response is None:
            return None
        # Instantiate output as an empty list
        output = []

        # Fill the output with the bytes in the appropriate order
        for i in range(0, num_regs):
            output.append((response.getRegister(i) & 0xFF00) >> 8)
            output.append(response.getRegister(i) & 0x00FF)

        # Output the result
        return output
