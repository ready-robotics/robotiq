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
# Modifed from the original comModbusTcp by Kelsey Hawkins @ Georgia Tech


import rospy
import ready_logging
from math import ceil
from threading import Lock
from teachmate.msg import (
    ReadRegistersGoal,
    ReadRegistersAction,
    WriteRegistersGoal,
    WriteRegistersAction
)
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from teachmate.modbus_constants import ROBOTIQ_ID


@ready_logging.logged
class Communication(object):
    def __init__(self):
        self.read_registers_ac = None
        self.write_registers_ac = None
        self.modbus_action_lock = Lock()

    def connect_to_device(self):
        self.read_registers_ac = SimpleActionClient('/teachmate_comms/read_registers', ReadRegistersAction)
        self.read_registers_ac.wait_for_server(rospy.Duration(0, 500000000))
        self.write_registers_ac = SimpleActionClient('/teachmate_comms/write_registers', WriteRegistersAction)
        self.write_registers_ac.wait_for_server(rospy.Duration(0, 500000000))

    def disconnect_from_device(self):
        """ Close connection """
        self.__log.err("Disconnecting Robotiq")
        self.stop_action_client(self.read_registers_ac)
        self.read_registers_ac = None
        self.stop_action_client(self.write_registers_ac)
        self.write_registers_ac = None

    @staticmethod
    def stop_action_client(ac):
        ac.cancel_all_goals()
        ac.action_client.pub_goal.unregister()
        ac.action_client.pub_cancel.unregister()

    def send_command(self, data):
        """
            Send a command to the Gripper - the method takes a list of uint8 as an argument.
            The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)
        """
        # make sure data has an even number of elements
        if len(data) % 2 == 1:
            data.append(0)

        # Initiate message as an empty list
        message = []

        # Fill message by combining two bytes in one register
        for i in range(0, len(data) / 2):
            message.append((data[2 * i] << 8) + data[2 * i + 1])

        if not self.write_registers_ac.wait_for_server(rospy.Duration(0, 600000000)):
            self.__log.err('Teachmate Modbus Communications could not be contacted!')
            return False

        request = WriteRegistersGoal()
        request.slave_id = ROBOTIQ_ID
        request.first_register = 0x03E8
        request.values = message
        with self.modbus_action_lock:
            self.write_registers_ac.send_goal(request)

            if not self.write_registers_ac.wait_for_result(rospy.Duration(1, 500000000)):
                self.__log.err('Timed Out While Sending Command')
                return False

            resp = self.write_registers_ac.get_result()
        if resp.result != resp.SUCCESS:
            self.__log.warn("Error Writing Registers")
            return False

        return True

    def get_status(self, num_bytes):
        """
            Sends a request to read, wait for the response and returns the Gripper status.
            The method gets the number of bytes to read as an argument
        """
        num_regs = int(ceil(num_bytes / 2.0))

        if not self.read_registers_ac.wait_for_server(rospy.Duration(1, 0)):
            self.__log.err('Teachmate Modbus Communications could not be contacted!')
            return None

        if self.read_registers_ac.get_state == GoalStatus.ACTIVE:
            self.__log.warn('Previous goal is stick executing: skipping request')
            return None

        request = ReadRegistersGoal()
        request.slave_id = ROBOTIQ_ID
        request.first_register = 0x07D0
        request.num_registers = num_regs
        with self.modbus_action_lock:
            self.read_registers_ac.send_goal(request)

            if not self.read_registers_ac.wait_for_result(rospy.Duration(1, 500000000)):
                self.__log.err('Timed Out on Response')
                return None

            resp = self.read_registers_ac.get_result()
        if resp.result != resp.SUCCESS:
            self.__log.warn('Error Reading Registers')
            return None

        # Instantiate output as an empty list
        output = []

        # Fill the output with the bytes in the appropriate order
        for i in xrange(len(resp.values)):
            output.append((resp.values[i] & 0xFF00) >> 8)
            output.append(resp.values[i] & 0x00FF)

        # Output the result
        return output