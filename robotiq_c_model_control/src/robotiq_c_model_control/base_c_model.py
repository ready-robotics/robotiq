
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


from robotiq_c_model_control.msg import CModel_robot_input


class BaseCModel(object):
    """
        Base class (communication protocol agnostic) for sending commands and receiving the status of the
        Robotic C-Model gripper
    """

    def __init__(self):
        pass

    @staticmethod
    def verify_command(command):
        """
            Function to verify that the value of each variable satisfy its limits.
        """

        # Verify that each variable is in its correct range
        command.rACT = max(0, command.rACT)
        command.rACT = min(1, command.rACT)

        command.rGTO = max(0, command.rGTO)
        command.rGTO = min(1, command.rGTO)

        command.rATR = max(0, command.rATR)
        command.rATR = min(1, command.rATR)

        command.rADR = max(0, command.rADR)
        command.rADR = min(1, command.rADR)

        command.rPR = max(0, command.rPR)
        command.rPR = min(255, command.rPR)

        command.rSP = max(0, command.rSP)
        command.rSP = min(255, command.rSP)

        command.rFR = max(0, command.rFR)
        command.rFR = min(255, command.rFR)

        # Return the modified command
        return command

    def create_command(self, command):
        """Function to update the command which will be sent during the next sendCommand() call."""

        # Limit the value of each variable
        new_cmd = self.verify_command(command)

        # Initiate command as an empty list
        cmd = list()

        # Build the command with each output variable
        # To-Do: add verification that all variables are in their authorized range
        cmd.append(new_cmd.rACT + (new_cmd.rGTO << 3) + (new_cmd.rATR << 4) + (new_cmd.rADR << 5))
        cmd.append(0)
        cmd.append(0)
        cmd.append(new_cmd.rPR)
        cmd.append(new_cmd.rSP)
        cmd.append(new_cmd.rFR)
        return cmd

    def send_command(self, command):
        """ Send the command to the Gripper."""
        cmd = self.create_command(command)
        return self.comms.send_command(cmd)

    def get_status(self):
        """Request the status from the gripper and return it in the CModel_robot_input msg type."""

        # Acquire status from the Gripper
        status = self.comms.get_status(6)
        if not status:
            return None

        # Message to output
        message = CModel_robot_input()

        # Assign the values to their respective variables
        message.gACT = (status[0] >> 0) & 0x01
        message.gGTO = (status[0] >> 3) & 0x01
        message.gSTA = (status[0] >> 4) & 0x03
        message.gOBJ = (status[0] >> 6) & 0x03
        message.gFLT = status[2]  # & 0xF Documentation is unclear as whether we get the second half of the byte
        message.gPR = status[3]
        message.gPO = status[4]
        message.gCU = status[5]

        return message