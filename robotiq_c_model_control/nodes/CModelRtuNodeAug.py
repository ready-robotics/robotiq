#!/usr/bin/env python

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

"""@package docstring
ROS node for controling a Robotiq C-Model gripper using the Modbus RTU protocol.

The script takes as an argument the IP address of the gripper. It initializes a baseCModel object and adds a
comModbusTcp client to it. It then loops forever, reading the gripper status and updating its command.
The gripper status is published on the 'CModelRobotInput' topic using the 'CModel_robot_input' msg type.
The node subscribes to the 'CModelRobotOutput' topic for new commands using the 'CModel_robot_output' msg type.
Examples are provided to control the gripper (CModelSimpleController.py) and interpreting its status
(CModelStatusListener.py).
"""

import rospy
import robotiq_c_model_control.baseCModelAug
import robotiq_modbus_rtu.comModbusRtu
import fcntl
from bondpy import bondpy  # this is the local version not the ros version
from std_msgs.msg import Empty
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg
from robotiq_c_model_control.msg import _CModel_augmented_robot_output as outputMsg


def mainLoop(devices):
    # Gripper is a C-Model with a TCP connection
    gripper = robotiq_c_model_control.baseCModelAug.robotiqBaseCModelAug()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    rospy.init_node('robotiqCModel', anonymous=True)
    # The Gripper status is published on the topic named 'CModelRobotInput'
    pub = rospy.Publisher('CModelRobotInput', inputMsg.CModel_robot_input, queue_size=1)
    watchdog_pub = rospy.Publisher('/robotiq_85mm_gripper/watchdog', Empty, queue_size=1)
    # The Gripper command is received from the topic named 'CModelRobotOutput'
    rospy.Subscriber('CModelRobotOutput', outputMsg.CModel_augmented_robot_output, gripper.refreshCommand)
    bond = bondpy.Bond('/robotiq_85mm_gripper', 'robotiq_85mm_gripper')
    # bond.on_broken = bond.shutdown
    bond.start()

    status = None
    connected = False
    for device in devices:
        # We connect to the address received as an argument
        try:
            flag = gripper.client.connectToDevice(device)
            if flag is False:
                raise Exception('Could Not Connect To Device!')
            fcntl.flock(gripper.client.client.socket, fcntl.LOCK_EX)
        except Exception as e:
            rospy.logwarn('Cannot connect to gripper on this port: {}'.format(e))
            gripper.client.disconnectFromDevice()
            continue
        rospy.logwarn('Device[{}] Connection: [{}]'.format(device, flag))
        try:
            status = gripper.getStatus()
        except AttributeError as ae:
            rospy.logwarn('Tried to connect to the wrong port: {}'.format(ae))
            fcntl.flock(gripper.client.client.socket, fcntl.LOCK_UN)
            gripper.client.disconnectFromDevice()
            continue

        # No exceptions means we connected to a device
        connected = True
        break

    # Even if we cannot connect to a device we still want to connect and break the bond so the UI doesn't wait forever
    if not rospy.is_shutdown():
        if status is not None:
            pub.publish(status)
        watchdog_pub.publish(Empty())
        bond.break_bond()

    while not rospy.is_shutdown() and connected:
        try:
            status = gripper.getStatus()
            watchdog_pub.publish(Empty())
            pub.publish(status)

            # Send the most recent command
            gripper.sendCommand()
        except Exception as exc:
            print(exc)

    # Release lock on shutdown
    if connected:
        fcntl.flock(gripper.client.client.socket, fcntl.LOCK_UN)

    return bond


if __name__ == '__main__':
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
    try:
        bond = mainLoop(ports)
        if not bond.is_shutdown:
            bond.shutdown()
    except Exception as exc:
        print('Main loop died: {}'.format(exc))
