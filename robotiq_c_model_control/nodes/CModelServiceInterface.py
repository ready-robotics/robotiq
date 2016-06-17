#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
import rospy

from std_msgs.msg import Bool, String

from robotiq_c_model_control.msg import _CModel_gripper_command  as commandMsg
from robotiq_c_model_control.msg import _CModel_gripper_state  as stateMsg

from robotiq_c_model_control.srv import *


class CModelControl():

    def __init__(self):
        rospy.init_node('c_gripper_control', anonymous=True)

        self.open_state_ = False
        self.close_state_ = False

        self.gripper_cmd_pub = rospy.Publisher("gripper_command", commandMsg.CModel_gripper_command)
        self.gripper_state_sub = rospy.Subscriber('gripper_state', stateMsg.CModel_gripper_state, self.gripper_state_cb,
                                                  queue_size=1)

        self.open_service = rospy.Service("/robotiq_c_model_control/Open", Open, self.service_open)
        rospy.logwarn('C-Model Gripper Service Layer: Interfaces Initialized')

        self.state_pub = rospy.Publisher("/gripper_state", String)
        
        self.last_gripper_state = None
        self.state = 'RESET'
        self.reported_state = 'RESET'
        self.initialized = False

        while not rospy.is_shutdown():
            self.update()
            self.state_pub.publish(self.reported_state)
            rospy.sleep(.01)
            pass

    def gripper_state_cb(self, msg):
        self.last_gripper_state = msg
        if msg.fault != 0:
            self.reset()
            self.initialized = False
            msg.status = 0
	    rospy.sleep(0.25)

        if not self.initialized:  # or msg.status != 3:
            # Initialize and Open
            self.initialize()
	    rospy.sleep(0.5)
            self.open()
            rospy.sleep(0.5)
            self.reported_state = 'OPEN'
            self.initialized = True

    def service_open(self, req):
        if req.wait is True:
            if req.state is True:  # Open Gripper
                if not self.state == 'OPEN':
                    self.open()
                    while not self.last_gripper_state.in_motion:
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(.1)
                    while self.last_gripper_state.in_motion:
                        rospy.loginfo('in motion')
                        self.reported_state = 'OPENING'
                        rospy.sleep(.1)
                    self.reported_state = 'OPEN'
                    return 'DONE - OPEN'
                else:
                    return 'FAILED - ALREADY OPEN'
            else:  # Close Gripper
                if not self.state == 'CLOSED':
                    self.close()
                    while not self.last_gripper_state.in_motion:
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(.1)
                    while self.last_gripper_state.in_motion:
                        rospy.loginfo('in motion')
                        self.reported_state = 'CLOSING'
                        rospy.sleep(.1)
                    self.reported_state = 'CLOSED'
                    return 'DONE - CLOSED'
                else:
                    return 'FAILED - ALREADY CLOSED'
        else:
            if req.state is True:  # Open Gripper
                if not self.state == 'OPEN':
                    self.open()
                    return 'DONE - OPEN'
                else:
                    return 'FAILED - ALREADY OPEN'
            else:  # Close Gripper
                if not self.state == 'CLOSED':
                    self.close()
                    return 'DONE - CLOSED'
                else:
                    return 'FAILED - ALREADY CLOSED'

    def open(self):
        bmsg = commandMsg.CModel_gripper_command()
        bmsg.reset = False
        bmsg.activate = False
        bmsg.open = True
        bmsg.close = False
        bmsg.release = False
        bmsg.request_pos = 0
        bmsg.speed = 255
        bmsg.force = 255
        self.gripper_cmd_pub.publish(bmsg)
        self.state = 'OPEN'
        self.reported_state = 'OPEN'

    def reset(self):
        bmsg = commandMsg.CModel_gripper_command()
        bmsg.reset = True
        bmsg.activate = False
        bmsg.open = True
        bmsg.close = False
        bmsg.release = False
        bmsg.request_pos = 0
        bmsg.speed = 255
        bmsg.force = 255
        self.gripper_cmd_pub.publish(bmsg)
        self.state = 'OPEN'
        self.reported_state = 'RESETTING'

    def initialize(self):
        bmsg = commandMsg.CModel_gripper_command()
        bmsg.reset = False
        bmsg.activate = True
        bmsg.open = True
        bmsg.close = False
        bmsg.release = False
        bmsg.request_pos = 0
        bmsg.speed = 255
        bmsg.force = 255
        self.gripper_cmd_pub.publish(bmsg)
        self.state = 'OPEN'
        self.reported_state = 'INITIALIZING'

    def close(self):
        bmsg = commandMsg.CModel_gripper_command()
        bmsg.reset = False
        bmsg.activate = False
        bmsg.open = False
        bmsg.close = True
        bmsg.release = False
        bmsg.request_pos = 0
        bmsg.speed = 255
        bmsg.force = 255
        self.gripper_cmd_pub.publish(bmsg)
        self.state = 'CLOSED'
        self.reported_state = 'CLOSED'

    def update(self):
        pass
if __name__ == '__main__':
    device = CModelControl()


# fault: 0
# reset: True
# standby: True
# object: False
# init: False
# activating: False
# status: 0
# at_requested_pos: False
# in_motion: True
# open_contact: False
# closed_contact: False
# current_pos: 13
# requested_pos: 0
# current: 0
# force: 0
# speed: 0
