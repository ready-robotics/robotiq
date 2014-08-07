#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from robotiq_c_model_control.msg import _CModel_gripper_command  as commandMsg
from robotiq_c_model_control.msg import _CModel_gripper_state  as stateMsg

from robotiq_c_model_control.srv import *

class CModelControl():

    def __init__(self):
        rospy.init_node('c_gripper_control',anonymous=True)

        self.open_state_ = False
        self.close_state_ = False

        self.gripper_cmd_pub = rospy.Publisher("gripper_command",commandMsg.CModel_gripper_command)
        self.gripper_state_sub = rospy.Subscriber('gripper_state',stateMsg.CModel_gripper_state,self.gripper_state_cb)
        # self.open_button_sub = rospy.Subscriber("open",Bool, self.open_cb)
        # self.close_button_sub = rospy.Subscriber("close",Bool, self.close_cb)
        # self.toggle_sub = rospy.Subscriber("toggle",Bool, self.toggle_cb)

        self.open_service = rospy.Service("/robotiq_c_model_control/Open",Open,self.service_open)
        rospy.logwarn('C-Model Gripper Service Layer: Interfaces Initialized')
        
        self.last_gripper_state = None
        self.state = 'RESET'

        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(.01)
            pass

    def gripper_state_cb(self,msg):
        self.last_gripper_state = msg

    def service_open(self,req):
        if req.wait == True:
            if req.state == True: # Open Gripper
                if not self.state == 'OPEN':
                    self.open()
                    while not self.last_gripper_state.in_motion:
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(.1)
                    while self.last_gripper_state.in_motion:
                        rospy.loginfo('in motion')
                        rospy.sleep(.1)
                    return 'DONE - OPEN'
                else:
                    return 'FAILED - ALREADY OPEN'
            else: # Close Gripper
                if not self.state == 'CLOSED':
                    self.close()
                    while not self.last_gripper_state.in_motion:
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(.1)
                    while self.last_gripper_state.in_motion:
                        rospy.loginfo('in motion')
                        rospy.sleep(.1)
                    return 'DONE - CLOSED'
                else:
                    return 'FAILED - ALREADY CLOSED'
        else:
            if req.state == True: # Open Gripper
                if not self.state == 'OPEN':
                    self.open()
                    return 'DONE - OPEN'
                else:
                    return 'FAILED - ALREADY OPEN'
            else: # Close Gripper
                if not self.state == 'CLOSED':
                    self.close()
                    return 'DONE - CLOSED'
                else:
                    return 'FAILED - ALREADY CLOSED'

    def open(self):
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

    def close(self):
        bmsg = commandMsg.CModel_gripper_command()
        bmsg.reset = False
        bmsg.activate = True
        bmsg.open = False
        bmsg.close = True
        bmsg.release = False
        bmsg.request_pos = 0
        bmsg.speed = 255
        bmsg.force = 255
        self.gripper_cmd_pub.publish(bmsg)
        self.state = 'CLOSED'

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