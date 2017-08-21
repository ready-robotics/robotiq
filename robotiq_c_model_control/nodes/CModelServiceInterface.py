#!/usr/bin/env python
import rospy
from threading import Lock
from std_msgs.msg import String, Empty
from robotiq_c_model_control.msg import _CModel_gripper_command as commandMsg
from robotiq_c_model_control.msg import _CModel_gripper_state as stateMsg
from robotiq_c_model_control.srv import *


class CModelControl():

    def __init__(self):
        rospy.init_node('c_gripper_control', anonymous=True)

        self.open_state_ = False
        self.close_state_ = False

        self.gripper_cmd_pub = rospy.Publisher('gripper_command', commandMsg.CModel_gripper_command, queue_size=1)
        self.gripper_state_sub = rospy.Subscriber('gripper_state', stateMsg.CModel_gripper_state, self.gripper_state_cb,
                                                  queue_size=1)

        self.open_service = rospy.Service('/robotiq_c_model_control/Open', Open, self.service_open)
        rospy.logwarn('C-Model Gripper Service Layer: Interfaces Initialized')

        self.state_pub = rospy.Publisher('/gripper_state', String, queue_size=1)
        self.last_update = rospy.Time.now()
        self.last_update_lock = Lock()
        self.watchdog_sub = rospy.Subscriber('/robotiq_85mm_gripper/watchdog', Empty, self.on_watchdog_update)

        # TODO: Need to check if the com layer comes up to see if the gripper service should work
        self.last_gripper_state = None
        self.state = 'RESET'
        self.reported_state = 'RESET'
        self.initialized = False

        while not rospy.is_shutdown():
            with self.last_update_lock:
                last_update = self.last_update
            if (rospy.Time.now() - last_update).to_sec() > 2.0:
                rospy.signal_shutdown('Did not receive update from Robotiq gripper')
            self.state_pub.publish(self.reported_state)
            rospy.sleep(.01)

    def on_watchdog_update(self, msg):
        with self.last_update_lock:
            self.last_update = rospy.Time.now()

    def gripper_state_cb(self, msg):
        self.last_gripper_state = msg
        if msg.fault != 0:
            self.reset()
            self.initialized = False
            rospy.sleep(0.5)

        if not self.initialized:  # or msg.status != 3:
            # Initialize and Open
            self.initialize()
            rospy.sleep(1.5)
            self.open()
            rospy.sleep(0.5)
            self.reported_state = 'OPEN'
            self.initialized = True

    def service_open(self, req):
        if self.last_gripper_state is None:
            return 'FAILED - NO GRIPPER DATA'

        if req.wait is True:
            if req.direction == req.OPEN:  # Open Gripper
                if not self.state == 'OPEN':
                    self.open()
                    while not self.last_gripper_state.in_motion and self.last_gripper_state.at_requested_pos:
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(0.1)
                    while self.last_gripper_state.in_motion:
                        rospy.loginfo('in motion')
                        self.reported_state = 'OPENING'
                        rospy.sleep(0.1)
                    rospy.sleep(0.4)
                    self.reported_state = 'OPEN'
                    return 'DONE - OPEN'
                else:
                    return 'SUCCEEDED - ALREADY OPEN'
            elif req.direction == req.CLOSE:  # Close Gripper
                if not self.state == 'CLOSED':
                    self.close()
                    while not self.last_gripper_state.in_motion and self.last_gripper_state.at_requested_pos:
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(0.1)
                    while self.last_gripper_state.in_motion:
                        rospy.loginfo('in motion')
                        self.reported_state = 'CLOSING'
                        rospy.sleep(0.1)
                    rospy.sleep(0.4)
                    self.reported_state = 'CLOSED'
                    return 'DONE - CLOSED'
                else:
                    return 'SUCCEEDED - ALREADY CLOSED'
            else:
                # The gripper may not close or open to the actual limits
                if req.pos > 229:
                    req.pos = 229
                elif req.pos < 3:
                    req.pos = 3
                if self.last_gripper_state.current_pos != int(req.pos):
                    self.custom(req.pos, req.force)
                    while not self.last_gripper_state.in_motion and self.last_gripper_state.current_pos != int(req.pos):
                        rospy.loginfo('waiting for motion')
                        rospy.sleep(0.1)
                    while self.last_gripper_state.in_motion and self.last_gripper_state.current_pos != int(req.pos):
                        rospy.loginfo('in motion')
                        self.reported_state = 'AT CUSTOM POSITION'
                        rospy.sleep(0.1)
                    rospy.sleep(0.4)
                    return 'DONE'
                else:
                    return 'SUCCEEDED - ALREADY AT POSITION'
        else:
            if req.direction is req.OPEN:  # Open Gripper
                if not self.state == 'OPEN':
                    self.open()
                    return 'DONE - OPEN'
                else:
                    return 'SUCCEEDED - ALREADY OPEN'
            elif req.direction == req.CLOSE:  # Close Gripper
                if not self.state == 'CLOSED':
                    self.close()
                    return 'DONE - CLOSED'
                else:
                    return 'SUCCEEDED - ALREADY CLOSED'
            else:
                # Custom position, we just go there
                self.custom(req.pos, req.force)
                return 'DONE'

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
        self.reported_state = 'OPENING'

    def custom(self, pos, force):
        bmsg = commandMsg.CModel_gripper_command()
        bmsg.reset = False
        bmsg.activate = False
        bmsg.open = False
        bmsg.close = False
        bmsg.release = False
        bmsg.request_pos = int(pos)
        bmsg.speed = 255
        bmsg.force = force
        self.gripper_cmd_pub.publish(bmsg)
        self.state = 'CUSTOM'
        self.reported_state = 'CUSTOM'

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
        self.reported_state = 'CLOSING'

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
