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

    while not rospy.is_shutdown():
        self.update()
        rospy.sleep(.01)
        pass

  def gripper_state_cb(self,msg):
    self.last_gripper_state = msg

  def service_open(self,req):
    if req.wait == True:
        if req.state == True: # Open Gripper
            self.open()
            while not self.last_gripper_state.in_motion:
                rospy.sleep(.01)
            while self.last_gripper_state.in_motion:
                rospy.sleep(.01)
            return 'DONE'
        else: # Close Gripper
            self.close()
            while not self.last_gripper_state.in_motion:
                rospy.sleep(.01)
            while self.last_gripper_state.in_motion:
                rospy.sleep(.01)
            return 'DONE'
            # tries = 0
            # done = False
            # while not done:
            #     if tries > 2: break
            #     s = self.last_gripper_state.current_pos
            #     if not self.last_gripper_state.in_motion:
            #         self.open()
            #         rospy.sleep(.05)
            #         self.close()
            #         rospy.sleep(.1)
            #         tries+=1
    else:
        pass

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


  # def open_cb(self,msg):
  #   self.open_state_ = msg.data
  #   if self.open_state_ == True:
  #       self.close_state_ = False
  #   pass

  # def close_cb(self,msg):
  #   self.close_state_ = msg.data
  #   if self.close_state_ == True:
  #       self.open_state_ = False
  #   pass

  # def toggle_cb(self,msg):
  #   if msg.data == True: # pressed but not released
  #       if self.open_state_ == False and self.close_state_ == False:
  #           self.open_state_ = True
  #           self.close_state_ = False
  #       elif self.open_state_ == True and self.close_state_ == False:
  #           self.open_state_ = False
  #           self.close_state_ = True
  #       elif self.open_state_ == False and self.close_state_ == True:
  #           self.open_state_ = True
  #           self.close_state_ = False

  def update(self):
    # print 'open: ' + str(self.open_state_) + ' -- close: ' + str(self.close_state_)
    pass
    # bmsg = commandMsg.CModel_gripper_command()

    # if self.open_state_ == True and self.close_state_ == False:
    #     bmsg.reset = False
    #     bmsg.activate = True
    #     bmsg.open = True
    #     bmsg.close = False
    #     bmsg.release = False
    #     bmsg.request_pos = 0
    #     bmsg.speed = 255
    #     bmsg.force = 255
    #     self.gripper_cmd_pub.publish(bmsg)
    # elif self.open_state_ == False and self.close_state_ == True:
    #     bmsg.reset = False
    #     bmsg.activate = True
    #     bmsg.open = False
    #     bmsg.close = True
    #     bmsg.release = False
    #     bmsg.request_pos = 0
    #     bmsg.speed = 255
    #     bmsg.force = 255
    #     self.gripper_cmd_pub.publish(bmsg)
    # pass



if __name__ == '__main__':
  device = CModelControl()
