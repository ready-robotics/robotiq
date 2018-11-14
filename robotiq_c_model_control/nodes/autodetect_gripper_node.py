"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
from robotiq_c_model_control.constants import (
    MODBUS,
    ORIGINAL
)
from robotiq_c_model_control.gripper_driver import (
    SerialGripperNode,
    TeachmateGripperNode
)

def main():
    """ Launch the ROS node. """
    rospy.init_node('robotiq_auto_gripper')

    teachmate_type = rospy.get_param('/teachmate_configuration/type', ORIGINAL)
    node = TeachmateGripperNode() if teachmate_type == MODBUS else SerialGripperNode()

    rospy.loginfo('Attempting to autodetect a gripper')
    if not node.autodetect(find_count=1):
        rospy.logerr('Failed to detect a gripper')
        exit(-1)

    node.start()
    rospy.spin()

if __name__ == '__main__':
    main()
