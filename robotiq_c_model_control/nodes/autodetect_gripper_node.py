#!/usr/bin/env python
"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
import errno
import rospy
import sys
from robotiq_c_model_control.attachment_session import AttachmentSession
from robotiq_c_model_control.constants import (
    MODBUS,
    ORIGINAL
)
from robotiq_c_model_control.gripper_driver import (
    SerialGripperDriver,
    TeachmateGripperDriver
)

def main():
    """ Launch the ROS node. """
    rospy.init_node('robotiq_auto_gripper')

    teachmate_type = rospy.get_param('/teachmate_configuration/type', ORIGINAL)
    driver = TeachmateGripperDriver() if teachmate_type == MODBUS else SerialGripperDriver()

    rospy.loginfo('Attempting to autodetect a gripper')

    # AttachmentSession synchronizes the launching of this attachment with
    # ready_runtime_manager. The attachment manager will not progress until the
    # detection is complete.
    with AttachmentSession('/robotiq_single_gripper', 'robotiq_single_gripper'):
        if not driver.autodetect(find_count=1):
            rospy.logerr('Failed to detect a gripper')
            sys.exit(errno.ENXIO)

        driver.start()

    rospy.spin()

if __name__ == '__main__':
    main()
