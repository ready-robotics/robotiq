#!/usr/bin/env python
# Copyright 2018 by READY Robotics Corporation.
# All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
# use or modify this software without first obtaining a license from the READY Robotics Corporation.
import errno
import rospy
from robotiq_c_model_control.attachment_session import AttachmentSession
from robotiq_c_model_control.constants import (
    MODBUS,
    ORIGINAL
)
from robotiq_c_model_control.gripper_driver import TeachmateGripperDriver
from robotiq_c_model_control.params import get_supported_modbus_ids


def main():
    """ Launch the ROS node. """
    rospy.init_node('robotiq_auto_gripper')

    driver = TeachmateGripperDriver()

    modbus_ids = get_supported_modbus_ids()
    id_str = ', '.join(str(id) for id in modbus_ids)
    rospy.loginfo('Supported MODBUS IDs: {}'.format(id_str))

    # AttachmentSession synchronizes the launching of this attachment with
    # ready_runtime_manager. The attachment manager will not progress until the
    # detection is complete.
    bond_topic = rospy.get_param('~bond_topic', 'unspecified_bond_topic')
    num_grippers = rospy.get_param('~num_grippers', 1)
    with AttachmentSession('/{}'.format(bond_topic), bond_topic):
        if not driver.autodetect(modbus_ids, find_count=num_grippers):
            rospy.logerr('Failed to detect a gripper')
            return errno.ENXIO

        driver.start()
        rospy.spin()


if __name__ == '__main__':
    main()
