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
    SerialGripperDriver,
    TeachmateGripperDriver
)

def main():
    """ Launch a ROS node controlling 2 grippers. """
    rospy.init_node('robotiq_dual_gripper')

    teachmate_type = rospy.get_param('/teachmate_configuration/type', ORIGINAL)
    driver = TeachmateGripperDriver() if teachmate_type == MODBUS else SerialGripperDriver()

    modbus_ids = [9, 15]

    id_str = ' '.join(str(id) for id in modbus_ids)
    rospy.loginfo('Attempting to detect grippers: {}'.format(id_str))

    if not driver.detect(modbus_ids):
        rospy.logerr('Failed to detect a gripper')
        exit(-1)

    driver.start()
    rospy.spin()

if __name__ == '__main__':
    main()
