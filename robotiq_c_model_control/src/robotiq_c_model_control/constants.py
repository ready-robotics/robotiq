"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
ORIGINAL = 'original'
MODBUS = 'modbus'

# The slave/device IDs where we expect to find grippers on the MODBUS
DEFAULT_DEVICE_IDS = (9, 15)

# The maximum number of grippers that can run simultaneously in this node
MAX_GRIPPER_COUNT = 2
