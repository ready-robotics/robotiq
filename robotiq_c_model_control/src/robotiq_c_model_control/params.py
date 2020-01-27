#!/usr/bin/env python
"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
import rospy
from robotiq_c_model_control.constants import DEFAULT_MODBUS_IDS


def get_supported_modbus_ids():
    return rospy.get_param('/robotiq/gripper/supported_modbus_ids', DEFAULT_MODBUS_IDS)
