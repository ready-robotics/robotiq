"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
from string import ascii_lowercase

def gripper_name_generator():
    """
    Generate valid gripper names

        This generator should probably be used in tandem with zip() to ensure
        that output is safely cut short when valid names run out.
    """
    return ('/robotiq_grip_{}'.format(ch) for ch in ascii_lowercase)
