# Copyright 2018 by READY Robotics Corporation.
# All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
# use or modify this software without first obtaining a license from the READY Robotics Corporation.
from bondpy import bondpy


class AttachmentSession(object):
    """
    A context that synchronizes the execution of the runtime system and this
    attachment using bondpy. Once the context is exited, the runtime system can
    proceed.
    """
    def __init__(self, topic, key):
        self.bond = bondpy.Bond(topic, key)

    def __enter__(self):
        self.bond.start()

    def __exit__(self, exc_type, exc_obj, exc_tb):
        self.bond.break_bond()
