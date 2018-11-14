"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
import ready_logging
import rospy
from robotiq_c_model_control.robotiq_gripper_action_interface import RobotiqGripperActionInterface
from std_msgs.msg import Empty
from threading import (
    Lock,
    Thread
)


@ready_logging.logged
class RobotiqGripper(object):
    """ Interface to the gripper as required by the ROS node executable. """

    def __init__(self, name, comms):
        self.grip_interface = RobotiqGripperActionInterface(name, comms)

        self.terminate_lock = Lock()
        self.terminate = False
        self.refresh_thread = Thread(target=self.refresh_loop, name='{} refresh loop'.format(name))

        self.watchdog_pub = rospy.Publisher('{}/watchdog'.format(name), Empty, queue_size=1)

    @property
    def resetting(self):
        """ Is the gripper currently in reset? """
        return self.grip_interface.resetting

    def refresh_loop(self):
        """
        Continually query the state of the state of the gripper. If a fault is
        detected, reset the gripper and its calibration.
        """
        acquisition_rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Check to see if the thread needs to stop
            with self.terminate_lock:
                if self.terminate:
                    self.__log.info('{} terminated'.format(self.refresh_thread.name))
                    return

            try:
                self.grip_interface.refresh_status()
                self.watchdog_pub.publish(Empty())
                acquisition_rate.sleep()
            except Exception as exc:
                msg = '{} failed: {}'.format(self.refresh_thread.name, exc)
                self.__log.error(msg)
                rospy.signal_shutdown(msg)
                return

    def start(self):
        """ Start the ROS interfaces. """
        self.grip_interface.start()
        self.refresh_thread.start()
        self.grip_interface.reset_gripper()

    def shutdown(self):
        """ Shutdown the ROS interfaces. """
        # Notify the thread to shutdown and wait for it to shutdown
        with self.terminate_lock:
            self.terminate = True
        try:
            self.refresh_thread.join()
        except RuntimeError as exc:
            self.__log.error('{} join failed: {}'.format(self.refresh_thread.name, exc))

        # Shutdown ROS topics
        self.grip_interface.shutdown()
        self.watchdog_pub.unregister()
