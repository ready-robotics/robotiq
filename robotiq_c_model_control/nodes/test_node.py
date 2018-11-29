"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
import rospy
from actionlib.simple_action_client import SimpleActionClient
from robotiq_c_model_control.msg import (
    GripperAction,
    GripperGoal
)
from ready_debug_tools import ipdb_helpers

def main():
    rospy.init_node('dual_gripper_test_node')

    grip_a_ac = SimpleActionClient('/robotiq_grip_a/grip_action', GripperAction)
    grip_b_ac = SimpleActionClient('/robotiq_grip_b/grip_action', GripperAction)
    grip_a_ac.wait_for_server()
    grip_b_ac.wait_for_server()

    close_goal = GripperGoal()
    close_goal.direction = close_goal.CLOSE
    close_goal.auto_release = close_goal.DISABLED
    close_goal.wait = False

    open_goal = GripperGoal()
    open_goal.direction = close_goal.OPEN
    open_goal.auto_release = close_goal.DISABLED
    open_goal.wait = False

    close_goal_wait = GripperGoal()
    close_goal_wait.direction = close_goal.CLOSE
    close_goal_wait.auto_release = close_goal.DISABLED
    close_goal_wait.wait = True

    open_goal_wait = GripperGoal()
    open_goal_wait.direction = close_goal.OPEN
    open_goal_wait.auto_release = close_goal.DISABLED
    open_goal_wait.wait = True

    grip_a_ac.send_goal_and_wait(close_goal)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(close_goal)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    rospy.sleep(rospy.Duration(.1))

    grip_a_ac.send_goal_and_wait(open_goal)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(open_goal)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(close_goal_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(open_goal_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_b_ac.send_goal_and_wait(open_goal_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(close_goal_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(open_goal_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    gran_close_wait = GripperGoal()
    gran_close_wait.direction = gran_close_wait.CUSTOM
    gran_close_wait.position = 200
    gran_close_wait.auto_release = close_goal.DISABLED
    gran_close_wait.wait = True

    gran_open_wait = GripperGoal()
    gran_open_wait.direction = gran_open_wait.CUSTOM
    gran_open_wait.position = 20
    gran_open_wait.auto_release = open_goal.DISABLED
    gran_open_wait.wait = True

    gran_close_no_wait = GripperGoal()
    gran_close_no_wait.direction = gran_close_no_wait.CUSTOM
    gran_close_no_wait.position = 100
    gran_close_no_wait.auto_release = close_goal.DISABLED
    gran_close_no_wait.wait = True

    gran_open_no_wait = GripperGoal()
    gran_open_no_wait.direction = gran_open_no_wait.CUSTOM
    gran_open_no_wait.position = 50
    gran_open_no_wait.auto_release = open_goal.DISABLED
    gran_open_no_wait.wait = True

    grip_a_ac.send_goal_and_wait(gran_close_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(gran_close_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(gran_open_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(gran_open_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(gran_close_no_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(gran_close_no_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(gran_open_no_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(gran_open_no_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(open_goal_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)
    grip_b_ac.send_goal_and_wait(open_goal_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(close_goal_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)

    grip_b_ac.send_goal_and_wait(close_goal_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

    grip_a_ac.send_goal_and_wait(open_goal_wait)
    result = grip_a_ac.get_result()
    rospy.loginfo(result)

    grip_b_ac.send_goal_and_wait(open_goal_wait)
    result = grip_b_ac.get_result()
    rospy.loginfo(result)

    raw_input("Press enter to continue...")

if __name__ == '__main__':
    main()

