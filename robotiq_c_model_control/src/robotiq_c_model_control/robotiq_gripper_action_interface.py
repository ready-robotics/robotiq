import time
import rospy
import ready_logging
from threading import (
    Lock,
    Thread
)
from copy import deepcopy
from robotiq_c_model_control.msg import (
    GripperGoal,
    GripperAction,
    GripperResult,
    CModel_robot_output
)
from robotiq_c_model_control.base_c_model import BaseCModel
from plc_interface.base_action_server import BaseActionServer

fault_text = {'/0x05': 'Action delayed, activation (reactivation) must be completed prior to renewed action.',
              '/0x07': 'The activation bit must be set prior to action.',
              '/0x08': 'Maximum operating temperature exceeded, wait for cool-down.',
              '/0x0A': 'Under minimum operating voltage.',
              '/0x0B': 'Automatic release in progress.',
              '/0x0C': 'Internal processor fault.',
              '/0x0D': 'Activation fault, verify that no interference or other error occurred.',
              '/0x0E': 'Overcurrent triggered.',
              '/0x0F': 'Automatic release completed.',
              }


@ready_logging.logged
class RobotiqGripperActionInterface(BaseCModel):
    has_goal_ = False
    interrupted_ = False
    resetting_ = False
    has_goal_lock = Lock()
    interrupt_lock = Lock()
    resetting_lock = Lock()

    @property
    def has_goal(self):
        with self.has_goal_lock:
            return self.has_goal_

    @has_goal.setter
    def has_goal(self, on_goal):
        with self.has_goal_lock:
            self.has_goal_ = on_goal

    @property
    def interrupted(self):
        with self.interrupt_lock:
            return self.interrupted_

    @interrupted.setter
    def interrupted(self, interrupt):
        with self.interrupt_lock:
            self.interrupted_ = interrupt

    @property
    def resetting(self):
        with self.resetting_lock:
            return self.resetting_

    @resetting.setter
    def resetting(self, in_reset):
        with self.resetting_lock:
            self.resetting_ = in_reset

    def __init__(self):
        super(RobotiqGripperActionInterface, self).__init__()
        self.gripper_as_ = None
        self.goal_thread = None
        self.last_goal_wait = False
        self.reset_thread = None
        self.max_closed = 255
        self.max_open = 3

    def initialize_as(self):
        self.gripper_as_ = BaseActionServer('/robotiq_gripper_action', GripperAction, self.goal_cb, auto_start=False)
        self.gripper_as_.start()

    def shutdown_as(self):
        if self.gripper_as_:
            if self.has_goal:
                self.interrupted = True
                if self.goal_thread:
                    try:
                        self.status_cv.notify_all()
                        self.goal_thread.join(3.0)
                    except RuntimeError:
                        pass
            self.gripper_as_.cleanup()
        self.gripper_as_ = None

    def goal_cb(self, goal_handle):
        """
            This cb handles all gripper requests from chiron, whether it be from a node or the robot_interface of the
            robot control panel.
            We first complete checks to verify if we can execute the request. We adjust the position depending on the
            calibration and then execute. If wait is true, we wait until the actual gripper position is within 3 values
            of the requested position before returning success. We return failure only if the communication fails, the
            command is interrupted, or rospy is shutdown.
        Args:
            :param goal_handle: A Robotiq Gripper Action goal handle. The goal is of type GripperGoal and has the
                following fields:
                    direction: This field has three options: CUSTOM - use value from the position field, OPEN, or CLOSE
                    position: The position of the gripper (0: open, 255: closed)
                    force: The stall torque of the gripper while moving (0: no re-grasping , 255: max force)
                    auto_release: Activate auto release on robot emergency stop
                    auto_release_direction: The direction of auto release, either OPEN or CLOSE
                    wait: Wait for current request to finish before executing a new request
            :type goal_handle: ServerGoalHandle
        """
        if self.has_goal and self.last_goal_wait:
            goal_handle.set_rejected(text='Not Accepting Multiple Goals at Once')
            return
        elif self.resetting:
            goal_handle.set_rejected(text='Gripper is Currently Resetting')
            return
        elif self.has_goal and not self.last_goal_wait:
            # Preempting previous goal
            self.interrupted = True
            if self.goal_thread:
                self.goal_thread.join()

        self.has_goal = True
        goal_handle.set_accepted()
        position = self.map_position_to_calibration(goal_handle.get_goal())
        with self.status_cv:
            self.status_cv.wait(0.25)
            status = deepcopy(self.last_status)
        if abs(status.gPO - position) < 3:
            result = GripperResult()
            result.result_code = result.SUCCESS
            result.position = status.gPO
            result.object_detection = status.gOBJ
            goal_handle.set_succeeded(result, text='Already at goal')
            self.has_goal = False
            self.last_goal_wait = False
        else:
            self.goal_thread = Thread(target=self.on_goal, name='Robotiq Goal Execution', args=(goal_handle,))
            self.goal_thread.start()

    def on_goal(self, gh):
        goal = gh.get_goal()
        goal.position = self.map_position_to_calibration(goal)
        result = self.send_gripper_command(goal)
        if not result:
            result = self.generate_failure_result()
            gh.set_succeeded(result, text='Communication Timed Out With Gripper')
            self.has_goal = False
            self.last_goal_wait = False
            return

        self.interrupted = False
        if goal.wait:
            self.last_goal_wait = True
            # Wait for motion
            while not rospy.is_shutdown() and not self.interrupted:
                with self.status_cv:
                    self.status_cv.wait(0.25)
                    status = deepcopy(self.last_status)
                if status.gOBJ == 0 or abs(status.gPO - goal.position) < 3:
                    break

            if rospy.is_shutdown():
                result = self.generate_failure_result()
                gh.set_succeeeded(result, text='Failure - ROS shutdown')
                self.has_goal = False
                self.last_goal_wait = False
                return

            # Wait until we reach the desired position
            while not rospy.is_shutdown() and not self.interrupted:
                with self.status_cv:
                    self.status_cv.wait(0.25)
                    status = deepcopy(self.last_status)
                if status.gOBJ != 0 or abs(status.gPO - goal.position) < 3:
                    break

            with self.status_cv:
                self.status_cv.wait(0.25)
                status = deepcopy(self.last_status)
            result = self.generate_failure_result()
            if self.interrupted or rospy.is_shutdown():
                gh.set_aborted(result, text=self.get_fault_text(int(status.gFLT)))
            else:
                result.result_code = result.SUCCESS
                gh.set_succeeded(result, text='Succeeded')
        else:
            result = GripperResult()
            result.result_code = result.SUCCESS
            gh.set_succeeded(result)
        self.has_goal = False
        self.last_goal_wait = False

    def map_position_to_calibration(self, goal):
        """
            This function maps the gripper request to an actionable position. If direction is set to OPEN or CLOSE, the
            position is set to the calibrated max_open or max_closed positions. If direction is set to CUSTOM, the
            requested positional value is checked to ensure it is within the calibration bounds.
        Args:
            :param cmd: The requested position and direction to move the gripper.
            :type cmd: OpenRequest
        Returns:
            :param pos: The calibrated position to move the gripper. Values can range from 0 to 255
            :type pos: int
        """
        if goal.direction == goal.OPEN:
            pos = self.max_open
        elif goal.direction == goal.CLOSE:
            pos = self.max_closed
        elif goal.position > self.max_closed:
            pos = self.max_closed
        elif goal.position < self.max_open:
            pos = self.max_open
        else:
            pos = int(goal.position)

        return pos

    def generate_failure_result(self):
        with self.status_cv:
            self.status_cv.wait(0.25)
            status = deepcopy(self.last_status)
        result = GripperResult()
        result.position = status.gPO
        result.object_detection = status.gOBJ
        result.result_code = result.FAILURE
        return result

    def reset_gripper(self):
        # If the gripper is currently running a goal we must stop it
        self.resetting = True
        self.reset_thread = Thread(target=self.reset, name='Robotiq Gripper Reset Thread')
        self.reset_thread.start()

    def reset(self):
        # Reset the gripper during startup or after a fault. A specific order of states must be sent to the gripper.
        # We recalibrate after reset to be thorough.

        # If we are currently executing a goal, terminate it
        if self.has_goal:
            self.interrupted = True
            if self.goal_thread:
                self.goal_thread.join()

        # Lock out the action server
        self.has_goal = True

        # First check if we were previously initialized
        with self.status_cv:
            self.status_cv.wait(0.25)
            status = deepcopy(self.last_status)
        if status.gACT == 1:
            # Deactive Gripper
            goal = CModel_robot_output()
            goal.rACT = 0
            self.send_gripper_command(goal, parse=False)
            while not rospy.is_shutdown() and not self.interrupted:
                with self.status_cv:
                    self.status_cv.wait(0.25)
                    status = deepcopy(self.last_status)
                if status.gSTA == 0:
                    break

            if rospy.is_shutdown():
                self.resetting = False
                self.has_goal = False
                return

        # Active Gripper
        goal = CModel_robot_output()
        goal.rACT = 1
        self.send_gripper_command(goal, parse=False)
        while not rospy.is_shutdown() and not self.interrupted:
            with self.status_cv:
                self.status_cv.wait(0.25)
                status = deepcopy(self.last_status)
            if status.gSTA == 3:
                break

        if rospy.is_shutdown():
            self.resetting = False
            self.has_goal = False
            return

        rospy.sleep(1.0)
        # Get Actual Calibrated Values
        goal = GripperGoal()
        goal.force = 255
        goal.direction = goal.CLOSE
        goal.auto_release = goal.DISABLED
        if not self.send_gripper_command(goal):
            self.has_goal = False
            self.resetting = False
            rospy.signal_shutdown('Failed Close Calibration')
            return

        rospy.sleep(1.25)
        with self.status_cv:
            self.status_cv.wait(0.25)
            self.max_closed = self.last_status.gPO

        goal = GripperGoal()
        goal.force = 255
        goal.direction = goal.OPEN
        goal.auto_release = goal.DISABLED
        if not self.send_gripper_command(goal):
            self.has_goal = False
            self.resetting = False
            rospy.signal_shutdown('Failed Open Calibration')
            return

        rospy.sleep(1.25)
        with self.status_cv:
            self.status_cv.wait(0.25)
            self.max_open = self.last_status.gPO

        self.has_goal = False
        self.resetting = False

    def parse_cmd(self, goal):
        cmd = CModel_robot_output()

        # Auto Release
        if goal.auto_release == goal.ENABLED:
            cmd.rATR = 1
            cmd.rADR = 1 if goal.auto_release_direction == goal.OPEN else 0
        else:
            cmd.rATR = 0

        # Set active and go to
        cmd.rACT = 1
        cmd.rGTO = 1

        # Position
        cmd.rPR = self.map_position_to_calibration(goal)

        # Speed - Always set to max for now
        cmd.rSP = 255

        # Force
        if goal.force > 255:
            cmd.rFR = 255
        elif goal.force < 0:
            cmd.rFR = 0
        else:
            cmd.rFR = int(goal.force)

        self.command_pub.publish(cmd)
        return cmd

    def send_gripper_command(self, goal, parse=True):
        try:
            if parse:
                goal = self.parse_cmd(goal)
            return self.send_command(goal)
        except Exception as exc:
            self.__log.err('Error While Sending Command [{}] - Shutting Down'.format(exc))
            rospy.signal_shutdown('Error While Sending Command - Shutting Down')
            return False

    @staticmethod
    def get_fault_text(fault_code):
        if fault_code in fault_text:
            return fault_text[fault_code]
        else:
            return 'Unknown Error Code.'
