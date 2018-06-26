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
from robotiq_c_model_control.srv import GetCalibrationParametersResponse
from robotiq_c_model_control.base_c_model import BaseCModel
from plc_interface.base_action_server import BaseActionServer

def generate_grip_result(grip_status, result_code=GripperResult.SUCCESS):
    result = GripperResult()
    result.position = grip_status.gPO
    result.object_detection = grip_status.gOBJ
    result.result_code = result_code
    return result

class RobotiqCommandTimeout(object):
    """Track command timeouts."""
    def __init__(self, seconds):
        """Create a command timer.

        Args:
            :param seconds: timeout duration in seconds
            :type seconds: float
        """
        super(RobotiqCommandTimeout, self).__init__()
        self.start_time = rospy.get_rostime()
        self.duration = rospy.Duration(seconds)

    def start(self):
        """Start the timer."""
        self.start_time = rospy.get_rostime()

    def expired(self):
        """Check if the timer has expired.

        Returns:
            True if timeout is expired
        """
        return rospy.get_rostime() - self.start_time > self.duration


@ready_logging.logged
class RobotiqGoalRunner(Thread):
    WF_START = 0
    WF_FINISH = 1

    def __init__(self, goal_handle, robotiq, target_pos):
        self._interrupt_lock = Lock()
        self._interrupted = False
        super(RobotiqGoalRunner, self).__init__(target=self.do_goal, name='Robotiq Goal Runner', args=(goal_handle, robotiq, target_pos))

    def preempt(self):
        with self._interrupt_lock:
            self._interrupted = True
        self.join()

    @property
    def interrupted(self):
        with self._interrupt_lock:
            return self._interrupted

    def is_running(self):
        return not rospy.is_shutdown() and not self.interrupted

    def do_goal(self, goal_handle, robotiq, target_pos):
        self.__log.info('starting to do goal')
        state = self.WF_START
        timer = RobotiqCommandTimeout(seconds=3.0)
        timer.start()

        while self.is_running():
            status = robotiq.wait_for_next_status()

            if self.WF_START == state:
                if status.gOBJ != 3:
                    timer.start()
                    self.__log.info('wait for finish')
                    state = self.WF_FINISH
            elif self.WF_FINISH == state:
                if status.gOBJ != 0:
                    self.__log.info('wrapping up')
                    grip_result = generate_grip_result(status, GripperResult.SUCCESS)
                    robotiq.goal_finished(goal_handle, grip_result, 'Object detected')
                    return

            if abs(status.gPO - target_pos) < 3:
                self.__log.info('position reached')
                grip_result = generate_grip_result(status, GripperResult.SUCCESS)
                robotiq.goal_finished(goal_handle, grip_result, 'Reached target position')
                return
            elif timer.expired():
                self.__log.err('Goal timeout w/ status: "{}"'.format(str(status)))
                grip_result = generate_grip_result(status, GripperResult.SUCCESS)
                robotiq.goal_finished(goal_handle, grip_result, 'Timeout!')
                return

        self.__log.info('goal aborted')
        grip_result = generate_grip_result(GripperResult.FAILURE)
        robotiq.goal_aborted(goal_handle, grip_result, 'Aborted with fault "{}"'.format(status.get_fault_text()))

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
            if self.goal_thread:
                self.goal_thread.preempt()
            self.gripper_as_.cleanup()
        self.gripper_as_ = None

    def wait_for_next_status(self, seconds=0.25):
        with self.status_cv:
            self.status_cv.wait(seconds)
            return deepcopy(self.last_status)

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
        if self.resetting:
            goal_handle.set_rejected(text='Gripper is Currently Resetting')
            return

        if self.has_goal:
            if self.last_goal_wait:
                goal_handle.set_rejected(text='Not Accepting Multiple Goals at Once')
                return

            # Preempt the pending goal
            if self.goal_thread:
                self.goal_thread.preempt()

        self.has_goal = True
        goal_handle.set_accepted()
        goal = goal_handle.get_goal()
        goal.position = self.map_position_to_calibration(goal)

        status = self.wait_for_next_status()
        if abs(status.gPO - goal.position) < 3:
            self.__log.info('already at target')
            result = GripperResult()
            result.result_code = result.SUCCESS
            result.position = status.gPO
            result.object_detection = status.gOBJ
            self.goal_finished(goal_handle, result, 'Already at goal')
            return

        if not self.send_gripper_command(goal):
            self.__log.err('Could not send position command.')
            status = self.wait_for_next_status()
            result = GripperResult()
            result.result_code = result.FAILURE
            result.position = status.gPO
            result.object_detection = status.gOBJ
            self.goal_finished(goal_handle, result, 'Could not send command!')
            return
            
        if goal.wait:
            self.last_goal_wait = True
            self.goal_thread = RobotiqGoalRunner(goal_handle, self, goal.position)
            self.goal_thread.start()
        else:
            result = GripperResult()
            result.result_code = result.SUCCESS
            self.goal_finished(goal_handle, result, 'Not waiting for the goal to complete.')

    def goal_finished(self, goal_handle, grip_result, text):
        self.__log.info('goal finished')
        goal_handle.set_succeeded(grip_result, text)
        self.set_goal_done()

    def goal_aborted(self, goal_handle, grip_result, text):
        self.__log.info('goal aborted')
        goal_handle.set_aborted(grip_result, text)
        self.set_goal_done()

    def map_position_to_calibration(self, goal):
        """
            This function maps the gripper request to an actionable position. If direction is set to OPEN or CLOSE, the
            position is set to the calibrated max_open or max_closed positions. If direction is set to CUSTOM, the
            requested positional value is checked to ensure it is within the calibration bounds.
        Args:
            :param goal: The requested position and direction to move the gripper.
            :type goal: GripperGoal
        Returns:
            :rparam pos: The calibrated position to move the gripper. Values can range from 0 to 255
            :rtype pos: int
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

    def set_goal_done(self):
        """Set flags indicating that there is no pending goal."""
        self.has_goal = False
        self.last_goal_wait = False

    def reset_gripper(self):
        # If the gripper is currently running a goal we must stop it
        self.resetting = True
        self.reset_thread = Thread(target=self.reset, name='Robotiq Gripper Reset Thread')
        self.reset_thread.start()

    def do_calibration_move(self, direction=GripperGoal.OPEN, wait_seconds=1.25):
        """Do a calibration command.

        Send a calibration command to the gripper and wait for the next status after completion.

        Args:
            :param direction: gripper open/close
            :type direction GripperGoal direction
            :param wait_seconds: time in seconds before getting a status update.
            :type wait_seconds: float
        Returns:
            True if command sent correctly.
        """
        goal = GripperGoal()
        goal.force = 255
        goal.direction = direction
        goal.auto_release = goal.DISABLED
        sent = self.send_gripper_command(goal)
        if sent:
            rospy.sleep(wait_seconds)
            status = self.wait_for_next_status()
            if direction == GripperGoal.OPEN:
                self.max_open = status.gPO
            elif direction == GripperGoal.CLOSE:
                self.max_closed = status.gPO
        return sent

    def reset(self):
        # Reset the gripper during startup or after a fault. A specific order of states must be sent to the gripper.
        # We recalibrate after reset to be thorough.

        # If we are currently executing a goal, terminate it
        if self.goal_thread:
            self.goal_thread.preempt()

        # Lock out the action server
        self.has_goal = True

        # First check if we were previously initialized
        status = self.wait_for_next_status()
        if status.gACT == 1:
            # Deactive Gripper
            goal = CModel_robot_output()
            goal.rACT = 0
            self.send_gripper_command(goal, parse=False)

            # Wait for the gripper to deactivate
            timer = RobotiqCommandTimeout(seconds=3.0)
            timer.start()
            while not rospy.is_shutdown() and not self.interrupted:
                status = self.wait_for_next_status()
                if status.gSTA == 0:
                    break
                if timer.expired():
                    self.__log.err('Timeout on deactivate w/ status: "{}"'.format(str(status)))
                    rospy.signal_shutdown('Failed to Deactivate Gripper -- timeout')
                    break

            if rospy.is_shutdown():
                self.resetting = False
                self.has_goal = False
                return

        # Active Gripper
        goal = CModel_robot_output()
        goal.rACT = 1
        self.send_gripper_command(goal, parse=False)

        # Wait for gripper to activate
        timer = RobotiqCommandTimeout(seconds=3.0)
        timer.start()
        while not rospy.is_shutdown() and not self.interrupted:
            status = self.wait_for_next_status()
            if status.gSTA == 3:
                break
            if timer.expired():
                self.__log.err('Timeout on activate w/ status: "{}"'.format(str(status)))
                rospy.signal_shutdown('Failed to Activate Gripper -- timeout')
                break

        if rospy.is_shutdown():
            self.resetting = False
            self.has_goal = False
            return

        rospy.sleep(1.0)

        if not self.do_calibration_move(GripperGoal.CLOSE, wait_seconds=1.25):
            self.has_goal = False
            self.resetting = False
            self.__log.err('Could not send calibration close command.')
            rospy.signal_shutdown('Failed Close Calibration')
            return

        if not self.do_calibration_move(GripperGoal.OPEN, wait_seconds=1.25):
            self.has_goal = False
            self.resetting = False
            self.__log.err('Could not send calibration open command.')
            rospy.signal_shutdown('Failed Open Calibration')
            return

        self.has_goal = False
        self.resetting = False

    def get_calibration_parameters(self, request):
        response = GetCalibrationParametersResponse()
        response.max_close = self.max_closed
        response.max_open = self.max_open
        return response

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
