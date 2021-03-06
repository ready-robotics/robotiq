# Copyright 2018 by READY Robotics Corporation.
# All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
# use or modify this software without first obtaining a license from the READY Robotics Corporation.
import ready_logging
import rospy
from copy import copy
from plc_interface.base_action_server import BaseActionServer
from robotiq_c_model_control.base_c_model import BaseCModel
from robotiq_c_model_control.msg import (
    CModel_robot_input,
    CModel_robot_output,
    GripperAction,
    GripperGoal,
    GripperResult
)
from robotiq_c_model_control.srv import (
    GetRobotiqGripState,
    GetRobotiqGripStateResponse
)
from threading import (
    Condition,
    Lock,
    Thread
)

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
class RobotiqGripperROSInterface:
    """ ROS Interface for interacting with a Robotiq Gripper """
    has_goal_ = False
    interrupted_ = False
    resetting_ = False
    has_goal_lock = Lock()
    interrupt_lock = Lock()
    resetting_lock = Lock()

    @staticmethod
    def grip_status_to_str(status):
        """Convert Gripper response status into a string.

        Args:
            :param status: the status registers from the gripper.
            :type status: CModel_robot_input
        Return: str
        """
        return 'gACT:{}, gGTO:{}, gSTA:{}, gOBJ:{}, ' \
               'gFLT:{}, gPR:{}, gPO:{}, gCU:{}'.format(
                       status.gACT,
                       status.gGTO,
                       status.gSTA,
                       status.gOBJ,
                       status.gFLT,
                       status.gPR,
                       status.gPO,
                       status.gCU
                       )

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

    def __init__(self, name, comms):
        self.cmodel = BaseCModel(comms)
        self.name = name

        self.max_closed = 255
        self.max_open = 3

        self.goal_thread = None
        self.last_goal_wait = False
        self.reset_thread = None
        self.gripper_as_ = BaseActionServer('/robotiq_gripper_action', GripperAction, self.goal_cb, auto_start=False)

        self.reg_status = self.cmodel.get_status()
        self.reg_status_lock = Lock()
        self.reg_status_cv = Condition(self.reg_status_lock)

        self.grip_state = GetRobotiqGripStateResponse.UNKNOWN
        self.grip_state_lock_ = Lock()
        self.grip_state_srv_ = rospy.Service('/robotiq/get_grip_state', GetRobotiqGripState, self.handle_grip_state_service)

        self.command_pub = rospy.Publisher('/robotiq_input_command', CModel_robot_output, queue_size=1)
        self.reg_status_pub = rospy.Publisher('/robotiq_state', CModel_robot_input, queue_size=1)

    def start(self):
        self.gripper_as_.start()

    def shutdown(self):
        if self.has_goal:
            self.interrupted = True
            # Wake anyone waiting on updated status
            with self.reg_status_cv:
                self.reg_status_cv.notify_all()
            if self.goal_thread:
                try:
                    self.goal_thread.join(3.0)
                except RuntimeError:
                    pass

        self.gripper_as_.cleanup()

        with self.grip_state_lock_:
            self.grip_state_srv_.shutdown()

        self.command_pub.unregister()
        self.reg_status_pub.unregister()

    def refresh_status(self):
        """ Query the gripper status and update topics/state accordingly. """
        status = self.cmodel.get_status()
        if status is not None:
            self.reg_status_pub.publish(status)

            with self.reg_status_cv:
                self.reg_status = status
                self.reg_status_cv.notify_all()

            if status.gFLT != 0 and not self.resetting:
                self.__log.info('RESETTING Robotiq {}'.format(self.name))
                self.reset_gripper()
        return status

    def wait_for_next_status(self, seconds=0.25):
        with self.reg_status_cv:
            self.reg_status_cv.wait(seconds)
            return copy(self.reg_status)

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

        status = self.wait_for_next_status()
        if abs(status.gPO - position) < 3:
            result = self.generate_grip_result(status=status, result_code=GripperResult.SUCCESS)
            self.finish_goal(goal_handle, result, 'Already at goal')
        else:
            thread_name = '{} goal executor'.format(self.name)
            self.goal_thread = Thread(target=self.on_goal, name=thread_name, args=(goal_handle,))
            self.goal_thread.start()

    def on_goal(self, gh):
        goal = gh.get_goal()
        goal.position = self.map_position_to_calibration(goal)
        result = self.send_gripper_command(goal)
        if not result:
            self.__log.err('Could not send position command.')
            self.finish_failed_goal(gh, 'Communication Timed Out With Gripper')
            return

        self.interrupted = False
        if goal.wait:
            self.last_goal_wait = True

            # Check goal direction
            status = self.wait_for_next_status()
            if goal.position < status.gPO:
                # Gripper moving toward the open position
                is_in_motion = lambda status: status.gOBJ == 0
            else:
                # Gripper moving toward the closed position
                is_in_motion = lambda status: status.gOBJ == 0 or status.gOBJ == 2

            # Wait for motion
            timer = RobotiqCommandTimeout(seconds=3.0)
            timer.start()
            while not rospy.is_shutdown() and not self.interrupted:
                status = self.wait_for_next_status()
                if is_in_motion(status) or abs(status.gPO - goal.position) < 3:
                    break
                if timer.expired():
                    self.__log.err('Timeout on motion start w/ status: "{}"'.format(self.grip_status_to_str(status)))
                    self.finish_failed_goal(gh, 'Timeout waiting for motion to start')
                    return

            if rospy.is_shutdown():
                self.finish_failed_goal(gh, 'ROS shutdown')
                return

            # Wait until we reach the desired position
            timer = RobotiqCommandTimeout(seconds=3.0)
            timer.start()
            while not rospy.is_shutdown() and not self.interrupted:
                status = self.wait_for_next_status()
                if status.gOBJ != 0 or abs(status.gPO - goal.position) < 3:
                    break
                if timer.expired():
                    self.__log.err('Timeout on position goal w/ status: "{}"'.format(self.grip_status_to_str(status)))
                    self.finish_failed_goal(gh, 'Timeout waiting to reach desired position')
                    return

            status = self.wait_for_next_status()
            if self.interrupted or rospy.is_shutdown():
                self.abort_goal(gh, self.get_fault_text(int(status.gFLT)))
            else:
                result = self.generate_grip_result(status=status, result_code=GripperResult.SUCCESS)
                self.finish_goal(gh, result, 'Succeeded')
        else:
            result = GripperResult()
            result.result_code = result.SUCCESS
            self.finish_goal(gh, result)

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

    def generate_grip_result(self, result_code=GripperResult.SUCCESS, status=None):
        """Create a GripperResult with the next status update.

        Args:
            :param result_code: the Gripper goal result code.
            :type result_code: GripperResult code
        Return: GripperResult based on the next received status.
        """
        if status is None:
            status = self.wait_for_next_status()
        result = GripperResult()
        result.position = status.gPO
        result.object_detection = status.gOBJ
        if status.gGTO != 0:
            grabbed_obj = (status.gOBJ == 1 or status.gOBJ == 2)
            result.has_object = result.HAS_OBJ_YES if grabbed_obj else result.HAS_OBJ_NO
        else:
            result.has_object = result.HAS_OBJ_UKNOWN
        result.result_code = result_code
        return result

    def set_goal_done(self):
        """Set flags indicating that there is no pending goal."""
        self.has_goal = False
        self.last_goal_wait = False

    def finish_failed_goal(self, goal_handle, text=""):
        """Finish a goal that failed to complete.

        Args:
            :param goal_handle: handle to notify the ActionServer
            :type goal_handle: ServerGoalHandle
            :param text: optional text status to be passed back to the
                ActionServer
            :type text: str
        """
        result = self.generate_grip_result(GripperResult.FAILURE)
        self.finish_goal(goal_handle, result, text)

    def finish_goal(self, goal_handle, result, text=""):
        """Finish a goal being handled with a given result.

        Args:
            :param goal_handle: handle to notify the ActionServer
            :type goal_handle: ServerGoalHandle
            :param result: the result of the gripper action
            :type result: GripperResult
            :param text: optional text status to be passed back to the
                ActionServer.
            :type text: str
        """
        goal_handle.set_succeeded(result, text)
        self.set_goal_done()

    def abort_goal(self, goal_handle, text=""):
        """Abort a goal being handled with a given result.

        Args:
            :param goal_handle: handle to notify the ActionServer
            :type goal_handle: ServerGoalHandle
            :param text: optional text status to be passed back to the
                ActionServer.
            :type text: str
        """
        result = self.generate_grip_result(GripperResult.FAILURE)
        goal_handle.set_aborted(result, text)
        self.set_goal_done()

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
        if self.has_goal:
            self.interrupted = True
            if self.goal_thread:
                self.goal_thread.join()

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
                    self.__log.err('Timeout on deactivate w/ status: "{}"'.format(self.grip_status_to_str(status)))
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
                self.__log.err('Timeout on activate w/ status: "{}"'.format(self.grip_status_to_str(status)))
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

        return cmd

    def send_gripper_command(self, goal, parse=True):
        try:
            if parse:
                goal = self.parse_cmd(goal)
            self.command_pub.publish(goal)
            return self.cmodel.send_command(goal)
        except Exception as exc:
            self.__log.err('Error While Sending Command [{}] - Shutting Down'.format(exc))
            rospy.signal_shutdown('Error While Sending Command - Shutting Down')
            return False

    def handle_grip_state_service(self, request):
        with self.grip_state_lock_:
            resp = GetRobotiqGripStateResponse()
            if self.resetting:
                resp.state = resp.UNKNOWN
            else:
                registers = self.wait_for_next_status()
                moved = registers.gGTO != 0
                if moved and registers.gOBJ == 1:
                    # Fingers stopped due to contact while opening
                    resp.state = GetRobotiqGripStateResponse.OPEN
                elif moved and registers.gOBJ == 2:
                    # Fingers stopped due to contact while closing
                    resp.state = GetRobotiqGripStateResponse.CLOSED
                elif moved and registers.gOBJ == 3:
                    # Fingers moved to the requested position without detecting an
                    # object. Use the gripper position to determine the current
                    # grip state.
                    position = int(registers.gPO)
                    position_median = abs(self.max_closed - self.max_open) / 2
                    if position < position_median:
                        resp.state = GetRobotiqGripStateResponse.OPEN
                    else:
                        resp.state = GetRobotiqGripStateResponse.CLOSED
                else:
                    # Gripper is either moving or performing a reset. Use the last
                    # known state.
                    resp.state = self.grip_state

            self.grip_state = resp.state
        return resp

    @staticmethod
    def get_fault_text(fault_code):
        if fault_code in fault_text:
            return fault_text[fault_code]
        else:
            return 'Unknown Error Code.'
