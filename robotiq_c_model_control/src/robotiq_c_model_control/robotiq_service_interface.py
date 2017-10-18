import time
import rospy
import ready_logging
from threading import (
    Lock,
    Thread
)
from copy import deepcopy
from robotiq_c_model_control.msg import (
    CModel_robot_output
)
from robotiq_c_model_control.srv import (
    Open,
    OpenResponse
)
from robotiq_c_model_control.base_c_model import BaseCModel


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
class RobotiqServiceInterface(BaseCModel):
    """
        This is the main class for interfacing with the Robotiq Gripper. It calibrates the gripper and also resets the
        gripper after faults. It is also an intermediate that translates requests from chiron and send corresponding
        commands to the gripper.
    """
    has_goal_ = False
    interrupted_ = False
    resetting_ = False
    has_goal_lock = Lock()
    interrupt_lock = Lock()
    resetting_lock = Lock()

    # In this class, all locks were implemented as properties. I actually like this paradigm better.
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
        super(RobotiqServiceInterface, self).__init__()
        self.gripper_service = None
        self.last_goal_wait = False
        self.reset_thread = None
        # The gripper register for position ranges from 0 to 255. In reality the range is smaller. This values are
        # populated correctly after calibration or a reset.
        self.max_closed = 255
        self.max_open = 3

    def initialize_service(self):
        self.gripper_service = rospy.Service('/robotiq_c_model_control/Open', Open, self.gripper_cb)

    def shutdown_gripper_service(self):
        if self.gripper_service:
            if self.has_goal:
                # If we are shutting down, interrupt the last goal and then notify all waiting threads so they can
                # shutdown.
                self.interrupted = True
                # Use python time since rospy may be dead
                start_time = time.time()
                while self.has_goal and time.time() - start_time < 3.0:
                    time.sleep(0.1)
                try:
                    self.status_cv.notify_all()
                except RuntimeError:
                    # If we notify when we can't
                    pass
            self.gripper_service.shutdown()
        self.gripper_service = None

    def gripper_cb(self, req):
        """
            This cb handles all gripper requests from chiron, whether it be from a node or the robot_interface of the
            robot control panel.

            We first complete checks to verify if we can execute the request. We adjust the position depending on the
            calibration and then execute. If wait is true, we wait until the actual gripper position is within 3 values
            of the requested position before returning success. We return failure only if the communication fails, the
            command is interrupted, or rospy is shutdown.

        Args:
            :param req: A Robotiq gripper request. The request has the following fields:
                direction: This field has three options: CUSTOM - use value from the position field, OPEN, or CLOSE
                position: The position of the gripper (0: open, 255: closed)
                force: The stall torque of the gripper while moving (0: no re-grasping , 255: max force)
                wait: Wait for current request to finish before executing a new request
            :type req: OpenRequest

        Returns:
            :param resp:
            :type resp: OpenResponse
        """
        resp = OpenResponse()
        if self.has_goal and self.last_goal_wait:
            resp.ack = 'FAILED - WAITING FOR PREVIOUS REQUEST TO FINISH'
            return resp
        elif self.resetting:
            resp.ack = 'FAILED - GRIPPER IS CURRENTLY RESETTING'
            return resp
        elif self.has_goal and not self.last_goal_wait:
            # Preempting previous goal
            self.interrupted = True
            start_time = rospy.Time.now()
            while self.has_goal and (rospy.Time.now() - start_time).to_sec() < 3.0:
                rospy.sleep(0.1)

        self.has_goal = True
        # Map the position to values within the calibration
        position = self.map_position_to_calibration(req)
        # Check if we are already at the goal
        with self.status_cv:
            self.status_cv.wait(0.25)
            status = deepcopy(self.last_status)
        if abs(status.gPO - position) < 3:
            resp.ack = 'SUCCEEDED - ALREADY AT REQUESTED POSITION'
            self.has_goal = False
            self.last_goal_wait = False
            return resp

        # Populate Command
        cmd = CModel_robot_output()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0  # No auto-releasing
        cmd.rSP = 255  # Always set speed to max b/c Robotiq is slow
        cmd.rPR = position
        if req.force > 255:
            cmd.rFR = 255
        elif req.force < 1:
            cmd.rFR = 1
        else:
            cmd.rFR = int(req.force)

        result = self.send_gripper_command(cmd)
        if not result:
            resp.ack = 'FAILED - COMMUNICATION TIMED OUT WITH GRIPPER'
            self.has_goal = False
            self.last_goal_wait = False
            return resp

        self.interrupted = False
        if req.wait:
            self.last_goal_wait = True
            # Wait for motion
            start_time = rospy.Time.now()
            # Prevent infinite blocking incase or position check has too much noise
            while not rospy.is_shutdown() and not self.interrupted and (rospy.Time.now() - start_time).to_sec() < 0.2:
                with self.status_cv:
                    self.status_cv.wait(0.25)
                    status = deepcopy(self.last_status)
                if status.gOBJ == 0 or abs(status.gPO - position) < 3:
                    break

            if rospy.is_shutdown():
                resp.ack = 'FAILED - ROS SHUTDOWN'
                self.has_goal = False
                self.last_goal_wait = False
                return resp

            # Wait until we reach the desired position
            while not rospy.is_shutdown() and not self.interrupted:
                with self.status_cv:
                    self.status_cv.wait(0.25)
                    status = deepcopy(self.last_status)
                if status.gOBJ != 0 or abs(status.gPO - position) < 3:
                    break

            with self.status_cv:
                self.status_cv.wait(0.25)
                status = deepcopy(self.last_status)
            if self.interrupted or rospy.is_shutdown():
                resp.ack = 'FAILED - {}'.format(self.get_fault_text(int(status.gFLT)).upper())
            else:
                resp.ack = 'SUCCEEDED - REACHED POSITION'
        else:
            resp.ack = 'SUCCEEDED'
        self.has_goal = False
        self.last_goal_wait = False
        return resp

    def map_position_to_calibration(self, cmd):
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
        if cmd.direction == cmd.OPEN:
            pos = self.max_open
        elif cmd.direction == cmd.CLOSE:
            pos = self.max_closed
        elif cmd.pos > self.max_closed:
            pos = self.max_closed
        elif cmd.pos < self.max_open:
            pos = self.max_open
        else:
            pos = int(cmd.pos)

        return pos

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
            start_time = rospy.Time.now()
            while self.has_goal and (rospy.Time.now() - start_time).to_sec() < 3.0:
                rospy.sleep(0.1)

        # Lock out the action server
        self.has_goal = True

        # First check if we were previously initialized
        with self.status_cv:
            self.status_cv.wait(0.25)
            status = deepcopy(self.last_status)
        if status.gACT == 1:
            # Deactive Gripper
            cmd = CModel_robot_output()
            cmd.rACT = 0
            self.send_gripper_command(cmd)
            # Ensure we are in the correct state before continuing
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
        cmd = CModel_robot_output()
        cmd.rACT = 1
        self.send_gripper_command(cmd)
        # Ensure we are in the correct state before continuing
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
        cmd = CModel_robot_output()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0
        cmd.rFR = 255
        cmd.rSP = 255
        cmd.rPR = self.max_closed
        if not self.send_gripper_command(cmd):
            self.has_goal = False
            self.resetting = False
            rospy.signal_shutdown('Failed Close Calibration')
            return

        rospy.sleep(1.25)
        with self.status_cv:
            self.status_cv.wait(0.25)
            self.max_closed = self.last_status.gPO

        cmd = CModel_robot_output()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0
        cmd.rFR = 255
        cmd.rSP = 255
        cmd.rPR = self.max_open
        if not self.send_gripper_command(cmd):
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

    def send_gripper_command(self, cmd):
        # Send a command to the gripper. The command is sent through pymodbus to over modbus to the gripper.
        try:
            sent_resp = self.send_command(cmd)
            self.command_pub.publish(cmd)
        except Exception as exc:
            self.__log.err('Error While Sending Command [{}] - Shutting Down'.format(exc))
            rospy.signal_shutdown('Error While Sending Command - Shutting Down')
            return False
        return sent_resp

    @staticmethod
    def get_fault_text(fault_code):
        if fault_code in fault_text:
            return fault_text[fault_code]
        else:
            return 'Unknown Error Code.'
