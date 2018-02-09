#!/usr/bin/env python
import rospy
import fcntl
import ready_logging
from threading import (
    Lock,
    Thread,
    Condition
)
from bondpy import bondpy
from std_msgs.msg import Empty
from robotiq_c_model_control.msg import (
    CModel_robot_input,
    CModel_robot_output
)
from robotiq_c_model_control.srv import GetCalibrationParameters
from robotiq_modbus_rtu.com_modbus_rtu import Communication
from robotiq_modbus_rtu.single_com_modbus_rtu import SingleCommunication
from robotiq_c_model_control.robotiq_gripper_action_interface import RobotiqGripperActionInterface

MODBUS = 'modbus'


@ready_logging.logged
class RobotiqGripper(RobotiqGripperActionInterface):
    def __init__(self, teachmate):
        super(RobotiqGripper, self).__init__()
        if teachmate == MODBUS:
            self.comms = Communication()
        else:
            self.comms = SingleCommunication()
        self.status_lock = Lock()
        self.last_status = None
        self.initialized = False
        self.watchdog_pub = None
        self.command_pub = None
        self.gripper_state_pub = None
        self.main_thread = None
        self.get_calibration_parameters_srv = None
        self.status_cv = Condition(self.status_lock)
        rospy.on_shutdown(self.shutdown)

    def initialize(self, teachmate):
        self.initialized = self.initialize_device(teachmate)
        if not self.initialized:
            self.__log.err('Failed to Initialize Device')
            rospy.signal_shutdown('Could not connect to device')
            return False
        self.initialize_comms()
        return True

    def initialize_device(self, teachmate):
        """
            This function connections the Robotiq device. Depending on the teachmate selected it either connects to
            a port or tests the already established connection for connectivity.
            Args:
                :param teachmate: The type of teachmate, this is specified in the ready_launch config file
                    for each robot
            Returns:
                :param connected: A flag for if a connection could be established
                :type connected: bool
        """
        connected = False
        bond = bondpy.Bond('/robotiq_85mm_gripper', 'robotiq_85mm_gripper')
        bond.on_broken = bond.shutdown
        bond.start()
        if teachmate != MODBUS:
            for device in ['/dev/ttyUSB0', '/dev/ttyUSB1']:
                # We connect to the address received as an argument
                try:
                    flag = self.comms.connect_to_device(device)
                    if flag is False:
                        continue
                    if self.comms.client.socket is not None:
                        fcntl.flock(self.comms.client.socket, fcntl.LOCK_EX)
                except Exception as e:
                    self.__log.warn('Cannot connect to gripper on this port: {}'.format(e))
                    if self.comms.client.socket is not None:
                        fcntl.flock(self.comms.client.socket, fcntl.LOCK_UN)
                    self.comms.disconnect_from_device()
                    continue
                self.__log.warn('Device[{}] Connection: [{}]'.format(device, flag))
                try:
                    status = self.get_status()
                    if status is not None:
                        with self.status_cv:
                            self.last_status = status
                            self.status_cv.notify_all()
                        connected = True
                        break
                except AttributeError as ae:
                    self.__log.warn('Tried to connect to the wrong port: {}'.format(ae))
                    if self.comms.client.socket is not None:
                        fcntl.flock(self.comms.client.socket, fcntl.LOCK_UN)
                    self.comms.disconnect_from_device()
        else:
            self.comms.connect_to_device()
            try:
                status = self.get_status()
                if status is not None:
                    with self.status_cv:
                        self.last_status = status
                        self.status_cv.notify_all()
                    connected = True
            except AttributeError as ae:
                self.__log.warn('Tried to connect to the wrong port: {}'.format(ae))
                self.comms.disconnect_from_device()
        bond.break_bond()
        return connected

    def initialize_comms(self):
        self.watchdog_pub = rospy.Publisher('/robotiq_85mm_gripper/watchdog', Empty, queue_size=1)
        self.command_pub = rospy.Publisher('/robotiq_input_command', CModel_robot_output, queue_size=1)
        self.gripper_state_pub = rospy.Publisher('/robotiq_state', CModel_robot_input, queue_size=1)
        self.initialize_as()

    def advertise_calibration_service(self):
        self.get_calibration_parameters_srv = rospy.Service('/robotiq/get_calibration_parameters',
                                                            GetCalibrationParameters, self.get_calibration_parameters)

    def start(self):
        self.main_thread = Thread(target=self.main, name='Robotiq Main Thread')
        self.main_thread.start()
        self.reset_gripper()

    def main(self):
        """
            Continually query the state of the state of the gripper. If a fault is detected, reset the gripper and its
            calibration.
        """
        acquisition_rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.initialized:
            try:
                status = self.get_status()
                if status is not None:
                    with self.status_cv:
                        self.gripper_state_pub.publish(status)
                        self.last_status = status
                        self.status_cv.notify_all()
                    if status.gFLT != 0 and not self.resetting:
                        self.__log.info('RESETTING Robotiq Gripper')
                        self.reset_gripper()
                self.watchdog_pub.publish(Empty())
                acquisition_rate.sleep()
            except Exception as exc:
                self.__log.error('CModelRtuNode: {}'.format(exc))
                break

        if not rospy.is_shutdown():
            rospy.signal_shutdown('Exception Occurred')
            return

    def shutdown(self):
        if self.initialized:
            if self.main_thread.isAlive():
                self.initialized = False
                try:
                    self.main_thread.join(2.0)
                except RuntimeError:
                    pass
            self.shutdown_as()
            if self.watchdog_pub is not None:
                self.watchdog_pub.unregister()
            if self.command_pub is not None:
                self.command_pub.unregister()
            if self.gripper_state_pub is not None:
                self.gripper_state_pub.unregister()
            if self.get_calibration_parameters_srv is not None:
                self.get_calibration_parameters_srv.shutdown()
            if isinstance(self.comms, SingleCommunication):
                if self.comms.client.socket is not None:
                    fcntl.flock(self.comms.client.socket, fcntl.LOCK_UN)
            self.comms.disconnect_from_device()


if __name__ == '__main__':
    rospy.init_node('Robotiq_Gripper')
    teachmate_type = rospy.get_param('/robot_configuration/teachmate_type', 'original')
    gripper = RobotiqGripper(teachmate_type)
    if gripper.initialize(teachmate_type):
        gripper.start()
        # Wait for the first reset and calibration to finish
        while not rospy.is_shutdown() and gripper.resetting:
            rospy.sleep(0.05)
        gripper.advertise_calibration_service()
    rospy.spin()
