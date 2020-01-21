"""
Copyright 2018 by READY Robotics Corporation.
All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
use or modify this software without first obtaining a license from the READY Robotics Corporation.
"""
import ready_logging
import rospy
from robotiq_c_model_control.base_c_model import BaseCModel
from robotiq_c_model_control.constants import (
    DEFAULT_DEVICE_IDS,
    MAX_GRIPPER_COUNT,
    MODBUS,
    ORIGINAL
)
from robotiq_c_model_control.robotiq_gripper import RobotiqGripper
from robotiq_c_model_control.util import gripper_name_generator
from robotiq_modbus_rtu.com_modbus_rtu import Communication
from robotiq_modbus_rtu.single_com_modbus_rtu import (
    SingleCommunication,
    SingleCommClient
)


@ready_logging.logged
class GripperDriver(object):
    """ Driver for running multiple Robotiq grippers on the same MODBUS. """
    @staticmethod
    def activate(grippers):
        """ Activate all the provided grippers simultaneously. """
        # Start gripper threads
        for gripper in grippers:
            gripper.start()

        # Wait for all gripper resets to complete.
        for gripper in grippers:
            while gripper.resetting:
                if rospy.is_shutdown():
                    break
                rospy.sleep(0.05)

    @staticmethod
    def build_grippers(clients):
        """ Build grippers with unique names using comm clients provided. """
        grippers = []
        for name, client in zip(gripper_name_generator(), clients):
            grippers.append(RobotiqGripper(name, client))
        return grippers

    def __init__(self):
        rospy.on_shutdown(self.shutdown)

    def detect(self, modbus_ids):
        """ Attempt to detect specified devices on the MODBUS. """
        return self._detect(modbus_ids)

    def _detect(self, modbus_ids):
        """ The child specific implementation of gripper detection. """
        raise NotImplementedError('Child must override _detect')

    def autodetect(self, supported_device_ids, find_count=None):
        """ Attempt to autodetect the number of grippers specified. """
        if find_count is None or \
           not 1 <= find_count <= MAX_GRIPPER_COUNT:
            find_count = MAX_GRIPPER_COUNT
        return self._autodetect(supported_device_ids, find_count)

    def _autodetect(self, supported_device_ids, find_count):
        """ The child specific implementation of gripper autodetection. """
        raise NotImplementedError('Child must override _autodetect')

    def _is_gripper_attached(self, dev_id, comms):
        """
        Attempt to query a gripper with MODBUS ID 'dev_id' using a given
        communication channel.
        """
        try:
            found = BaseCModel.read_comm_status(comms)
            if not found:
                self.__log.warn('Device {}: Could not be detected'.format(dev_id))
        except AttributeError as exc:
            self.__log.warn('Device {}: Could not be detected, {}'.format(dev_id, exc))
            found = False
        return found

    def start(self):
        """ Start the ROS node. """
        self._start()

    def _start(self):
        """ The child specific implementation for starting the node. """
        raise NotImplementedError('Child must override _start')

    def shutdown(self):
        """ Shutdown the ROS node. """
        self._shutdown()

    def _shutdown(self):
        """ The child specific implementation for starting the node. """
        raise NotImplementedError('Child must override _shutdown')


@ready_logging.logged
class SerialGripperDriver(GripperDriver):
    """ Driver for running multiple Robotiq grippers on the same USB to RS-485 converter. """
    DEVICES = ('/dev/ttyUSB0', '/dev/ttyUSB1')

    def __init__(self):
        super(SerialGripperDriver, self).__init__()
        self.grippers = []
        self.comm_channel = None

    def _detect(self, modbus_ids):
        return any(self._detect_grippers_with_dev(dev, modbus_ids) for dev in self.DEVICES)

    def _detect_grippers_with_dev(self, dev_path, modbus_ids):
        """
        Try to detect grippers using the USB to RS-485 converter at the
        specified path.
        """
        comm_channel = SingleCommunication()
        if not comm_channel.connect(dev_path):
            self.__log.warn('{}: Could not connect'.format(dev_path))
            return False

        self.__log.info('{}: Comms connected'.format(dev_path))

        # Scan for grippers attached at all dev_ids
        dev_clients = []
        all_detected = True
        for dev_id in modbus_ids:
            comm_client = SingleCommClient(dev_id, comm_channel)
            dev_clients.append(comm_client)
            if not self._is_gripper_attached(dev_id, comm_client):
                all_detected = False
                break

        if all_detected:
            self.__log.info('All grippers detected!')
            self.comm_channel = comm_channel
            self.grippers = self.build_grippers(dev_clients)
            return True
        else:
            comm_channel.disconnect()
            return False

    def _autodetect(self, supported_device_ids, find_count):
        return any(self._autodetect_grippers_with_dev(dev, supported_device_ids, find_count) for dev in self.DEVICES)

    def _autodetect_grippers_with_dev(self, dev_path, find_count):
        """
        Try to detect grippers using the USB to RS-485 converter at the
        specified path.
        """
        comm_channel = SingleCommunication()
        if not comm_channel.connect(dev_path):
            self.__log.warn('{}: Could not connect'.format(dev_path))
            return False

        self.__log.info('{}: Comms connected'.format(dev_path))

        # Scan for grippers attached at all dev_ids
        dev_clients = []
        for dev_id in VALID_DEVICE_IDS:
            comm_client = SingleCommClient(dev_id, comm_channel)
            if self._is_gripper_attached(dev_id, comm_client):
                dev_clients.append(comm_client)
                self.__log.info('Autodetected {}'.format(dev_id))
                find_count -= 1
                if find_count == 0:
                    break

        if find_count == 0:
            self.__log.info('Autodetect successful')
            self.comm_channel = comm_channel
            self.grippers = self.build_grippers(dev_clients)
            return True
        else:
            comm_channel.disconnect()
            return False

    def _start(self):
        if self.grippers:
            self.activate(self.grippers)

    def _shutdown(self):
        for gripper in self.grippers:
            gripper.shutdown()

        if self.comm_channel is not None:
            self.comm_channel.disconnect()


@ready_logging.logged
class TeachmateGripperDriver(GripperDriver):
    """ Driver for running multiple Robotiq grippers on the Teachmate MODBUS. """
    def __init__(self):
        super(TeachmateGripperDriver, self).__init__()
        self.grippers = []
        self.clients = []

    def _detect_gripper_with_comms(self, dev_id, comms):
        """
        Attempt to connect to a gripper using the specified communication
        channel.
        """
        if not comms.connect():
            self.__log.warn('Device {}: Could not connect'.format(dev_id))
            return False

        return self._is_gripper_attached(dev_id, comms)

    def _detect(self, modbus_ids):
        """
        Attempt to detect all the specified grippers.

            Each gripper communicates using its own unique client.
        """
        for dev_id in modbus_ids:
            comms = Communication(dev_id)
            self.clients.append(comms)
            if not self._detect_gripper_with_comms(dev_id, comms):
                return False

        self.__log.info('All grippers detected!')
        self.grippers = self.build_grippers(self.clients)
        return True

    def _autodetect(self, supported_device_ids, find_count):
        for dev_id in supported_device_ids:
            comms = Communication(dev_id)
            if self._detect_gripper_with_comms(dev_id, comms):
                self.clients.append(comms)
                self.__log.info('Autodetected {}'.format(dev_id))
                find_count -= 1
                if find_count == 0:
                    break
            else:
                comms.disconnect()

        if find_count == 0:
            self.__log.info('Autodetect successful')
            self.grippers = self.build_grippers(self.clients)
            return True
        return False

    def _start(self):
        if self.grippers:
            self.activate(self.grippers)

    def _shutdown(self):
        for gripper in self.grippers:
            gripper.shutdown()

        for comms in self.clients:
            comms.disconnect()
