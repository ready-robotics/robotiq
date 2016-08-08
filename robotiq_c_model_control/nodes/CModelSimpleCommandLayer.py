#!/usr/bin/env python
import roslib
import rospy
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg
from robotiq_c_model_control.msg import _CModel_augmented_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_gripper_command as commandMsg
from robotiq_c_model_control.msg import _CModel_gripper_state as stateMsg
roslib.load_manifest('robotiq_c_model_control')


class gripper_control():
    def __init__(self):
        self.commands_saved_ = False
        rospy.init_node('c_model_command_layer', anonymous=True)
        # Adjutant Feed Publishers
        self.output_pub = rospy.Publisher("CModelRobotOutput", outputMsg.CModel_augmented_robot_output, queue_size=1)
        self.state_pub = rospy.Publisher("gripper_state", stateMsg.CModel_gripper_state, queue_size=1)
        # Adjutant Raw Device Subscribers
        self.input_sub = rospy.Subscriber("CModelRobotInput", inputMsg.CModel_robot_input, self.input_cb)
        self.command_sub = rospy.Subscriber("gripper_command", commandMsg.CModel_gripper_command, self.command_cb)
        rospy.logwarn('C-Model Gripper Command Layer: Interfaces Initialized')
        # Start ROS Process
        rospy.spin()
        pass

    def input_cb(self, input_msg):
        print input_msg
        state = stateMsg.CModel_gripper_state()
        # Reset
        if input_msg.gACT == 0:
            state.reset = True
        else:
            state.reset = False

        # Standby
        if input_msg.gGTO == 0:
            state.standby = True
        else:
            state.standby = False

        # Status
        state.status = input_msg.gSTA
        if input_msg.gSTA == 1:
            state.activating = True
        else:
            state.activating = False

        # Object and At Requested Position
        if input_msg.gOBJ == 0:
            state.object = False
            state.open_contact = False
            state.closed_contact = False
            state.at_requested_pos = False
            state.in_motion = True
        elif input_msg.gOBJ == 1:
            state.object = True
            state.open_contact = True
            state.closed_contact = False
            state.at_requested_pos = False
            state.in_motion = False
        elif input_msg.gOBJ == 2:
            state.object = True
            state.open_contact = False
            state.closed_contact = True
            state.at_requested_pos = False
            state.in_motion = False
        elif input_msg.gOBJ == 3:
            state.object = False
            state.open_contact = False
            state.closed_contact = False
            state.at_requested_pos = True
            state.in_motion = False

        # Fault
        state.fault = input_msg.gFLT
        if input_msg.gFLT == 0x00:
            state.init = False
        elif input_msg.gFLT == 0x09:
            state.init = True

        # Position
        state.current_pos = input_msg.gPO
        # Requested Position
        state.requested_pos = input_msg.gPR
        # Force
        if self.commands_saved_ is True:
            state.force = self.saved_command_msg_.force
        else:
            state.force = 0
        # Current
        state.current = input_msg.gCU
        # Speed
        if self.commands_saved_ is True:
            state.speed = self.saved_command_msg_.speed
        else:
            state.speed = 0

        # Publish Message
        self.state_pub.publish(state)

    def command_cb(self, command_msg):
        output = outputMsg.CModel_augmented_robot_output()
        output.rATR = 0  # set release to normal

        # Auto Release
        if command_msg.release is True:
            output.rADR = 1
            output.rATR = 1

        if command_msg.reset is not True:
            # Initialize and Activate
            if command_msg.activate is True:
                output.rACT = 1
                output.rGTO = 1
            # Keep activated for action
            else:
                output.rACT = 1
                output.rGTO = 1
        # Reset
        else:
            output.rACT = 0
            output.rGTO = 1

        # Position
        if command_msg.open is True:
            output.rPR = 0
        elif command_msg.close is True:
            output.rPR = 255
        elif command_msg.request_pos > 255:
            output.rPR = 255
        elif command_msg.request_pos < 0:
            output.rPR = 0
        else:
            output.rPR = int(command_msg.request_pos)

        # Speed
        if command_msg.speed > 255:
            output.rSP = 255
        elif command_msg.speed < 0:
            output.rSP = 0
        else:
            output.rSP = int(command_msg.speed)

        # Force
        if command_msg.force > 255:
            output.rFR = 255
        elif command_msg.force < 0:
            output.rFR = 0
        else:
            output.rFR = int(command_msg.force)

            # Save current commands
        self.saved_output_ = output
        self.saved_command_msg_ = command_msg
        self.commands_saved_ = True

        # Publish
        self.output_pub.publish(output)

        pass


if __name__ == '__main__':
    device = gripper_control()
