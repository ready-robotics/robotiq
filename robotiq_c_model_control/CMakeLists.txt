# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_c_model_control)
find_package(catkin REQUIRED COMPONENTS rospy actionlib actionlib_msgs message_generation std_msgs bondpy)

catkin_python_setup()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin message generation     ##
###################################
add_action_files(
  FILES
  Gripper.action
)

add_service_files(
  FILES
  GetRobotiqGripState.srv
)

add_message_files(
  FILES
  CModel_robot_input.msg
  CModel_robot_output.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS rospy message_runtime
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS nodes/autodetect_gripper_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS nodes/dual_gripper_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
