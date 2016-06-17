# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_c_model_control)
find_package(catkin REQUIRED COMPONENTS rospy message_generation std_msgs)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin message generation     ##
###################################
add_service_files(
  FILES
  Open.srv
)

add_message_files(
  FILES
  CModel_robot_input.msg
  CModel_robot_output.msg
  CModel_augmented_robot_output.msg
  CModel_gripper_command.msg
  CModel_gripper_state.msg
)

catkin_python_setup()

generate_messages(
	DEPENDENCIES std_msgs)

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

install(PROGRAMS nodes/CModelSimpleController.py nodes/CModelStatusListener.py 
    nodes/CModelTcpNode.py nodes/CModelRtuNode.py nodes/CModelRtuNodeAug.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})