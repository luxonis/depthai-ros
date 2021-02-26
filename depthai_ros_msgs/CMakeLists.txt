cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_ros_msgs VERSION 1.0.0)

find_package (catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
  vision_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files (
  FILES
  DetectionDai.msg
  DetectionDaiArray.msg
  AutoFocusCtrl.msg
)

## Generate services in the 'srv' folder
add_service_files (
  FILES
  TriggerNamed.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages (
  DEPENDENCIES
  sensor_msgs std_msgs vision_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package (
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
)
