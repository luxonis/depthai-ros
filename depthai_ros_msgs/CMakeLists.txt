cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_ros_msgs VERSION 2.7.5)

if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(catkin_FOUND 1)
add_definitions( -DUSING_ROS )

find_package (catkin REQUIRED COMPONENTS
  geometry_msgs
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
  AutoFocusCtrl.msg
  HandLandmark.msg
  HandLandmarkArray.msg
  # ImageMarker.msg
  # ImageMarkerArray.msg
  SpatialDetection.msg
  SpatialDetectionArray.msg
  ImuWithMagneticField.msg
)

## Generate services in the 'srv' folder
add_service_files (
  FILES
  TriggerNamed.srv
  NormalizedImageCrop.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages (
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs vision_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package (
  CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs vision_msgs
)

