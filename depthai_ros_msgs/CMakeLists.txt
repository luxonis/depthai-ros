cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project(depthai_ros_msgs VERSION 2.7.4)

if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake REQUIRED)

# Not adding -DUSING_ROS since xml_parsing.cpp hasn't been ported to ROS2
message(STATUS "------------------------------------------")
message(STATUS "Depthai ROS MSGS/INTERFACES is being built using AMENT.")
message(STATUS "------------------------------------------")

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoFocusCtrl.msg"
  "msg/HandLandmark.msg"
  "msg/HandLandmarkArray.msg"
  "msg/ImuWithMagneticField.msg"
  # "msg/ImageMarker.msg"
  # "msg/ImageMarkerArray.msg"
  "msg/SpatialDetection.msg"
  "msg/SpatialDetectionArray.msg"
  "srv/TriggerNamed.srv"
  "srv/NormalizedImageCrop.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs vision_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

