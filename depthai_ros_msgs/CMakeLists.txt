cmake_minimum_required (VERSION 3.5)  # CMake version in Ubuntu 18.04 LTS

project (depthai_ros_msgs VERSION 1.0.0)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoFocusCtrl.msg"
  "msg/DetectionDai.msg"
  "msg/DetectionDaiArray.msg"
  "srv/TriggerNamed.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
