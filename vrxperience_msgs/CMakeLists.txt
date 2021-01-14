# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

cmake_minimum_required(VERSION 3.5)
project(vrxperience_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoadLinePolynom.msg"
  "msg/RoadLinesPolynoms.msg"
  "msg/MovableTargetBoundingBox.msg"
  "msg/MovableTargetsBoundingBoxes.msg"
  "msg/GPS.msg"
  "msg/LaserMeterPoint.msg"
  "msg/LaserMeter.msg"
  "msg/MovableTarget.msg"
  "msg/MovableTargets.msg"
  "msg/CabToModelCorrective.msg"
  "msg/CabToSteeringCorrective.msg"
  "msg/VehicleOutput.msg"
  "msg/Num.msg"
  DEPENDENCIES std_msgs geometry_msgs
 )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
