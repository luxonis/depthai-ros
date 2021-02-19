cmake_minimum_required(VERSION 3.0.2)

get_filename_component (DEPTHAI_CORE "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core" REALPATH)
include ("${DEPTHAI_CORE}/cmake/HunterGate.cmake")

set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")


HunterGate (
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
    SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
    FILEPATH ${DEPTHAI_CORE}/cmake/Hunter/config.cmake # Add depthai-core config  (hunter limitation)
)

project(depthai_examples)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  depthai_ros_msgs
  roscpp
  sensor_msgs
  std_msgs
)

add_subdirectory(${DEPTHAI_CORE} depthai-core)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depthai_examples
#  CATKIN_DEPENDS depthai_ros_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stereo_node
  src/stereo_publisher.cpp
)

add_dependencies(stereo_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
)

target_link_libraries(stereo_node 
  ${catkin_LIBRARIES}
  depthai-core
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depthai_examples.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
