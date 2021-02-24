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
  cv_bridge
  vision_msgs
)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  set(_opencv_version 3)
endif()

add_subdirectory(${DEPTHAI_CORE} depthai-core)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS depthai_ros_msgs roscpp sensor_msgs std_msgs camera_info_manager 
 DEPENDS
 OpenCV
 )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  lib/stereo_pipeline.cpp
  lib/rgb_pipeline.cpp
  lib/nn_pipeline.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} depthai-core
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  depthai-core
)

macro(dai_add_node node_name node_src)
    add_executable("${node_name}" "${node_src}")
    
    add_dependencies("${node_name}"
      ${catkin_EXPORTED_TARGETS} 
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${PROJECT_NAME} 
    )

    target_link_libraries("${node_name}" 
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    depthai-core
    )
    
endmacro() 


dai_add_node(stereo_node src/stereo_publisher.cpp)
dai_add_node(rgb_node src/rgb_publisher.cpp)

# add_executable(stereo_node
#   src/stereo_publisher.cpp
# )

# add_dependencies(stereo_node
#   ${catkin_EXPORTED_TARGETS} 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${PROJECT_NAME} 
# )

# target_link_libraries(stereo_node 
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
#   depthai-core
# )

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
