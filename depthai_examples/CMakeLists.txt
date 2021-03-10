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
# add_compile_options(-std=c++11)
project(depthai_examples)
# add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
  depthai_bridge
  message_filters
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
 CATKIN_DEPENDS depthai_ros_msgs roscpp sensor_msgs std_msgs camera_info_manager depthai_bridge vision_msgs cv_bridge message_filters
 DEPENDS
 OpenCV
 )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# message(STATUS "Include directoires ------------->")
# message(STATUS "libraries directoires ------------->")
# message(${catkin_INCLUDE_DIRS})
message(${catkin_LIBRARIES})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB EXAMPLE_LIB
   "lib/*.cpp"
)

add_library(${PROJECT_NAME} ${EXAMPLE_LIB})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} depthai-core
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  depthai-core
  ${OpenCV_LIBRARIES}
)


add_library(nodelet_stereo src/stereo_nodelet.cpp)

add_dependencies(nodelet_stereo ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} depthai-core
  ${PROJECT_NAME}
)

target_link_libraries(nodelet_stereo
  ${catkin_LIBRARIES}
  depthai-core
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)


hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob"
    SHA1 "f0e14978b3f77a4f93b9f969cd39e58bb7aef490"
    FILE "mobilenet-ssd_openvino_2021.2_6shave.blob"
    LOCATION mobilenet_blob
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
dai_add_node(mobilenet_node src/mobilenet_publisher.cpp)
dai_add_node(rgb_stereo_node src/rgb_stereo_node.cpp)
dai_add_node(stereo_mimic_node src/stereo_mimic_node.cpp)

target_compile_definitions(mobilenet_node PRIVATE BLOB_PATH="${mobilenet_blob}")

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
