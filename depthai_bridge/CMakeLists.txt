cmake_minimum_required(VERSION 3.0.2)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_bridge)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()


find_package(catkin REQUIRED COMPONENTS  
  cv_bridge
  camera_info_manager
  depthai_ros_msgs
  image_transport
  roscpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

find_package(Boost REQUIRED)

find_package(depthai CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS depthai_ros_msgs camera_info_manager roscpp sensor_msgs std_msgs vision_msgs image_transport cv_bridge stereo_msgs
)

list(APPEND DEPENDENCY_PUBLIC_LIBRARIES ${catkin_LIBRARIES})
set(BUILD_TOOL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

include_directories(
  include
  ${BUILD_TOOL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${depthai}
)

FILE(GLOB LIB_SRC
   "src/*Converter.cpp"
)

## Declare a C++ library
add_library(${PROJECT_NAME}  
${LIB_SRC}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  depthai::core
  opencv_imgproc
  opencv_highgui
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
