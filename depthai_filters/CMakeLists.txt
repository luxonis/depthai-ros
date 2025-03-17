cmake_minimum_required(VERSION 3.10.2)
project(depthai_filters VERSION 2.11.2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-g)
add_definitions(-std=c++14)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure message_filters sensor_msgs vision_msgs visualization_msgs cv_bridge image_transport depthai_ros_msgs nodelet)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/wls.cfg
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp dynamic_reconfigure message_filters sensor_msgs vision_msgs visualization_msgs cv_bridge image_transport nodelet
 DEPENDS OpenCV
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

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
    src/detection2d_overlay.cpp
    src/segmentation_overlay.cpp
    src/wls_filter.cpp
    src/spatial_bb.cpp
    src/feature_tracker_overlay.cpp
    src/features_3d.cpp
    src/thermal_temp.cpp
    src/utils.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
