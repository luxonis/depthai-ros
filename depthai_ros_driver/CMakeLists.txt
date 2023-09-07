cmake_minimum_required(VERSION 3.16.3)
project(depthai_ros_driver)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()
set(CMAKE_LINKER_FLAGS "-pthread" CACHE STRING "Linker Flags" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_LINKER_FLAGS}" CACHE STRING "" FORCE)

set(opencv_version 4)
find_package(OpenCV ${opencv_version} QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(opencv_version 3)
  find_package(OpenCV ${opencv_version} REQUIRED COMPONENTS imgproc highgui)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  dynamic_reconfigure
  depthai_ros_msgs
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  vision_msgs
  depthai_bridge
  message_filters
  image_transport
  nodelet
  diagnostic_updater
  diagnostic_msgs
)
find_package(Threads REQUIRED)


find_package(depthai CONFIG REQUIRED)
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)


include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


catkin_package(
  LIBRARIES 
  ${PROJECT_NAME} 
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs camera_info_manager depthai_bridge vision_msgs cv_bridge message_filters image_transport diagnostic_msgs diagnostic_updater
  DEPENDS
  OpenCV
)

set(COMMON_LIB_NAME ${PROJECT_NAME}_common)
add_library(
  ${COMMON_LIB_NAME} SHARED
  src/utils.cpp
  src/dai_nodes/base_node.cpp
  src/dai_nodes/sys_logger.cpp
  src/dai_nodes/sensors/sensor_helpers.cpp # TODO: Figure out different place for this 
  src/param_handlers/camera_param_handler.cpp
  src/param_handlers/imu_param_handler.cpp
  src/param_handlers/nn_param_handler.cpp
  src/param_handlers/sensor_param_handler.cpp
  src/param_handlers/feature_tracker_param_handler.cpp
  src/param_handlers/stereo_param_handler.cpp
)

add_dependencies(${COMMON_LIB_NAME}
${catkin_EXPORTED_TARGETS} 
${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(${COMMON_LIB_NAME}
  ${catkin_LIBRARIES}
  depthai::core
  opencv_imgproc
  opencv_highgui
)

set(SENSOR_LIB_NAME ${PROJECT_NAME}_sensor_nodes)
add_library(
  ${SENSOR_LIB_NAME} SHARED
  src/dai_nodes/sensors/imu.cpp
  src/dai_nodes/sensors/rgb.cpp
  src/dai_nodes/sensors/mono.cpp
  src/dai_nodes/sensors/feature_tracker.cpp
  src/dai_nodes/sensors/sensor_wrapper.cpp
  src/dai_nodes/stereo.cpp
)

target_link_libraries(
  ${SENSOR_LIB_NAME}
  ${COMMON_LIB_NAME}
)
set(NN_LIB_NAME ${PROJECT_NAME}_nn_nodes)
add_library(
  ${NN_LIB_NAME} SHARED
  src/dai_nodes/nn/nn_wrapper.cpp
  src/dai_nodes/nn/spatial_nn_wrapper.cpp
  src/dai_nodes/nn/segmentation.cpp
)

target_link_libraries(
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/pipeline/pipeline_generator.cpp
  src/pipeline/base_types.cpp
  src/camera.cpp
)


target_link_libraries(
  ${PROJECT_NAME}
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
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
install(TARGETS
${PROJECT_NAME} 
${SENSOR_LIB_NAME}
${NN_LIB_NAME}
${COMMON_LIB_NAME}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(FILES nodelet_plugins.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Install Python executables
install(
  PROGRAMS
  scripts/obj_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugins.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    DIRECTORY include/
    DESTINATION include
)
