cmake_minimum_required(VERSION 3.8)
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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
)

find_package(depthai CONFIG REQUIRED)
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs camera_info_manager depthai_bridge vision_msgs cv_bridge message_filters image_transport 
  DEPENDS
  OpenCV
)

include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(
  ${PROJECT_NAME} SHARED
  src/camera.cpp
  src/pipeline_generator.cpp
  src/dai_nodes/sensors/imu.cpp
  src/dai_nodes/sensors/rgb.cpp
  src/dai_nodes/sensors/mono.cpp
  src/dai_nodes/sensors/camera_sensor.cpp
  src/dai_nodes/sensors/sensor_helpers.cpp
  src/dai_nodes/stereo.cpp
  src/dai_nodes/nn/nn_wrapper.cpp
  src/dai_nodes/nn/spatial_nn_wrapper.cpp
  src/dai_nodes/nn/segmentation.cpp
  src/dai_nodes/nn/mobilenet.cpp
  src/dai_nodes/nn/yolo.cpp
  src/dai_nodes/nn/spatial_mobilenet.cpp
  src/dai_nodes/nn/spatial_yolo.cpp
  src/param_handlers/camera_param_handler.cpp
  src/param_handlers/imu_param_handler.cpp
  src/param_handlers/nn_param_handler.cpp
  src/param_handlers/rgb_param_handler.cpp
  src/param_handlers/mono_param_handler.cpp
  src/param_handlers/stereo_param_handler.cpp
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

install(
    DIRECTORY include/
    DESTINATION include
)
