cmake_minimum_required(VERSION 3.10.2) # CMake version in Ubuntu 18.04 LTS
project(
  depthai_examples
  VERSION 3.0.1
  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(backward_ros REQUIRED)

set(dependencies
    ${camera_info_manager_TARGETS}
    ${cv_bridge_TARGETS}
    ${depthai_ros_msgs_TARGETS}
    ${depthai_bridge_TARGETS}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${stereo_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${vision_msgs_TARGETS})

include_directories(include ${ament_INCLUDE_DIRS})

macro(add_example node_name node_src)
    add_executable("${node_name}" "${node_src}")

    target_link_libraries("${node_name}" depthai::core opencv_imgproc
                        opencv_highgui)

    target_link_libraries("${node_name}" ${dependencies})

endmacro()

add_example(rgbd_spatial_detections src/rgbd_spatial_detections.cpp)
add_example(feature_tracker src/feature_tracker_publisher.cpp)
add_example(converter_to_cv src/converter_to_cv.cpp)
add_example(imu_publisher src/imu_publisher.cpp)
add_example(rgb_publisher src/rgb_publisher.cpp)
add_example(rgbd_publisher src/rgbd_publisher.cpp)
add_example(rgb_subscriber src/rgb_subscriber.cpp)
add_example(rgb_compressed_publisher src/rgb_compressed_publisher.cpp)
add_example(disparity_publisher src/disparity_publisher.cpp)
add_example(detection_publisher src/detection_publisher.cpp)
add_example(thermal_publisher src/thermal_publisher.cpp)
add_example(tof_publisher src/tof_publisher.cpp)
add_example(tof_rgbd src/tof_rgbd.cpp)

install(
  TARGETS rgbd_spatial_detections
          feature_tracker
          converter_to_cv
          imu_publisher
          rgb_publisher
          rgbd_publisher
          rgb_subscriber
          rgb_compressed_publisher
          disparity_publisher
          detection_publisher
          tof_publisher
          tof_rgbd
          thermal_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
