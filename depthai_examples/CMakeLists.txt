cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(depthai_examples VERSION 2.11.2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## is used, also find other catkin packages
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

set(dependencies
${camera_info_manager_TARGETS}
${cv_bridge_TARGETS}
${depthai_ros_msgs_TARGETS}
  ${depthai_bridge_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${stereo_msgs_TARGETS}
  ${std_msgs_TARGETS}
    ${vision_msgs_TARGETS}
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")

  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui)

  target_link_libraries("${node_name}"
                            ${dependencies})

endmacro()


# dai_add_node_ros2(crop_control_service src/crop_control_service.cpp)
# dai_add_node_ros2(mobilenet_node src/mobilenet_publisher.cpp)
# dai_add_node_ros2(rgb_stereo_node src/rgb_stereo_node.cpp)
dai_add_node_ros2(stereo_inertial_node src/stereo_inertial_publisher.cpp)
# dai_add_node_ros2(feature_tracker src/feature_tracker_publisher.cpp)
# dai_add_node_ros2(stereo_node src/stereo_publisher.cpp)
# dai_add_node_ros2(yolov4_spatial_node src/yolov4_spatial_publisher.cpp)
# dai_add_node_ros2(yolov4_node src/yolov4_publisher.cpp)
# dai_add_node_ros2(tracker_yolov4_node src/tracker_yolov4_publisher.cpp)
# dai_add_node_ros2(tracker_yolov4_spatial_node src/tracker_yolov4_spatial_publisher.cpp)




install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.py")

install(TARGETS
        stereo_inertial_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()

