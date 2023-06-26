cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(depthai_examples VERSION 2.7.4 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

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

# find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai")
set(tiny_yolo_v4_blob_name "yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob")
set(mobilenet_blob_name "mobilenet-ssd_openvino_2021.2_6shave.blob")
set(deeplab_blob_name "deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob")

set(mobilenet_blob "${PROJECT_SOURCE_DIR}/resources/${mobilenet_blob_name}")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob"
${mobilenet_blob}
TIMEOUT 60  # seconds
EXPECTED_HASH SHA1=f0e14978b3f77a4f93b9f969cd39e58bb7aef490
TLS_VERIFY ON)

set(tiny_yolo_v4_blob "${PROJECT_SOURCE_DIR}/resources/${tiny_yolo_v4_blob_name}")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/tiny-yolo-v4_openvino_2021.2_6shave.blob"
  ${tiny_yolo_v4_blob}
  TIMEOUT 60  # seconds
  EXPECTED_HASH SHA1=219d949610a5760e62a8458941e1300b81c3fe4a
  TLS_VERIFY ON)

set(deeplab_blob "${PROJECT_SOURCE_DIR}/resources/${deeplab_blob_name}")
file(DOWNLOAD
"https://github.com/luxonis/depthai-experiments/blob/master/gen2-deeplabv3_multiclass/models/deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob?raw=true"
  ${deeplab_blob}
  TIMEOUT 60  # seconds
  EXPECTED_HASH SHA1=bdc2529f765f34c4799b478690fb4e32f1ad5d89
  TLS_VERIFY ON)

find_package(ament_cmake REQUIRED)

message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

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
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
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

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()


dai_add_node_ros2(crop_control_service src/crop_control_service.cpp)
dai_add_node_ros2(mobilenet_node src/mobilenet_publisher.cpp)
dai_add_node_ros2(rgb_stereo_node src/rgb_stereo_node.cpp)
dai_add_node_ros2(stereo_inertial_node src/stereo_inertial_publisher.cpp)
dai_add_node_ros2(stereo_node src/stereo_publisher.cpp)
dai_add_node_ros2(yolov4_spatial_node src/yolov4_spatial_publisher.cpp)

target_compile_definitions(mobilenet_node PRIVATE BLOB_NAME="${mobilenet_blob_name}")
target_compile_definitions(yolov4_spatial_node PRIVATE BLOB_NAME="${tiny_yolo_v4_blob_name}")
target_compile_definitions(stereo_inertial_node PRIVATE BLOB_NAME="${tiny_yolo_v4_blob_name}")

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  target_compile_definitions(rgb_stereo_node PRIVATE IS_GALACTIC)
endif()


install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.py")

install(TARGETS
        crop_control_service
        mobilenet_node
        rgb_stereo_node
        stereo_inertial_node
        stereo_node
        yolov4_spatial_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()

