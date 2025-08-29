cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_bridge VERSION 3.0.4 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  image_transport
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  composition_interfaces
  ffmpeg_image_transport_msgs
)

include_directories(
    include
    ${BUILD_TOOL_INCLUDE_DIRS}

)

file(GLOB LIB_SRC
    "src/BaseConverter.cpp"
    "src/DisparityConverter.cpp"
    "src/ImageConverter.cpp"
    "src/ImgDetectionConverter.cpp"
    "src/ImuConverter.cpp"
    "src/PointCloudConverter.cpp"
    "src/SpatialDetectionConverter.cpp"
    "src/TFPublisher.cpp"
    "src/TrackedFeaturesConverter.cpp"
    "src/TrackDetectionConverter.cpp"
    "src/TrackSpatialDetectionConverter.cpp"
)

add_library(${PROJECT_NAME} SHARED ${LIB_SRC})

target_link_libraries(depthai_bridge PUBLIC
      ${composition_interfaces_TARGETS}
      ${depthai_ros_msgs_TARGETS}
      ${ffmpeg_image_transport_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${stereo_msgs_TARGETS}
      ${vision_msgs_TARGETS}
      ament_index_cpp::ament_index_cpp
      camera_info_manager::camera_info_manager
      cv_bridge::cv_bridge
      image_transport::image_transport
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
      tf2::tf2
      tf2_geometry_msgs::tf2_geometry_msgs
      tf2_ros::static_transform_broadcaster_node
      tf2_ros::tf2_ros
      depthai::core
      opencv_imgproc
      opencv_highgui
      opencv_calib3d)


ament_export_targets(depthai_bridgeTargets HAS_LIBRARY_TARGET)

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS depthai_bridge
      EXPORT depthai_bridgeTargets
              ARCHIVE DESTINATION lib
              LIBRARY DESTINATION lib
              RUNTIME DESTINATION bin
              INCLUDES DESTINATION include

)
install(EXPORT depthai_bridgeTargets
      DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    macro(bridge_test test_name test_src)
        ament_add_gtest("${PROJECT_NAME}_${test_name}" "test/${test_src}")
        target_include_directories("${PROJECT_NAME}_${test_name}" PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
        )
        target_link_libraries("${PROJECT_NAME}_${test_name}" depthai_bridge)

    endmacro()
    bridge_test(disparity_converter_test DisparityConverterTest.cpp)
    bridge_test(image_converter_test ImageConverterTest.cpp)
    bridge_test(img_det_converter_test ImgDetectionConverterTest.cpp)
    bridge_test(imu_converter_test ImuConverterTest.cpp)
    bridge_test(pointcloud_converter_test PointCloudConverterTest.cpp)
    bridge_test(spatial_detection_converter_test SpatialDetectionConverterTest.cpp)
    bridge_test(track_detection_converter_test TrackDetectionConverterTest.cpp)
    bridge_test(track_spatial_detection_converter_test TrackSpatialDetectionConverterTest.cpp)
    bridge_test(tracked_features_converter_test TrackedFeaturesConverterTest.cpp)

    #   # for calib.json
    install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
endif()


ament_export_include_directories(include)
ament_export_libraries(depthai_bridge)
ament_export_dependencies(${dependencies})

ament_package()

