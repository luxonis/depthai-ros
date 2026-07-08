cmake_minimum_required(VERSION 3.8)
project(
  depthai_filters_v3
  VERSION 3.3.0
  LANGUAGES CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
include_directories(include)
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/detection2d_overlay.cpp
  src/segmentation_overlay.cpp
  src/spatial_bb.cpp
  src/wls_filter.cpp
  src/feature_tracker_overlay.cpp
  src/features_3d.cpp
  src/thermal_temp.cpp
  src/utils.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME}
  "depthai_filters::Detection2DOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME}
  "depthai_filters::SegmentationOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME} "depthai_filters::WLSFilter")
rclcpp_components_register_nodes(${PROJECT_NAME} "depthai_filters::SpatialBB")
rclcpp_components_register_nodes(${PROJECT_NAME}
  "depthai_filters::FeatureTrackerOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME} "depthai_filters::Features3D")
rclcpp_components_register_nodes(${PROJECT_NAME} "depthai_filters::ThermalTemp")

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
