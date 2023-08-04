cmake_minimum_required(VERSION 3.8)
project(depthai_filters VERSION 2.7.4 LANGUAGES CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/detection2d_overlay.cpp
  src/segmentation_overlay.cpp
  src/spatial_bb.cpp
  src/wls_filter.cpp
  src/utils.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::Detection2DOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::SegmentationOverlay")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::WLSFilter")
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::SpatialBB")
ament_export_include_directories(include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(
  include
)
install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()
