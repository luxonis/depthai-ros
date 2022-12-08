cmake_minimum_required(VERSION 3.8)
project(depthai_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/camera.cpp
  src/dai_nodes/sensors/imu.cpp
  src/dai_nodes/sensors/rgb.cpp
  src/dai_nodes/sensors/mono.cpp
  src/dai_nodes/sensors/camera_sensor.cpp
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

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(
  ${PROJECT_NAME}
  depthai::opencv
  depthai::core
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::Camera")
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch models config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(
  PROGRAMS
  scripts/obj_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()
