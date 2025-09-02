cmake_minimum_required(VERSION 3.22)
project(depthai_ros_driver VERSION 3.0.5)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_SHARED_LIBS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Wno-maybe-uninitialized)
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(backward_ros REQUIRED)
ament_auto_find_build_dependencies()

set(COMMON_DEPS
${camera_info_manager_TARGETS}
cv_bridge::cv_bridge
depthai::core
${depthai_bridge_TARGETS}
image_transport::image_transport
rclcpp::rclcpp
${sensor_msgs_TARGETS}
diagnostic_updater::diagnostic_updater
${diagnostic_msgs_TARGETS}
${ffmpeg_image_transport_msgs_TARGETS}
)

set(SENSOR_DEPS
cv_bridge::cv_bridge
${camera_info_manager_TARGETS}
depthai::core
${depthai_bridge_TARGETS}
rclcpp::rclcpp
${std_msgs_TARGETS}
${sensor_msgs_TARGETS}
image_transport::image_transport
)

set(NN_DEPS
depthai::core
${depthai_bridge_TARGETS}
rclcpp::rclcpp
${vision_msgs_TARGETS}
)

set(CAM_DEPS
depthai::core
rclcpp::rclcpp
${rclcpp_components_TARGETS}
${std_srvs_TARGETS}
${pluginlib_TARGETS}
)


include_directories(include)

set(COMMON_LIB_NAME ${PROJECT_NAME}_common)
add_library(
  ${COMMON_LIB_NAME} SHARED
  src/utils.cpp
  src/dai_nodes/base_node.cpp
  src/dai_nodes/sys_logger.cpp
  src/dai_nodes/sensors/img_pub.cpp
  src/dai_nodes/sensors/sensor_helpers.cpp # TODO: Figure out different place for this
  src/param_handlers/driver_param_handler.cpp
  src/param_handlers/imu_param_handler.cpp
  src/param_handlers/nn_param_handler.cpp
  src/param_handlers/sensor_param_handler.cpp
  src/param_handlers/feature_tracker_param_handler.cpp
  src/param_handlers/rgbd_param_handler.cpp
  src/param_handlers/stereo_param_handler.cpp
  src/param_handlers/thermal_param_handler.cpp
  src/param_handlers/tof_param_handler.cpp
  src/param_handlers/pipeline_gen_param_handler.cpp
  src/param_handlers/sync_param_handler.cpp
)

target_link_libraries(${COMMON_LIB_NAME}
PUBLIC
  ${COMMON_DEPS}
)

set(SENSOR_LIB_NAME ${PROJECT_NAME}_sensor_nodes)
add_library(
  ${SENSOR_LIB_NAME} SHARED
  src/dai_nodes/sensors/imu.cpp
  src/dai_nodes/sensors/camera.cpp
  src/dai_nodes/sensors/feature_tracker.cpp
  src/dai_nodes/sensors/sensor_wrapper.cpp
  src/dai_nodes/sensors/stereo.cpp
  src/dai_nodes/sensors/thermal.cpp
  src/dai_nodes/sensors/rgbd.cpp
  src/dai_nodes/sensors/tof.cpp
  src/dai_nodes/sensors/sync.cpp
)

target_link_libraries(${SENSOR_LIB_NAME}
PUBLIC
  ${SENSOR_DEPS}
  ${COMMON_LIB_NAME}
)

set(NN_LIB_NAME ${PROJECT_NAME}_nn_nodes)
add_library(
  ${NN_LIB_NAME} SHARED
  src/dai_nodes/nn/nn_wrapper.cpp
  src/dai_nodes/nn/spatial_nn_wrapper.cpp
  src/dai_nodes/nn/segmentation.cpp
)

target_link_libraries(${NN_LIB_NAME}
PUBLIC
  ${NN_DEPS}
  ${COMMON_LIB_NAME}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/driver.cpp
  src/pipeline/pipeline_generator.cpp
  src/pipeline/base_types.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
PUBLIC
  ${CAM_DEPS}
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
)
add_executable(
 driver_node
 src/driver_node.cpp)
target_link_libraries(driver_node
PUBLIC
  ${PROJECT_NAME}
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME})
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::Driver")
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${COMMON_DEPS} ${SENSOR_DEPS} ${NN_DEPS} ${CAM_DEPS})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}  ${SENSOR_LIB_NAME}
${NN_LIB_NAME}
${COMMON_LIB_NAME}
EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
      DESTINATION share/${PROJECT_NAME}/cmake)
install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(
  include
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()
