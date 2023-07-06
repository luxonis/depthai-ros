cmake_minimum_required(VERSION 3.22)
project(depthai_ros_driver VERSION 2.7.4)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_SHARED_LIBS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
ament_auto_find_build_dependencies()

set(COMMON_DEPS
camera_info_manager
cv_bridge
depthai
depthai_bridge
image_transport
rclcpp
sensor_msgs
)

set(SENSOR_DEPS
cv_bridge 
camera_info_manager
depthai 
depthai_bridge 
rclcpp  
std_msgs 
sensor_msgs 
image_transport)

set(NN_DEPS
depthai 
depthai_bridge 
rclcpp 
vision_msgs 
)

set(CAM_DEPS
depthai 
rclcpp 
rclcpp_components 
std_srvs
pluginlib
)


include_directories(include)

set(COMMON_LIB_NAME ${PROJECT_NAME}_common)
add_library(
  ${COMMON_LIB_NAME} SHARED
  src/utils.cpp
  src/dai_nodes/base_node.cpp
  src/dai_nodes/sensors/sensor_helpers.cpp # TODO: Figure out different place for this 
  src/param_handlers/camera_param_handler.cpp
  src/param_handlers/imu_param_handler.cpp
  src/param_handlers/nn_param_handler.cpp
  src/param_handlers/sensor_param_handler.cpp
  src/param_handlers/stereo_param_handler.cpp
)

ament_target_dependencies(${COMMON_LIB_NAME} ${COMMON_DEPS})

set(SENSOR_LIB_NAME ${PROJECT_NAME}_sensor_nodes)
add_library(
  ${SENSOR_LIB_NAME} SHARED
  src/dai_nodes/sensors/imu.cpp
  src/dai_nodes/sensors/rgb.cpp
  src/dai_nodes/sensors/mono.cpp
  src/dai_nodes/sensors/sensor_wrapper.cpp
  src/dai_nodes/stereo.cpp
)

ament_target_dependencies(${SENSOR_LIB_NAME} ${SENSOR_DEPS})
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
ament_target_dependencies(${NN_LIB_NAME} ${NN_DEPS})
target_link_libraries(
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/camera.cpp
  src/pipeline/pipeline_generator.cpp
  src/pipeline/base_types.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${CAM_DEPS})
target_link_libraries(
  ${PROJECT_NAME}
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
)


rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::Camera")
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
