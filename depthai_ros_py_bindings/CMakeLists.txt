cmake_minimum_required(VERSION 3.15)  # CMake version in Ubuntu 18.04 LTS
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_ros_py_bindings VERSION 2.8.2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()
cmake_policy(SET CMP0074 NEW)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(opencv_version 4)
find_package(OpenCV ${opencv_version} QUIET COMPONENTS imgproc highgui calib3d)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(opencv_version 3)
  find_package(OpenCV ${opencv_version} REQUIRED COMPONENTS imgproc highgui calib3d)
endif()


message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
set(DAI_ROS_BRIDGE_PATH "" CACHE STRING "Path to depthai-ros-bridge project")
add_subdirectory(${DAI_ROS_BRIDGE_PATH} depthai_bridge)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
hunter_add_package(TIFF)
hunter_add_package(PNG)
find_package(rtabmap_slam REQUIRED)
set(spectacularai_ros2_DIR "" CACHE STRING "Path to spectacularai_ros2 project")
find_package(spectacularai_ros2 REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11 REQUIRED)
include_directories(
    include
    ${BUILD_TOOL_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
)

set(dependencies
  cv_bridge
  depthai_ros_msgs
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  rtabmap_slam
  spectacularai_ros2
  pybind11
)
add_library(${PROJECT_NAME} SHARED src/bindings.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})
set(DAI_ROS_BRIDGE_INCLUDE_DIR "${DAI_ROS_BRIDGE_PATH}/include")
target_include_directories(${PROJECT_NAME} PUBLIC src ${DAI_ROS_BRIDGE_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
                      depthai_bridge
                      depthai::core
                      opencv_imgproc
                      opencv_highgui
                      opencv_calib3d)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
      EXPORT ${PROJECT_NAME}Targets
              ARCHIVE DESTINATION lib
              LIBRARY DESTINATION lib
              RUNTIME DESTINATION bin
              INCLUDES DESTINATION include

)
install(EXPORT ${PROJECT_NAME}Targets
      DESTINATION share/${PROJECT_NAME}/cmake)

      ament_export_include_directories(include)
      ament_export_libraries(${PROJECT_NAME})
      ament_export_dependencies(${dependencies})

ament_package()
