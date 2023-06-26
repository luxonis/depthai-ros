cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_bridge VERSION 2.7.4 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(opencv_version 4)
find_package(OpenCV ${opencv_version} QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(opencv_version 3)
  find_package(OpenCV ${opencv_version} REQUIRED COMPONENTS imgproc highgui)
endif()


message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

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

set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  image_transport
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

include_directories(
    include
    ${BUILD_TOOL_INCLUDE_DIRS}

)

file(GLOB LIB_SRC
"src/DisparityConverter.cpp"
"src/ImageConverter.cpp"
"src/ImgDetectionConverter.cpp"
"src/SpatialDetectionConverter.cpp"
"src/ImuConverter.cpp"
)

add_library(${PROJECT_NAME} SHARED ${LIB_SRC})

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})
target_compile_definitions(${PROJECT_NAME} PUBLIC IS_ROS2)
message(STATUS "--------------------------$ENV{ROS_DISTRO} ---")
message(STATUS $ENV{ROS_DISTRO})

if($ENV{ROS_DISTRO} STREQUAL "galactic" OR ($ENV{ROS_DISTRO} STREQUAL "rolling" AND "${ubuntu_version}" STREQUAL "20.04\n"))
  target_compile_definitions(${PROJECT_NAME} PRIVATE IS_GALACTIC)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
  target_compile_definitions(${PROJECT_NAME} PRIVATE IS_HUMBLE)
endif()

target_link_libraries(${PROJECT_NAME}
                      depthai::core
                      opencv_imgproc
                      opencv_highgui)

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



install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(depthai_bridge)
ament_export_dependencies(${dependencies})

ament_package()

