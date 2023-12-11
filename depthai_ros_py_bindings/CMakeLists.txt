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

set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})


find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rtabmap_slam REQUIRED)
find_package(pybind11 REQUIRED)
find_package(spectacularai_ros2 REQUIRED)
find_package(image_proc REQUIRED)
find_package(laserscan_kinect REQUIRED)
find_package(ira_laser_tools REQUIRED)
find_package(depthai_bridge REQUIRED)


find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11 REQUIRED)
include_directories(
    include
    ${BUILD_TOOL_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
)

set(dependencies
  cv_bridge
  depthai
  depthai_bridge
  depthai_ros_msgs
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  rtabmap_slam
  spectacularai_ros2
  image_proc
  laserscan_kinect
  ira_laser_tools
  pybind11
  Python3
)

function(ament_target_dependencies_private target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    set(definitions "")
    set(include_dirs "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARGN})
      if(NOT ${${package_name}_FOUND})
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      list_append_unique(definitions ${${package_name}_DEFINITIONS})
      list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
      list(APPEND libraries ${${package_name}_LIBRARIES})
      list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
    endforeach()
    target_compile_definitions(${target}
      PUBLIC ${definitions})
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_include_directories(${target}
      PUBLIC ${ordered_include_dirs})
    ament_libraries_deduplicate(unique_libraries ${libraries})
    target_link_libraries(${target} PRIVATE
      ${unique_libraries})
    foreach(link_flag IN LISTS link_flags)
      set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
    endforeach()
  endif()
endfunction()

pybind11_add_module(dai_ros_py SHARED src/bindings.cpp)

ament_target_dependencies_private(dai_ros_py ${dependencies})
target_link_libraries(dai_ros_py depthai::core)
set_target_properties(dai_ros_py PROPERTIES
  PREFIX ""
  OUTPUT_NAME "dai_ros_py"
  SUFFIX ".so"
)
ament_python_install_package(${PROJECT_NAME})
install(TARGETS dai_ros_py
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
        )
ament_package()
