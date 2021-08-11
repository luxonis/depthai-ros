cmake_minimum_required(VERSION 3.5)


set (CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_bridge VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)

find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai") 

# find_package(depthai CONFIG REQUIRED) 

set(dependencies
rclcpp
sensor_msgs
std_msgs
vision_msgs
depthai_ros_msgs 
camera_info_manager
)


include_directories(
  include
  ${ament_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB LIB_SRC
   "src/*Converter.cpp"
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
"src/ImageConverter.cpp"
"src/ImgDetectionConverter.cpp"
"src/SpatialDetectionConverter.cpp")

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME} 
                      depthai::core 
                      opencv_imgproc
                      opencv_highgui) 

install(TARGETS depthai_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(depthai_bridge)
ament_export_dependencies(${dependencies})

ament_package()
