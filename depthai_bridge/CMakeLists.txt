cmake_minimum_required(VERSION 3.0.2)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_bridge)
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

message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")

find_package(depthai CONFIG REQUIRED)
message(STATUS ${depthai_VERSION})
if(depthai_VERSION LESS_EQUAL 2.14.1)
  message( FATAL_ERROR "Installed DEPTHAI VERSION is less than 2.15.0.  Please upgrade to the latest version which solves the timesync issues" )
endif()

# find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai") 
find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )
    # set(IS_ROS2 ON)
    # Not adding -DUSING_ROS since xml_parsing.cpp hasn't been ported to ROS2

    message(STATUS "------------------------------------------")
    message(STATUS "Depthai Bridge is being built using AMENT.")
    message(STATUS "------------------------------------------")

    set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS}})

    find_package(camera_info_manager REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(depthai_ros_msgs REQUIRED)
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
        # ${OpenCV_INCLUDE_DIRS}
    )

    FILE(GLOB LIB_SRC
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
    
    if($ENV{ROS_DISTRO} STREQUAL "galactic")
      target_compile_definitions(${PROJECT_NAME} PRIVATE IS_GALACTIC)
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


      install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
              FILES_MATCHING PATTERN "*.py")
      install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
      install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

      ament_export_include_directories(include)
      ament_export_libraries(depthai_bridge)
      ament_export_dependencies(${dependencies})

      ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )

    find_package(Boost REQUIRED)

    find_package(catkin REQUIRED COMPONENTS  
      cv_bridge
      camera_info_manager
      depthai_ros_msgs
      image_transport
      roscpp
      sensor_msgs
      stereo_msgs
      std_msgs
      vision_msgs
    )
    find_package(Boost REQUIRED)

    FILE(GLOB LIB_SRC
    "src/DisparityConverter.cpp"
    "src/ImageConverter.cpp"
    "src/ImgDetectionConverter.cpp"
    "src/SpatialDetectionConverter.cpp"
    "src/ImuConverter.cpp"
    )
    
    message(STATUS "------------------------------------------")
    message(STATUS "Depthai Bridge is being built using CATKIN.")
    message(STATUS "------------------------------------------")

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS depthai_ros_msgs camera_info_manager roscpp sensor_msgs std_msgs vision_msgs image_transport cv_bridge stereo_msgs
    )

    list(APPEND DEPENDENCY_PUBLIC_LIBRARIES ${catkin_LIBRARIES})
    set(BUILD_TOOL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

    include_directories(
      include
      ${BUILD_TOOL_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${depthai}
    )

    ## Declare a C++ library
    add_library(${PROJECT_NAME}  
    ${LIB_SRC}
    )

    add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

    target_link_libraries(${PROJECT_NAME}
      ${catkin_LIBRARIES}
      depthai::core
      opencv_imgproc
      opencv_highgui
    )

    install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

    ## Mark cpp header files for installation
    install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.hpp"
    )

    install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch")

    install(DIRECTORY
      urdf
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

endif()


# find_package(catkin REQUIRED COMPONENTS  
#   cv_bridge
#   camera_info_manager
#   depthai_ros_msgs
#   image_transport
#   roscpp
#   sensor_msgs
#   stereo_msgs
#   std_msgs
#   vision_msgs
# )


# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS depthai_ros_msgs camera_info_manager roscpp sensor_msgs std_msgs vision_msgs image_transport cv_bridge stereo_msgs
# )

# list(APPEND DEPENDENCY_PUBLIC_LIBRARIES ${catkin_LIBRARIES})
# set(BUILD_TOOL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

# include_directories(
#   include
#   ${BUILD_TOOL_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
#   ${depthai}
# )

# FILE(GLOB LIB_SRC
#    "src/*Converter.cpp"
# )

# ## Declare a C++ library
# add_library(${PROJECT_NAME}  
# ${LIB_SRC}
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   depthai::core
#   opencv_imgproc
#   opencv_highgui
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )

# install(DIRECTORY
#   launch
#   urdf
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
