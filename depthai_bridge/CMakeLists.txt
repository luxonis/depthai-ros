cmake_minimum_required(VERSION 3.0.2)

# get_filename_component (DEPTHAI_CORE "${CMAKE_CURRENT_LIST_DIR}/../external/depthai-core" REALPATH)
# include ("${DEPTHAI_CORE}/cmake/HunterGate.cmake")

set (CMAKE_POSITION_INDEPENDENT_CODE ON)
# set (HUNTER_CONFIGURATION_TYPES "Release" CACHE STRING "Hunter dependencies list of build configurations")

# HunterGate (
#     URL "https://github.com/cpp-pm/hunter/archive/v0.23.258.tar.gz"
#     SHA1 "062a19ab13ce8dffa9a882b6ce3e43bdabdf75d3"
#     FILEPATH ${DEPTHAI_CORE}/cmake/Hunter/config.cmake # Add depthai-core config  (hunter limitation)
# )


project(depthai_bridge)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


# find_package(ament_cmake QUIET)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()


# if ( ament_cmake_FOUND )

#     # Not adding -DUSING_ROS since xml_parsing.cpp hasn't been ported to ROS2

#     find_package(depthai_ros_msgs REQUIRED)
#     find_package(rclcpp REQUIRED)
#     find_package(sensor_msgs REQUIRED)
#     find_package(std_msgs REQUIRED)
#     find_package(vision_msgs REQUIRED)

#     message(STATUS "------------------------------------------")
#     message(STATUS "Depthai bridge is being built using AMENT.")
#     message(STATUS "------------------------------------------")

#     list(APPEND DEPENDENCY_PUBLIC_LIBRARIES 
#     [depthai_ros_msgs, 
#     rclcpp, 
#     sensor_msgs, 
#     std_msgs, 
#     vision_msgs])

#     set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

# elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    set(catkin_FOUND 1)
    # add_definitions( -DUSING_ROS )
    # message(STATUS "------------------------------------------")
    
    find_package(catkin REQUIRED COMPONENTS  
      camera_info_manager
      roscpp
      sensor_msgs
      stereo_msgs
      std_msgs
      vision_msgs
      depthai_ros_msgs
    )
    # find_package(GTest)
    find_package(Boost REQUIRED)

    message(STATUS "------------------------------------------")
    message(STATUS "Depthai bridge is being built using CATKIN.")
    message(STATUS "------------------------------------------")

    find_package(depthai CONFIG REQUIRED)
    # find_package(depthai REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake")

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS depthai_ros_msgs camera_info_manager roscpp sensor_msgs std_msgs vision_msgs 
    )

    # catkin_package(
    #     INCLUDE_DIRS include # do not include "3rdparty" here
    #     LIBRARIES ${BEHAVIOR_TREE_LIBRARY}
    #     CATKIN_DEPENDS roslib
    #   )

    list(APPEND DEPENDENCY_PUBLIC_LIBRARIES ${catkin_LIBRARIES})
    set(BUILD_TOOL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
# endif()

# set(DAI_LIBRARIES "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib")
# set(DAI_INCLUDE_LIBRARIES "/home/sachin/Desktop/luxonis/depthai-core/build/install/include")

include_directories(
  include
  ${BUILD_TOOL_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${depthai}
)

FILE(GLOB LIB_SRC
   "src/*Converter.cpp"
)

## Declare a C++ library
add_library(${PROJECT_NAME}  
${LIB_SRC}
)
message("----------------------------------------------------------------------")
message("${CMAKE_MODULE_PATH}")
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  depthai::core
  opencv_imgproc
  opencv_highgui
)

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(
#   ${DEPENDENCY_PUBLIC_LIBRARIES}
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/depthai_bridge.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/depthai_bridge_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depthai_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
