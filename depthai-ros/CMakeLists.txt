cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project(depthai-ros VERSION 2.5.3 LANGUAGES CXX C)

if(CMAKE_CXX_STANDARD LESS 14)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )
    ament_package()
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
 find_package(catkin REQUIRED)
 catkin_metapackage()
 endif()
