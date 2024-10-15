cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project(depthai-ros VERSION 2.10.3 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)


ament_package()
