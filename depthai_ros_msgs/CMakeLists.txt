cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS

project (depthai_ros_msgs VERSION 2.5.3)

if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )
    # Not adding -DUSING_ROS since xml_parsing.cpp hasn't been ported to ROS2
    message(STATUS "------------------------------------------")
    message(STATUS "Depthai ROS MSGS/INTERFACES is being built using AMENT.")
    message(STATUS "------------------------------------------")
    
    find_package(geometry_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(vision_msgs REQUIRED)
    # set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/AutoFocusCtrl.msg"
      "msg/HandLandmark.msg"
      "msg/HandLandmarkArray.msg"
      # "msg/ImageMarker.msg"
      # "msg/ImageMarkerArray.msg"
      "msg/SpatialDetection.msg"
      "msg/SpatialDetectionArray.msg"
      "srv/TriggerNamed.srv"
      "srv/NormalizedImageCrop.srv"
      DEPENDENCIES builtin_interfaces geometry_msgs std_msgs vision_msgs
    )
    
    ament_export_dependencies(rosidl_default_runtime)
    ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )

    find_package (catkin REQUIRED COMPONENTS
      geometry_msgs
      message_generation
      sensor_msgs
      std_msgs
      vision_msgs
    )

    ################################################
    ## Declare ROS messages, services and actions ##
    ################################################
    ## Generate messages in the 'msg' folder
    add_message_files (
      FILES
      AutoFocusCtrl.msg
      HandLandmark.msg
      HandLandmarkArray.msg
      # ImageMarker.msg
      # ImageMarkerArray.msg
      SpatialDetection.msg
      SpatialDetectionArray.msg
  )

    ## Generate services in the 'srv' folder
    add_service_files (
      FILES
      TriggerNamed.srv
      NormalizedImageCrop.srv
    )

    ## Generate added messages and services with any dependencies listed here
    generate_messages (
      DEPENDENCIES
      geometry_msgs sensor_msgs std_msgs vision_msgs
    )

    ###################################
    ## catkin specific configuration ##
    ###################################
    catkin_package (
      CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs vision_msgs
    )
endif()
