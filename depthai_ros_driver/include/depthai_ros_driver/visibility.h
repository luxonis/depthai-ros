#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

    #ifdef __GNUC__
        #define DEPTHAI_ROS_DRIVER_EXPORT __attribute__((dllexport))
        #define DEPTHAI_ROS_DRIVER_IMPORT __attribute__((dllimport))
    #else
        #define DEPTHAI_ROS_DRIVER_EXPORT __declspec(dllexport)
        #define DEPTHAI_ROS_DRIVER_IMPORT __declspec(dllimport)
    #endif

    #ifdef DEPTHAI_ROS_DRIVER_DLL
        #define DEPTHAI_ROS_DRIVER_PUBLIC DEPTHAI_ROS_DRIVER_EXPORT
    #else
        #define DEPTHAI_ROS_DRIVER_PUBLIC DEPTHAI_ROS_DRIVER_IMPORT
    #endif

    #define DEPTHAI_ROS_DRIVER_PUBLIC_TYPE DEPTHAI_ROS_DRIVER_PUBLIC

    #define DEPTHAI_ROS_DRIVER_LOCAL

#else

    #define DEPTHAI_ROS_DRIVER_EXPORT __attribute__((visibility("default")))
    #define DEPTHAI_ROS_DRIVER_IMPORT

    #if __GNUC__ >= 4
        #define DEPTHAI_ROS_DRIVER_PUBLIC __attribute__((visibility("default")))
        #define DEPTHAI_ROS_DRIVER_LOCAL __attribute__((visibility("hidden")))
    #else
        #define DEPTHAI_ROS_DRIVER_PUBLIC
        #define DEPTHAI_ROS_DRIVER_LOCAL
    #endif

    #define DEPTHAI_ROS_DRIVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif
