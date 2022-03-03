#pragma once

#include <chrono>
#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
#else
    #include <ros/ros.h>
#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2

rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                          std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                          std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    // uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto rclStamp = rclBaseTime + elapsedTime;
    ROS_DEBUG_STREAM_NAMED("PRINT TIMESTAMP: ", "rosStamp -> " << rclStamp << "rosBaseTime -> " << rclBaseTime);
    return rclStamp;
}

#else

::ros::Time getFrameTime(::ros::Time rosBaseTime,
                         std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                         std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto rosStamp = rosBaseTime.fromNSec(nSec);
    ROS_DEBUG_STREAM_NAMED("PRINT TIMESTAMP: ", "rosStamp -> " << rosStamp << "rosBaseTime -> " << rosBaseTime);
    return rosStamp;
}
#endif

template <typename T>
T lerp(const T& a, const T& b, const double t) {
    return a * (1.0 - t) + b * t;
}

template <typename T>
T lerpImu(const T& a, const T& b, const double t) {
    T res;
    res.x = lerp(a.x, b.x, t);
    res.y = lerp(a.y, b.y, t);
    res.z = lerp(a.z, b.z, t);
    return res;
}

}  // namespace ros
}  // namespace dai