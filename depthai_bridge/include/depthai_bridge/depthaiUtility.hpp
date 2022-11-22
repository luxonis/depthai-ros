#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace dai {

namespace ros {

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

#define DEPTHAI_ROS_LOG_STREAM(loggerName, level, isOnce, args)                 \
    switch(level) {                                                             \
        case dai::ros::LogLevel::DEBUG:                                         \
            if(isOnce) {                                                        \
                RCLCPP_DEBUG_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
        case dai::ros::LogLevel::INFO:                                          \
            if(isOnce) {                                                        \
                RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
            } else {                                                            \
                RCLCPP_INFO_STREAM(rclcpp::get_logger(loggerName), args);       \
            }                                                                   \
            break;                                                              \
        case dai::ros::LogLevel::WARN:                                          \
            if(isOnce) {                                                        \
                RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
            } else {                                                            \
                RCLCPP_WARN_STREAM(rclcpp::get_logger(loggerName), args);       \
            }                                                                   \
            break;                                                              \
        case dai::ros::LogLevel::ERROR:                                         \
            if(isOnce) {                                                        \
                RCLCPP_ERROR_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
        case dai::ros::LogLevel::FATAL:                                         \
            if(isOnce) {                                                        \
                RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_FATAL_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
    }

// DEBUG stream macros on top of ROS logger
#define DEPTHAI_ROS_DEBUG_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::DEBUG, false, args)

#define DEPTHAI_ROS_DEBUG_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::DEBUG, true, args)

// INFO stream macros on top of ROS logger
#define DEPTHAI_ROS_INFO_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::INFO, false, args)

#define DEPTHAI_ROS_INFO_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::INFO, true, args)

// WARN stream macros on top of ROS logger
#define DEPTHAI_ROS_WARN_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::WARN, false, args)

#define DEPTHAI_ROS_WARN_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::WARN, true, args)

// ERROR stream macros on top of ROS logger
#define DEPTHAI_ROS_ERROR_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::ERROR, false, args)

#define DEPTHAI_ROS_ERROR_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::ERROR, true, args)

// FATAL stream macros on top of ROS logger
#define DEPTHAI_ROS_FATAL_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::FATAL, false, args)

#define DEPTHAI_ROS_FATAL_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, dai::ros::LogLevel::FATAL, true, args)

inline rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    // uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto rclStamp = rclBaseTime + elapsedTime;
    // DEPTHAI_ROS_DEBUG_STREAM("PRINT TIMESTAMP: ", "rosStamp -> " << rclStamp << "  rosBaseTime -> " << rclBaseTime);
    return rclStamp;
}

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