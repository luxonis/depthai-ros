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

static const int64_t ZERO_TIME_DELTA_NS{100};

inline rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    // uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto rclStamp = rclBaseTime + elapsedTime;
    // DEPTHAI_ROS_DEBUG_STREAM("PRINT TIMESTAMP: ", "rosStamp -> " << rclStamp << "  rosBaseTime -> " << rclBaseTime);
    return rclStamp;
}

inline void updateBaseTime(std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime, rclcpp::Time rclBaseTime, int64_t& totalNsChange) {
    rclcpp::Time currentRosTime = rclcpp::Clock().now();
    std::chrono::time_point<std::chrono::steady_clock> currentSteadyTime = std::chrono::steady_clock::now();
    // In nanoseconds
    auto expectedOffset = std::chrono::duration_cast<std::chrono::nanoseconds>(currentSteadyTime - steadyBaseTime).count();
    uint64_t previousBaseTimeNs = rclBaseTime.nanoseconds();
    rclBaseTime = rclcpp::Time(currentRosTime.nanoseconds() - expectedOffset);
    uint64_t newBaseTimeNs = rclBaseTime.nanoseconds();
    int64_t diff = static_cast<int64_t>(newBaseTimeNs - previousBaseTimeNs);
    totalNsChange += diff;
    if(::abs(diff) > ZERO_TIME_DELTA_NS) {
        // Has been updated
        DEPTHAI_ROS_DEBUG_STREAM("ROS BASE TIME CHANGE: ",
                                 "ROS base time changed by " << std::to_string(diff) << " ns. Total change: " << std::to_string(totalNsChange)
                                                             << " ns. New time: " << std::to_string(rclBaseTime.nanoseconds()) << " ns.");
    }
}

}  // namespace ros
}  // namespace dai