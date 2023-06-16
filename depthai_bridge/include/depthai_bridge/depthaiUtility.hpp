#pragma once

#include <ros/ros.h>

#include <chrono>

namespace dai {

namespace ros {

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

#define DEPTHAI_ROS_LOG_STREAM(loggerName, level, isOnce, args)                                                                       \
    if(isOnce) {                                                                                                                      \
        ROS_LOG_STREAM_ONCE(static_cast<::ros::console::Level>(level), std::string(ROSCONSOLE_NAME_PREFIX) + "." + loggerName, args); \
    } else {                                                                                                                          \
        ROS_LOG_STREAM(static_cast<::ros::console::Level>(level), std::string(ROSCONSOLE_NAME_PREFIX) + "." + loggerName, args);      \
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

inline ::ros::Time getFrameTime(::ros::Time rosBaseTime,
                                std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                                std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto currTime = rosBaseTime;
    auto rosStamp = currTime.fromNSec(nSec);
    DEPTHAI_ROS_DEBUG_STREAM("PRINT TIMESTAMP: ", "rosStamp -> " << rosStamp << "  rosBaseTime -> " << rosBaseTime);
    return rosStamp;
}

inline void updateBaseTime(std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime, ::ros::Time& rosBaseTime, int64_t& totalNsChange) {
    ::ros::Time currentRosTime = ::ros::Time::now();
    std::chrono::time_point<std::chrono::steady_clock> currentSteadyTime = std::chrono::steady_clock::now();
    // In nanoseconds
    auto expectedOffset = std::chrono::duration_cast<std::chrono::nanoseconds>(currentSteadyTime - steadyBaseTime).count();
    uint64_t previousBaseTimeNs = rosBaseTime.toNSec();
    rosBaseTime = rosBaseTime.fromNSec(currentRosTime.toNSec() - expectedOffset);
    uint64_t newBaseTimeNs = rosBaseTime.toNSec();
    int64_t diff = static_cast<int64_t>(newBaseTimeNs - previousBaseTimeNs);
    totalNsChange += diff;
    if(::abs(diff) > ZERO_TIME_DELTA_NS) {
        // Has been updated
        DEPTHAI_ROS_DEBUG_STREAM("ROS BASE TIME CHANGE: ",
                                 "ROS base time changed by " << std::to_string(diff) << " ns. Total change: " << std::to_string(totalNsChange)
                                                             << " ns. New time: " << std::to_string(rosBaseTime.toNSec()) << " ns.");
    }
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