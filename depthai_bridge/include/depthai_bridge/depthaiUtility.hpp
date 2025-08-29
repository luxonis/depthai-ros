#pragma once

#include <chrono>
#include <stdexcept>

#include "depthai/common/CameraBoardSocket.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

namespace depthai_bridge {

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

#define DEPTHAI_ROS_LOG_STREAM(loggerName, level, isOnce, args)                 \
    switch(level) {                                                             \
        case depthai_bridge::LogLevel::DEBUG:                                   \
            if(isOnce) {                                                        \
                RCLCPP_DEBUG_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::INFO:                                    \
            if(isOnce) {                                                        \
                RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
            } else {                                                            \
                RCLCPP_INFO_STREAM(rclcpp::get_logger(loggerName), args);       \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::WARN:                                    \
            if(isOnce) {                                                        \
                RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
            } else {                                                            \
                RCLCPP_WARN_STREAM(rclcpp::get_logger(loggerName), args);       \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::ERROR:                                   \
            if(isOnce) {                                                        \
                RCLCPP_ERROR_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
        case depthai_bridge::LogLevel::FATAL:                                   \
            if(isOnce) {                                                        \
                RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
            } else {                                                            \
                RCLCPP_FATAL_STREAM(rclcpp::get_logger(loggerName), args);      \
            }                                                                   \
            break;                                                              \
    }

// DEBUG stream macros on top of ROS logger
#define DEPTHAI_ROS_DEBUG_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::DEBUG, false, args)

#define DEPTHAI_ROS_DEBUG_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::DEBUG, true, args)

// INFO stream macros on top of ROS logger
#define DEPTHAI_ROS_INFO_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::INFO, false, args)

#define DEPTHAI_ROS_INFO_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::INFO, true, args)

// WARN stream macros on top of ROS logger
#define DEPTHAI_ROS_WARN_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::WARN, false, args)

#define DEPTHAI_ROS_WARN_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::WARN, true, args)

// ERROR stream macros on top of ROS logger
#define DEPTHAI_ROS_ERROR_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::ERROR, false, args)

#define DEPTHAI_ROS_ERROR_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::ERROR, true, args)

// FATAL stream macros on top of ROS logger
#define DEPTHAI_ROS_FATAL_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::FATAL, false, args)

#define DEPTHAI_ROS_FATAL_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, depthai_bridge::LogLevel::FATAL, true, args)

static const int64_t ZERO_TIME_DELTA_NS{100};

inline rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                                 std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    auto rclStamp = rclBaseTime + elapsedTime;
    return rclStamp;
}

inline void updateBaseTime(std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime, rclcpp::Time& rclBaseTime, int64_t& totalNsChange) {
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
enum class DeviceNames { OAK_1, OAK_D, OAK_D_PRO, OAK_D_POE, OAK_THERMAL, OAK_SR, OAK_D_SR_POE };

const std::unordered_map<DeviceNames, std::string> deviceNameMap = {
    {DeviceNames::OAK_1, "OAK-1"},
    {DeviceNames::OAK_D, "OAK-D"},
    {DeviceNames::OAK_D_PRO, "OAK-D-PRO"},
    {DeviceNames::OAK_D_POE, "OAK-D-POE"},
    {DeviceNames::OAK_THERMAL, "OAK-T"},
    {DeviceNames::OAK_SR, "OAK-SR"},
    {DeviceNames::OAK_D_SR_POE, "OAK-D-SR-POE"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> defaultSocketMap = {
    {dai::CameraBoardSocket::AUTO, "rgb"},
    {dai::CameraBoardSocket::CAM_A, "rgb"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
    {dai::CameraBoardSocket::CAM_D, "left_back"},
    {dai::CameraBoardSocket::CAM_E, "right_back"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> letterSocketMap = {
    {dai::CameraBoardSocket::AUTO, "auto"},
    {dai::CameraBoardSocket::CAM_A, "cam_a"},
    {dai::CameraBoardSocket::CAM_B, "cam_b"},
    {dai::CameraBoardSocket::CAM_C, "cam_c"},
    {dai::CameraBoardSocket::CAM_D, "cam_d"},
    {dai::CameraBoardSocket::CAM_E, "cam_e"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> srDPoeSocketMap = {
    {dai::CameraBoardSocket::AUTO, "tof"},
    {dai::CameraBoardSocket::CAM_A, "tof"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> thermalSocketMap = {
    {dai::CameraBoardSocket::AUTO, "rgb"},
    {dai::CameraBoardSocket::CAM_A, "rgb"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
    {dai::CameraBoardSocket::CAM_D, "left_back"},
    {dai::CameraBoardSocket::CAM_E, "thermal"},
};

const std::unordered_map<dai::CameraBoardSocket, std::string> rsSocketNameMap = {
    {dai::CameraBoardSocket::AUTO, "color"},
    {dai::CameraBoardSocket::CAM_A, "color"},
    {dai::CameraBoardSocket::CAM_B, "infra2"},
    {dai::CameraBoardSocket::CAM_C, "infra1"},
    {dai::CameraBoardSocket::CAM_E, "infra4"},
    {dai::CameraBoardSocket::CAM_D, "infra3"},
};

inline std::string getFrameName(const std::string& prefix, const std::string& frameName) {
    return prefix + "_" + frameName;
}

inline std::string getOpticalFrameName(const std::string& prefix, const std::string& frameName, bool rsCompat = false) {
    std::string suffix = "_camera_optical_frame";
    if(rsCompat) {
        suffix = "_optical_frame";
    }
    return getFrameName(prefix, frameName) + suffix;
}

inline std::string getSocketName(dai::CameraBoardSocket socketNum, const std::string& deviceName = "", bool rsCompat = false, bool useSocketNames = false) {
    std::string name = "";
    try {
        name = defaultSocketMap.at(socketNum);
        if(rsCompat) {
            name = rsSocketNameMap.at(socketNum);
        }
        if(deviceName.empty()) {
            if(useSocketNames) {
                name = letterSocketMap.at(socketNum);
            }
        } else {
            if(deviceName == deviceNameMap.at(DeviceNames::OAK_D_SR_POE)) {
                name = srDPoeSocketMap.at(socketNum);
            } else if(deviceName == deviceNameMap.at(DeviceNames::OAK_THERMAL)) {
                name = thermalSocketMap.at(socketNum);
            }
        }
    } catch(std::out_of_range) {
        DEPTHAI_ROS_ERROR_STREAM("depthai_bridge", "Couldn't find socket name for device: " << deviceName << ". Socket ID: " << static_cast<int>(socketNum));
        throw std::runtime_error("Socket name not found");
    }
    return name;
}

}  // namespace depthai_bridge
