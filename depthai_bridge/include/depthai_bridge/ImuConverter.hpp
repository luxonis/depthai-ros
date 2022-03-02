#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unordered_map>

#include "depthai/depthai.hpp"

#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/imu.hpp"
#else
    #include <ros/ros.h>

    #include <boost/make_shared.hpp>

    #include "sensor_msgs/Imu.h"
#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2
namespace ImuMsgs = sensor_msgs::msg;
using ImuPtr = ImuMsgs::Imu::SharedPtr;
#else
namespace ImuMsgs = sensor_msgs;
using ImuPtr = ImuMsgs::Imu::Ptr;
#endif
class ImuConverter {
   public:
    ImuConverter(const std::string& frameName, imuSyncMethod syncMode);

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsg);
    enum imuSyncMethod{COPY, LINEAR_INTERPOLATE_GYRO, LINEAR_INTERPOLATE_ACCEL};
   
   private:
   void FillImuData_LinearInterpolation(std::vector<IMUPacket>& imuPackets,  std::deque<ImuMsgs::Imu>& imuMsgs);
    uint32_t _sequenceNum;
    const std::string _frameName = "";
    imuSyncMethod _syncMode;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
