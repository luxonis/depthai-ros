#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unordered_map>

#include "depthai/depthai.hpp"

#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/imu.hpp"
namespace ImuMsgs = sensor_msgs::msg;
using ImuPtr = ImuMsgs::Imu::SharedPtr;
#else
    #include <ros/ros.h>

    #include <boost/make_shared.hpp>

    #include "sensor_msgs/Imu.h"
namespace ImuMsgs = sensor_msgs;
using ImuPtr = ImuMsgs::Imu::Ptr;
#endif

namespace dai::ros {

class ImuConverter {
   public:
    ImuConverter(const std::string& frameName);

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, ImuMsgs::Imu& outImuMsg);
    ImuPtr toRosMsgPtr(const std::shared_ptr<dai::IMUData> inData);

   private:
    uint32_t _sequenceNum;
    const std::string _frameName = "";
};

}  // namespace dai::ros
namespace rosBridge = ros;
