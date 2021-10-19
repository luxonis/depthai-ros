#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "sensor_msgs/Imu.h"

namespace dai::rosBridge {

class ImuConverter {
   public:
    ImuConverter(const std::string& frameName);

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, sensor_msgs::Imu& outImuMsg);
    sensor_msgs::Imu::Ptr toRosMsgPtr(const std::shared_ptr<dai::IMUData> inData);

   private:
    uint32_t _sequenceNum;
    const std::string _frameName = "";
};

}  // namespace dai::rosBridge