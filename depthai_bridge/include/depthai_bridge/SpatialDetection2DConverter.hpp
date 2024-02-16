#pragma once

#include <deque>
#include <memory>
#include <string>

#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

namespace ros {

namespace VisionMsgs = vision_msgs::msg;
using SpatialDetection2DArrayPtr = VisionMsgs::Detection2DArray;

class SpatialDetection2DConverter {
   public:
    SpatialDetection2DConverter(std::string frameName, int width, int height, bool normalized = false, bool getBaseDeviceTimestamp = false);
    ~SpatialDetection2DConverter();
    void toRosVisionMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsg);

   private:
    int _width, _height;
    const std::string _frameName;
    bool _normalized;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;

    rclcpp::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
