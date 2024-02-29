#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace dai {

namespace ros {

namespace VisionMsgs = vision_msgs::msg;
namespace DepthaiMsgs = depthai_ros_msgs::msg;
using Detection2DArrayPtr = VisionMsgs::Detection2DArray::SharedPtr;
using TrackDetection2DArrayPtr = DepthaiMsgs::TrackDetection2DArray::SharedPtr;

class TrackSpatialDetectionConverter {
   public:
    TrackSpatialDetectionConverter(std::string frameName, int width, int height, bool normalized = false, float thresh = 0.0, bool getBaseDeviceTimestamp = false);
    ~TrackSpatialDetectionConverter();
    void toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<DepthaiMsgs::TrackDetection2DArray>& opDetectionMsgs);

    TrackDetection2DArrayPtr toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData);

   private:
    int _width, _height;
    const std::string _frameName;
    bool _normalized;
    float _thresh;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    rclcpp::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai