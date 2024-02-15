#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <depthai_ros_msgs/msg/track_detection2_d_array.hpp>

#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "rclcpp/time.hpp"

namespace dai {

namespace ros {

namespace VisionMsgs = vision_msgs::msg;
namespace DepthaiMsgs = depthai_ros_msgs::msg;
using Detection2DArrayPtr = VisionMsgs::Detection2DArray::SharedPtr;
using TrackDetection2DArrayPtr = DepthaiMsgs::TrackDetection2DArray::SharedPtr;

class TrackDetectionConverter {
   public:
    TrackDetectionConverter(std::string frameName, int width, int height, bool normalized = false, float thresh = 0.0, bool getBaseDeviceTimestamp = false);
    ~TrackDetectionConverter();
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

/** TODO(sachin): Do we need to have ros msg -> dai bounding box ?
 * is there any situation where we would need to have xlinkin to take bounding
 * box as input. One scenario would to take this as input and use ImageManip
 * node to crop the roi of the image. Since it is not available yet. Leaving
 * it out for now to speed up on other tasks feel free to raise a issue if you
 * feel that feature is good to have...
 */

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai