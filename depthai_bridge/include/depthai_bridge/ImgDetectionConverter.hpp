#pragma once

#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <depthai/depthai.hpp>
#include <depthai_bridge/depthaiUtility.hpp>
#include <deque>

namespace dai {

namespace ros {

namespace VisionMsgs = vision_msgs;
using Detection2DArrayPtr = VisionMsgs::Detection2DArray::Ptr;
class ImgDetectionConverter {
   public:
    // DetectionConverter() = default;
    ImgDetectionConverter(std::string frameName, int width, int height, bool normalized = false);

    void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs);

    Detection2DArrayPtr toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData);

   private:
    int _width, _height;
    const std::string _frameName;
    bool _normalized;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    ::ros::Time _rosBaseTime;
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
