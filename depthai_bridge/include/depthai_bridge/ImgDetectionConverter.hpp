#pragma once

#include <depthai/depthai.hpp>
#include <depthai_bridge/depthaiUtility.hpp>
#include <deque>

#ifdef IS_ROS2
    #include <vision_msgs/msg/detection2_d_array.hpp>

    #include "rclcpp/rclcpp.hpp"
#else
    #include <ros/ros.h>
    #include <vision_msgs/Detection2DArray.h>

    #include <boost/make_shared.hpp>
    #include <boost/shared_ptr.hpp>
#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2
namespace VisionMsgs = vision_msgs::msg;
using Detection2DArrayPtr = VisionMsgs::Detection2DArray::SharedPtr;
#else
namespace VisionMsgs = vision_msgs;
using Detection2DArrayPtr = VisionMsgs::Detection2DArray::Ptr;
#endif
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
#ifdef IS_ROS2
    rclcpp::Time _rosBaseTime;
#else
    ::ros::Time _rosBaseTime;
#endif
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
