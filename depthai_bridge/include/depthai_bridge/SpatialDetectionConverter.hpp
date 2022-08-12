#pragma once

#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/depthaiUtility.hpp>
#include "depthai/depthai.hpp"
#ifdef IS_ROS2
    #include <depthai_ros_msgs/msg/spatial_detection_array.hpp>

    #include "rclcpp/rclcpp.hpp"
#else
    #include <depthai_ros_msgs/SpatialDetectionArray.h>
    #include <ros/ros.h>

    #include <boost/make_shared.hpp>
    #include <boost/shared_ptr.hpp>
#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2
namespace SpatialMessages = depthai_ros_msgs::msg;
using SpatialDetectionArrayPtr = SpatialMessages::SpatialDetectionArray::SharedPtr;
#else
namespace SpatialMessages = depthai_ros_msgs;
using SpatialDetectionArrayPtr = SpatialMessages::SpatialDetectionArray::Ptr;
#endif
class SpatialDetectionConverter {
   public:
    // DetectionConverter() = default;
    SpatialDetectionConverter(std::string frameName, int width, int height, bool normalized = false);

    void toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData, std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsg);

    SpatialDetectionArrayPtr toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData);

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
