#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace dai {

namespace ros {

namespace SpatialMessages = depthai_ros_msgs::msg;
using SpatialDetectionArrayPtr = SpatialMessages::SpatialDetectionArray::SharedPtr;

class SpatialDetectionConverter {
   public:
    SpatialDetectionConverter(std::string frameName, int width, int height, bool normalized = false, bool getBaseDeviceTimestamp = false);
    ~SpatialDetectionConverter();

    /**
     * @brief Handles cases in which the ROS time shifts forward or backward
     *  Should be called at regular intervals or on-change of ROS time, depending
     *  on monitoring.
     *
     */
    void updateRosBaseTime();

    /**
     * @brief Commands the converter to automatically update the ROS base time on message conversion based on variable
     *
     * @param update: bool whether to automatically update the ROS base time on message conversion
     */
    void setUpdateRosBaseTimeOnToRosMsg(bool update = true) {
        _updateRosBaseTimeOnToRosMsg = update;
    }

    void toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData, std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsg);
    void toRosVisionMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData, std::deque<vision_msgs::msg::Detection3DArray>& opDetectionMsg);

    SpatialDetectionArrayPtr toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData);

   private:
    int _width, _height;
    const std::string _frameName;
    bool _normalized;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;

    rclcpp::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
    // For handling ROS time shifts and debugging
    int64_t _totalNsChange{0};
    // Whether to update the ROS base time on each message conversion
    bool _updateRosBaseTimeOnToRosMsg{false};
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
