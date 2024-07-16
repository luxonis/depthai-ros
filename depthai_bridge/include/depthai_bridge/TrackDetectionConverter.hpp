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

class TrackDetectionConverter {
   public:
    TrackDetectionConverter(std::string frameName, int width, int height, bool normalized = false, float thresh = 0.0, bool getBaseDeviceTimestamp = false);
    ~TrackDetectionConverter();

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

    void toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<depthai_ros_msgs::msg::TrackDetection2DArray>& opDetectionMsgs);

    depthai_ros_msgs::msg::TrackDetection2DArray::SharedPtr toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData);

   private:
    int _width, _height;
    const std::string _frameName;
    bool _normalized;
    float _thresh;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    rclcpp::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
    // For handling ROS time shifts and debugging
    int64_t _totalNsChange{0};
    // Whether to update the ROS base time on each message conversion
    bool _updateRosBaseTimeOnToRosMsg{false};
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
