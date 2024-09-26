
#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/TransformData.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/time.hpp"

namespace dai {

namespace ros {

class TransformDataConverter {
   public:
    // DetectionConverter() = default;
    explicit TransformDataConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    ~TransformDataConverter();

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

    void toRosMsg(std::shared_ptr<dai::TransformData> inFeatures, std::deque<geometry_msgs::msg::TransformStamped>& transformMsgs);

   private:
    const std::string _frameName;
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
