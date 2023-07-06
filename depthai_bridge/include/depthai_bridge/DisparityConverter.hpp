#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace dai {

namespace ros {

namespace DisparityMsgs = stereo_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using DisparityImagePtr = DisparityMsgs::DisparityImage::SharedPtr;

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class DisparityConverter {
   public:
    DisparityConverter(
        const std::string frameName, float focalLength, float baseline = 7.5, float minDepth = 80, float maxDepth = 1100, bool getBaseDeviceTimestamp = false);
    ~DisparityConverter();

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

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outImageMsg);
    DisparityImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

   private:
    const std::string _frameName = "";
    const float _focalLength = 882.2, _baseline = 7.5, _minDepth = 80, _maxDepth;
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
