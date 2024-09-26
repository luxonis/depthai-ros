
#include "depthai_bridge/TransformDataConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {

namespace ros {

TransformDataConverter::TransformDataConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : _frameName(frameName), _steadyBaseTime(std::chrono::steady_clock::now()), _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    _rosBaseTime = rclcpp::Clock().now();
}

TransformDataConverter::~TransformDataConverter() = default;

void TransformDataConverter::updateRosBaseTime() {
    updateBaseTime(_steadyBaseTime, _rosBaseTime, _totalNsChange);
}

void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inFeatures, std::deque<geometry_msgs::msg::TransformStamped>& transformMsgs) {
    if(_updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inFeatures->getTimestampDevice();
    else
        tstamp = inFeatures->getTimestamp();

    geometry_msgs::msg::TransformStamped msg;

    msg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    msg.header.frame_id = _frameName;
    transformMsgs.push_back(msg);
}

}  // namespace ros
}  // namespace dai
