#include "depthai_bridge/TrackedFeaturesConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {

namespace ros {

TrackedFeaturesConverter::TrackedFeaturesConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : _frameName(frameName), _steadyBaseTime(std::chrono::steady_clock::now()), _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    _rosBaseTime = rclcpp::Clock().now();
}

TrackedFeaturesConverter::~TrackedFeaturesConverter() = default;

void TrackedFeaturesConverter::updateRosBaseTime() {
    updateBaseTime(_steadyBaseTime, _rosBaseTime, _totalNsChange);
}

void TrackedFeaturesConverter::toRosMsg(std::shared_ptr<dai::TrackedFeatures> inFeatures, std::deque<depthai_ros_msgs::msg::TrackedFeatures>& featureMsgs) {
    if(_updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inFeatures->getTimestampDevice();
    else
        tstamp = inFeatures->getTimestamp();

    depthai_ros_msgs::msg::TrackedFeatures msg;

    msg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    msg.header.frame_id = _frameName;
    msg.features.reserve(inFeatures->trackedFeatures.size());

    for(const auto& feature : inFeatures->trackedFeatures) {
        depthai_ros_msgs::msg::TrackedFeature ft;
        ft.header = msg.header;
        ft.position.x = feature.position.x;
        ft.position.y = feature.position.y;
        ft.age = feature.age;
        ft.id = feature.id;
        ft.harris_score = feature.harrisScore;
        ft.tracking_error = feature.trackingError;
        msg.features.emplace_back(ft);
    }
    featureMsgs.push_back(msg);
}

}  // namespace ros
}  // namespace dai
