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
    msg.features.resize(inFeatures->trackedFeatures.size());

    for(int i = 0; i < inFeatures->trackedFeatures.size(); ++i) {
        msg.features[i].header = msg.header;
        msg.features[i].position.x = inFeatures->trackedFeatures[i].position.x;
        msg.features[i].position.y = inFeatures->trackedFeatures[i].position.y;
        msg.features[i].age = inFeatures->trackedFeatures[i].age;
        msg.features[i].id = inFeatures->trackedFeatures[i].id;
        msg.features[i].harris_score = inFeatures->trackedFeatures[i].harrisScore;
        msg.features[i].tracking_error = inFeatures->trackedFeatures[i].trackingError;
    }
    featureMsgs.push_back(msg);
}

}  // namespace ros
}  // namespace dai