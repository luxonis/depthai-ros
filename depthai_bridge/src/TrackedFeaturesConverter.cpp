#include "depthai_bridge/TrackedFeaturesConverter.hpp"

namespace depthai_bridge {

TrackedFeaturesConverter::TrackedFeaturesConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp) {}

TrackedFeaturesConverter::~TrackedFeaturesConverter() = default;

void TrackedFeaturesConverter::toRosMsg(std::shared_ptr<dai::TrackedFeatures> inFeatures, std::deque<depthai_ros_msgs::msg::TrackedFeatures>& featureMsgs) {
    depthai_ros_msgs::msg::TrackedFeatures msg;

    msg.header = getRosHeader(inFeatures);
    msg.header.frame_id = outputRotation.frame(frameName);
    if(outputRotation.enabled() && (width <= 0 || height <= 0)) throw std::invalid_argument("Feature rotation requires image dimensions");
    msg.features.reserve(inFeatures->trackedFeatures.size());

    for(const auto& feature : inFeatures->trackedFeatures) {
        depthai_ros_msgs::msg::TrackedFeature ft;
        ft.header = msg.header;
        ft.position.x = feature.position.x;
        ft.position.y = feature.position.y;
        auto xy = outputRotation.pixel(ft.position.x, ft.position.y, double(width), double(height));
        ft.position.x = xy.first;
        ft.position.y = xy.second;
        ft.age = feature.age;
        ft.id = feature.id;
        ft.harris_score = feature.harrisScore;
        ft.tracking_error = feature.trackingError;
        msg.features.emplace_back(ft);
    }
    featureMsgs.push_back(msg);
}

}  // namespace depthai_bridge
