// Compile an actual client against the unmodified develop public header.
// Fixture revision: 703ad4a4fda81fb88428a1c543d0a70a0dbb1687.
#include "fixtures/develop/TFPublisher.hpp"

std::size_t developTFPublisherSize() {
    return sizeof(depthai_bridge::TFPublisher);
}

std::string developTFPublisherClient(std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calibration, const std::string& model) {
    auto publisher = std::make_unique<depthai_bridge::TFPublisher>(node, calibration, std::vector<dai::CameraFeatures>{}, "oak", model);
    return publisher->getURDF();
}
