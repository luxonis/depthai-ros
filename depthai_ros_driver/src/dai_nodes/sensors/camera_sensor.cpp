#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/sensors/mono.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
CameraSensor::CameraSensor(const std::string& daiNodeName,
                           rclcpp::Node* node,
                           std::shared_ptr<dai::Pipeline> pipeline,
                           std::shared_ptr<dai::Device> device,
                           dai::CameraBoardSocket socket,
                           bool publish)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s base", daiNodeName.c_str());

    auto sensorName = device->getCameraSensorNames().at(socket);
    for (auto & c: sensorName) c = toupper(c);
    std::vector<sensor_helpers::ImageSensor>::iterator sensorIt =
        std::find_if(sensor_helpers::availableSensors.begin(), sensor_helpers::availableSensors.end(), [&sensorName](const sensor_helpers::ImageSensor& s) {
            return s.name == sensorName;
        });
    RCLCPP_DEBUG(node->get_logger(), "Node %s has sensor %s", daiNodeName.c_str(), sensorName.c_str());
    if((*sensorIt).color) {
        sensorNode = std::make_unique<RGB>(daiNodeName, node, pipeline, socket, (*sensorIt), publish);
    } else {
        sensorNode = std::make_unique<Mono>(daiNodeName, node, pipeline, socket, (*sensorIt), publish);
    }

    RCLCPP_DEBUG(node->get_logger(), "Base node %s created", daiNodeName.c_str());
}
void CameraSensor::setNames() {}

void CameraSensor::setXinXout(std::shared_ptr<dai::Pipeline> /*pipeline*/) {}

void CameraSensor::setupQueues(std::shared_ptr<dai::Device> device) {
    sensorNode->setupQueues(device);
}
void CameraSensor::closeQueues() {
    sensorNode->closeQueues();
}

void CameraSensor::link(const dai::Node::Input& in, int linkType) {
    dai::Node::Input& input = const_cast<dai::Node::Input&>(in);
    sensorNode->link(input, linkType);
}

void CameraSensor::updateParams(const std::vector<rclcpp::Parameter>& params) {
    sensorNode->updateParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
