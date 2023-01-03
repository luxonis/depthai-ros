#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/sensors/mono.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
CameraSensor::CameraSensor(const std::string& daiNodeName,
                           ros::NodeHandle node,
                           std::shared_ptr<dai::Pipeline> pipeline,
                           std::shared_ptr<dai::Device> device,
                           dai::CameraBoardSocket socket)
    : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s base", daiNodeName.c_str());

    auto sensorName = device->getCameraSensorNames().at(socket);

    std::vector<sensor_helpers::ImageSensor>::iterator sensorIt =
        std::find_if(sensor_helpers::availableSensors.begin(), sensor_helpers::availableSensors.end(), [&sensorName](const sensor_helpers::ImageSensor& s) {
            return s.name == sensorName;
        });
    ROS_DEBUG("Node %s has sensor %s", daiNodeName.c_str(), sensorName.c_str());
    if((*sensorIt).color) {
        sensorNode = std::make_unique<RGB>(daiNodeName, node, pipeline, socket, (*sensorIt));
    } else {
        sensorNode = std::make_unique<Mono>(daiNodeName, node, pipeline, socket, (*sensorIt));
    }

    ROS_DEBUG("Base node %s created", daiNodeName.c_str());
};
void CameraSensor::setNames() {}

void CameraSensor::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {}

void CameraSensor::setupQueues(std::shared_ptr<dai::Device> device) {
    sensorNode->setupQueues(device);
}
void CameraSensor::closeQueues() {
    sensorNode->closeQueues();
}

void CameraSensor::link(const dai::Node::Input& in, int linkType) {
    sensorNode->link(in, linkType);
}

dai::Node::Input CameraSensor::getInput(int linkType) {
    return sensorNode->getInput(linkType);
}

void CameraSensor::updateParams(parametersConfig& config) {
    sensorNode->updateParams(config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
