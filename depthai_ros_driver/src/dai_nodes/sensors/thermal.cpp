
#include "depthai_ros_driver/dai_nodes/sensors/thermal.hpp"

#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Thermal.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/thermal_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Thermal::Thermal(
    const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline, const std::string& deviceName, bool rsCompat)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    thermalNode = pipeline->create<dai::node::Thermal>();
    ph = std::make_unique<param_handlers::ThermalParamHandler>(node, getName(), deviceName, rsCompat);
    ph->declareParams(thermalNode);
    boardSocket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>(param_handlers::ParamNames::BOARD_SOCKET_ID));
    setInOut(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
Thermal::~Thermal() = default;
void Thermal::setNames() {
    thermalQName = getName() + "_thermal";
    rawQName = getName() + "_thermal_raw";
}

void Thermal::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        utils::VideoEncoderConfig encConfig;
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_low_bandwidth_profile"));
        encConfig.bitrate = ph->getParam<int>("i_low_bandwidth_bitrate");
        encConfig.frameFreq = ph->getParam<int>("i_low_bandwidth_frame_freq");
        encConfig.quality = ph->getParam<int>("i_low_bandwidth_quality");
        encConfig.enabled = ph->getParam<bool>("i_low_bandwidth");

        thermalPub = setupOutput(pipeline, thermalQName, &thermalNode->color, ph->getParam<bool>("i_synced"), encConfig);
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        thermalRawPub = setupOutput(pipeline, rawQName, &thermalNode->temperature, ph->getParam<bool>("i_synced"));
    }
}

void Thermal::setupQueues(std::shared_ptr<dai::Device> device) {
    using param_handlers::ParamNames;
    if(ph->getParam<bool>(ParamNames::PUBLISH_TOPIC)) {
        auto tfPrefix = getOpticalFrameName(getSocketName(boardSocket));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP);
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG);
        convConfig.lowBandwidth = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);
        convConfig.encoding = dai::ImgFrame::Type::RGB888i;
        convConfig.addExposureOffset = ph->getParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET);
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(ParamNames::EXPOSURE_OFFSET));
        convConfig.reverseSocketOrder = ph->getParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER);

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER);
        pubConfig.socket = ph->getSocketID();
        pubConfig.calibrationFile = ph->getParam<std::string>(ParamNames::CALIBRATION_FILE);
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>(ParamNames::WIDTH);
        pubConfig.height = ph->getParam<int>(ParamNames::HEIGHT);
        pubConfig.maxQSize = ph->getParam<int>(ParamNames::MAX_Q_SIZE);

        thermalPub->setup(device, convConfig, pubConfig);
    }
    if(ph->getParam<bool>(ParamNames::PUBLISH_RAW)) {
        auto tfPrefix = getOpticalFrameName(getSocketName(boardSocket));

        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP);
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG);
        convConfig.lowBandwidth = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);
        convConfig.encoding = dai::ImgFrame::Type::RGB888i;
        convConfig.addExposureOffset = ph->getParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET);
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(ParamNames::EXPOSURE_OFFSET));
        convConfig.reverseSocketOrder = ph->getParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER);

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName() + "/raw_data";
        pubConfig.lazyPub = ph->getParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER);
        pubConfig.socket = ph->getSocketID();
        pubConfig.calibrationFile = ph->getParam<std::string>(ParamNames::CALIBRATION_FILE);
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>(ParamNames::WIDTH);
        pubConfig.height = ph->getParam<int>(ParamNames::HEIGHT);
        pubConfig.maxQSize = ph->getParam<int>(ParamNames::MAX_Q_SIZE);
        thermalRawPub->setup(device, convConfig, pubConfig);
    }
    confQ = thermalNode->inputConfig.createInputQueue();
}

void Thermal::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        thermalPub->closeQueue();
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        thermalRawPub->closeQueue();
    }
}

void Thermal::link(dai::Node::Input& in, int /*linkType*/) {
    thermalNode->temperature.link(in);
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> Thermal::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> pubs;
    if(ph->getParam<bool>("i_publish_topic") && ph->getParam<bool>("i_synced")) {
        pubs.push_back(thermalPub);
    }
    if(ph->getParam<bool>("i_publish_raw") && ph->getParam<bool>("i_synced")) {
        pubs.push_back(thermalRawPub);
    }
    return pubs;
}

void Thermal::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto thermalConf = ph->setRuntimeParams(params);
    confQ->send(thermalConf);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
