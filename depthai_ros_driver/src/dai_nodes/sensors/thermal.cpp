
#include "depthai_ros_driver/dai_nodes/sensors/thermal.hpp"

#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Thermal.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
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
    camNode = pipeline->create<dai::node::Thermal>();
    ph = std::make_unique<param_handlers::ThermalParamHandler>(node, getName(), deviceName, rsCompat);
    ph->declareParams(camNode);
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

        thermalPub = setupOutput(pipeline, thermalQName, &camNode->color, ph->getParam<bool>("i_synced"), encConfig);
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        thermalRawPub = setupOutput(pipeline, rawQName, &camNode->temperature, ph->getParam<bool>("i_synced"));
    }
}

void Thermal::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getOpticalFrameName(getSocketName(boardSocket));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        convConfig.encoding = dai::ImgFrame::Type::RGB888i;
        convConfig.addExposureOffset = ph->getParam<bool>("i_add_exposure_offset");
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
        convConfig.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        pubConfig.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
        pubConfig.calibrationFile = ph->getParam<std::string>("i_calibration_file");
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>("i_width");
        pubConfig.height = ph->getParam<int>("i_height");
        pubConfig.maxQSize = ph->getParam<int>("i_max_q_size");

        thermalPub->setup(device, convConfig, pubConfig);
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        auto tfPrefix = getOpticalFrameName(getSocketName(boardSocket));

        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        convConfig.encoding = dai::ImgFrame::Type::RGB888i;
        convConfig.addExposureOffset = ph->getParam<bool>("i_add_exposure_offset");
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
        convConfig.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName() + "/raw_data";
        pubConfig.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        pubConfig.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
        pubConfig.calibrationFile = ph->getParam<std::string>("i_calibration_file");
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>("i_width");
        pubConfig.height = ph->getParam<int>("i_height");
        pubConfig.maxQSize = ph->getParam<int>("i_max_q_size");
        thermalRawPub->setup(device, convConfig, pubConfig);
    }
}

void Thermal::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        thermalPub->closeQueue();
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        thermalRawPub->closeQueue();
    }
}

void Thermal::link(dai::Node::Input in, int /*linkType*/) {
    camNode->temperature.link(in);
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
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
