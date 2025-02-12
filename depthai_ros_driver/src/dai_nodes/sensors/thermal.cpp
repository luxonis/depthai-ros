
#include "depthai_ros_driver/dai_nodes/sensors/thermal.hpp"

#include <memory>
#include <sensor_msgs/image_encodings.hpp>

#include "cv_bridge/cv_bridge.h"
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Thermal::Thermal(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline, dai::CameraFeatures camFeatures)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    camNode = pipeline->create<dai::node::Camera>();
    boardSocket = camFeatures.socket;
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, boardSocket);
    ph->declareParams(camNode, camFeatures, true);
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
Thermal::~Thermal() = default;
void Thermal::setNames() {
    thermalQName = getName() + "_thermal";
    rawQName = getName() + "thermal_raw";
}

void Thermal::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        utils::VideoEncoderConfig encConfig;
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_low_bandwidth_profile"));
        encConfig.bitrate = ph->getParam<int>("i_low_bandwidth_bitrate");
        encConfig.frameFreq = ph->getParam<int>("i_low_bandwidth_frame_freq");
        encConfig.quality = ph->getParam<int>("i_low_bandwidth_quality");
        encConfig.enabled = ph->getParam<bool>("i_low_bandwidth");

        thermalPub = setupOutput(pipeline, thermalQName, [&](auto input) { camNode->video.link(input); }, ph->getParam<bool>("i_synced"), encConfig);
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        xoutRaw = pipeline->create<dai::node::XLinkOut>();
        xoutRaw->setStreamName(rawQName);
        camNode->raw.link(xoutRaw->input);
    }
}

void Thermal::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getOpticalTFPrefix(getSocketName(boardSocket));

        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        convConfig.encoding = dai::RawImgFrame::Type::RGB888i;
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
        rawPub = image_transport::create_camera_publisher(getROSNode().get(), "~/" + getName() + "/raw_data/image_raw");
        rawQ = device->getOutputQueue(rawQName, ph->getParam<int>("i_max_q_size"), false);
        rawQ->addCallback([this](const std::string& name, const std::shared_ptr<dai::ADatatype>& data) { thermalRawCB(name, data); });
    }
}

void Thermal::thermalRawCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto temp = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    auto frame = temp->getCvFrame();
    cv_bridge::CvImage imgBridge;
    cv::Mat frameFp32(temp->getHeight(), temp->getWidth(), CV_32F);
    frame.convertTo(frameFp32, CV_32F);
    std_msgs::msg::Header header;
    sensor_msgs::msg::Image img_msg;
    header.stamp = getROSNode()->get_clock()->now();
    auto tfPrefix = getOpticalTFPrefix(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
    header.frame_id = tfPrefix;
    rawInfo.header = header;
    imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, frameFp32);
    imgBridge.toImageMsg(img_msg);
    rawPub.publish(img_msg, rawInfo);
}

void Thermal::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        thermalPub->closeQueue();
    }
    if(ph->getParam<bool>("i_publish_raw")) {
        rawQ->close();
    }
}

void Thermal::link(dai::Node::Input in, int /*linkType*/) {
    camNode->video.link(in);
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> Thermal::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> pubs;
    if(ph->getParam<bool>("i_publish_topic") && ph->getParam<bool>("i_synced")) {
        pubs.push_back(thermalPub);
    }
    return pubs;
}

void Thermal::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
