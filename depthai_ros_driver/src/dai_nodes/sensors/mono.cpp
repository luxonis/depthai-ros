#include "depthai_ros_driver/dai_nodes/sensors/mono.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Mono::Mono(const std::string& daiNodeName,
           std::shared_ptr<rclcpp::Node> node,
           std::shared_ptr<dai::Pipeline> pipeline,
           dai::CameraBoardSocket socket,
           dai_nodes::sensor_helpers::ImageSensor sensor,
           bool publish = true)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    monoCamNode = pipeline->create<dai::node::MonoCamera>();
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, socket);
    ph->declareParams(monoCamNode, sensor, publish);
    setXinXout(pipeline);
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Mono::~Mono() = default;
void Mono::setNames() {
    monoQName = getName() + "_mono";
    controlQName = getName() + "_control";
}

void Mono::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        utils::VideoEncoderConfig encConfig;
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_low_bandwidth_profile"));
        encConfig.bitrate = ph->getParam<int>("i_low_bandwidth_bitrate");
        encConfig.frameFreq = ph->getParam<int>("i_low_bandwidth_frame_freq");
        encConfig.quality = ph->getParam<int>("i_low_bandwidth_quality");
        encConfig.enabled = ph->getParam<bool>("i_low_bandwidth");

        imagePublisher = setupOutput(
            pipeline, monoQName, [&](auto input) { monoCamNode->out.link(input); }, ph->getParam<bool>("i_synced"), encConfig);
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(monoCamNode->inputControl);
}

void Mono::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getOpticalTFPrefix(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        utils::ImgConverterConfig convConf;
        convConf.tfPrefix = tfPrefix;
        convConf.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConf.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConf.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        convConf.encoding = dai::RawImgFrame::Type::GRAY8;
        convConf.addExposureOffset = ph->getParam<bool>("i_add_exposure_offset");
        convConf.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
        convConf.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");

        utils::ImgPublisherConfig pubConf;
        pubConf.daiNodeName = getName();
        pubConf.topicName = "~/" + getName();
        pubConf.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        pubConf.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
        pubConf.calibrationFile = ph->getParam<std::string>("i_calibration_file");
        pubConf.rectified = false;
        pubConf.width = ph->getParam<int>("i_width");
        pubConf.height = ph->getParam<int>("i_height");
        pubConf.maxQSize = ph->getParam<int>("i_max_q_size");
        pubConf.publishCompressed = ph->getParam<bool>("i_publish_compressed");

        imagePublisher->setup(device, convConf, pubConf);
    }
    controlQ = device->getInputQueue(controlQName);
}
void Mono::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        imagePublisher->closeQueue();
    }
    controlQ->close();
}

void Mono::link(dai::Node::Input in, int /*linkType*/) {
    monoCamNode->out.link(in);
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> Mono::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> publishers;
    if(ph->getParam<bool>("i_publish_topic") && ph->getParam<bool>("i_synced")) {
        publishers.push_back(imagePublisher);
    }
    return publishers;
}

void Mono::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
