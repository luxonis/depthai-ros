#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/param_handlers/tof_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
ToF::ToF(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline, dai::CameraBoardSocket socket)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    camNode = pipeline->create<dai::node::Camera>();
    tofNode = pipeline->create<dai::node::ToF>();
    boardSocket = socket;
    ph = std::make_unique<param_handlers::ToFParamHandler>(node, daiNodeName);
    ph->declareParams(camNode, tofNode);
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
ToF::~ToF() = default;
void ToF::setNames() {
    tofQName = getName() + "_tof";
}

void ToF::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        camNode->raw.link(tofNode->input);
        bool align = boardSocket == dai::CameraBoardSocket::CAM_A;
        std::function<void(dai::Node::Input)> tofLinkChoice;
        if(align) {
            tofLinkChoice = [&](auto input) { tofNode->depth.link(input); };
        } else {
            alignNode = pipeline->create<dai::node::ImageAlign>();
            tofNode->depth.link(alignNode->input);
            tofLinkChoice = [&](auto input) { alignNode->outputAligned.link(input); };
        }
        utils::VideoEncoderConfig encConfig;
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_low_bandwidth_profile"));
        encConfig.bitrate = ph->getParam<int>("i_low_bandwidth_bitrate");
        encConfig.frameFreq = ph->getParam<int>("i_low_bandwidth_frame_freq");
        encConfig.quality = ph->getParam<int>("i_low_bandwidth_quality");
        encConfig.enabled = ph->getParam<bool>("i_low_bandwidth");

        tofPub = setupOutput(pipeline, tofQName, tofLinkChoice, ph->getParam<bool>("i_synced"), encConfig);
    }
}

void ToF::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getOpticalTFPrefix(getSocketName(boardSocket));

        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        convConfig.encoding = dai::RawImgFrame::Type::RAW8;
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

        tofPub->setup(device, convConfig, pubConfig);
    }
}
void ToF::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        tofPub->closeQueue();
    }
}

dai::Node::Input ToF::getInput(int /*linkType*/) {
    return alignNode->inputAlignTo;
}

void ToF::link(dai::Node::Input in, int /*linkType*/) {
    tofNode->depth.link(in);
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> ToF::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> pubs;
    if(ph->getParam<bool>("i_publish_topic") && ph->getParam<bool>("i_synced")) {
        pubs.push_back(tofPub);
    }
    return pubs;
}

void ToF::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
