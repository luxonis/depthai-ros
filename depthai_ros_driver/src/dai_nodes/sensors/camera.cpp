#include "depthai_ros_driver/dai_nodes/sensors/camera.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Camera::Camera(const std::string& daiNodeName,
               std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<dai::Pipeline> pipeline,
               const std::string& deviceName, bool rsCompat,
               dai::CameraBoardSocket socket = dai::CameraBoardSocket::CAM_A,
               bool publish = true)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    camNode = pipeline->create<dai::node::Camera>()->build(socket);
    ;
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);
    // ph->declareParams(colorCamNode, sensor, publish);
    setInOut(pipeline);
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Camera::~Camera() = default;
void Camera::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
}

void Camera::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {
    for(auto outputName : outputNames) {
        if(ph->getParam<bool>(outputName + "_i_publish_topic")) {
            utils::VideoEncoderConfig encConfig;
            bool lowBandwidth = ph->getParam<bool>(outputName + "_i_low_bandwidth");
            encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>(outputName + "_i_low_bandwidth_profile"));
            encConfig.bitrate = ph->getParam<int>(outputName + "_i_low_bandwidth_bitrate");
            encConfig.frameFreq = ph->getParam<int>(outputName + "_i_low_bandwidth_frame_freq");
            encConfig.quality = ph->getParam<int>(outputName + "_i_low_bandwidth_quality");
            encConfig.enabled = lowBandwidth;
            auto width = ph->getParam<int>(outputName + "_i_width");
            auto height = ph->getParam<int>(outputName + "_i_height");
            auto fps = ph->getParam<float>(outputName + "i_fps");

            auto output = camNode->requestOutput(std::pair<int, int>(width, height));
            outputs.push_back(std::pair(outputName, output));
            rgbPub = setupOutput(pipeline, ispQName, *output, ph->getParam<bool>("i_synced"), encConfig);
        }
    }
    // xinControl = pipeline->create<dai::node::XLinkIn>();
    // xinControl->setStreamName(controlQName);
    // xinControl->out.link(colorCamNode->inputControl);
    controlQ = camNode->inputControl.createInputQueue(8, false);
}

void Camera::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getOpticalFrameName(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
        convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
        // if(ph->getParam<std::string>("i_color_order") == "BGR") {
        //     convConfig.encoding = dai::RawImgFrame::Type::BGR888i;
        // } else {
        //     convConfig.encoding = dai::RawImgFrame::Type::Camera888i;
        // }
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
        pubConfig.publishCompressed = ph->getParam<bool>("i_publish_compressed");

        rgbPub->setup(device, convConfig, pubConfig);
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        auto tfPrefix = getOpticalFrameName(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        pubConfig.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
        pubConfig.calibrationFile = ph->getParam<std::string>("i_calibration_file");
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>("i_preview_width");
        pubConfig.height = ph->getParam<int>("i_preview_height");
        pubConfig.maxQSize = ph->getParam<int>("i_max_q_size");
        pubConfig.topicSuffix = "/preview/image_raw";

        previewPub->setup(device, convConfig, pubConfig);
    };
    // controlQ = device->getInputQueue(controlQName);
}

void Camera::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        rgbPub->closeQueue();
        if(ph->getParam<bool>("i_enable_preview")) {
            previewPub->closeQueue();
        }
    }
    // controlQ->close();
}

void Camera::link(dai::Node::Input in, int linkType) {
    // if(linkType == static_cast<int>(link_types::CameraLinkType::video)) {
    //     colorCamNode->video.link(in);
    // } else if(linkType == static_cast<int>(link_types::CameraLinkType::isp)) {
    //     colorCamNode->isp.link(in);
    // } else if(linkType == static_cast<int>(link_types::CameraLinkType::preview)) {
    //     colorCamNode->preview.link(in);
    // } else {
    //     throw std::runtime_error("Link type not supported");
    // }
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> Camera::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> publishers;
    if(ph->getParam<bool>("i_synced")) {
        publishers.push_back(rgbPub);
    }
    return publishers;
}

void Camera::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(params);
    // controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
