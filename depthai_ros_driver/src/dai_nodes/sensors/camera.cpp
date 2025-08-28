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
               const std::string& deviceName,
               bool rsCompat,
               dai::CameraBoardSocket socket = dai::CameraBoardSocket::CAM_A,
               bool publish = true)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    camNode = pipeline->create<dai::node::Camera>()->build(socket);
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);
    ph->declareParams(camNode, publish);
    setInOut(pipeline);
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Camera::~Camera() = default;
void Camera::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
}

std::shared_ptr<dai::node::Camera> Camera::getUnderlyingNode() {
    return camNode;
}
void Camera::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {
    using ParamNames = param_handlers::ParamNames;
    auto width = ph->getParam<int>(ParamNames::WIDTH);
    auto height = ph->getParam<int>(ParamNames::HEIGHT);
    auto fps = ph->getParam<float>(ParamNames::FPS);
    dai::ImgFrame::Type type = dai::ImgFrame::Type::NV12;
    if(ph->getParam<bool>("i_enable_default_output")) {
        defaultOut = camNode->requestOutput(std::pair<int, int>(width, height),
                                            type,
                                            utils::getValFromMap(ph->getParam<std::string>(ParamNames::RESIZE_MODE), sensor_helpers::resizeModeMap),
                                            fps,
                                            ph->getParam<bool>(ParamNames::UNDISTORTED));
    }
    if(ph->getParam<bool>(ParamNames::PUBLISH_TOPIC)) {
        utils::VideoEncoderConfig encConfig;
        bool lowBandwidth = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>(ParamNames::LOW_BANDWIDTH_PROFILE));
        encConfig.bitrate = ph->getParam<int>(ParamNames::LOW_BANDWIDTH_BITRATE);
        encConfig.frameFreq = ph->getParam<int>(ParamNames::LOW_BANDWIDTH_FRAME_FREQ);
        encConfig.quality = ph->getParam<int>(ParamNames::LOW_BANDWIDTH_QUALITY);
        encConfig.enabled = lowBandwidth;

        rgbPub = setupOutput(pipeline, ispQName, defaultOut, ph->getParam<bool>(ParamNames::SYNCED), encConfig);
    }
}

void Camera::setupQueues(std::shared_ptr<dai::Device> device) {
    using ParamNames = param_handlers::ParamNames;
    controlQ = camNode->inputControl.createInputQueue(8, false);
    if(ph->getParam<bool>(ParamNames::PUBLISH_TOPIC)) {
        auto tfPrefix = getOpticalFrameName(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>(ParamNames::BOARD_SOCKET_ID))));
        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP);
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG);
        convConfig.lowBandwidth = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);
        convConfig.addExposureOffset = ph->getParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET);
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(ParamNames::EXPOSURE_OFFSET));
        convConfig.reverseSocketOrder = ph->getParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER);

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER);
        pubConfig.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>(ParamNames::BOARD_SOCKET_ID));
        pubConfig.calibrationFile = ph->getParam<std::string>(ParamNames::CALIBRATION_FILE);
        pubConfig.undistorted = ph->getParam<bool>(ParamNames::UNDISTORTED);
        pubConfig.width = ph->getParam<int>(ParamNames::WIDTH);
        pubConfig.height = ph->getParam<int>(ParamNames::HEIGHT);
        pubConfig.maxQSize = ph->getParam<int>(ParamNames::MAX_Q_SIZE);
        pubConfig.publishCompressed = ph->getParam<bool>(ParamNames::PUBLISH_COMPRESSED);

        rgbPub->setup(device, convConfig, pubConfig);
    }
}

void Camera::closeQueues() {
    if(ph->getParam<bool>(param_handlers::ParamNames::PUBLISH_TOPIC)) {
        rgbPub->closeQueue();
    }
}

void Camera::link(dai::Node::Input& in, int /* linkType */) {
    if(ph->getParam<bool>("i_enable_default_output")) {
        defaultOut->link(in);
    } else {
        throw std::runtime_error("Default output is disabled! Please reenable it via \"i_enable_default_out\" parameter");
    }
}

dai::Node::Output* Camera::getDefaultOut() {
    return defaultOut;
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> Camera::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> publishers;
    if(ph->getParam<bool>(param_handlers::ParamNames::PUBLISH_TOPIC)) {
        publishers.push_back(rgbPub);
    }
    return publishers;
}

void Camera::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
