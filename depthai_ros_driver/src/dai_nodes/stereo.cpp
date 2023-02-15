#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device)
    : BaseNode(daiNodeName, node, pipeline), it(node) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();
    left = std::make_unique<CameraSensor>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT, false);
    right = std::make_unique<CameraSensor>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT, false);

    ph = std::make_unique<param_handlers::StereoParamHandler>(daiNodeName);
    ph->declareParams(node, stereoCamNode);
    setXinXout(pipeline);
    left->link(stereoCamNode->left);
    right->link(stereoCamNode->right);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
}

void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutStereo = pipeline->create<dai::node::XLinkOut>();
    xoutStereo->setStreamName(stereoQName);
    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
        videoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>(getROSNode(), "i_low_bandwidth_quality"));
        stereoCamNode->disparity.link(videoEnc->input);
        videoEnc->bitstream.link(xoutStereo->input);
    } else {
        stereoCamNode->depth.link(xoutStereo->input);
    }
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    left->setupQueues(device);
    right->setupQueues(device);
    stereoQ = device->getOutputQueue(stereoQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    std::string tfPrefix;
    if(ph->getParam<bool>(getROSNode(), "i_align_depth")) {
        tfPrefix = getTFPrefix("rgb");
    } else {
        tfPrefix = getTFPrefix("right");
    }
    imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);

    stereoPub = it.advertiseCamera(getName() + "/image_raw", 1);
    infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
    auto info = sensor_helpers::getCalibInfo(*imageConverter,
                                             device,
                                             static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                             ph->getParam<int>(getROSNode(), "i_width"),
                                             ph->getParam<int>(getROSNode(), "i_height"));
    auto calibHandler = device->readCalibration();
    info.P[3] = calibHandler.getBaselineDistance() * 10.0;  // baseline in mm
    infoManager->setCameraInfo(info);

    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
        if(ph->getParam<bool>(getROSNode(), "i_output_disparity")) {
            stereoQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                           std::placeholders::_1,
                                           std::placeholders::_2,
                                           *imageConverter,
                                           stereoPub,
                                           infoManager,
                                           dai::RawImgFrame::Type::GRAY8));
        } else {
            // converting disp->depth
            stereoQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                           std::placeholders::_1,
                                           std::placeholders::_2,
                                           *imageConverter,
                                           stereoPub,
                                           infoManager,
                                           dai::RawImgFrame::Type::RAW8));
        }
    } else {
        stereoQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, stereoPub, infoManager));
    }
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    stereoQ->close();
}

void Stereo::link(const dai::Node::Input& in, int /*linkType*/) {
    stereoCamNode->depth.link(in);
}

dai::Node::Input Stereo::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
        return stereoCamNode->left;
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
        return stereoCamNode->right;
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}

void Stereo::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
