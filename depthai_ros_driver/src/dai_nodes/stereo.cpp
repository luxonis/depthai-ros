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
};
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
}

void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutStereo = pipeline->create<dai::node::XLinkOut>();
    xoutStereo->setStreamName(stereoQName);
    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
        videoEnc = pipeline->create<dai::node::VideoEncoder>();
        videoEnc->setQuality(ph->getParam<int>(getROSNode(), "i_low_bandwidth_quality"));
        videoEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
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
    std::string frameName;
    if(ph->getParam<bool>(getROSNode(), "i_align_depth")) {
        frameName = "rgb";
    } else {
        frameName = "right";
    }
    auto tfPrefix = std::string(getROSNode().getNamespace()) + "_" + frameName;
    tfPrefix.erase(0, 1);
    imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
    stereoQ->addCallback(std::bind(&Stereo::stereoQCB, this, std::placeholders::_1, std::placeholders::_2));
    stereoPub = it.advertiseCamera(getName() + "/image_raw", 1);
    auto calibHandler = device->readCalibration();
    try {
        stereoInfo = imageConverter->calibrationToCameraInfo(calibHandler,
                                                             static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                             ph->getParam<int>(getROSNode(), "i_width"),
                                                             ph->getParam<int>(getROSNode(), "i_height"));
        stereoInfo.P[3] = calibHandler.getBaselineDistance() * 10.0;  // baseline in mm
    } catch(std::runtime_error& e) {
        ROS_ERROR("No calibration! Publishing empty camera_info.");
    }
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    stereoQ->close();
}
void Stereo::stereoQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::deque<sensor_msgs::Image> deq;
    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth"))
        imageConverter->toRosMsgFromBitStream(img, deq, dai::RawImgFrame::Type::RAW8, stereoInfo);
    else
        imageConverter->toRosMsg(img, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        stereoInfo.header = currMsg.header;
        stereoPub.publish(currMsg, stereoInfo);
        deq.pop_front();
    }
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
