#include "depthai_ros_driver/dai_nodes/nn/spatial_mobilenet.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

SpatialMobilenet::SpatialMobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    mobileNode = pipeline->create<dai::node::MobileNetSpatialDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
    ph->declareParams(mobileNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(mobileNode->input);
    setXinXout(pipeline);
}
SpatialMobilenet::~SpatialMobilenet() = default;
void SpatialMobilenet::setNames() {
    nnQName = getName() + "_nn";
}

void SpatialMobilenet::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    mobileNode->out.link(xoutNN->input);
}

void SpatialMobilenet::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode()->get_name());
    detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&SpatialMobilenet::mobilenetCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection3DArray>("~/" + getName() + "/detections", 10);
}
void SpatialMobilenet::closeQueues() {
    nnQ->close();
}

void SpatialMobilenet::mobilenetCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
    std::deque<vision_msgs::msg::Detection3DArray> deq;
    detConverter->toRosVisionMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        detPub->publish(currMsg);
        deq.pop_front();
    }
}

void SpatialMobilenet::link(const dai::Node::Input& in, int /*linkType*/) {
    mobileNode->out.link(in);
}

dai::Node::Input SpatialMobilenet::getInput(int linkType) {
    if(linkType == static_cast<int>(nn_helpers::link_types::SpatialNNLinkType::input)) {
        if(ph->getParam<bool>("i_disable_resize")) {
            return mobileNode->input;
        }
        return imageManip->inputImage;
    } else {
        return mobileNode->inputDepth;
    }
}

void SpatialMobilenet::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver