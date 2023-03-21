#include "depthai_ros_driver/dai_nodes/nn/detection.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Detection::Detection(const std::string& daiNodeName,
                     rclcpp::Node* node,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     std::shared_ptr<dai::node::DetectionNetwork> nn)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    detectionNode = nn;
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
    ph->declareParams(detectionNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(detectionNode->input);
    setXinXout(pipeline);
}
Detection::~Detection() = default;
void Detection::setNames() {
    nnQName = getName() + "_nn";
    ptQName = getName() + "_pt";
}

void Detection::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    detectionNode->out.link(xoutNN->input);
    if(ph->getParam<bool>("i_enable_passthrough")) {
        xoutPT = pipeline->create<dai::node::XLinkOut>();
        xoutPT->setStreamName(ptQName);
        detectionNode->passthrough.link(xoutPT->input);
    }
}

void Detection::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
    auto tfPrefix = getTFPrefix("rgb");
    detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
        tfPrefix + "_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10);
    nnQ->addCallback(std::bind(&Detection::detectionCB, this, std::placeholders::_1, std::placeholders::_2));
    
    if(ph->getParam<bool>("i_enable_passthrough")) {
        ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                *imageConverter,
                                                                device,
                                                                dai::CameraBoardSocket::RGB,
                                                                imageManip->initialConfig.getResizeWidth(),
                                                                imageManip->initialConfig.getResizeWidth()));

        ptPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough");
        ptQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, ptPub, infoManager));
    }
}
void Detection::closeQueues() {
    nnQ->close();
    if(ph->getParam<bool>("i_enable_passthrough")) {
    ptQ->close();
    }
}

void Detection::detectionCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    std::deque<vision_msgs::msg::Detection2DArray> deq;
    detConverter->toRosMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        detPub->publish(currMsg);
        deq.pop_front();
    }
}

void Detection::link(const dai::Node::Input& in, int /*linkType*/) {
    detectionNode->out.link(in);
}

dai::Node::Input Detection::getInput(int /*linkType*/) {
    if(ph->getParam<bool>("i_disable_resize")) {
        return detectionNode->input;
    }
    return imageManip->inputImage;
}

void Detection::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver