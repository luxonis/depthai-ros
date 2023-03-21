#include "depthai_ros_driver/dai_nodes/nn/spatial_detection.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

SpatialDetection::SpatialDetection(const std::string& daiNodeName,
                                   rclcpp::Node* node,
                                   std::shared_ptr<dai::Pipeline> pipeline,
                                   std::shared_ptr<dai::node::SpatialDetectionNetwork> nn)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    spatialNode = nn;
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
    ph->declareParams(spatialNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(spatialNode->input);
    setXinXout(pipeline);
}
SpatialDetection::~SpatialDetection() = default;
void SpatialDetection::setNames() {
    nnQName = getName() + "_nn";
    ptQName = getName() + "_pt";
    ptDepthQName = getName() + "_pt_depth";
}

void SpatialDetection::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    spatialNode->out.link(xoutNN->input);
    if(ph->getParam<bool>("i_enable_passthrough")) {
        xoutPT = pipeline->create<dai::node::XLinkOut>();
        xoutPT->setStreamName(ptQName);
        spatialNode->passthrough.link(xoutPT->input);
    }
    if(ph->getParam<bool>("i_enable_passthrough_depth")) {
        xoutPTDepth = pipeline->create<dai::node::XLinkOut>();
        xoutPTDepth->setStreamName(ptDepthQName);
        spatialNode->passthroughDepth.link(xoutPTDepth->input);
    }
}

void SpatialDetection::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
    auto tfPrefix = getTFPrefix("rgb");
    detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(
        tfPrefix + "_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&SpatialDetection::spatialCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection3DArray>("~/" + getName() + "/spatial_detections", 10);

    if(ph->getParam<bool>("i_enable_passthrough")) {
        ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
        ptImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        ptInfoMan = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        ptInfoMan->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                              *ptImageConverter,
                                                              device,
                                                              dai::CameraBoardSocket::RGB,
                                                              imageManip->initialConfig.getResizeWidth(),
                                                              imageManip->initialConfig.getResizeWidth()));

        ptPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough");
        ptQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *ptImageConverter, ptPub, ptInfoMan));
    }

    if(ph->getParam<bool>("i_enable_passthrough_depth")) {
        dai::CameraBoardSocket socket = dai::CameraBoardSocket::RGB;
        if(!getROSNode()->get_parameter("stereo.i_align_depth").as_bool()) {
            tfPrefix = getTFPrefix("right");
            socket = dai::CameraBoardSocket::RIGHT;
        };
        ptDepthQ = device->getOutputQueue(ptDepthQName, ph->getParam<int>("i_max_q_size"), false);
        ptDepthImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        ptDepthInfoMan = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        ptDepthInfoMan->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                   *ptDepthImageConverter,
                                                                   device,
                                                                   socket,
                                                                   getROSNode()->get_parameter("stereo.i_width").as_int(),
                                                                   getROSNode()->get_parameter("stereo.i_height").as_int()));

        ptDepthPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough_depth");
        ptDepthQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *ptDepthImageConverter, ptDepthPub, ptDepthInfoMan));
    }
}
void SpatialDetection::closeQueues() {
    nnQ->close();
    if(ph->getParam<bool>("i_enable_passthrough")) {
        ptQ->close();
    }
    if(ph->getParam<bool>("i_enable_passthrough_depth")) {
        ptDepthQ->close();
    }
}

void SpatialDetection::spatialCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
    std::deque<vision_msgs::msg::Detection3DArray> deq;
    detConverter->toRosVisionMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        detPub->publish(currMsg);
        deq.pop_front();
    }
}

void SpatialDetection::link(const dai::Node::Input& in, int /*linkType*/) {
    spatialNode->out.link(in);
}

dai::Node::Input SpatialDetection::getInput(int linkType) {
    if(linkType == static_cast<int>(nn_helpers::link_types::SpatialNNLinkType::input)) {
        if(ph->getParam<bool>("i_disable_resize")) {
            return spatialNode->input;
        }
        return imageManip->inputImage;
    } else {
        return spatialNode->inputDepth;
    }
}

void SpatialDetection::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver