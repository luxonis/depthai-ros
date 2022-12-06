#include "depthai_ros_driver/dai_nodes/nn/mobilenet.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Mobilenet::Mobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    mobileNode = pipeline->create<dai::node::MobileNetDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, mobileNode, imageManip);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(mobileNode->input);
    setXinXout(pipeline);
}

void Mobilenet::setNames() {
    nnQName = getName() + "_nn";
}

void Mobilenet::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    mobileNode->out.link(xoutNN->input);
}

void Mobilenet::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    // nnPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
    nnQ->addCallback(std::bind(&Mobilenet::mobilenetCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10);
}
void Mobilenet::closeQueues() {
    nnQ->close();
}

void Mobilenet::mobilenetCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    auto detections = inDet->detections;
    vision_msgs::msg::Detection2DArray rosDet;
    rosDet.header.stamp = getROSNode()->get_clock()->now();
    rosDet.header.frame_id = std::string(getROSNode()->get_name()) + "_rgb_camera_optical_frame";
    rosDet.detections.resize(detections.size());
    auto labelMap = ph->getParam<std::vector<std::string>>(getROSNode(), "i_label_map");
    for(size_t i = 0; i < detections.size(); i++) {
        uint16_t label = detections[i].label;
        std::string labelName = std::to_string(label);
        if(label < labelMap.size()) {
            labelName = labelMap[label];
        }
        rosDet.detections[i].results.resize(1);
        rosDet.detections[i].results[0].hypothesis.class_id = labelName;
        rosDet.detections[i].results[0].hypothesis.score = detections[i].confidence;
        rosDet.detections[i].results[0].hypothesis.score = detections[i].confidence;
        int xMin, yMin, xMax, yMax;
        xMin = detections[i].xmin;
        yMin = detections[i].ymin;
        xMax = detections[i].xmax;
        yMax = detections[i].ymax;
        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;
        rosDet.detections[i].bbox.center.position.x = xCenter;
        rosDet.detections[i].bbox.center.position.y = yCenter;
        rosDet.detections[i].bbox.size_x = xSize;
        rosDet.detections[i].bbox.size_y = ySize;
    }

    detPub->publish(rosDet);
}

void Mobilenet::link(const dai::Node::Input& in, int linkType) {
    mobileNode->out.link(in);
}

dai::Node::Input Mobilenet::getInput(int linkType) {
    return imageManip->inputImage;
}

void Mobilenet::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}

}  // namespace nn_wrappers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver