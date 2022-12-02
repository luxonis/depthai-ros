#include "depthai_ros_driver/dai_nodes/nn_wrappers/yolo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn_wrappers {

Yolo::Yolo(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    yoloNode = pipeline->create<dai::node::YoloDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    paramHandler = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    paramHandler->declareParams(node, yoloNode, imageManip);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(yoloNode->input);
    setXinXout(pipeline);
}

void Yolo::setNames() {
    nnQName = getName() + "_nn";
}

void Yolo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    yoloNode->out.link(xoutNN->input);

}

void Yolo::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
    // nnPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
    nnQ->addCallback(std::bind(&Yolo::yoloCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10);
}

void Yolo::yoloCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    auto detections = inDet->detections;
    vision_msgs::msg::Detection2DArray rosDet;

    rosDet.detections.resize(detections.size());
    auto labelMap = paramHandler->get_param<std::vector<std::string>>(getROSNode(), "i_label_map");
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

void Yolo::link(const dai::Node::Input& in, int linkType) {
    yoloNode->out.link(in);
}

dai::Node::Input Yolo::getInput(int linkType) {
    return imageManip->inputImage;
}

void Yolo::updateParams(const std::vector<rclcpp::Parameter>& params) {
    paramHandler->setRuntimeParams(getROSNode(), params);
}
}  // namespace nn_wrappers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver