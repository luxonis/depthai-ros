#include "depthai_ros_driver/dai_nodes/nn_wrappers/segmentation.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn_wrappers {

Segmentation::Segmentation(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    segNode = pipeline->create<dai::node::NeuralNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, segNode, imageManip);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(segNode->input);
    setXinXout(pipeline);
}

void Segmentation::setNames() {
    nnQName = getName() + "_nn";
}

void Segmentation::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    segNode->out.link(xoutNN->input);
}

void Segmentation::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    nnPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
    nnQ->addCallback(std::bind(&Segmentation::segmentationCB, this, std::placeholders::_1, std::placeholders::_2));
}

void Segmentation::closeQueues() {
    nnQ->close();
}

void Segmentation::segmentationCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);
    std::vector<std::int32_t> nn_frame = in_det->getFirstLayerInt32();
    cv::Mat nn_mat = cv::Mat(nn_frame);
    nn_mat = nn_mat.reshape(0, 256);
    cv::Mat cv_frame = decodeDeeplab(nn_mat);
    auto currTime = getROSNode()->get_clock()->now();
    cv_bridge::CvImage imgBridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    imgBridge.toImageMsg(img_msg);
    nnPub.publish(img_msg, nnInfo);
}
cv::Mat Segmentation::decodeDeeplab(cv::Mat mat) {
    cv::Mat out = mat.mul(255 / 21);
    out.convertTo(out, CV_8UC1);
    cv::Mat colors = cv::Mat(256, 1, CV_8UC3);
    cv::applyColorMap(out, colors, cv::COLORMAP_JET);
    for(int row = 0; row < out.rows; ++row) {
        uchar* p = out.ptr(row);
        for(int col = 0; col < out.cols; ++col) {
            if(*p++ == 0) {
                colors.at<cv::Vec3b>(row, col)[0] = 0;
                colors.at<cv::Vec3b>(row, col)[1] = 0;
                colors.at<cv::Vec3b>(row, col)[2] = 0;
            }
        }
    }
    return colors;
}
void Segmentation::link(const dai::Node::Input& in, int linkType) {
    segNode->out.link(in);
}

dai::Node::Input Segmentation::getInput(int linkType) {
    return imageManip->inputImage;
}

void Segmentation::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}
}  // namespace nn_wrappers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver