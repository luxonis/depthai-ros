#include "depthai_ros_driver/dai_nodes/spatial_detections.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace daiNodes {
NN::NN(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    nnNode = pipeline->create<dai::node::NeuralNetwork>();
    paramHandler = std::make_unique<paramHandlers::NNParamHandler>(daiNodeName);
    paramHandler->declareParams(node, nnNode);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void NN::setNames() {
    nnQName_ = getName() + "_nn";
}

void NN::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    nnNode->out.link(xoutNN->input);
}

void NN::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName_, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
    nnQ->addCallback(std::bind(&NN::nnQCB, this, std::placeholders::_1, std::placeholders::_2));
    nnPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
}

void NN::nnQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);
    std::vector<std::int32_t> nn_frame = in_det->getFirstLayerInt32();
    cv::Mat nn_mat = cv::Mat(nn_frame);
    nn_mat = nn_mat.reshape(0, 256);
    cv::Mat cv_frame = decodeDeeplab(nn_mat);
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    img_bridge.toImageMsg(img_msg);
    nnPub.publish(img_msg, nnInfo);
}

void NN::link(const dai::Node::Input& in, int linkType) {
    nnNode->out.link(in);
}

dai::Node::Input NN::getInput(int linkType) {
    return nnNode->input;
}

void NN::updateParams(const std::vector<rclcpp::Parameter>& params) {
    paramHandler->setRuntimeParams(getROSNode(), params);
}

cv::Mat NN::decodeDeeplab(cv::Mat mat) {
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

}  // namespace daiNodes
}  // namespace depthai_ros_driver
