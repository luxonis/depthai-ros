#include "depthai_ros_driver/dai_nodes/nn.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
NN::NN(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(dai_node_name, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", dai_node_name.c_str());
    set_names();
    nn_node_ = pipeline->create<dai::node::NeuralNetwork>();
    param_handler_ = std::make_unique<param_handlers::NNParamHandler>(dai_node_name);
    param_handler_->declareParams(node, nn_node_);
    set_xin_xout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", dai_node_name.c_str());
};
void NN::set_names() {
    nn_q_name_ = getName() + "_nn";
}

void NN::set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) {
    xout_nn_ = pipeline->create<dai::node::XLinkOut>();
    xout_nn_->setStreamName(nn_q_name_);
    nn_node_->out.link(xout_nn_->input);
}

void NN::setupQueues(std::shared_ptr<dai::Device> device) {
    nn_q_ = device->getOutputQueue(nn_q_name_, param_handler_->get_param<int>(getROSNode(), "i_max_q_size"), false);
    nn_q_->addCallback(std::bind(&NN::nn_q_cb, this, std::placeholders::_1, std::placeholders::_2));
    nn_pub_ = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
}

void NN::nn_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);
    std::vector<std::int32_t> nn_frame = in_det->getFirstLayerInt32();
    cv::Mat nn_mat = cv::Mat(nn_frame);
    nn_mat = nn_mat.reshape(0, 256);
    cv::Mat cv_frame = decode_deeplab(nn_mat);
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    img_bridge.toImageMsg(img_msg);
    nn_pub_.publish(img_msg, nn_info_);
}

void NN::link(const dai::Node::Input& in, int link_type) {
    nn_node_->out.link(in);
}

dai::Node::Input NN::get_input(int link_type) {
    return nn_node_->input;
}

void NN::updateParams(const std::vector<rclcpp::Parameter>& params) {
    param_handler_->setRuntimeParams(getROSNode(), params);
}

cv::Mat NN::decode_deeplab(cv::Mat mat) {
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

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
