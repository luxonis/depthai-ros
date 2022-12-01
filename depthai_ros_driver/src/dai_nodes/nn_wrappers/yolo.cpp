#include "depthai_ros_driver/dai_nodes/nn_wrappers/yolo.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn_wrappers {

Yolo::Yolo(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline){
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    yoloNode = pipeline->create<dai::node::YoloDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
}
void Yolo::yoloCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
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

}
}  // namespace dai_nodes
}  // namespace depthai_ros_driver