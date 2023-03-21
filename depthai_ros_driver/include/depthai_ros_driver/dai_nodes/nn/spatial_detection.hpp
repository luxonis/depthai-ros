#pragma once

#include <memory>
#include <string>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "image_transport/camera_publisher.hpp"
#include "rclcpp/publisher.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class SpatialDetectionNetwork;
class ImageManip;
class XLinkOut;
}  // namespace node
namespace ros {
class SpatialDetectionConverter;
class ImageConverter;
}
}  // namespace dai
namespace camera_info_manager {
class CameraInfoManager;
}
namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class NNParamHandler;
}
namespace dai_nodes {
namespace nn {

class SpatialDetection : public BaseNode {
   public:
    SpatialDetection(const std::string& daiNodeName,
                     rclcpp::Node* node,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     std::shared_ptr<dai::node::SpatialDetectionNetwork> nn);
    ~SpatialDetection();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void spatialCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detPub;
    std::unique_ptr<dai::ros::ImageConverter> ptImageConverter, ptDepthImageConverter;
    image_transport::CameraPublisher ptPub, ptDepthPub;
    sensor_msgs::msg::CameraInfo ptInfo, ptDepthInfo;
    std::shared_ptr<camera_info_manager::CameraInfoManager> ptInfoMan, ptDepthInfoMan;
    std::shared_ptr<dai::node::SpatialDetectionNetwork> spatialNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ, ptQ, ptDepthQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN, xoutPT, xoutPTDepth;
    std::string nnQName, ptQName, ptDepthQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver