#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace dai {
class Pipeline;
class Device;
class ADatatype;
namespace node {
class ImageAlign;
class RGBD;
}  // namespace node
}  // namespace dai

namespace depthai_bridge {
class PointCloudConverter;
};

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class RGBDParamHandler;
}
namespace dai_nodes {
namespace link_types {
enum class RGBDLinkType { rgb, depth };
};
class SensorWrapper;
class ToF;
class RGBD : public BaseNode {
   public:
    explicit RGBD(const std::string& daiNodeName,
                  std::shared_ptr<rclcpp::Node> node,
                  std::shared_ptr<dai::Pipeline> pipeline,
                  std::shared_ptr<dai::Device> device,
                  bool rsCompat,
                  SensorWrapper& camNode,
                  std::shared_ptr<dai::node::StereoDepth> stereo,
                  bool aligned = false);
    explicit RGBD(const std::string& daiNodeName,
                  std::shared_ptr<rclcpp::Node> node,
                  std::shared_ptr<dai::Pipeline> pipeline,
                  std::shared_ptr<dai::Device> device,
                  bool rsCompat,
                  SensorWrapper& camNode,
                  ToF& tofNode,
                  bool aligned = false);
    ~RGBD();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input& getInput(int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::shared_ptr<dai::node::RGBD> getUnderlyingNode();

   private:
    std::unique_ptr<depthai_bridge::PointCloudConverter> pclConv;
    void pclCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPub;
    std::shared_ptr<dai::node::RGBD> rgbdNode;
    std::shared_ptr<dai::node::ImageAlign> align;
    std::unique_ptr<param_handlers::RGBDParamHandler> ph;
    std::shared_ptr<dai::MessageQueue> pclQ;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
