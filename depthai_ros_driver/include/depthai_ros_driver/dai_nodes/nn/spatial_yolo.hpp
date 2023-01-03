#pragma once

#include <string>

#include "depthai/depthai.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "vision_msgs/Detection3DArray.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {
class SpatialYolo : public BaseNode {
   public:
    SpatialYolo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline);
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void yoloCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    image_transport::CameraPublisher nnPub;
    std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> yoloNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    ros::Publisher detPub;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN;
    std::string nnQName;
};
}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver