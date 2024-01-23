#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class NeuralNetwork;
class ImageManip;
class XLinkOut;
}  // namespace node
namespace ros {
class ImageConverter;
}
}  // namespace dai
namespace camera_info_manager {
class CameraInfoManager;
}
namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class NNParamHandler;
}
namespace dai_nodes {
namespace nn {
class Segmentation : public BaseNode {
   public:
    Segmentation(const std::string& daiNodeName,
                 ros::NodeHandle node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A);
    ~Segmentation();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0);
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    cv::Mat decodeDeeplab(cv::Mat mat);
    image_transport::ImageTransport it;
    void segmentationCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::vector<std::string> labelNames;
    image_transport::CameraPublisher nnPub, ptPub;
    sensor_msgs::CameraInfo nnInfo;
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::node::NeuralNetwork> segNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ, ptQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN, xoutPT;
    std::string nnQName, ptQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver