#pragma once

#include "camera_info_manager/camera_info_manager.h"
#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { left, right };
}
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device);
    virtual ~Stereo() = default;
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0);
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void stereoQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    image_transport::ImageTransport it;
    image_transport::CameraPublisher stereoPub;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> videoEnc;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<BaseNode> left;
    std::unique_ptr<BaseNode> right;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> stereoQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string stereoQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver