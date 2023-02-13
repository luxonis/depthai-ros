#pragma once

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

class RGB : public BaseNode {
   public:
    explicit RGB(const std::string& daiNodeName,
                 rclcpp::Node* node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 dai::CameraBoardSocket socket,
                 sensor_helpers::ImageSensor sensor,
                 bool publish);
    virtual ~RGB() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::CameraPublisher rgbPub, previewPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager, previewInfoManager;
    std::shared_ptr<dai::node::ColorCamera> colorCamNode;
    std::shared_ptr<dai::node::VideoEncoder> videoEnc;
    std::unique_ptr<param_handlers::RGBParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> colorQ, previewQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutColor, xoutPreview;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string ispQName, previewQName, controlQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver