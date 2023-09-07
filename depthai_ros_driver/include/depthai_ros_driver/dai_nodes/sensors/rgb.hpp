#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class DataInputQueue;
enum class CameraBoardSocket;
class ADatatype;
namespace node {
class ColorCamera;
class XLinkIn;
class XLinkOut;
class VideoEncoder;
}  // namespace node
namespace ros {
class ImageConverter;
}
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler;
}
namespace dai_nodes {

namespace sensor_helpers {
struct ImageSensor;
}

class RGB : public BaseNode {
   public:
    explicit RGB(const std::string& daiNodeName,
                 ros::NodeHandle node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 dai::CameraBoardSocket socket,
                 sensor_helpers::ImageSensor sensor,
                 bool publish);
    ~RGB();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::ImageTransport it;
    image_transport::CameraPublisher rgbPubIT, previewPubIT;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager, previewInfoManager;
    std::shared_ptr<dai::node::ColorCamera> colorCamNode;
    std::shared_ptr<dai::node::VideoEncoder> videoEnc;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> colorQ, previewQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutColor, xoutPreview;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string ispQName, previewQName, controlQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver