#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class StereoDepth;
class XLinkOut;
class VideoEncoder;
}  // namespace node
namespace ros {
class ImageConverter;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler;
}

namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { left, right };
};

struct StereoSensorInfo {
    std::string name;
    dai::CameraBoardSocket socket;
};

class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName,
                    rclcpp::Node* node,
                    std::shared_ptr<dai::Pipeline> pipeline,
                    std::shared_ptr<dai::Device> device,
                    StereoSensorInfo leftInfo = StereoSensorInfo{"left", dai::CameraBoardSocket::LEFT},
                    StereoSensorInfo rightInfo = StereoSensorInfo{"right", dai::CameraBoardSocket::RIGHT});
    ~Stereo();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void setupStereoQueue(std::shared_ptr<dai::Device> device);
    void setupLeftRectQueue(std::shared_ptr<dai::Device> device);
    void setupRightRectQueue(std::shared_ptr<dai::Device> device);
    std::unique_ptr<dai::ros::ImageConverter> stereoConv, leftRectConv, rightRectConv;
    image_transport::CameraPublisher stereoPub, leftRectPub, rightRectPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> stereoIM, leftRectIM, rightRectIM;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> stereoEnc, leftRectEnc, rightRectEnc;
    std::unique_ptr<SensorWrapper> left;
    std::unique_ptr<SensorWrapper> right;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> stereoQ, leftRectQ, rightRectQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo, xoutLeftRect, xoutRightRect;
    std::string stereoQName, leftRectQName, rightRectQName;
    StereoSensorInfo leftSensInfo, rightSensInfo;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver