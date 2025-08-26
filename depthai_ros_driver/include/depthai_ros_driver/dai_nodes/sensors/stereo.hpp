#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"

namespace dai {
class Pipeline;
class Device;
class MessageQueue;
class ADatatype;
class ImgFrame;
namespace node {
class ImageAlign;
class StereoDepth;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler;
}

namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { stereo, left, right, align };
};

namespace sensor_helpers {
class ImagePubliser;
}
class RGBD;
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName,
                    std::shared_ptr<rclcpp::Node> node,
                    std::shared_ptr<dai::Pipeline> pipeline,
                    std::shared_ptr<dai::Device> device,
                    bool rsCompat,
                    dai::CameraBoardSocket leftSocket = dai::CameraBoardSocket::CAM_B,
                    dai::CameraBoardSocket rightSocket = dai::CameraBoardSocket::CAM_C);
    ~Stereo();
    void setupQueues(std::shared_ptr<dai::Device> dvice) override;
    void link(dai::Node::Input& in, int linkType = 1) override;
    dai::Node::Input& getInput(int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;
    std::shared_ptr<dai::node::StereoDepth> getUnderlyingNode();
    bool isAligned();
    dai::CameraBoardSocket getSocketID();
    std::shared_ptr<SensorWrapper> getLeftSensor();
    std::shared_ptr<SensorWrapper> getRightSensor();

   private:
    void setupStereoQueue(std::shared_ptr<dai::Device> device);
    void setupLeftRectQueue(std::shared_ptr<dai::Device> device);
    void setupRightRectQueue(std::shared_ptr<dai::Device> device);
    void setupRectQueue(std::shared_ptr<dai::Device> device, dai::CameraFeatures& sensorInfo, std::shared_ptr<sensor_helpers::ImagePublisher> pub, bool isLeft);
    std::shared_ptr<sensor_helpers::ImagePublisher> stereoPub, leftRectPub, rightRectPub;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::ImageAlign> alignNode;
    dai::Platform platform;
    std::unique_ptr<RGBD> rgbdNodeLeft, rgbdNodeRight;
    std::shared_ptr<SensorWrapper> left, right;
    std::unique_ptr<BaseNode> featureTrackerLeftR, featureTrackerRightR, nnNodeLeft, nnNodeRight;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::MessageQueue> leftRectQ, rightRectQ;
    std::string stereoQName, leftRectQName, rightRectQName;
    dai::CameraFeatures leftSensInfo, rightSensInfo;
    bool aligned;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
