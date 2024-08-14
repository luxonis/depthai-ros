#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
class ImgFrame;
namespace node {
class StereoDepth;
class XLinkOut;
class VideoEncoder;
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
enum class StereoLinkType { stereo, left, right };
};

namespace sensor_helpers {
class ImagePubliser;
}
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName,
                    std::shared_ptr<rclcpp::Node> node,
                    std::shared_ptr<dai::Pipeline> pipeline,
                    std::shared_ptr<dai::Device> device,
                    dai::CameraBoardSocket leftSocket = dai::CameraBoardSocket::CAM_B,
                    dai::CameraBoardSocket rightSocket = dai::CameraBoardSocket::CAM_C);
    ~Stereo();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> dvice) override;
    void link(dai::Node::Input in, int linkType = 1) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;

   private:
    void setupStereoQueue(std::shared_ptr<dai::Device> device);
    void setupLeftRectQueue(std::shared_ptr<dai::Device> device);
    void setupRightRectQueue(std::shared_ptr<dai::Device> device);
    void setupRectQueue(std::shared_ptr<dai::Device> device, dai::CameraFeatures& sensorInfo, std::shared_ptr<sensor_helpers::ImagePublisher> pub, bool isLeft);
    /*
     * This callback is used to synchronize left and right rectified frames
     * It is called every 10ms and it publishes the frames if they are synchronized
     * If they are not synchronized, it prints a warning message
     */
    void syncTimerCB();
    std::shared_ptr<sensor_helpers::ImagePublisher> stereoPub, leftRectPub, rightRectPub;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> stereoEnc, leftRectEnc, rightRectEnc;
    std::unique_ptr<SensorWrapper> left;
    std::unique_ptr<SensorWrapper> right;
    std::unique_ptr<BaseNode> featureTrackerLeftR, featureTrackerRightR, nnNode;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> leftRectQ, rightRectQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo, xoutLeftRect, xoutRightRect;
    std::string stereoQName, leftRectQName, rightRectQName;
    dai::CameraFeatures leftSensInfo, rightSensInfo;
    rclcpp::TimerBase::SharedPtr syncTimer;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
