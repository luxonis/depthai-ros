#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/parametersConfig.h"
namespace dai {
class Pipeline;
class Device;
class ADatatype;
class ImgFrame;
class DataOutputQueue;
namespace node {
class StereoDepth;
}  // namespace node
namespace ros {
class Timer;
}  // namespace ros
}  // namespace dai

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler;
}

namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { stereo, left, right };
}

namespace sensor_helpers {
class ImagePubliser;
}
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName,
                    ros::NodeHandle node,
                    std::shared_ptr<dai::Pipeline> pipeline,
                    std::shared_ptr<dai::Device> device,
                    dai::CameraBoardSocket leftSocket = dai::CameraBoardSocket::CAM_B,
                    dai::CameraBoardSocket rightSocket = dai::CameraBoardSocket::CAM_C);
    ~Stereo();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
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

    /**
     * This callback is used to synchronize left and right rectified frames
     * It is called every 10ms and it publishes the frames if they are synchronized
     * If they are not synchronized, it prints a warning message
     */
    void syncTimerCB();
    std::shared_ptr<sensor_helpers::ImagePublisher> stereoPub, leftRectPub, rightRectPub;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::unique_ptr<SensorWrapper> left;
    std::unique_ptr<SensorWrapper> right;
    std::unique_ptr<BaseNode> featureTrackerLeftR, featureTrackerRightR, nnNode;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> stereoQ, leftRectQ, rightRectQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo, xoutLeftRect, xoutRightRect;
    std::string stereoQName, leftRectQName, rightRectQName;
    dai::CameraFeatures leftSensInfo, rightSensInfo;
    std::shared_ptr<ros::Timer> syncTimer;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
