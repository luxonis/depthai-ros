#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
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
namespace ros {
class ImageConverter;
class Timer;
}  // namespace ros
}  // namespace dai

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
    dai::Node::Input getInput(int linkType = 0);
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void setupStereoQueue(std::shared_ptr<dai::Device> device);
    void setupLeftRectQueue(std::shared_ptr<dai::Device> device);
    void setupRightRectQueue(std::shared_ptr<dai::Device> device);
    void setupRectQueue(std::shared_ptr<dai::Device> device,
                        dai::CameraFeatures& sensorInfo,
                        const std::string& queueName,
                        std::unique_ptr<dai::ros::ImageConverter>& conv,
                        std::shared_ptr<camera_info_manager::CameraInfoManager>& im,
                        std::shared_ptr<dai::DataOutputQueue>& q,
                        image_transport::CameraPublisher& pubIT,
                        bool isLeft);
    /**
     * This callback is used to synchronize left and right rectified frames
     * It is called every 10ms and it publishes the frames if they are synchronized
     * If they are not synchronized, it prints a warning message
     */
    void syncTimerCB();
    image_transport::ImageTransport it;
    std::unique_ptr<dai::ros::ImageConverter> stereoConv, leftRectConv, rightRectConv;
    image_transport::CameraPublisher stereoPubIT, leftRectPubIT, rightRectPubIT;
    std::shared_ptr<camera_info_manager::CameraInfoManager> stereoIM, leftRectIM, rightRectIM;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> stereoEnc, leftRectEnc, rightRectEnc;
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