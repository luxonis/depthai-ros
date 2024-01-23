#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

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

class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName,
                    rclcpp::Node* node,
                    std::shared_ptr<dai::Pipeline> pipeline,
                    std::shared_ptr<dai::Device> device,
                    dai::CameraBoardSocket leftSocket = dai::CameraBoardSocket::CAM_B,
                    dai::CameraBoardSocket rightSocket = dai::CameraBoardSocket::CAM_C);
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
    void setupRectQueue(std::shared_ptr<dai::Device> device,
                        dai::CameraFeatures& sensorInfo,
                        const std::string& queueName,
                        std::unique_ptr<dai::ros::ImageConverter>& conv,
                        std::shared_ptr<camera_info_manager::CameraInfoManager>& im,
                        std::shared_ptr<dai::DataOutputQueue>& q,
                        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
                        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub,
                        image_transport::CameraPublisher& pubIT,
                        bool isLeft);
    /*
     * This callback is used to synchronize left and right rectified frames
     * It is called every 10ms and it publishes the frames if they are synchronized
     * If they are not synchronized, it prints a warning message
     */
    void syncTimerCB();
    std::unique_ptr<dai::ros::ImageConverter> stereoConv, leftRectConv, rightRectConv;
    image_transport::CameraPublisher stereoPubIT, leftRectPubIT, rightRectPubIT;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereoPub, leftRectPub, rightRectPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr stereoInfoPub, leftRectInfoPub, rightRectInfoPub;
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
    rclcpp::TimerBase::SharedPtr syncTimer;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver