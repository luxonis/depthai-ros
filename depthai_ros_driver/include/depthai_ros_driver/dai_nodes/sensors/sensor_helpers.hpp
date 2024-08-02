#pragma once

#include <deque>
#include <string>
#include <vector>

#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "image_transport/camera_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dai {
class Device;
class Pipeline;
class DataOutputQueue;
namespace node {
class VideoEncoder;
}
namespace ros {
class ImageConverter;
}
}  // namespace dai

namespace rclcpp {
class Logger;
}

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class RGBLinkType { video, isp, preview };
};
namespace sensor_helpers {
class ImagePublisher {
   public:
    ImagePublisher(rclcpp::Node* node,
                   const std::string& name,
                   bool lazyPub,
                   bool ipcEnabled,
                   std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
                   std::shared_ptr<dai::ros::ImageConverter> converter);
    ~ImagePublisher();
    void addQueueCB(const std::shared_ptr<dai::DataOutputQueue>& queue);
    void removeQueueCB();
    void publish(const std::shared_ptr<dai::ADatatype>& data);

   private:
    std::string name;
    bool lazyPub;
    bool ipcEnabled;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::ros::ImageConverter> converter;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;
    image_transport::CameraPublisher imgPubIT;
    std::shared_ptr<dai::DataOutputQueue> dataQ;
    int cbID;
};
enum class NodeNameEnum { RGB, Left, Right, Stereo, IMU, NN };
struct ImageSensor {
    std::string name;
    std::string defaultResolution;
    std::vector<std::string> allowedResolutions;
    bool color;
};
extern std::vector<ImageSensor> availableSensors;
extern const std::unordered_map<dai::CameraBoardSocket, std::string> socketNameMap;
extern const std::unordered_map<dai::CameraBoardSocket, std::string> rsSocketNameMap;
extern const std::unordered_map<NodeNameEnum, std::string> rsNodeNameMap;
extern const std::unordered_map<NodeNameEnum, std::string> NodeNameMap;
extern const std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
extern const std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap;
extern const std::unordered_map<std::string, dai::CameraControl::FrameSyncMode> fSyncModeMap;
extern const std::unordered_map<std::string, dai::CameraImageOrientation> cameraImageOrientationMap;
bool rsCompabilityMode(rclcpp::Node* node);
std::string getSocketName(rclcpp::Node* node, dai::CameraBoardSocket socket);
std::string getNodeName(rclcpp::Node* node, NodeNameEnum name);
void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager);

void cameraPub(const std::string& /*name*/,
               const std::shared_ptr<dai::ADatatype>& data,
               std::shared_ptr<dai::ros::ImageConverter> converter,
               image_transport::CameraPublisher& pub,
               std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
               bool lazyPub = true);

void splitPub(const std::string& /*name*/,
              const std::shared_ptr<dai::ADatatype>& data,
              dai::ros::ImageConverter& converter,
              rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub,
              rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub,
              std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
              bool lazyPub = true);

sensor_msgs::msg::CameraInfo getCalibInfo(const rclcpp::Logger& logger,
                                          dai::ros::ImageConverter& converter,
                                          std::shared_ptr<dai::Device> device,
                                          dai::CameraBoardSocket socket,
                                          int width = 0,
                                          int height = 0);
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                       int quality,
                                                       dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG);
bool detectSubscription(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                        const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& infoPub);
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
