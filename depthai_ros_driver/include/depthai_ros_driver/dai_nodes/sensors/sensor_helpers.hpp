#pragma once

#include <deque>
#include <string>
#include <vector>

#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/pipeline/Node.hpp"
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
class XLinkOut;
}  // namespace node
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
struct ImgConverterConfig {
    std::string tfPrefix = "";
    bool interleaved = false;
    bool getBaseDeviceTimestamp = false;
    bool updateROSBaseTimeOnRosMsg = false;
    bool lowBandwidth = false;
    dai::RawImgFrame::Type encoding = dai::RawImgFrame::Type::BGR888i;
    bool addExposureOffset = false;
    dai::CameraExposureOffset expOffset = dai::CameraExposureOffset::START;
    bool reverseSocketOrder = false;
    bool alphaScalingEnabled = false;
    double alphaScaling = 1.0;
    bool outputDisparity = false;
};

struct ImgPublisherConfig {
    std::string daiNodeName = "";
    std::string topicName = "";
    bool lazyPub = false;
    dai::CameraBoardSocket socket = dai::CameraBoardSocket::AUTO;
    dai::CameraBoardSocket leftSocket = dai::CameraBoardSocket::CAM_B;
    dai::CameraBoardSocket rightSocket = dai::CameraBoardSocket::CAM_C;
    std::string calibrationFile = "";
    std::string topicSuffix = "/image_raw";
    std::string infoMgrSuffix = "";
    bool rectified = false;
    int width = 0;
    int height = 0;
    int maxQSize = 8;
    bool qBlocking = false;
};
class ImagePublisher {
   public:
    /**
     * @brief Construct a new Image Publisher object
     *
     * Creates XLinkOut if synced and VideoEncoder if lowBandwidth is enabled. linkFunc is stored and returned when link is called.
     */
    ImagePublisher(std::shared_ptr<rclcpp::Node> node,
                   std::shared_ptr<dai::Pipeline> pipeline,
                   const std::string& qName,
                   std::function<void(dai::Node::Input in)> linkFunc,
                   bool synced = false,
                   bool ipcEnabled = false,
                   bool lowBandwidth = false,
                   int lowBandwidthQuality = 50);

    ~ImagePublisher();
    /**
     * @brief Setup the image publisher
     *
     * Creates Publishers, ImageConverter and CameraInfoManager. Creates a Queue and adds a callback if not synced.
     */
    void setup(std::shared_ptr<dai::Device> device, const ImgConverterConfig& convConf, const ImgPublisherConfig& pubConf);
    void createImageConverter(std::shared_ptr<dai::Device> device);
    void createInfoManager(std::shared_ptr<dai::Device> device);
    void addQueueCB(const std::shared_ptr<dai::DataOutputQueue>& queue);
    void closeQueue();
    std::shared_ptr<dai::DataOutputQueue> getQueue();
    void link(dai::Node::Input in);
    std::string getQueueName();
    void publish(const std::shared_ptr<dai::ADatatype>& data);
    void publish(std::pair<sensor_msgs::msg::Image::UniquePtr, sensor_msgs::msg::CameraInfo::UniquePtr> data);
    std::pair<sensor_msgs::msg::Image::UniquePtr, sensor_msgs::msg::CameraInfo::UniquePtr> convertData(const std::shared_ptr<dai::ADatatype> data);

   private:
    std::shared_ptr<rclcpp::Node> node;
    ImgPublisherConfig pubConfig;
    ImgConverterConfig convConfig;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::ros::ImageConverter> converter;
    std::shared_ptr<dai::node::XLinkOut> xout;
    std::shared_ptr<dai::node::VideoEncoder> encoder;
    std::function<void(dai::Node::Input in)> linkCB;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;
    image_transport::CameraPublisher imgPubIT;
    std::shared_ptr<dai::DataOutputQueue> dataQ;
    int cbID;
    std::string qName;
    bool ipcEnabled;
    bool synced;
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
bool rsCompabilityMode(std::shared_ptr<rclcpp::Node> node);
std::string getSocketName(std::shared_ptr<rclcpp::Node> node, dai::CameraBoardSocket socket);
std::string getNodeName(std::shared_ptr<rclcpp::Node> node, NodeNameEnum name);
void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager);

sensor_msgs::msg::CameraInfo getCalibInfo(const rclcpp::Logger& logger,
                                          std::shared_ptr<dai::ros::ImageConverter> converter,
                                          std::shared_ptr<dai::Device> device,
                                          dai::CameraBoardSocket socket,
                                          int width = 0,
                                          int height = 0);
std::shared_ptr<dai::node::XLinkOut> setupXout(std::shared_ptr<dai::Pipeline> pipeline, const std::string& name);
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                       int quality,
                                                       dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG);
bool detectSubscription(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                        const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& infoPub);
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
