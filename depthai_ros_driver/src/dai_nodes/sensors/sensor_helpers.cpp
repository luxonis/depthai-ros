#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logger.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {

void ImagePublisher::setup(std::shared_ptr<rclcpp::Node> node,
                           const std::string& name,
                           bool lazyPub,
                           bool ipcEnabled,
                           std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
                           std::shared_ptr<dai::ros::ImageConverter> converter,
                           const std::string& topicSuffix) {
    this->name = name;
    this->lazyPub = lazyPub;
    this->ipcEnabled = ipcEnabled;
    this->infoManager = infoManager;
    this->converter = converter;
    if(ipcEnabled) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        imgPub = node->create_publisher<sensor_msgs::msg::Image>(name + topicSuffix, qos);
        infoPub = node->create_publisher<sensor_msgs::msg::CameraInfo>(name + "/camera_info", qos);
    } else {
        imgPubIT = image_transport::create_camera_publisher(node.get(), name + topicSuffix);
    }
}

ImagePublisher::~ImagePublisher() = default;

void ImagePublisher::addQueueCB(const std::shared_ptr<dai::DataOutputQueue>& queue) {
	if(synced){
		return;
	}
    dataQ = queue;
    qName = queue->getName();
    cbID = dataQ->addCallback([this](const std::shared_ptr<dai::ADatatype>& data) { publish(data); });
}

void ImagePublisher::removeQueueCB() {
    if(dataQ) {
        dataQ->removeCallback(cbID);
    }
}

std::string ImagePublisher::getQueueName() {
    return qName;
}
void ImagePublisher::setQueueName(const std::string& name) {
    qName = name;
}
void ImagePublisher::setSynced(bool sync) {
	synced = sync;
}
std::pair<sensor_msgs::msg::Image::UniquePtr, sensor_msgs::msg::CameraInfo::UniquePtr> ImagePublisher::convertData(const std::shared_ptr<dai::ADatatype> data) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    auto info = infoManager->getCameraInfo();
    auto rawMsg = converter->toRosMsgRawPtr(img, info);
    info.header = rawMsg.header;
    sensor_msgs::msg::CameraInfo::UniquePtr infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
    sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(rawMsg);
    return {std::move(msg), std::move(infoMsg)};
}

void ImagePublisher::publish(std::pair<sensor_msgs::msg::Image::UniquePtr, sensor_msgs::msg::CameraInfo::UniquePtr> data) {
    if(ipcEnabled && (!lazyPub || detectSubscription(imgPub, infoPub))) {
        imgPub->publish(std::move(data.first));
        infoPub->publish(std::move(data.second));
    } else {
        if(!lazyPub || imgPubIT.getNumSubscribers() > 0) imgPubIT.publish(*data.first, *data.second);
    }
}

void ImagePublisher::publish(const std::shared_ptr<dai::ADatatype>& data) {
    if(rclcpp::ok()) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter->toRosMsgRawPtr(img, info);
        info.header = rawMsg.header;
        if(ipcEnabled && (!lazyPub || detectSubscription(imgPub, infoPub))) {
            sensor_msgs::msg::CameraInfo::UniquePtr infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
            sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(rawMsg);
            imgPub->publish(std::move(msg));
            infoPub->publish(std::move(infoMsg));
        } else {
            if(!lazyPub || imgPubIT.getNumSubscribers() > 0) imgPubIT.publish(rawMsg, info);
        }
    }
}

std::vector<ImageSensor> availableSensors = {{"IMX378", "1080P", {"12MP", "4K", "1080P"}, true},
                                             {"OV9282", "720P", {"800P", "720P", "400P"}, false},
                                             {"OV9782", "720P", {"800P", "720P", "400P"}, true},
                                             {"OV9281", "720P", {"800P", "720P", "400P"}, true},
                                             {"IMX214", "1080P", {"13MP", "12MP", "4K", "1080P"}, true},
                                             {"IMX412", "1080P", {"13MP", "12MP", "4K", "1080P"}, true},
                                             {"OV7750", "480P", {"480P", "400P"}, false},
                                             {"OV7251", "480P", {"480P", "400P"}, false},
                                             {"IMX477", "1080P", {"12MP", "4K", "1080P"}, true},
                                             {"IMX577", "1080P", {"12MP", "4K", "1080P"}, true},
                                             {"AR0234", "1200P", {"1200P"}, true},
                                             {"IMX582", "4K", {"48MP", "12MP", "4K"}, true},
                                             {"LCM48", "4K", {"48MP", "12MP", "4K"}, true}};
const std::unordered_map<dai::CameraBoardSocket, std::string> socketNameMap = {
    {dai::CameraBoardSocket::AUTO, "rgb"},
    {dai::CameraBoardSocket::CAM_A, "rgb"},
    {dai::CameraBoardSocket::CAM_B, "left"},
    {dai::CameraBoardSocket::CAM_C, "right"},
    {dai::CameraBoardSocket::CAM_D, "left_back"},
    {dai::CameraBoardSocket::CAM_E, "right_back"},
};
const std::unordered_map<dai::CameraBoardSocket, std::string> rsSocketNameMap = {
    {dai::CameraBoardSocket::AUTO, "color"},
    {dai::CameraBoardSocket::CAM_A, "color"},
    {dai::CameraBoardSocket::CAM_B, "infra2"},
    {dai::CameraBoardSocket::CAM_C, "infra1"},
    {dai::CameraBoardSocket::CAM_D, "infra4"},
    {dai::CameraBoardSocket::CAM_E, "infra3"},
};
const std::unordered_map<NodeNameEnum, std::string> rsNodeNameMap = {
    {NodeNameEnum::RGB, "color"},
    {NodeNameEnum::Left, "infra2"},
    {NodeNameEnum::Right, "infra1"},
    {NodeNameEnum::Stereo, "depth"},
    {NodeNameEnum::IMU, "imu"},
    {NodeNameEnum::NN, "nn"},
};

const std::unordered_map<NodeNameEnum, std::string> NodeNameMap = {
    {NodeNameEnum::RGB, "rgb"},
    {NodeNameEnum::Left, "left"},
    {NodeNameEnum::Right, "right"},
    {NodeNameEnum::Stereo, "stereo"},
    {NodeNameEnum::IMU, "imu"},
    {NodeNameEnum::NN, "nn"},
};

bool rsCompabilityMode(std::shared_ptr<rclcpp::Node> node) {
    return node->get_parameter("camera.i_rs_compat").as_bool();
}
std::string getNodeName(std::shared_ptr<rclcpp::Node> node, NodeNameEnum name) {
    if(rsCompabilityMode(node)) {
        return rsNodeNameMap.at(name);
    }
    return NodeNameMap.at(name);
}

std::string getSocketName(std::shared_ptr<rclcpp::Node> node, dai::CameraBoardSocket socket) {
    if(rsCompabilityMode(node)) {
        return rsSocketNameMap.at(socket);
    }
    return socketNameMap.at(socket);
}
const std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap = {
    {"400P", dai::MonoCameraProperties::SensorResolution::THE_400_P},
    {"480P", dai::MonoCameraProperties::SensorResolution::THE_480_P},
    {"720P", dai::MonoCameraProperties::SensorResolution::THE_720_P},
    {"800P", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    {"1200P", dai::MonoCameraProperties::SensorResolution::THE_1200_P},
};

const std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap = {
    {"720P", dai::ColorCameraProperties::SensorResolution::THE_720_P},
    {"1080P", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
    {"4K", dai::ColorCameraProperties::SensorResolution::THE_4_K},
    {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
    {"13MP", dai::ColorCameraProperties::SensorResolution::THE_13_MP},
    {"800P", dai::ColorCameraProperties::SensorResolution::THE_800_P},
    {"1200P", dai::ColorCameraProperties::SensorResolution::THE_1200_P},
    {"5MP", dai::ColorCameraProperties::SensorResolution::THE_5_MP},
    {"4000x3000", dai::ColorCameraProperties::SensorResolution::THE_4000X3000},
    {"5312X6000", dai::ColorCameraProperties::SensorResolution::THE_5312X6000},
    {"48MP", dai::ColorCameraProperties::SensorResolution::THE_48_MP},
    {"1440X1080", dai::ColorCameraProperties::SensorResolution::THE_1440X1080}};

const std::unordered_map<std::string, dai::CameraControl::FrameSyncMode> fSyncModeMap = {
    {"OFF", dai::CameraControl::FrameSyncMode::OFF},
    {"OUTPUT", dai::CameraControl::FrameSyncMode::OUTPUT},
    {"INPUT", dai::CameraControl::FrameSyncMode::INPUT},
};
const std::unordered_map<std::string, dai::CameraImageOrientation> cameraImageOrientationMap = {
    {"NORMAL", dai::CameraImageOrientation::NORMAL},
    {"ROTATE_180_DEG", dai::CameraImageOrientation::ROTATE_180_DEG},
    {"AUTO", dai::CameraImageOrientation::AUTO},
    {"HORIZONTAL_MIRROR", dai::CameraImageOrientation::HORIZONTAL_MIRROR},
    {"VERTICAL_FLIP", dai::CameraImageOrientation::VERTICAL_FLIP},
};

void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager) {
    if(rclcpp::ok() && (pub.getNumSubscribers() > 0)) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img);
        info.header = rawMsg.header;
        pub.publish(rawMsg, info);
    }
}

void cameraPub(const std::string& /*name*/,
               const std::shared_ptr<dai::ADatatype>& data,
               dai::ros::ImageConverter& converter,
               image_transport::CameraPublisher& pub,
               std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
               bool lazyPub) {
    if(rclcpp::ok() && (!lazyPub || pub.getNumSubscribers() > 0)) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img, info);
        info.header = rawMsg.header;
        pub.publish(rawMsg, info);
    }
}

void splitPub(const std::string& /*name*/,
              const std::shared_ptr<dai::ADatatype>& data,
              dai::ros::ImageConverter& converter,
              rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub,
              rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub,
              std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
              bool lazyPub) {
    if(rclcpp::ok() && (!lazyPub || detectSubscription(imgPub, infoPub))) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img, info);
        info.header = rawMsg.header;
        sensor_msgs::msg::CameraInfo::UniquePtr infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
        sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(rawMsg);
        imgPub->publish(std::move(msg));
        infoPub->publish(std::move(infoMsg));
    }
}

sensor_msgs::msg::CameraInfo getCalibInfo(const rclcpp::Logger& logger,
                                          dai::ros::ImageConverter& converter,
                                          std::shared_ptr<dai::Device> device,
                                          dai::CameraBoardSocket socket,
                                          int width,
                                          int height) {
    sensor_msgs::msg::CameraInfo info;
    auto calibHandler = device->readCalibration();
    try {
        info = converter.calibrationToCameraInfo(calibHandler, socket, width, height);
    } catch(std::runtime_error& e) {
        RCLCPP_ERROR(logger, "No calibration for socket %d! Publishing empty camera_info.", static_cast<int>(socket));
    }
    return info;
}
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline, int quality, dai::VideoEncoderProperties::Profile profile) {
    auto enc = pipeline->create<dai::node::VideoEncoder>();
    enc->setQuality(quality);
    enc->setProfile(profile);
    return enc;
}

bool detectSubscription(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                        const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& infoPub) {
    return (pub->get_subscription_count() > 0 || pub->get_intra_process_subscription_count() > 0 || infoPub->get_subscription_count() > 0
            || infoPub->get_intra_process_subscription_count() > 0);
}
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
