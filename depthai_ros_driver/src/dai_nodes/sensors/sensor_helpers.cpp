#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include <depthai/pipeline/node/XLinkOut.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logger.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {

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

sensor_msgs::msg::CameraInfo getCalibInfo(const rclcpp::Logger& logger,
                                          std::shared_ptr<dai::ros::ImageConverter> converter,
                                          std::shared_ptr<dai::Device> device,
                                          dai::CameraBoardSocket socket,
                                          int width,
                                          int height) {
    sensor_msgs::msg::CameraInfo info;
    auto calibHandler = device->readCalibration();
    try {
        info = converter->calibrationToCameraInfo(calibHandler, socket, width, height);
    } catch(std::runtime_error& e) {
        RCLCPP_ERROR(logger, "No calibration for socket %d! Publishing empty camera_info.", static_cast<int>(socket));
    }
    return info;
}

}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
