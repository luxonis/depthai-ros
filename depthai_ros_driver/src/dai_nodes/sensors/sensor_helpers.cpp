#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logger.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {

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

const std::unordered_map<std::string, dai::ColorCameraProperties::ColorOrder> colorOrderMap = {
    {"BGR", dai::ColorCameraProperties::ColorOrder::BGR},
    {"RGB", dai::ColorCameraProperties::ColorOrder::RGB},
};

bool rsCompabilityMode(std::shared_ptr<rclcpp::Node> node) {
    return node->get_parameter("driver.i_rs_compat").as_bool();
}
std::string getNodeName(std::shared_ptr<rclcpp::Node> node, NodeNameEnum name) {
    if(rsCompabilityMode(node)) {
        return rsNodeNameMap.at(name);
    }
    return NodeNameMap.at(name);
}

const std::unordered_map<std::string, dai::ImgResizeMode> resizeModeMap = {
    {"CROP", dai::ImgResizeMode::CROP},
    {"LETTERBOX", dai::ImgResizeMode::LETTERBOX},
    {"STRETCH", dai::ImgResizeMode::STRETCH},
};

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
                    depthai_bridge::ImageConverter& converter,
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
                                          std::shared_ptr<depthai_bridge::ImageConverter> converter,
                                          dai::CalibrationHandler calHandler,
                                          dai::CameraBoardSocket socket,
                                          int width,
                                          int height) {
    sensor_msgs::msg::CameraInfo info;
    try {
        info = converter->calibrationToCameraInfo(calHandler, socket, width, height);
    } catch(std::runtime_error& e) {
        RCLCPP_ERROR(logger, "No calibration for socket %d! Publishing empty camera_info.", static_cast<int>(socket));
    }
    return info;
}

}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
