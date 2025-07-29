#pragma once

#include <string>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/ColorCameraProperties.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"
#include "image_transport/camera_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dai {
class Device;
class CalibrationHandler;
class Pipeline;
class MessageQueue;
namespace node {
class VideoEncoder;
}  // namespace node
}  // namespace dai

namespace depthai_bridge {
class ImageConverter;
}
namespace rclcpp {
class Logger;
}

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
enum class NodeNameEnum { RGB, Left, Right, Stereo, IMU, NN };
extern const std::unordered_map<NodeNameEnum, std::string> rsNodeNameMap;
extern const std::unordered_map<NodeNameEnum, std::string> NodeNameMap;
extern const std::unordered_map<std::string, dai::CameraControl::FrameSyncMode> fSyncModeMap;
extern const std::unordered_map<std::string, dai::CameraImageOrientation> cameraImageOrientationMap;
extern const std::unordered_map<std::string, dai::ImgResizeMode> resizeModeMap;
bool rsCompabilityMode(std::shared_ptr<rclcpp::Node> node);
std::string getNodeName(std::shared_ptr<rclcpp::Node> node, NodeNameEnum name);
void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    depthai_bridge::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager);

sensor_msgs::msg::CameraInfo getCalibInfo(const rclcpp::Logger& logger,
                                          std::shared_ptr<depthai_bridge::ImageConverter> converter,
                                          dai::CalibrationHandler calHandler,
                                          dai::CameraBoardSocket socket,
                                          int width = 0,
                                          int height = 0);
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
