#pragma once

#include <deque>
#include <string>
#include <vector>

#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "image_transport/camera_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace dai {
class Device;
class Pipeline;
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
struct ImageSensor {
    std::string name;
    std::vector<std::string> allowedResolutions;
    bool color;
    void getSizeFromResolution(const dai::ColorCameraProperties::SensorResolution& res, int& width, int& height);
};
extern std::vector<ImageSensor> availableSensors;

void imgCB(const std::string& /*name*/,
           const std::shared_ptr<dai::ADatatype>& data,
           dai::ros::ImageConverter& converter,
           image_transport::CameraPublisher& pub,
           std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager);

void compressedImgCB(const std::string& /*name*/,
                     const std::shared_ptr<dai::ADatatype>& data,
                     dai::ros::ImageConverter& converter,
                     image_transport::CameraPublisher& pub,
                     std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
                     dai::RawImgFrame::Type dataType);

sensor_msgs::msg::CameraInfo getCalibInfo(const rclcpp::Logger& logger,
                                          dai::ros::ImageConverter& converter,
                                          std::shared_ptr<dai::Device> device,
                                          dai::CameraBoardSocket socket,
                                          int width = 0,
                                          int height = 0);
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                       int quality,
                                                       dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG);
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver