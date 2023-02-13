#pragma once

#include <deque>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/camera_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

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
    void getSizeFromResolution(const dai::ColorCameraProperties::SensorResolution& res, int& width, int& height) {
        switch(res) {
            case dai::ColorCameraProperties::SensorResolution::THE_720_P: {
                width = 1280;
                height = 720;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_800_P: {
                width = 1280;
                height = 800;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_1080_P: {
                width = 1920;
                height = 1080;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_4_K: {
                width = 3840;
                height = 2160;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_12_MP: {
                height = 4056;
                width = 3040;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_1200_P: {
                height = 1920;
                width = 1200;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_5_MP: {
                height = 2592;
                width = 1944;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_13_MP: {
                height = 4208;
                width = 3120;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_4000X3000: {
                height = 4000;
                width = 3000;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_5312X6000: {
                height = 5312;
                width = 6000;
                break;
            }
            case dai::ColorCameraProperties::SensorResolution::THE_48_MP: {
                height = 8000;
                width = 6000;
                break;
            }
        }
    }
};
inline std::vector<ImageSensor> availableSensors{
    {"IMX378", {"12mp", "4k"}, true},
    {"OV9282", {"800P", "720p", "400p"}, false},
    {"OV9782", {"800P", "720p", "400p"}, true},
    {"OV9281", {"800P", "720p", "400p"}, true},
    {"IMX214", {"13mp", "12mp", "4k", "1080p"}, true},
    {"OV7750", {"480P", "400p"}, false},
    {"OV7251", {"480P", "400p"}, false},
    {"IMX477", {"12mp", "4k", "1080p"}, true},
    {"IMX577", {"12mp", "4k", "1080p"}, true},
    {"AR0234", {"1200P"}, true},
    {"IMX582", {"48mp", "12mp", "4k"}, true},
};

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