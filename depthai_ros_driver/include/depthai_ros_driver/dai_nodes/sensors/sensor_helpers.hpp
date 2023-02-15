#pragma once

#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/camera_publisher.h"
#include "sensor_msgs/CameraInfo.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class RGBLinkType { video, isp, preview };
}
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
            default: {
                throw std::runtime_error("Resolution not supported!");
            }
        }
    }
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
sensor_msgs::CameraInfo getCalibInfo(
    dai::ros::ImageConverter& converter, std::shared_ptr<dai::Device> device, dai::CameraBoardSocket socket, int width = 0, int height = 0);
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                       int quality,
                                                       dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG);
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver