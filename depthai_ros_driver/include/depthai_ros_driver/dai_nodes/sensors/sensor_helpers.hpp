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
#include "image_transport/camera_publisher.h"
#include "sensor_msgs/CameraInfo.h"

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

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class RGBLinkType { video, isp, preview };
}
namespace sensor_helpers {
struct ImageSensor {
    std::string name;
    std::string defaultResolution;
    std::vector<std::string> allowedResolutions;
    bool color;
    void getSizeFromResolution(const dai::ColorCameraProperties::SensorResolution& res, int& width, int& height);
};

extern std::vector<ImageSensor> availableSensors;
extern const std::unordered_map<dai::CameraBoardSocket, std::string> socketNameMap;
extern const std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
extern const std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap;
extern const std::unordered_map<std::string, dai::CameraControl::FrameSyncMode> fSyncModeMap;
extern const std::unordered_map<std::string, dai::CameraImageOrientation> cameraImageOrientationMap;

void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager);

void cameraPub(const std::string& /*name*/,
               const std::shared_ptr<dai::ADatatype>& data,
               dai::ros::ImageConverter& converter,
               image_transport::CameraPublisher& pub,
               std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
               bool lazyPub = true);

sensor_msgs::CameraInfo getCalibInfo(
    dai::ros::ImageConverter& converter, std::shared_ptr<dai::Device> device, dai::CameraBoardSocket socket, int width = 0, int height = 0);
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                       int quality,
                                                       dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG);

}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver