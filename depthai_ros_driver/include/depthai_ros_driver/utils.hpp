#pragma once

#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/common/CameraExposureOffset.hpp"

namespace dai {
class Pipeline;
namespace node {
class XLinkOut;
}  // namespace node
}  // namespace dai

namespace depthai_ros_driver {
namespace utils {
template <typename T>
T getValFromMap(const std::string& name, const std::unordered_map<std::string, T>& map) {
    try {
        return map.at(name);
    } catch(const std::out_of_range& e) {
        std::stringstream stream;
        stream << "Unable to find name " << name.c_str() << " in map.\n";
        stream << "Map values:\n";
        for(auto it = map.cbegin(); it != map.cend(); ++it) {
            stream << it->first << "\n";
        }
        throw std::out_of_range(stream.str());
    }
}
std::string getUpperCaseStr(const std::string& string);
struct VideoEncoderConfig {
    bool enabled = false;
    int quality = 50;
    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG;
    int frameFreq = 30;
    int bitrate = 0;
};
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
    std::string ffmpegEncoder = "libx264";
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
	std::string infoSuffix = "";
    std::string compressedTopicSuffix = "/image_raw/compressed";
    std::string infoMgrSuffix = "";
    bool rectified = false;
    int width = 0;
    int height = 0;
    int maxQSize = 8;
    bool qBlocking = false;
    bool publishCompressed = false;
};
std::shared_ptr<dai::node::XLinkOut> setupXout(std::shared_ptr<dai::Pipeline> pipeline, const std::string& name);
}  // namespace utils
}  // namespace depthai_ros_driver
