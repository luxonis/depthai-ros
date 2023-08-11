#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "rclcpp/logger.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
void ImageSensor::getSizeFromResolution(const dai::ColorCameraProperties::SensorResolution& res, int& width, int& height) {
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
            width = 4056;
            height = 3040;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_1200_P: {
            width = 1920;
            height = 1200;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_5_MP: {
            width = 2592;
            height = 1944;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_13_MP: {
            width = 4208;
            height = 3120;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_4000X3000: {
            width = 4000;
            height = 3000;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_5312X6000: {
            width = 5312;
            height = 6000;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_48_MP: {
            width = 8000;
            height = 6000;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_1440X1080: {
            width = 1440;
            height = 1080;
            break;
        }
        default: {
            throw std::runtime_error("Resolution not supported!");
        }
    }
}
std::vector<ImageSensor> availableSensors{
    {"IMX378", {"12mp", "4k"}, true},
    {"OV9282", {"800P", "720p", "400p"}, false},
    {"OV9782", {"800P", "720p", "400p"}, true},
    {"OV9281", {"800P", "720p", "400p"}, true},
    {"IMX214", {"13mp", "12mp", "4k", "1080p"}, true},
    {"IMX412", {"13mp", "12mp", "4k", "1080p"}, true},
    {"OV7750", {"480P", "400p"}, false},
    {"OV7251", {"480P", "400p"}, false},
    {"IMX477", {"12mp", "4k", "1080p"}, true},
    {"IMX577", {"12mp", "4k", "1080p"}, true},
    {"AR0234", {"1200P"}, true},
    {"IMX582", {"48mp", "12mp", "4k"}, true},
    {"LCM48", {"48mp", "12mp", "4k"}, true},
};

void basicCameraPub(const std::string& /*name*/,
                    const std::shared_ptr<dai::ADatatype>& data,
                    dai::ros::ImageConverter& converter,
                    image_transport::CameraPublisher& pub,
                    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
                    rclcpp::Node* node) {
    if(node->count_subscribers(pub.getTopic()) > 0) {
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
               rclcpp::Node* node,
               bool fromBitStream,
               bool dispToDepth,
               dai::RawImgFrame::Type type) {
    if(node->count_subscribers(pub.getTopic()) > 0) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img, fromBitStream, dispToDepth, type, info);
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
              rclcpp::Node* node,
              bool fromBitStream,
              bool dispToDepth,
              dai::RawImgFrame::Type type) {
    if(node->count_subscribers(imgPub->get_topic_name()) > 0 && node->count_subscribers(infoPub->get_topic_name()) > 0) {
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto info = infoManager->getCameraInfo();
        auto rawMsg = converter.toRosMsgRawPtr(img, fromBitStream, dispToDepth, type, info);
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
        RCLCPP_ERROR(logger, "No calibration! Publishing empty camera_info.");
    }
    return info;
}
std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline, int quality, dai::VideoEncoderProperties::Profile profile) {
    auto enc = pipeline->create<dai::node::VideoEncoder>();
    enc->setQuality(quality);
    enc->setProfile(profile);
    return enc;
}

}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver