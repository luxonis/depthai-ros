#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "rclcpp/logger.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {

std::vector<ImageSensor> availableSensors{{"IMX378", "4k", {"12mp", "4k"}, true},
                                          {"OV9282", "800p", {"800p", "720p", "400p"}, false},
                                          {"OV9782", "800p", {"800p", "720p", "400p"}, true},
                                          {"OV9281", "800p", {"800p", "720p", "400p"}, true},
                                          {"IMX214", "1080p", {"13mp", "12mp", "4k", "1080p"}, true},
                                          {"IMX412", "1080p", {"13mp", "12mp", "4k", "1080p"}, true},
                                          {"OV7750", "480p", {"480p", "400p"}, false},
                                          {"OV7251", "480p", {"480p", "400p"}, false},
                                          {"IMX477", "1080p", {"12mp", "4k", "1080p"}, true},
                                          {"IMX577", "1080p", {"12mp", "4k", "1080p"}, true},
                                          {"AR0234", "1200p", {"1200p"}, true},
                                          {"IMX582", "4k", {"48mp", "12mp", "4k"}, true},
                                          {"LCM48", "4k", {"48mp", "12mp", "4k"}, true}};

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

bool detectSubscription(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                        const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& infoPub) {
    return (pub->get_subscription_count() > 0 || pub->get_intra_process_subscription_count() > 0 || infoPub->get_subscription_count() > 0
            || infoPub->get_intra_process_subscription_count() > 0);
}
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver