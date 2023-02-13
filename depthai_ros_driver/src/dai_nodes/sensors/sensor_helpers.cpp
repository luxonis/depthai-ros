#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {

void compressedImgCB(const std::string& /*name*/,
                     const std::shared_ptr<dai::ADatatype>& data,
                     dai::ros::ImageConverter& converter,
                     image_transport::CameraPublisher& pub,
                     std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager,
                     dai::RawImgFrame::Type dataType) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::deque<sensor_msgs::msg::Image> deq;
    auto info = infoManager->getCameraInfo();
    converter.toRosMsgFromBitStream(img, deq, dataType, info);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        info.header = currMsg.header;
        pub.publish(currMsg, info);
        deq.pop_front();
    }
}
void imgCB(const std::string& /*name*/,
           const std::shared_ptr<dai::ADatatype>& data,
           dai::ros::ImageConverter& converter,
           image_transport::CameraPublisher& pub,
           std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::deque<sensor_msgs::msg::Image> deq;
    auto info = infoManager->getCameraInfo();
    converter.toRosMsg(img, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        info.header = currMsg.header;
        pub.publish(currMsg, info);
        deq.pop_front();
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