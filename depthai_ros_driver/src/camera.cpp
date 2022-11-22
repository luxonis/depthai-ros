#include "depthai_ros_driver/camera.hpp"

#include "depthai_ros_driver/dai_nodes/rgb.hpp"

namespace depthai_ros_driver {

Camera::Camera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("camera", options) {
    on_configure();
}
void Camera::on_configure() {
    declareParams();
    createPipeline();
}
void Camera::createPipeline() {
    pipeline_ = std::make_shared<dai::Pipeline>();
    if(this->get_parameter("pipeline_type").as_string() == "RGB") {
        rgb_ = std::make_unique<dai_nodes::RGB>();
        rgb_->initialize("color", this, pipeline_);
        startDevice();
        rgb_->setupQueues(device_);
    }

    RCLCPP_INFO(this->get_logger(), "Camera ready!");
}
void Camera::startDevice() {
    // dai::DeviceInfo info;
    // bool device_found;
    // if (!this->get_parameter("camera_mxid").as_string().empty()) {
    //   info = dai::DeviceInfo(this->get_parameter("camera_mxid").as_string());
    // } else if (!this->get_parameter("camera_ip").as_string().empty()) {
    //   info = dai::DeviceInfo(this->get_parameter("camera_mxid").as_string());
    // }
    // rclcpp::Rate r(1.0);
    // bool cam_setup;
    // while (!cam_setup) {
    //   try {
    //     dai::UsbSpeed speed = usb_speed_map_.at(base_config_.usb_speed);
    //     if (base_config_.camera_mxid.empty() && base_config_.camera_ip.empty()) {
    //       device_ = std::make_shared<dai::Device>(*pipeline_, speed);
    //     } else {
    //       device_ = std::make_shared<dai::Device>(*pipeline_, info, speed);
    //     }
    //     cam_setup = true;
    //   } catch (const std::runtime_error & e) {
    //     RCLCPP_ERROR(this->get_logger(), "Camera not found! Please connect it");
    //   }
    //   r.sleep();
    // }
    // cam_running_ = true;
    // device_name_ = device_->getMxId();
    // RCLCPP_INFO(this->get_logger(), "Camera %s connected!", device_name_.c_str());

    device_ = std::make_shared<dai::Device>(*pipeline_);
}
void Camera::declareParams() {
    this->declare_parameter<std::string>("pipeline_type", "RGB");
    this->declare_parameter<bool>("enable_imu", true);
    this->declare_parameter<std::string>("usb_speed", "SUPER");
    this->declare_parameter<std::string>("cam_type", "auto");
    this->declare_parameter<std::string>("mx_id", "");
    this->declare_parameter<std::string>("ip", "");
}
}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);