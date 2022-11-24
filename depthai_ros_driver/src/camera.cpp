#include "depthai_ros_driver/camera.hpp"

#include "depthai_ros_driver/dai_nodes/imu.hpp"
#include "depthai_ros_driver/dai_nodes/mono.hpp"
#include "depthai_ros_driver/dai_nodes/rgb.hpp"
#include "depthai_ros_driver/dai_nodes/stereo.hpp"

namespace depthai_ros_driver {

Camera::Camera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("camera", options) {
    on_configure();
}
void Camera::on_configure() {
    declareParams();
    createPipeline();
    startDevice();
    setupQueues();
    param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(&Camera::parameterCB, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Camera ready!");
}

void Camera::createPipeline() {
    pipeline_ = std::make_shared<dai::Pipeline>();
    dai_nodes::RGBFactory rgb_fac;
    dai_nodes::MonoFactory mono_fac;
    dai_nodes::StereoFactory stereo_fac;
    dai_nodes::ImuFactory imu_fac;
    if(this->get_parameter("pipeline_type").as_string() == "RGB") {
        auto rgb = rgb_fac.create("color", this, pipeline_);
        dai_nodes_.push_back(std::move(rgb));
    } else if(this->get_parameter("pipeline_type").as_string() == "RGBD") {
        auto rgb = rgb_fac.create("color", this, pipeline_);
        auto mono_left = mono_fac.create("mono_left", this, pipeline_);
        auto mono_right = mono_fac.create("mono_right", this, pipeline_);
        auto stereo = stereo_fac.create("stereo", this, pipeline_);

        mono_left->link(stereo->get_input(static_cast<int>(dai_nodes::link_types::StereoLinkType::left)),
                        static_cast<int>(dai_nodes::link_types::StereoLinkType::left));
        mono_right->link(stereo->get_input(static_cast<int>(dai_nodes::link_types::StereoLinkType::right)),
                         static_cast<int>(dai_nodes::link_types::StereoLinkType::right));
        dai_nodes_.push_back(std::move(rgb));
        dai_nodes_.push_back(std::move(mono_right));
        dai_nodes_.push_back(std::move(mono_left));
        dai_nodes_.push_back(std::move(stereo));
    }
    auto imu = imu_fac.create("imu", this, pipeline_);
    dai_nodes_.push_back(std::move(imu));
    RCLCPP_INFO(this->get_logger(), "Finished setting up pipeline.");
}

void Camera::setupQueues() {
    for(const auto& node : dai_nodes_) {
        node->setupQueues(device_);
    }
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

rcl_interfaces::msg::SetParametersResult Camera::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& node : dai_nodes_) {
        node->updateParams(params);
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
}
void Camera::declareParams() {
    this->declare_parameter<std::string>("pipeline_type", "RGBD");
    this->declare_parameter<bool>("enable_imu", true);
    this->declare_parameter<std::string>("usb_speed", "SUPER");
    this->declare_parameter<std::string>("cam_type", "auto");
    this->declare_parameter<std::string>("mx_id", "");
    this->declare_parameter<std::string>("ip", "");
}
}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);