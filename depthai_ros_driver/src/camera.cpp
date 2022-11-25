#include "depthai_ros_driver/camera.hpp"

#include "depthai_ros_driver/dai_nodes/imu.hpp"
#include "depthai_ros_driver/dai_nodes/mono.hpp"
#include "depthai_ros_driver/dai_nodes/nn.hpp"
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
    dai_nodes::NNFactory nn_fac;
    if(this->get_parameter("pipeline_type").as_string() == "RGB") {
        auto rgb = rgb_fac.create("color", this, pipeline_);
        dai_nodes_.push_back(std::move(rgb));
    } else if(this->get_parameter("pipeline_type").as_string() == "RGBD") {
        auto rgb = rgb_fac.create("color", this, pipeline_);
        auto mono_left = mono_fac.create("mono_left", this, pipeline_);
        auto mono_right = mono_fac.create("mono_right", this, pipeline_);
        auto stereo = stereo_fac.create("stereo", this, pipeline_);
        auto nn = nn_fac.create("nn", this, pipeline_);

        rgb->link(nn->get_input(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));

        mono_left->link(stereo->get_input(static_cast<int>(dai_nodes::link_types::StereoLinkType::left)),
                        static_cast<int>(dai_nodes::link_types::StereoLinkType::left));
        mono_right->link(stereo->get_input(static_cast<int>(dai_nodes::link_types::StereoLinkType::right)),
                         static_cast<int>(dai_nodes::link_types::StereoLinkType::right));

        dai_nodes_.push_back(std::move(rgb));
        dai_nodes_.push_back(std::move(nn));
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
    dai::DeviceInfo info;
    auto mxid = this->get_parameter("mx_id").as_string();
    auto ip = this->get_parameter("ip").as_string();
    if(!mxid.empty()) {
        RCLCPP_INFO(this->get_logger(), "Connecting to the camera using mxid: %s", mxid.c_str());
        info = dai::DeviceInfo(mxid);
    } else if(!ip.empty()) {
        RCLCPP_INFO(this->get_logger(), "Connecting to the camera using ip: %s", ip.c_str());
        info = dai::DeviceInfo(ip);
    } else {
        RCLCPP_INFO(this->get_logger(), "No ip/mxid specified, connecting to the next available device.");
    }
    rclcpp::Rate r(1.0);
    bool cam_running;
    while(!cam_running) {
        try {
            dai::UsbSpeed speed = usb_speed_map_.at(this->get_parameter("usb_speed").as_string());
            bool usb2mode = this->get_parameter("usb2mode").as_bool();
            if(mxid.empty() && ip.empty()) {
                if(usb2mode) {
                    device_ = std::make_shared<dai::Device>(*pipeline_, usb2mode);
                } else {
                    device_ = std::make_shared<dai::Device>(*pipeline_, speed);
                }
            } else {
                if(usb2mode) {
                    device_ = std::make_shared<dai::Device>(*pipeline_, info, usb2mode);
                } else {
                    device_ = std::make_shared<dai::Device>(*pipeline_, info, speed);
                }
            }
            cam_running = true;
        } catch(const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
        r.sleep();
    }

    auto device_name = device_->getMxId();
    RCLCPP_INFO(this->get_logger(), "Camera %s connected!", device_name.c_str());
    if(ip.empty()) {
        auto speed = usbStrings[static_cast<int32_t>(device_->getUsbSpeed())];
        RCLCPP_INFO(this->get_logger(), "USB SPEED: %s", speed.c_str());
    }
}

rcl_interfaces::msg::SetParametersResult Camera::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& p : params) {
        if(this->get_parameter("enable_ir").as_bool()) {
            if(p.get_name() == "laser_dot_brightness") {
                device_->setIrLaserDotProjectorBrightness(p.get_value<int>());
            } else if(p.get_name() == "floodlight_brightness") {
                device_->setIrFloodLightBrightness(p.get_value<int>());
            }
        }
    }
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
    this->declare_parameter<bool>("enable_ir", true);
    this->declare_parameter<std::string>("usb_speed", "SUPER_PLUS");
    this->declare_parameter<bool>("usb2mode", false);
    this->declare_parameter<std::string>("mx_id", "");
    this->declare_parameter<std::string>("ip", "");
    this->declare_parameter<int>("laser_dot_brightness", 0, param_handlers::get_ranged_int_descriptor(0, 1200));
    this->declare_parameter<int>("floodlight_brightness", 0, param_handlers::get_ranged_int_descriptor(0, 1500));
}
}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);