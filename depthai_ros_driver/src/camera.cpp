#include "depthai_ros_driver/camera.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_ros_driver/dai_nodes/imu.hpp"
#include "depthai_ros_driver/dai_nodes/mono.hpp"
#include "depthai_ros_driver/dai_nodes/nn.hpp"
#include "depthai_ros_driver/dai_nodes/spatial_detection.hpp"
#include "depthai_ros_driver/dai_nodes/rgb.hpp"
#include "depthai_ros_driver/dai_nodes/stereo.hpp"
#include "depthai_ros_driver/types.hpp"

namespace depthai_ros_driver {

Camera::Camera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("camera", options) {
    on_configure();
}
void Camera::on_configure() {
    declareParams();
    getDeviceType();
    createPipeline();
    startDevice();
    setupQueues();
    param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(&Camera::parameterCB, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Camera ready!");
}

void Camera::getDeviceType() {
    pipeline_ = std::make_shared<dai::Pipeline>();
    startDevice();
    auto name = device_->getDeviceName();
    RCLCPP_INFO(this->get_logger(), "Device type: %s", name.c_str());
    cam_type_ = std::make_unique<types::cam_types::CamType>(name);
    device_.reset();
}

void Camera::createPipeline() {
    dai_nodes::RGBFactory rgb_fac;
    dai_nodes::MonoFactory mono_fac;
    dai_nodes::StereoFactory stereo_fac;
    dai_nodes::ImuFactory imu_fac;
    dai_nodes::NNFactory nn_fac;
    dai_nodes::SpatialDetectionFactory spat_fac;
    if(this->get_parameter("i_pipeline_type").as_string() == "RGB") {
        auto rgb = rgb_fac.create("color", this, pipeline_);
        dai_nodes_.push_back(std::move(rgb));
    } else if(this->get_parameter("i_pipeline_type").as_string() == "RGBD" && cam_type_->stereo()) {
        auto rgb = rgb_fac.create("color", this, pipeline_);
        auto mono_left = mono_fac.create("mono_left", this, pipeline_);
        auto mono_right = mono_fac.create("mono_right", this, pipeline_);
        auto stereo = stereo_fac.create("stereo", this, pipeline_);
        mono_left->link(stereo->get_input(static_cast<int>(dai_nodes::link_types::StereoLinkType::left)),
                        static_cast<int>(dai_nodes::link_types::StereoLinkType::left));
        mono_right->link(stereo->get_input(static_cast<int>(dai_nodes::link_types::StereoLinkType::right)),
                         static_cast<int>(dai_nodes::link_types::StereoLinkType::right));

        auto nn_type = nn_type_map_.at(this->get_parameter("i_nn_type").as_string());
        switch (nn_type){
            case types::NNType::None:
                break;
            case types::NNType::Default:{
                auto nn = nn_fac.create("nn", this, pipeline_);
                rgb->link(nn->get_input(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                dai_nodes_.push_back(std::move(nn));
                break;
            }
            case types::NNType::Spatial:{
                auto nn = spat_fac.create("spatial_nn", this, pipeline_);
                rgb->link(nn->get_input(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                stereo->link(nn->get_input(static_cast<int>(dai_nodes::link_types::SpatialDetectionLinkType::inputDepth)));
                dai_nodes_.push_back(std::move(nn));
                break;
            }
            default:
                break;
        }

        dai_nodes_.push_back(std::move(rgb));
        dai_nodes_.push_back(std::move(mono_right));
        dai_nodes_.push_back(std::move(mono_left));
        dai_nodes_.push_back(std::move(stereo));
    } else {
        std::string configuration = this->get_parameter("i_pipeline_type").as_string() + " " + this->get_parameter("i_cam_type").as_string();
        throw std::runtime_error("UNKNOWN PIPELINE TYPE SPECIFIED/CAMERA DOESN'T SUPPORT GIVEN PIPELINE. Configuration: " + configuration);
    }
    if(this->get_parameter("i_enable_imu").as_bool() && cam_type_->imu_available()) {
        auto imu = imu_fac.create("imu", this, pipeline_);
        dai_nodes_.push_back(std::move(imu));
    }
    auto json = pipeline_->serializeToJson();
    RCLCPP_INFO(this->get_logger(), "Pipeline. %s", json.dump().c_str());
    RCLCPP_INFO(this->get_logger(), "Finished setting up pipeline.");
}

void Camera::setupQueues() {
    for(const auto& node : dai_nodes_) {
        node->setupQueues(device_);
    }
}

void Camera::startDevice() {
    dai::DeviceInfo info;
    auto mxid = this->get_parameter("i_mx_id").as_string();
    auto ip = this->get_parameter("i_ip").as_string();
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
            dai::UsbSpeed speed = usb_speed_map_.at(this->get_parameter("i_usb_speed").as_string());
            if(mxid.empty() && ip.empty()) {
                device_ = std::make_shared<dai::Device>(*pipeline_, speed);

            } else {
                device_ = std::make_shared<dai::Device>(*pipeline_, info, speed);
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
        if(this->get_parameter("i_enable_ir").as_bool() && cam_type_->ir_available()) {
            if(p.get_name() == "i_laser_dot_brightness") {
                device_->setIrLaserDotProjectorBrightness(p.get_value<int>());
            } else if(p.get_name() == "i_floodlight_brightness") {
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
    this->declare_parameter<std::string>("i_pipeline_type", "RGBD");
    this->declare_parameter<std::string>("i_nn_type", "default");
    this->declare_parameter<bool>("i_enable_imu", true);
    this->declare_parameter<bool>("i_enable_ir", true);
    this->declare_parameter<std::string>("i_usb_speed", "SUPER_PLUS");
    this->declare_parameter<std::string>("i_mx_id", "");
    this->declare_parameter<std::string>("i_ip", "");
    this->declare_parameter<int>("i_laser_dot_brightness", 0, param_handlers::get_ranged_int_descriptor(0, 1200));
    this->declare_parameter<int>("i_floodlight_brightness", 0, param_handlers::get_ranged_int_descriptor(0, 1500));
}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);