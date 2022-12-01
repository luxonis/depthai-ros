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
    onConfigure();
}
void Camera::onConfigure() {
    declareParams();
    getDeviceType();
    createPipeline();
    startDevice();
    setupQueues();
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&Camera::parameterCB, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Camera ready!");
}

void Camera::getDeviceType() {
    pipeline = std::make_shared<dai::Pipeline>();
    startDevice();
    auto name = device->getDeviceName();
    RCLCPP_INFO(this->get_logger(), "Device type: %s", name.c_str());

    cam_type = std::make_unique<types::cam_types::CamType>(name);
    device.reset();
}

void Camera::createPipeline() {
    dai_nodes::RGBFactory rgbFac;
    dai_nodes::MonoFactory monoFac;
    dai_nodes::StereoFactory stereoFac;
    dai_nodes::ImuFactory imuFac;
    dai_nodes::NNFactory nnFac;
    dai_nodes::SpatialDetectionFactory spat_fac;
    if(this->get_parameter("i_pipelinetype").as_string() == "RGB") {
        auto rgb = rgbFac.create("color", this, pipeline);
    
        auto nn_type = nnTypeMap.at(this->get_parameter("i_nn_type").as_string());
        switch (nn_type){
            case types::nn_types::NNType::None:
                break;
            case types::nn_types::NNType::Default:{
                auto nn = nnFac.create("nn", this, pipeline);
                rgb->link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                daiNodes.push_back(std::move(nn));
                break;
            }
            case types::nn_types::NNType::Spatial:{
                break;
            }
            default:
                break;
        }
        daiNodes.push_back(std::move(rgb));
    } else if(this->get_parameter("i_pipelinetype").as_string() == "RGBD" && cam_type->stereo()) {
        auto rgb = rgbFac.create("color", this, pipeline);
        auto mono_left = monoFac.create("mono_left", this, pipeline);
        auto mono_right = monoFac.create("mono_right", this, pipeline);
        auto stereo = stereoFac.create("stereo", this, pipeline);
        mono_left->link(stereo->getInput(static_cast<int>(dai_nodes::link_types::StereoLinkType::left)),
                        static_cast<int>(dai_nodes::link_types::StereoLinkType::left));
        mono_right->link(stereo->getInput(static_cast<int>(dai_nodes::link_types::StereoLinkType::right)),
                         static_cast<int>(dai_nodes::link_types::StereoLinkType::right));

        auto nn_type = nnTypeMap.at(this->get_parameter("i_nn_type").as_string());
        switch (nn_type){
            case types::nn_types::NNType::None:
                break;
            case types::nn_types::NNType::Default:{
                auto nn = nnFac.create("nn", this, pipeline);
                rgb->link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                daiNodes.push_back(std::move(nn));
                break;
            }
            case types::nn_types::NNType::Spatial:{
                break;
            }
            default:
                break;
        }

        daiNodes.push_back(std::move(rgb));
        daiNodes.push_back(std::move(mono_right));
        daiNodes.push_back(std::move(mono_left));
        daiNodes.push_back(std::move(stereo));
    } else {
        std::string configuration = this->get_parameter("i_pipelinetype").as_string() + " " + this->get_parameter("i_cam_type").as_string();
        throw std::runtime_error("UNKNOWN PIPELINE TYPE SPECIFIED/CAMERA DOESN'T SUPPORT GIVEN PIPELINE. Configuration: " + configuration);
    }
    if(this->get_parameter("i_enable_imu").as_bool() && cam_type->imu_available()) {
        auto imu = imuFac.create("imu", this, pipeline);
        daiNodes.push_back(std::move(imu));
    }
    auto json = pipeline->serializeToJson();
    std::ofstream file;
    file.open("/home/adaser/pipeline.json");
    file << json.dump().c_str() << std::endl;
    file.close();
    RCLCPP_INFO(this->get_logger(), "Finished setting up pipeline.");
}

void Camera::setupQueues() {
    for(const auto& node : daiNodes) {
        node->setupQueues(device);
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
            dai::UsbSpeed speed = usbSpeedMap.at(this->get_parameter("i_usb_speed").as_string());
            if(mxid.empty() && ip.empty()) {
                device = std::make_shared<dai::Device>(*pipeline, speed);

            } else {
                device = std::make_shared<dai::Device>(*pipeline, info, speed);
            }
            cam_running = true;
        } catch(const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
        r.sleep();
    }

    auto devicename = device->getMxId();
    RCLCPP_INFO(this->get_logger(), "Camera %s connected!", devicename.c_str());
    if(ip.empty()) {
        auto speed = usbStrings[static_cast<int32_t>(device->getUsbSpeed())];
        RCLCPP_INFO(this->get_logger(), "USB SPEED: %s", speed.c_str());
    }
}

rcl_interfaces::msg::SetParametersResult Camera::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& p : params) {
        if(this->get_parameter("i_enable_ir").as_bool() && cam_type->ir_available()) {
            if(p.get_name() == "i_laser_dot_brightness") {
                device->setIrLaserDotProjectorBrightness(p.get_value<int>());
            } else if(p.get_name() == "i_floodlight_brightness") {
                device->setIrFloodLightBrightness(p.get_value<int>());
            }
        }
    }
    for(const auto& node : daiNodes) {
        node->updateParams(params);
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
}
void Camera::declareParams() {
    this->declare_parameter<std::string>("i_pipelinetype", "RGBD");
    this->declare_parameter<std::string>("i_nn_type", "default");
    this->declare_parameter<bool>("i_enable_imu", true);
    this->declare_parameter<bool>("i_enable_ir", true);
    this->declare_parameter<std::string>("i_usb_speed", "SUPER_PLUS");
    this->declare_parameter<std::string>("i_mx_id", "");
    this->declare_parameter<std::string>("i_ip", "");
    this->declare_parameter<int>("i_laser_dot_brightness", 0, param_handlers::getRangedIntDescriptor(0, 1200));
    this->declare_parameter<int>("i_floodlight_brightness", 0, param_handlers::getRangedIntDescriptor(0, 1500));
}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);