#include "depthai_ros_driver/camera.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn.hpp"
#include "depthai_ros_driver/dai_nodes/stereo.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"

namespace depthai_ros_driver {

Camera::Camera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("camera", options) {
    ph = std::make_unique<param_handlers::CameraParamHandler>("camera");
    ph->declareParams(this);
    onConfigure();
}
void Camera::onConfigure() {
    getDeviceType();
    createPipeline();
    startDevice();
    setupQueues();
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&Camera::parameterCB, this, std::placeholders::_1));
    startSrv = this->create_service<Trigger>("~/start_camera", std::bind(&Camera::startCB, this, std::placeholders::_1, std::placeholders::_2));
    stopSrv = this->create_service<Trigger>("~/stop_camera", std::bind(&Camera::stopCB, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Camera ready!");
}

void Camera::startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    RCLCPP_INFO(this->get_logger(), "Starting camera!");
    onConfigure();
}
void Camera::stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    RCLCPP_INFO(this->get_logger(), "Stopping camera!");
    for(const auto& node : daiNodes) {
        node->closeQueues();
    }
    daiNodes.clear();
    device.reset();
    pipeline.reset();
}
void Camera::getDeviceType() {
    pipeline = std::make_shared<dai::Pipeline>();
    startDevice();
    auto name = device->getDeviceName();
    RCLCPP_INFO(this->get_logger(), "Device type: %s", name.c_str());
    for(auto& sensor : device->getCameraSensorNames()) {
        RCLCPP_INFO(this->get_logger(), "Socket %d - %s", static_cast<int>(sensor.first), sensor.second.c_str());
    }
    auto ir_drivers = device->getIrDrivers();
    if (ir_drivers.empty()){
        RCLCPP_INFO(this->get_logger(), "Device has no IR drivers");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "IR Drivers present");
    }

}

void Camera::createPipeline() {
    if(ph->getParam<std::string>(this, "i_pipeline_type") == "RGB") {
        auto rgb = std::make_unique<dai_nodes::CameraSensor>("rgb", this, pipeline, device, dai::CameraBoardSocket::RGB);
        auto nn_type = ph->getNNType(this);
        switch(nn_type) {
            case param_handlers::camera::NNType::None:
                break;
            case param_handlers::camera::NNType::RGB: {
                auto nn = std::make_unique<dai_nodes::NN>("nn", this, pipeline);
                rgb->link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                daiNodes.push_back(std::move(nn));
                break;
            }
            case param_handlers::camera::NNType::Spatial: {
                RCLCPP_WARN(this->get_logger(), "Spatial NN selected, but configuration is RGB.");
            }
            default:
                break;
        }
        daiNodes.push_back(std::move(rgb));
    } else if(ph->getParam<std::string>(this, "i_pipeline_type") == "RGBD") {
        auto rgb = std::make_unique<dai_nodes::CameraSensor>("rgb", this, pipeline,device, dai::CameraBoardSocket::RGB);
        auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", this, pipeline, device);
        auto nn_type = ph->getNNType(this);
        switch(nn_type) {
            case param_handlers::camera::NNType::None:
                break;
            case param_handlers::camera::NNType::RGB: {
                auto nn = std::make_unique<dai_nodes::NN>("nn", this, pipeline);
                rgb->link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                daiNodes.push_back(std::move(nn));
                break;
            }
            case param_handlers::camera::NNType::Spatial: {
                auto nn = std::make_unique<dai_nodes::SpatialNN>("nn", this, pipeline);
                rgb->link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::input)), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                stereo->link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::inputDepth)));
                daiNodes.push_back(std::move(nn));
                break;
            }
            default:
                break;
        }
        daiNodes.push_back(std::move(rgb));
        daiNodes.push_back(std::move(stereo));
    } else {
        std::string configuration =
            ph->getParam<std::string>(this, "i_pipeline_type") + " " + ph->getParam<std::string>(this, "i_cam_type");
        throw std::runtime_error("UNKNOWN PIPELINE TYPE SPECIFIED/CAMERA DOESN'T SUPPORT GIVEN PIPELINE. Configuration: " + configuration);
    }
    if(ph->getParam<bool>(this, "i_enable_imu")) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", this, pipeline);
        daiNodes.push_back(std::move(imu));
    }
    device.reset();
    RCLCPP_INFO(this->get_logger(), "Finished setting up pipeline.");
}

void Camera::setupQueues() {
    for(const auto& node : daiNodes) {
        node->setupQueues(device);
    }
}

void Camera::startDevice() {
    dai::DeviceInfo info;
    auto mxid = ph->getParam<std::string>(this, "i_mx_id");
    auto ip = ph->getParam<std::string>(this, "i_ip");
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
            dai::UsbSpeed speed = ph->getUSBSpeed(this);
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
        if(ph->getParam<bool>(this, "i_enable_ir") && !device->getIrDrivers().empty()) {
            if(p.get_name() == ph->getFullParamName("i_laser_dot_brightness")) {
                device->setIrLaserDotProjectorBrightness(p.get_value<int>());
            } else if(p.get_name() == ph->getFullParamName("i_floodlight_brightness")) {
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

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);