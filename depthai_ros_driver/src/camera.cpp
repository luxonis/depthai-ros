#include "depthai_ros_driver/camera.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_ros_driver/pipeline_generator.hpp"

namespace depthai_ros_driver {

Camera::Camera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("camera", options) {
    ph = std::make_unique<param_handlers::CameraParamHandler>("camera");
    ph->declareParams(this);
    onConfigure();
}
void Camera::onConfigure() {
    getDeviceType();
    createPipeline();
    device->startPipeline(*pipeline);
    setupQueues();
    setIR();
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&Camera::parameterCB, this, std::placeholders::_1));
    startSrv = this->create_service<Trigger>("~/start_camera", std::bind(&Camera::startCB, this, std::placeholders::_1, std::placeholders::_2));
    stopSrv = this->create_service<Trigger>("~/stop_camera", std::bind(&Camera::stopCB, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Camera ready!");
}

void Camera::startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    RCLCPP_INFO(this->get_logger(), "Starting camera.");
    onConfigure();
    res->success = true;
}
void Camera::stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    RCLCPP_INFO(this->get_logger(), "Stopping camera.");
    for(const auto& node : daiNodes) {
        node->closeQueues();
    }
    daiNodes.clear();
    device.reset();
    pipeline.reset();
    res->success = true;
}
void Camera::getDeviceType() {
    pipeline = std::make_shared<dai::Pipeline>();
    startDevice();
    auto name = device->getDeviceName();
    RCLCPP_INFO(this->get_logger(), "Device type: %s", name.c_str());
    for(auto& sensor : device->getCameraSensorNames()) {
        RCLCPP_DEBUG(this->get_logger(), "Socket %d - %s", static_cast<int>(sensor.first), sensor.second.c_str());
    }
    auto ir_drivers = device->getIrDrivers();
    if(ir_drivers.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Device has no IR drivers");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "IR Drivers present");
    }
}

void Camera::createPipeline() {
    auto generator = std::make_unique<pipeline_gen::PipelineGenerator>();
    daiNodes = generator->createPipeline(this,
                                         device,
                                         pipeline,
                                         ph->getParam<std::string>(this, "i_pipeline_type"),
                                         ph->getParam<std::string>(this, "i_nn_type"),
                                         ph->getParam<bool>(this, "i_enable_imu"));
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
    } else if(!ip.empty()) {
    }
    rclcpp::Rate r(1.0);
    bool cam_running = false;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
    while(!cam_running) {
        try {
            dai::UsbSpeed speed = ph->getUSBSpeed(this);
            if(mxid.empty() && ip.empty()) {
                RCLCPP_INFO(this->get_logger(), "No ip/mxid specified, connecting to the next available device.");
                device = std::make_shared<dai::Device>();
            } else {
                for(const auto& info : availableDevices) {
                    if(!mxid.empty() && info.getMxId() == mxid) {
                        RCLCPP_INFO(this->get_logger(), "Connecting to the camera using mxid: %s", mxid.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device with mxid %s is already booted in different process.");
                        }
                    } else if(!ip.empty() && info.name == ip) {
                        RCLCPP_INFO(this->get_logger(), "Connecting to the camera using ip: %s", ip.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info);
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device with ip %s is already booted in different process.");
                        }
                    }
                }
            }
            cam_running = true;
        } catch(const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
        r.sleep();
    }

    auto deviceName = device->getMxId();
    RCLCPP_INFO(this->get_logger(), "Camera %s connected!", deviceName.c_str());
    auto protocol = device->getDeviceInfo().getXLinkDeviceDesc().protocol;

    if(protocol != XLinkProtocol_t::X_LINK_TCP_IP) {
        auto speed = usbStrings[static_cast<int32_t>(device->getUsbSpeed())];
        RCLCPP_INFO(this->get_logger(), "USB SPEED: %s", speed.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "PoE camera detected. Consider enabling low bandwidth for specific image topics (see "
                    "readme).");
    }
}

void Camera::setIR() {
    if(ph->getParam<bool>(this, "i_enable_ir") && !device->getIrDrivers().empty()) {
        device->setIrLaserDotProjectorBrightness(ph->getParam<int>(this, "i_laser_dot_brightness"));
        device->setIrFloodLightBrightness(ph->getParam<int>(this, "i_floodlight_brightness"));
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