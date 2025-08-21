#include "depthai_ros_driver/driver.hpp"

#include <fstream>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace depthai_ros_driver {

Driver::Driver(const rclcpp::NodeOptions& options) : rclcpp::Node("driver", options) {
    //  Since we cannot use shared_from this before the object is initialized, we need to use a timer to start the device.
    startTimer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        start();
        startTimer->cancel();
    });
    rclcpp::on_shutdown([this]() { stop(); });
}
void Driver::onConfigure() {
    ph = std::make_unique<param_handlers::DriverParamHandler>(shared_from_this(), "driver");
    ph->declareParams();
    getDeviceType();
    createPipeline();
    setupQueues();
    setIR();
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&Driver::parameterCB, this, std::placeholders::_1));
    // If model name not set get one from the device
    std::string camModel = ph->getParam<std::string>("i_tf_device_model");
    if(camModel.empty()) {
        camModel = device->getDeviceName();
    }

    if(ph->getParam<bool>("i_publish_tf_from_calibration")) {
        tfPub = std::make_unique<depthai_bridge::TFPublisher>(shared_from_this(),
                                                              device->readCalibration(),
                                                              device->getConnectedCameraFeatures(),
                                                              ph->getParam<std::string>("i_tf_device_name"),
                                                              camModel,
                                                              ph->getParam<std::string>("i_tf_base_frame"),
                                                              ph->getParam<std::string>("i_tf_parent_frame"),
                                                              ph->getParam<std::string>("i_tf_cam_pos_x"),
                                                              ph->getParam<std::string>("i_tf_cam_pos_y"),
                                                              ph->getParam<std::string>("i_tf_cam_pos_z"),
                                                              ph->getParam<std::string>("i_tf_cam_roll"),
                                                              ph->getParam<std::string>("i_tf_cam_pitch"),
                                                              ph->getParam<std::string>("i_tf_cam_yaw"),
                                                              ph->getParam<std::string>("i_tf_imu_from_descr"),
                                                              ph->getParam<std::string>("i_tf_custom_urdf_location"),
                                                              ph->getParam<std::string>("i_tf_custom_xacro_args"),
                                                              ph->getParam<bool>("i_rs_compat"));
    }
    srvGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    startSrv = this->create_service<Trigger>(
        "~/start_driver", std::bind(&Driver::startCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    stopSrv = this->create_service<Trigger>(
        "~/stop_driver", std::bind(&Driver::stopCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    savePipelineSrv = this->create_service<Trigger>(
        "~/save_pipeline", std::bind(&Driver::savePipelineCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);
    saveCalibSrv = this->create_service<Trigger>(
        "~/save_calibration", std::bind(&Driver::saveCalibCB, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), srvGroup);

    diagSub = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10, std::bind(&Driver::diagCB, this, std::placeholders::_1));
    pipeline->start();
    RCLCPP_WARN(get_logger(),
                "Driver is still at beta stage! Expect further stability & usability improvements towards end of August 2025.\n In meantime, please report "
                "issues to GH: https://github.com/luxonis/depthai-ros/issues/719");
    RCLCPP_INFO(get_logger(), "Driver ready!");
}

void Driver::diagCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    for(const auto& status : msg->status) {
        if(status.name == get_name() + std::string(": sys_logger")) {
            if(status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
                RCLCPP_ERROR(get_logger(), "Driver diagnostics error: %s", status.message.c_str());
                if(ph->getParam<bool>("i_restart_on_diagnostics_error")) {
                    restart();
                };
            }
        }
    }
}

void Driver::start() {
    RCLCPP_INFO(this->get_logger(), "Starting driver.");
    if(!camRunning) {
        onConfigure();
    } else {
        RCLCPP_INFO(this->get_logger(), "Driver already running!.");
    }
}

void Driver::stop() {
    if(rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "Stopping driver.");
    }
    if(camRunning) {
        for(const auto& node : daiNodes) {
            node->closeQueues();
        }
        daiNodes.clear();
        device.reset();
        pipeline.reset();
        camRunning = false;
        if(rclcpp::ok()) {
            RCLCPP_INFO(get_logger(), "Driver stopped!");
        }
    } else {
        RCLCPP_INFO(get_logger(), "Driver already stopped!");
    }
}

void Driver::restart() {
    RCLCPP_ERROR(get_logger(), "Restarting driver");
    stop();
    start();
    if(camRunning) {
        return;
    } else {
        RCLCPP_ERROR(get_logger(), "Restarting driver failed.");
    }
}

void Driver::saveCalib() {
    auto calibHandler = device->readCalibration();
    std::stringstream savePath;
    savePath << "/tmp/" << device->getDeviceId().c_str() << "_calibration.json";
    RCLCPP_INFO(get_logger(), "Saving calibration to: %s", savePath.str().c_str());
    calibHandler.eepromToJsonFile(savePath.str());
}

void Driver::loadCalib(const std::string& path) {
    RCLCPP_INFO(get_logger(), "Reading calibration from: %s", path.c_str());
    dai::CalibrationHandler cH(path);
    pipeline->setCalibrationData(cH);
}

void Driver::saveCalibCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    saveCalib();
    res->success = true;
}

void Driver::savePipeline() {
    std::stringstream savePath;
    savePath << "/tmp/" << device->getDeviceId().c_str() << "_pipeline.json";
    RCLCPP_INFO(get_logger(), "Saving pipeline schema to: %s", savePath.str().c_str());
    std::ofstream file(savePath.str());
    file << pipeline->serializeToJson()["pipeline"];
    file.close();
}

void Driver::savePipelineCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    savePipeline();
    res->success = true;
}

void Driver::startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    start();
    res->success = true;
}
void Driver::stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    stop();
    res->success = true;
}
void Driver::getDeviceType() {
    startDevice();
    platform = device->getPlatform();
    pipeline = std::make_shared<dai::Pipeline>(device);
    deviceName = device->getDeviceName();
    RCLCPP_INFO(get_logger(), "Device type: %s", deviceName.c_str());
    for(auto& sensor : device->getCameraSensorNames()) {
        RCLCPP_DEBUG(get_logger(), "Socket %d - %s", static_cast<int>(sensor.first), sensor.second.c_str());
    }
    // not working on OAK4 right now
    if(platform == dai::Platform::RVC2) {
        auto ir_drivers = device->getIrDrivers();
        if(ir_drivers.empty()) {
            RCLCPP_DEBUG(get_logger(), "Device has no IR drivers");
        } else {
            RCLCPP_DEBUG(get_logger(), "IR Drivers present");
        }
    }
}

void Driver::createPipeline() {
    auto generator = std::make_unique<pipeline_gen::PipelineGenerator>();
    if(!ph->getParam<std::string>("i_external_calibration_path").empty()) {
        loadCalib(ph->getParam<std::string>("i_external_calibration_path"));
    }
    daiNodes = generator->createPipeline(shared_from_this(), device, pipeline, ph->getParam<bool>("i_rs_compat"));
    if(ph->getParam<bool>("i_pipeline_dump")) {
        savePipeline();
    }
    if(ph->getParam<bool>("i_calibration_dump")) {
        saveCalib();
    }
}

void Driver::setupQueues() {
    for(const auto& node : daiNodes) {
        node->setupQueues(device);
    }
}

void Driver::startDevice() {
    rclcpp::Rate r(1.0);
    while(rclcpp::ok() && !camRunning) {
        auto deviceId = ph->getParam<std::string>("i_device_id");
        auto ip = ph->getParam<std::string>("i_ip");
        auto usb_id = ph->getParam<std::string>("i_usb_port_id");
        try {
            if(deviceId.empty() && ip.empty() && usb_id.empty()) {
                RCLCPP_INFO(get_logger(), "No ip/ID specified, connecting to the next available device.");
                auto info = dai::Device::getAnyAvailableDevice();
                auto speed = ph->getUSBSpeed();
                device = std::make_shared<dai::Device>(std::get<1>(info), speed);
                camRunning = true;
            } else {
                std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
                if(availableDevices.size() == 0) {
                    // autodiscovery might not work so try connecting via IP directly if set
                    if(!ip.empty()) {
                        dai::DeviceInfo info(ip);
                        RCLCPP_INFO(this->get_logger(), "No devices detected by autodiscovery, trying to connect to device via IP: %s", ip.c_str());
                        availableDevices.push_back(info);
                    } else {
                        throw std::runtime_error("No devices detected!");
                    }
                }
                dai::UsbSpeed speed = ph->getUSBSpeed();
                for(const auto& info : availableDevices) {
                    if(!deviceId.empty() && info.getDeviceId() == deviceId) {
                        RCLCPP_INFO(get_logger(), "Connecting to the device using ID: %s", deviceId.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else if(!ip.empty() && info.name == ip) {
                        RCLCPP_INFO(get_logger(), "Connecting to the device using ip: %s", ip.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process...");
                        }
                    } else if(!usb_id.empty() && info.name == usb_id) {
                        RCLCPP_INFO(get_logger(), "Connecting to the device using USB ID: %s", usb_id.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else {
                        RCLCPP_INFO(get_logger(), "Ignoring device info: ID: %s, Name: %s", info.getDeviceId().c_str(), info.name.c_str());
                    }
                }
            }
        } catch(const std::runtime_error& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
        r.sleep();
    }

    // device = std::make_shared<dai::Device>();

    RCLCPP_INFO(get_logger(), "Driver with ID: %s and Name: %s connected!", device->getDeviceId().c_str(), device->getDeviceInfo().name.c_str());
    auto protocol = device->getDeviceInfo().getXLinkDeviceDesc().protocol;

    if(protocol != XLinkProtocol_t::X_LINK_TCP_IP) {
        auto speed = usbStrings[static_cast<int32_t>(device->getUsbSpeed())];
        RCLCPP_INFO(get_logger(), "USB SPEED: %s", speed.c_str());
    } else {
        RCLCPP_INFO(get_logger(),
                    "PoE device detected. Consider enabling low bandwidth for specific image topics (see "
                    "Readme->DepthAI ROS Driver->Specific device configurations).");
    }
}

void Driver::setIR() {
    bool hasIR = true;
    if(platform == dai::Platform::RVC2) {
        hasIR = !device->getIrDrivers().empty();
    }
    if(ph->getParam<bool>("i_enable_ir") && hasIR) {
        // Normalize laserdot brightness to 0-1 range, max value can be 1200mA
        float laserdotBrightness = float(ph->getParam<int>("i_laser_dot_brightness"));
        if(laserdotBrightness > 1.0) {
            laserdotBrightness = laserdotBrightness / 1200.0;
        }
        // Normalize floodlight brightness to 0-1 range, max value can be 1500mA
        float floodlightBrightness = float(ph->getParam<int>("i_floodlight_brightness"));
        if(floodlightBrightness > 1.0) {
            floodlightBrightness = floodlightBrightness / 1500.0;
        }
        device->setIrLaserDotProjectorIntensity(laserdotBrightness);
        device->setIrFloodLightIntensity(floodlightBrightness);
    }
}

rcl_interfaces::msg::SetParametersResult Driver::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& p : params) {
        bool hasIR = true;
        if(platform == dai::Platform::RVC2) {
            hasIR = !device->getIrDrivers().empty();
        }
        if(ph->getParam<bool>("i_enable_ir") && hasIR) {
            if(p.get_name() == ph->getFullParamName("i_laser_dot_brightness")) {
                float laserdotBrightness = float(p.get_value<int>());
                if(laserdotBrightness > 1.0) {
                    laserdotBrightness = laserdotBrightness / 1200.0;
                }
                device->setIrLaserDotProjectorIntensity(laserdotBrightness);
            } else if(p.get_name() == ph->getFullParamName("i_floodlight_brightness")) {
                float floodlightBrightness = float(p.get_value<int>());
                if(floodlightBrightness > 1.0) {
                    floodlightBrightness = floodlightBrightness / 1500.0;
                }
                device->setIrFloodLightIntensity(floodlightBrightness);
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
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Driver);
