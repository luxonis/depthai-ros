#include "depthai_ros_driver/camera.hpp"

#include <fstream>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace depthai_ros_driver {

Camera::Camera(const rclcpp::NodeOptions& options) : rclcpp::Node("camera", options) {
    //  Since we cannot use shared_from this before the object is initialized, we need to use a timer to start the device.
    startTimer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        ph = std::make_unique<param_handlers::CameraParamHandler>(shared_from_this(), "camera");
        ph->declareParams();
        start();
        startTimer->cancel();
    });
    rclcpp::on_shutdown([this]() { stop(); });
}
Camera::~Camera() {
    stop();
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
    savePipelineSrv = this->create_service<Trigger>("~/save_pipeline", std::bind(&Camera::savePipelineCB, this, std::placeholders::_1, std::placeholders::_2));
    saveCalibSrv = this->create_service<Trigger>("~/save_calibration", std::bind(&Camera::saveCalibCB, this, std::placeholders::_1, std::placeholders::_2));

    // If model name not set get one from the device
    std::string camModel = ph->getParam<std::string>("i_tf_camera_model");
    if(camModel.empty()) {
        camModel = device->getDeviceName();
    }

    if(ph->getParam<bool>("i_publish_tf_from_calibration")) {
        tfPub = std::make_unique<dai::ros::TFPublisher>(shared_from_this(),
                                                        device->readCalibration(),
                                                        device->getConnectedCameraFeatures(),
                                                        ph->getParam<std::string>("i_tf_camera_name"),
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
    diagSub = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10, std::bind(&Camera::diagCB, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Camera ready!");
}

void Camera::diagCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    for(const auto& status : msg->status) {
        if(status.name == get_name() + std::string(": sys_logger")) {
            if(status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
                RCLCPP_ERROR(get_logger(), "Camera diagnostics error: %s", status.message.c_str());
                if(ph->getParam<bool>("i_restart_on_diagnostics_error")) {
                    restart();
                };
            }
        }
    }
}

void Camera::start() {
    RCLCPP_INFO(this->get_logger(), "Starting camera.");
    if(!camRunning) {
        onConfigure();
    } else {
        RCLCPP_INFO(this->get_logger(), "Camera already running!.");
    }
}

void Camera::stop() {
    RCLCPP_INFO(get_logger(), "Stopping camera.");
    if(camRunning) {
        for(const auto& node : daiNodes) {
            node->closeQueues();
        }
        daiNodes.clear();
        device.reset();
        pipeline.reset();
        camRunning = false;
    } else {
        RCLCPP_INFO(get_logger(), "Camera already stopped!");
    }
}

void Camera::restart() {
    RCLCPP_ERROR(get_logger(), "Restarting camera");
    stop();
    start();
    if(camRunning) {
        return;
    } else {
        RCLCPP_ERROR(get_logger(), "Restarting camera failed.");
    }
}

void Camera::saveCalib() {
    auto calibHandler = device->readCalibration();
    std::stringstream savePath;
    savePath << "/tmp/" << device->getMxId().c_str() << "_calibration.json";
    RCLCPP_INFO(get_logger(), "Saving calibration to: %s", savePath.str().c_str());
    calibHandler.eepromToJsonFile(savePath.str());
}

void Camera::loadCalib(const std::string& path) {
    RCLCPP_INFO(get_logger(), "Reading calibration from: %s", path.c_str());
    dai::CalibrationHandler cH(path);
    pipeline->setCalibrationData(cH);
}

void Camera::saveCalibCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    saveCalib();
    res->success = true;
}

void Camera::savePipeline() {
    std::stringstream savePath;
    savePath << "/tmp/" << device->getMxId().c_str() << "_pipeline.json";
    RCLCPP_INFO(get_logger(), "Saving pipeline schema to: %s", savePath.str().c_str());
    std::ofstream file(savePath.str());
    file << pipeline->serializeToJson()["pipeline"];
    file.close();
}

void Camera::savePipelineCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    savePipeline();
    res->success = true;
}

void Camera::startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    start();
    res->success = true;
}
void Camera::stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res) {
    stop();
    res->success = true;
}
void Camera::getDeviceType() {
    pipeline = std::make_shared<dai::Pipeline>();
    startDevice();
    auto name = device->getDeviceName();
    RCLCPP_INFO(get_logger(), "Device type: %s", name.c_str());
    for(auto& sensor : device->getCameraSensorNames()) {
        RCLCPP_DEBUG(get_logger(), "Socket %d - %s", static_cast<int>(sensor.first), sensor.second.c_str());
    }
    auto ir_drivers = device->getIrDrivers();
    if(ir_drivers.empty()) {
        RCLCPP_DEBUG(get_logger(), "Device has no IR drivers");
    } else {
        RCLCPP_DEBUG(get_logger(), "IR Drivers present");
    }
}

void Camera::createPipeline() {
    auto generator = std::make_unique<pipeline_gen::PipelineGenerator>();
    if(!ph->getParam<std::string>("i_external_calibration_path").empty()) {
        loadCalib(ph->getParam<std::string>("i_external_calibration_path"));
    }
    daiNodes =
        generator->createPipeline(shared_from_this(), device, pipeline, ph->getParam<std::string>("i_pipeline_type"), ph->getParam<std::string>("i_nn_type"));
    if(ph->getParam<bool>("i_pipeline_dump")) {
        savePipeline();
    }
    if(ph->getParam<bool>("i_calibration_dump")) {
        saveCalib();
    }
}

void Camera::setupQueues() {
    for(const auto& node : daiNodes) {
        node->setupQueues(device);
    }
}

void Camera::startDevice() {
    rclcpp::Rate r(1.0);
    while(rclcpp::ok() && !camRunning) {
        auto mxid = ph->getParam<std::string>("i_mx_id");
        auto ip = ph->getParam<std::string>("i_ip");
        auto usb_id = ph->getParam<std::string>("i_usb_port_id");
        try {
            if(mxid.empty() && ip.empty() && usb_id.empty()) {
                RCLCPP_INFO(get_logger(), "No ip/mxid specified, connecting to the next available device.");
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
                        RCLCPP_INFO(this->get_logger(), "No devices detected by autodiscovery, trying to connect to camera via IP: %s", ip.c_str());
                        availableDevices.push_back(info);
                    } else {
                        throw std::runtime_error("No devices detected!");
                    }
                }
                dai::UsbSpeed speed = ph->getUSBSpeed();
                for(const auto& info : availableDevices) {
                    if(!mxid.empty() && info.getMxId() == mxid) {
                        RCLCPP_INFO(get_logger(), "Connecting to the camera using mxid: %s", mxid.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else if(!ip.empty() && info.name == ip) {
                        RCLCPP_INFO(get_logger(), "Connecting to the camera using ip: %s", ip.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process...");
                        }
                    } else if(!usb_id.empty() && info.name == usb_id) {
                        RCLCPP_INFO(get_logger(), "Connecting to the camera using USB ID: %s", usb_id.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else {
                        RCLCPP_INFO(get_logger(), "Ignoring device info: MXID: %s, Name: %s", info.getMxId().c_str(), info.name.c_str());
                    }
                }
            }
        } catch(const std::runtime_error& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
        r.sleep();
    }

    RCLCPP_INFO(get_logger(), "Camera with MXID: %s and Name: %s connected!", device->getMxId().c_str(), device->getDeviceInfo().name.c_str());
    auto protocol = device->getDeviceInfo().getXLinkDeviceDesc().protocol;

    if(protocol != XLinkProtocol_t::X_LINK_TCP_IP) {
        auto speed = usbStrings[static_cast<int32_t>(device->getUsbSpeed())];
        RCLCPP_INFO(get_logger(), "USB SPEED: %s", speed.c_str());
    } else {
        RCLCPP_INFO(get_logger(),
                    "PoE camera detected. Consider enabling low bandwidth for specific image topics (see "
                    "Readme->DepthAI ROS Driver->Specific camera configurations).");
    }
}

void Camera::setIR() {
    if(ph->getParam<bool>("i_enable_ir") && !device->getIrDrivers().empty()) {
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

rcl_interfaces::msg::SetParametersResult Camera::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& p : params) {
        if(ph->getParam<bool>("i_enable_ir") && !device->getIrDrivers().empty()) {
            if(p.get_name() == ph->getFullParamName("i_laser_dot_brightness")) {
                float laserdotBrightness = float(p.get_value<int>());
                if(laserdotBrightness > 1.0) {
                    laserdotBrightness = 1200.0 / laserdotBrightness;
                }
                device->setIrLaserDotProjectorIntensity(laserdotBrightness);
            } else if(p.get_name() == ph->getFullParamName("i_floodlight_brightness")) {
                float floodlightBrightness = float(p.get_value<int>());
                if(floodlightBrightness > 1.0) {
                    floodlightBrightness = 1500.0 / floodlightBrightness;
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
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::Camera);
