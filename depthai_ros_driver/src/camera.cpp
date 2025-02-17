#include "depthai_ros_driver/camera.hpp"

#include <XLink/XLinkPublicDefines.h>

#include <fstream>
#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"
#include "dynamic_reconfigure/server.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depthai_ros_driver {

void Camera::onInit() {
    pNH = getPrivateNodeHandle();
    ph = std::make_unique<param_handlers::CameraParamHandler>(pNH, "camera");
    ph->declareParams();
    paramServer = std::make_shared<dynamic_reconfigure::Server<parametersConfig>>(pNH);
    paramServer->setCallback(std::bind(&Camera::parameterCB, this, std::placeholders::_1, std::placeholders::_2));
    start();
    startSrv = pNH.advertiseService("start_camera", &Camera::startCB, this);
    stopSrv = pNH.advertiseService("stop_camera", &Camera::stopCB, this);
    savePipelineSrv = pNH.advertiseService("save_pipeline", &Camera::startCB, this);
    saveCalibSrv = pNH.advertiseService("save_calibration", &Camera::stopCB, this);

    diagSub = pNH.subscribe("/diagnostics", 1, &Camera::diagCB, this);
}

Camera::~Camera() {
    stop();
}

void Camera::diagCB(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
    for(const auto& status : msg->status) {
        std::string nodeletName = pNH.getNamespace() + "_nodelet_manager";
        nodeletName.erase(nodeletName.begin());
        if(status.name == nodeletName + std::string(": sys_logger")) {
            if(status.level == diagnostic_msgs::DiagnosticStatus::ERROR) {
                ROS_ERROR("Camera diagnostics error: %s", status.message.c_str());
                if(ph->getParam<bool>("i_restart_on_diagnostics_error")) {
                    restart();
                }
            }
        }
    }
}

void Camera::start() {
    ROS_INFO("Starting camera.");
    if(!camRunning) {
        onConfigure();
    } else {
        ROS_INFO("Camera already running!.");
    }
}
void Camera::stop() {
    ROS_INFO("Stopping camera.");
    if(camRunning) {
        for(const auto& node : daiNodes) {
            node->closeQueues();
        }
        daiNodes.clear();
        device.reset();
        pipeline.reset();
        camRunning = false;
    } else {
        ROS_INFO("Camera already stopped!");
    }
}

void Camera::restart() {
    ROS_INFO("Restarting camera");
    stop();
    start();
    if(camRunning) {
        return;
    } else {
        ROS_ERROR("Restarting camera failed.");
    }
}

void Camera::saveCalib() {
    auto calibHandler = device->readCalibration();
    std::stringstream savePath;
    savePath << "/tmp/" << device->getMxId().c_str() << "_calibration.json";
    ROS_INFO("Saving calibration to: %s", savePath.str().c_str());
    calibHandler.eepromToJsonFile(savePath.str());
}

void Camera::loadCalib(const std::string& path) {
    ROS_INFO("Reading calibration from: %s", path.c_str());
    dai::CalibrationHandler cH(path);
    pipeline->setCalibrationData(cH);
}

void Camera::savePipeline() {
    std::stringstream savePath;
    savePath << "/tmp/" << device->getMxId().c_str() << "_pipeline.json";
    ROS_INFO("Saving pipeline schema to: %s", savePath.str().c_str());
    std::ofstream file(savePath.str());
    file << pipeline->serializeToJson()["pipeline"];
    file.close();
}

bool Camera::saveCalibCB(Trigger::Request& /*req*/, Trigger::Response& res) {
    saveCalib();
    res.success = true;
    return true;
}

bool Camera::savePipelineCB(Trigger::Request& /*req*/, Trigger::Response& res) {
    savePipeline();
    res.success = true;
    return true;
}

bool Camera::startCB(Trigger::Request& /*req*/, Trigger::Response& res) {
    start();
    res.success = true;
    return true;
}
bool Camera::stopCB(Trigger::Request& /*req*/, Trigger::Response& res) {
    stop();
    res.success = true;
    return true;
}

void Camera::parameterCB(parametersConfig& config, uint32_t /*level*/) {
    enableIR = config.camera_i_enable_ir;
    floodlightBrighness = config.camera_i_floodlight_brightness;
    laserDotBrightness = config.camera_i_laser_dot_brightness;
    setIR();
    if(!daiNodes.empty()) {
        for(const auto& node : daiNodes) {
            try {
                node->updateParams(config);
            } catch(std::runtime_error& e) {
                ROS_ERROR("%s", e.what());
            }
        }
    }
}

void Camera::onConfigure() {
    try {
        getDeviceType();
        createPipeline();
        device->startPipeline(*pipeline);
        // If model name not set get one from the device
        std::string camModel = ph->getParam<std::string>("i_tf_camera_model");
        if(camModel.empty()) {
            camModel = device->getDeviceName();
        }

        if(ph->getParam<bool>("i_publish_tf_from_calibration")) {
            tfPub = std::make_unique<dai::ros::TFPublisher>(pNH,
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
                                                            ph->getParam<std::string>("i_tf_custom_xacro_args"));
        }
        setupQueues();
        setIR();
    } catch(const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        throw std::runtime_error("Failed to start up the camera.");
    }
    ROS_INFO("Camera ready!");
}

void Camera::getDeviceType() {
    pipeline = std::make_shared<dai::Pipeline>();
    startDevice();
    auto name = device->getDeviceName();
    ROS_INFO("Device type: %s", name.c_str());
    for(auto& sensor : device->getCameraSensorNames()) {
        ROS_DEBUG("Socket %d - %s", static_cast<int>(sensor.first), sensor.second.c_str());
    }
    auto ir_drivers = device->getIrDrivers();
    if(ir_drivers.empty()) {
        ROS_WARN("Device has no IR drivers");
    } else {
        ROS_DEBUG("IR Drivers present");
    }
}

void Camera::createPipeline() {
    auto generator = std::make_unique<pipeline_gen::PipelineGenerator>();
    daiNodes = generator->createPipeline(pNH, device, pipeline, ph->getParam<std::string>("i_pipeline_type"), ph->getParam<std::string>("i_nn_type"));
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
    ros::Rate r(1.0);
    while(!camRunning) {
        try {
            auto mxid = ph->getParam<std::string>("i_mx_id");
            auto ip = ph->getParam<std::string>("i_ip");
            auto usb_id = ph->getParam<std::string>("i_usb_port_id");
            if(mxid.empty() && ip.empty() && usb_id.empty()) {
                ROS_INFO("No ip/mxid/usb_id specified, connecting to the next available device.");
                auto info = dai::Device::getAnyAvailableDevice();
                auto speed = ph->getUSBSpeed();
                device = std::make_shared<dai::Device>(std::get<1>(info), speed);
                camRunning = true;
            } else {
                std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
                if(availableDevices.size() == 0) {
                    if(!ip.empty()) {
                        dai::DeviceInfo info(ip);
                        ROS_INFO("No devices detected by autodiscovery, trying to connect to camera via IP: %s", ip.c_str());
                        availableDevices.push_back(info);
                    } else {
                        throw std::runtime_error("No devices detected!");
                    }
                }
                dai::UsbSpeed speed = ph->getUSBSpeed();

                for(const auto& info : availableDevices) {
                    if(!mxid.empty() && info.getMxId() == mxid) {
                        ROS_INFO("Connecting to the camera using mxid: %s", mxid.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else if(!ip.empty() && info.name == ip) {
                        ROS_INFO("Connecting to the camera using ip: %s", ip.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER || info.state == X_LINK_ANY_STATE) {
                            device = std::make_shared<dai::Device>(info);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else if(!usb_id.empty() && info.name == usb_id) {
                        ROS_INFO("Connecting to the camera using USB ID: %s", usb_id.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                            camRunning = true;
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device is already booted in different process.");
                        }
                    } else {
                        ROS_INFO("Ignoring device info: MXID: %s, Name: %s", info.getMxId().c_str(), info.name.c_str());
                    }
                }
            }
        } catch(const std::runtime_error& e) {
            ROS_ERROR("%s", e.what());
        }
        r.sleep();
    }

    ROS_INFO("Camera with MXID: %s and Name: %s connected!", device->getMxId().c_str(), device->getDeviceInfo().name.c_str());
    auto protocol = device->getDeviceInfo().getXLinkDeviceDesc().protocol;

    if(protocol != XLinkProtocol_t::X_LINK_TCP_IP) {
        auto speed = usbStrings[static_cast<int32_t>(device->getUsbSpeed())];
        ROS_INFO("USB SPEED: %s", speed.c_str());
    } else {
        ROS_INFO("PoE camera detected. Consider enabling low bandwidth for specific image topics (see readme).");
    }
}

void Camera::setIR() {
    if(camRunning && enableIR && !device->getIrDrivers().empty()) {
        float laserdotIntensity = float(laserDotBrightness);
        if(laserdotIntensity > 1.0) {
            laserdotIntensity = laserdotIntensity / 1200.0;
        }
        device->setIrLaserDotProjectorIntensity(laserdotIntensity);
        float floodlightIntensity = float(floodlightBrighness);
        if(floodlightIntensity > 1.0) {
            floodlightIntensity = floodlightIntensity / 1500.0;
        }
        device->setIrFloodLightIntensity(floodlightIntensity);
    }
}

}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::Camera, nodelet::Nodelet)
