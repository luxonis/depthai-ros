#include "depthai_ros_driver/camera.hpp"

#include <memory>

#include "depthai_ros_driver/pipeline_generator.hpp"
#include "dynamic_reconfigure/server.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depthai_ros_driver {

void Camera::onInit() {
    pNH = getPrivateNodeHandle();
    paramServer = std::make_shared<dynamic_reconfigure::Server<parametersConfig>>(pNH);
    paramServer->setCallback(std::bind(&Camera::parameterCB, this, std::placeholders::_1, std::placeholders::_2));
    ph = std::make_unique<param_handlers::CameraParamHandler>("camera");
    ph->declareParams(pNH);
    onConfigure();
    startSrv = pNH.advertiseService("start_camera", &Camera::startCB, this);
    stopSrv = pNH.advertiseService("stop_camera", &Camera::stopCB, this);
}

void Camera::parameterCB(parametersConfig& config, uint32_t /*level*/) {
    enableIR = config.camera_i_enable_ir;
    floodlightBrighness = config.camera_i_floodlight_brightness;
    laserDotBrightness = config.camera_i_laser_dot_brightness;
    if(camRunning && enableIR && !device->getIrDrivers().empty()) {
        device->setIrFloodLightBrightness(floodlightBrighness);
        device->setIrLaserDotProjectorBrightness(laserDotBrightness);
    }
    if(!daiNodes.empty()) {
        for(const auto& node : daiNodes) {
            node->updateParams(config);
        }
    }
}

void Camera::onConfigure() {
    getDeviceType();
    createPipeline();
    device->startPipeline(*pipeline);
    setupQueues();
    setIR();
    ROS_INFO("Camera ready!");
}

bool Camera::startCB(Trigger::Request& /*req*/, Trigger::Response& res) {
    ROS_INFO("Starting camera!");
    onConfigure();
    res.success = true;
    return true;
}
bool Camera::stopCB(Trigger::Request& /*req*/, Trigger::Response& res) {
    ROS_INFO("Stopping camera!");
    for(const auto& node : daiNodes) {
        node->closeQueues();
    }
    daiNodes.clear();
    device.reset();
    pipeline.reset();
    camRunning = false;
    res.success = true;
    return true;
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
    daiNodes = generator->createPipeline(pNH, device, pipeline, ph->getParam<std::string>(pNH, "i_pipeline_type"), ph->getParam<std::string>(pNH, "i_nn_type"));
}

void Camera::setupQueues() {
    for(const auto& node : daiNodes) {
        node->setupQueues(device);
    }
}

void Camera::startDevice() {
    dai::DeviceInfo info;
    auto mxid = ph->getParam<std::string>(pNH, "i_mx_id");
    auto ip = ph->getParam<std::string>(pNH, "i_ip");
    ros::Rate r(1.0);

    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
    while(!camRunning) {
        try {
            dai::UsbSpeed speed = ph->getUSBSpeed(pNH);
            if(mxid.empty() && ip.empty()) {
                ROS_INFO("No ip/mxid specified, connecting to the next available device.");
                device = std::make_shared<dai::Device>();
            } else {
                for(const auto& info : availableDevices) {
                    if(!mxid.empty() && info.getMxId() == mxid) {
                        ROS_INFO("Connecting to the camera using mxid: %s", mxid.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info, speed);
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device with mxid %s is already booted in different process.");
                        }
                    } else if(!ip.empty() && info.name == ip) {
                        ROS_INFO("Connecting to the camera using ip: %s", ip.c_str());
                        if(info.state == X_LINK_UNBOOTED || info.state == X_LINK_BOOTLOADER) {
                            device = std::make_shared<dai::Device>(info);
                        } else if(info.state == X_LINK_BOOTED) {
                            throw std::runtime_error("Device with ip %s is already booted in different process.");
                        }
                    }
                }
            }
            camRunning = true;
        } catch(const std::runtime_error& e) {
            ROS_ERROR("%s", e.what());
        }
        r.sleep();
    }

    auto devicename = device->getMxId();
    ROS_INFO("Camera %s connected!", devicename.c_str());
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
        device->setIrFloodLightBrightness(floodlightBrighness);
        device->setIrLaserDotProjectorBrightness(laserDotBrightness);
    }
}

}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::Camera, nodelet::Nodelet)
