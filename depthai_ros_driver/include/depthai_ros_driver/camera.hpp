#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "dynamic_reconfigure/server.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

namespace depthai_ros_driver {
using Trigger = std_srvs::Trigger;
class Camera : public nodelet::Nodelet {
   public:
    void onInit() override;
    void onConfigure();

   private:
    void getDeviceType();
    void createPipeline();
    void loadNodes();
    void startDevice();
    void rgbPipeline();
    void setupQueues();
    void setIR();
    void parameterCB(parametersConfig& config, uint32_t level);
    std::shared_ptr<dynamic_reconfigure::Server<parametersConfig>> paramServer;
    std::unique_ptr<param_handlers::CameraParamHandler> ph;
    ros::ServiceServer startSrv, stopSrv;
    bool startCB(Trigger::Request& /*req*/, Trigger::Response& res);
    bool stopCB(Trigger::Request& /*req*/, Trigger::Response& res);

    std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::Device> device;
    ros::NodeHandle pNH;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    bool camRunning = false;
    bool enableIR = false;
    double floodlightBrighness;
    double laserDotBrightness;
};
}  // namespace depthai_ros_driver