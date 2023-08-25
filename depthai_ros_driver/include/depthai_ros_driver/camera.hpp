#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"
#include "rclcpp/node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace depthai_ros_driver {
using Trigger = std_srvs::srv::Trigger;
class Camera : public rclcpp::Node {
   public:
    explicit Camera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void onConfigure();

   private:
    void getDeviceType();
    void createPipeline();
    void loadNodes();
    void startDevice();
    void rgbPipeline();
    void setupQueues();
    void setIR();
    void savePipeline();
    void saveCalib();
    void loadCalib(const std::string& path);
    rcl_interfaces::msg::SetParametersResult parameterCB(const std::vector<rclcpp::Parameter>& params);
    OnSetParametersCallbackHandle::SharedPtr paramCBHandle;
    std::unique_ptr<param_handlers::CameraParamHandler> ph;
    rclcpp::Service<Trigger>::SharedPtr startSrv, stopSrv, savePipelineSrv, saveCalibSrv;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;
    void stop();
    void start();
    void restart();
    void diagCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    void startCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    void stopCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    void saveCalibCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    void savePipelineCB(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res);
    std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::Device> device;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    bool camRunning = false;

};
}  // namespace depthai_ros_driver