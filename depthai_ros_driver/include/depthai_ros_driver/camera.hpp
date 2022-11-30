#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/types.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {

class Camera : public rclcpp::Node {
   public:
    explicit Camera(const rclcpp::NodeOptions& options);
    void onConfigure();

   private:
    void getDeviceType();
    void createPipeline();
    void loadNodes();
    void declareParams();
    void startDevice();
    void rgbPipeline();
    void setupQueues();
    rcl_interfaces::msg::SetParametersResult parameterCB(const std::vector<rclcpp::Parameter>& params);
    OnSetParametersCallbackHandle::SharedPtr paramCBHandle;
    std::unique_ptr<types::cam_types::CamType> cam_type;
    std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    std::unordered_map<std::string, dai::UsbSpeed> usbSpeedMap = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
    std::unordered_map<std::string, types::NNType> nnTypeMap = {
        {"", types::NNType::None},
        {"none", types::NNType::None},
        {"default", types::NNType::Default},
        {"spatial", types::NNType::Spatial},
    };
    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::Device> device;
    std::vector<std::unique_ptr<daiNodes::BaseNode>> daiNodes;
};
}  // namespace depthai_ros_driver