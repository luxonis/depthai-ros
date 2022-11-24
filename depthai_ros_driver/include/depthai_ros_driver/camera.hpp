#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/rgb.hpp"
#include "depthai_ros_driver/visibility.h"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
class Camera : public rclcpp::Node {
   public:
    explicit Camera(const rclcpp::NodeOptions& options);
    void on_configure();

   private:
    void createPipeline();
    void loadNodes();
    void declareParams();
    void startDevice();
    void rgbPipeline();
    void setupQueues();
    rcl_interfaces::msg::SetParametersResult parameterCB(const std::vector<rclcpp::Parameter>& params);
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    // enum class PipelineTypes{
    //     RGB
    //     RGBD
    //     RGBDNN
    //     RGBDC
    // };
    // std::unordered_map<PipelineTypes, std::function<void>()> pipeline_creation ={
    //     {RGBD, }
    // };
    std::unordered_map<std::string, dai::UsbSpeed> usb_speed_map_ = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
    std::shared_ptr<dai::Pipeline> pipeline_;
    std::shared_ptr<dai::Device> device_;
    std::unique_ptr<dai_nodes::BaseNode> rgb_;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> dai_nodes_;
};
}  // namespace depthai_ros_driver