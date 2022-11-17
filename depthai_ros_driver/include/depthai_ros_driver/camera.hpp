#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
class Camera : public rclcpp::Node {
   public:
    explicit Camera(const std::string& name, const rclcpp::NodeOptions& options);
    void on_configure();

   private:
    void createPipeline();
    void loadNodes();
    void declareParams();
    void startDevice();
    pluginlib::ClassLoader<dai_nodes::BaseNode> node_loader_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::unordered_map<std::string, dai::UsbSpeed> usb_speed_map_ = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
    std::shared_ptr<dai::Pipeline> pipeline_;
    std::shared_ptr<dai::Device> device_;
    std::vector<std::shared_ptr<dai_nodes::BaseNode>> dai_node_vec_;
}
}  // namespace depthai_ros_driver