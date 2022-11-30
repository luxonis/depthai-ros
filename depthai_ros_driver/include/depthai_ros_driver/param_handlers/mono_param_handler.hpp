#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"


namespace depthai_ros_driver {
namespace paramHandlers {
class MonoParamHandler : public BaseParamHandler {
   public:
    explicit MonoParamHandler(const std::string& name);
    ~MonoParamHandler(){};
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MonoCamera> mono_cam);
    dai::CameraControl setRuntimeParams(rclcpp::Node* node,const std::vector<rclcpp::Parameter>& params) override;

   private:
    std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap = {
        {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
        {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
        {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
        {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    };
};
}  // namespace paramHandlers
}  // namespace depthai_ros_driver