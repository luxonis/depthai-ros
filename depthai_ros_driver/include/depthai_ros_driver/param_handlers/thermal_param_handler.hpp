#pragma once

#include <memory>
#include <string>

#include "depthai/pipeline/datatype/ThermalConfig.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class Thermal;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class ThermalParamHandler : public BaseParamHandler {
   public:
    explicit ThermalParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat);
    ~ThermalParamHandler();
    void declareParams(std::shared_ptr<dai::node::Thermal> thermal);
    std::shared_ptr<dai::ThermalConfig> setRuntimeParams(const std::vector<rclcpp::Parameter>& params);

   private:
    std::unordered_map<std::string, dai::ThermalConfig::ThermalImageOrientation> thermalOrientMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
