
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class Sync;
}
}  // namespace dai

namespace ros {
class NodeHandle;
class Parameter;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class SyncParamHandler : public BaseParamHandler {
   public:
    explicit SyncParamHandler(ros::NodeHandle node, const std::string& name);
    ~SyncParamHandler();
    void declareParams(std::shared_ptr<dai::node::Sync> sync);
    dai::CameraControl setRuntimeParams(parametersConfig& config) override;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
