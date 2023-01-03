#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler : public BaseParamHandler {
   public:
    explicit StereoParamHandler(const std::string& name);
    ~StereoParamHandler();
    void declareParams(ros::NodeHandle node, std::shared_ptr<dai::node::StereoDepth> stereo);
    dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig& config) override;

   private:
    std::unordered_map<std::string, dai::node::StereoDepth::PresetMode> depthPresetMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode> decimationModeMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode> temporalPersistencyMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver