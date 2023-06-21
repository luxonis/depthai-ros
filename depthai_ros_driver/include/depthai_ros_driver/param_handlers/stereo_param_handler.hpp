#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler : public BaseParamHandler {
   public:
    explicit StereoParamHandler(ros::NodeHandle node, const std::string& name);
    ~StereoParamHandler();
    void declareParams(std::shared_ptr<dai::node::StereoDepth> stereo, const std::string& rightName);
    dai::CameraControl setRuntimeParams(parametersConfig& config) override;

   private:
    std::unordered_map<std::string, dai::node::StereoDepth::PresetMode> depthPresetMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::CostMatching::DisparityWidth> disparityWidthMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode> decimationModeMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode> temporalPersistencyMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver