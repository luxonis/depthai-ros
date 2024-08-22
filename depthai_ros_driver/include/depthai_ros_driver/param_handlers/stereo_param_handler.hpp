#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
class CameraFeatures;
}

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler : public BaseParamHandler {
   public:
    explicit StereoParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name);
    ~StereoParamHandler();
    void declareParams(std::shared_ptr<dai::node::StereoDepth> stereo);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
    void updateSocketsFromParams(dai::CameraBoardSocket& left, dai::CameraBoardSocket& right, dai::CameraBoardSocket& align);

   private:
    std::unordered_map<std::string, dai::node::StereoDepth::PresetMode> depthPresetMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::CostMatching::DisparityWidth> disparityWidthMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode> decimationModeMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode> temporalPersistencyMap;
    dai::CameraBoardSocket alignSocket;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
