#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
StereoParamHandler::StereoParamHandler(const std::string& name) : BaseParamHandler(name) {
    depthPresetMap = {
        {"HIGH_DENSITY", dai::node::StereoDepth::PresetMode::HIGH_DENSITY},
        {"HIGH_ACCURACY", dai::node::StereoDepth::PresetMode::HIGH_ACCURACY},

    };
};
StereoParamHandler::~StereoParamHandler() = default;
void StereoParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::StereoDepth> stereo) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);
    stereo->setLeftRightCheck(declareAndLogParam<bool>(node, "i_lr_check", true));
    if(declareAndLogParam<bool>(node, "i_align_depth", true)) {
        declareAndLogParam<int>(node, "i_board_socket_id", 0);
        stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
        declareAndLogParam<int>(node, "i_width", 1920);
        declareAndLogParam<int>(node, "i_height", 1080);
    } else {
        declareAndLogParam<int>(node, "i_board_socket_id", 2);
        stereo->setDepthAlign(dai::CameraBoardSocket::RIGHT);
    }
    stereo->setDefaultProfilePreset(depthPresetMap.at(declareAndLogParam<std::string>(node, "i_depth_preset", "HIGH_DENSITY")));
    stereo->initialConfig.setLeftRightCheckThreshold(declareAndLogParam<int>(node, "i_lrc_threshold", 5));
    stereo->initialConfig.setMedianFilter(static_cast<dai::MedianFilter>(declareAndLogParam<int>(node, "i_depth_filter_size", 7)));
    stereo->initialConfig.setConfidenceThreshold(declareAndLogParam<int>(node, "i_stereo_conf_threshold", 255));
    // stereo->initialConfig.setSubpixel(declareAndLogParam<bool>(node, "i_subpixel", true));
    stereo->setExtendedDisparity(declareAndLogParam<bool>(node, "i_extender_disp", false));
    stereo->setRectifyEdgeFillColor(declareAndLogParam<int>(node, "i_rectify_edge_fill_color", -1));
    auto config = stereo->initialConfig.get();
    config.postProcessing.speckleFilter.enable = declareAndLogParam<bool>(node, "i_enable_speckle_filter", false);
    config.postProcessing.speckleFilter.speckleRange = declareAndLogParam<int>(node, "i_speckle_range", 50);
    config.postProcessing.spatialFilter.enable = declareAndLogParam<bool>(node, "i_enable_temporal_filter", true);
    config.postProcessing.temporalFilter.enable = declareAndLogParam<bool>(node, "i_enable_spatial_filter", true);
    config.postProcessing.spatialFilter.holeFillingRadius = declareAndLogParam<int>(node, "i_hole_filling_radius", 2);
    config.postProcessing.spatialFilter.numIterations = declareAndLogParam<int>(node, "i_spatal_filter_iterations", 1);
    config.postProcessing.thresholdFilter.minRange = declareAndLogParam<int>(node, "i_threshold_filter_min_range", 400);
    config.postProcessing.thresholdFilter.maxRange = declareAndLogParam<int>(node, "i_threshold_filter_max_range", 15000);
    config.postProcessing.decimationFilter.decimationFactor = declareAndLogParam<int>(node, "i_decimation_factor", 1);
    stereo->initialConfig.set(config);
}
dai::CameraControl StereoParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver