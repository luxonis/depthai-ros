#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
StereoParamHandler::StereoParamHandler(const std::string& name) : BaseParamHandler(name) {
    depthPresetMap = {
        {"HIGH_ACCURACY", dai::node::StereoDepth::PresetMode::HIGH_ACCURACY},
        {"HIGH_DENSITY", dai::node::StereoDepth::PresetMode::HIGH_DENSITY},
    };
    decimationModeMap = {{"PIXEL_SKIPPING", dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING},
                         {"NON_ZERO_MEDIAN", dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEDIAN},
                         {"NON_ZERO_MEAN", dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEAN}};

    temporalPersistencyMap = {
        {"PERSISTENCY_OFF", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::PERSISTENCY_OFF},
        {"VALID_8_OUT_OF_8", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8},
        {"VALID_2_IN_LAST_3", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_3},
        {"VALID_2_IN_LAST_4", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4},
        {"VALID_2_OUT_OF_8", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_OUT_OF_8},
        {"VALID_1_IN_LAST_2", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_2},
        {"VALID_1_IN_LAST_5", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_5},
        {"VALID_1_IN_LAST_8", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_8},
        {"PERSISTENCY_INDEFINITELY", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::PERSISTENCY_INDEFINITELY},

    };
}

StereoParamHandler::~StereoParamHandler() = default;
void StereoParamHandler::declareParams(ros::NodeHandle node, std::shared_ptr<dai::node::StereoDepth> stereo) {
    getParam<int>(node, "i_max_q_size");
    stereo->setLeftRightCheck(getParam<bool>(node, "i_lr_check"));
    if(getParam<bool>(node, "i_align_depth")) {
        stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    } else {
        stereo->setDepthAlign(dai::CameraBoardSocket::RIGHT);
    }
    stereo->setDefaultProfilePreset(depthPresetMap.at(getParam<std::string>(node, "i_depth_preset")));
    stereo->enableDistortionCorrection(getParam<bool>(node, "i_enable_distortion_correction"));

    stereo->initialConfig.setBilateralFilterSigma(getParam<int>(node, "i_bilateral_sigma"));
    stereo->initialConfig.setLeftRightCheckThreshold(getParam<int>(node, "i_lrc_threshold"));
    stereo->initialConfig.setMedianFilter(static_cast<dai::MedianFilter>(getParam<int>(node, "i_depth_filter_size")));
    stereo->initialConfig.setConfidenceThreshold(getParam<int>(node, "i_stereo_conf_threshold"));
    stereo->initialConfig.setSubpixel(getParam<bool>(node, "i_subpixel"));
    stereo->setRectifyEdgeFillColor(getParam<int>(node, "i_rectify_edge_fill_color"));
    auto config = stereo->initialConfig.get();
    config.postProcessing.temporalFilter.enable = getParam<bool>(node, "i_enable_temporal_filter");
    if(config.postProcessing.temporalFilter.enable) {
        config.postProcessing.temporalFilter.alpha = getParam<float>(node, "i_temporal_filter_alpha");
        config.postProcessing.temporalFilter.delta = getParam<int>(node, "i_temporal_filter_delta");
        config.postProcessing.temporalFilter.persistencyMode = temporalPersistencyMap.at(getParam<std::string>(node, "i_temporal_filter_persistency"));
    }
    if(config.postProcessing.speckleFilter.enable) {
        config.postProcessing.speckleFilter.enable = getParam<bool>(node, "i_enable_speckle_filter");
        config.postProcessing.speckleFilter.speckleRange = getParam<int>(node, "i_speckle_filter_speckle_range");
    }
    config.postProcessing.spatialFilter.enable = getParam<bool>(node, "i_enable_spatial_filter");
    if(config.postProcessing.spatialFilter.enable) {
        config.postProcessing.spatialFilter.holeFillingRadius = getParam<int>(node, "i_spatial_filter_hole_filling_radius");
        config.postProcessing.spatialFilter.alpha = getParam<float>(node, "i_spatial_filter_alpha");
        config.postProcessing.spatialFilter.delta = getParam<int>(node, "i_spatial_filter_delta");
        config.postProcessing.spatialFilter.numIterations = getParam<int>(node, "i_spatial_filter_iterations");
    }
    if(getParam<bool>(node, "i_enable_threshold_filter")) {
        config.postProcessing.thresholdFilter.minRange = getParam<int>(node, "i_threshold_filter_min_range");
        config.postProcessing.thresholdFilter.maxRange = getParam<int>(node, "i_threshold_filter_max_range");
    }
    if(getParam<bool>(node, "i_enable_decimation_filter")) {
        config.postProcessing.decimationFilter.decimationMode = decimationModeMap.at(getParam<std::string>(node, "i_decimation_filter_decimation_mode"));
        config.postProcessing.decimationFilter.decimationFactor = getParam<int>(node, "i_decimation_filter_decimation_factor");
    }
    stereo->initialConfig.set(config);
}
dai::CameraControl StereoParamHandler::setRuntimeParams(ros::NodeHandle /*node*/, parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver