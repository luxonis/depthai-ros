#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"

#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "depthai_ros_driver/utils.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
StereoParamHandler::StereoParamHandler(ros::NodeHandle node, const std::string& name) : BaseParamHandler(node, name) {
    depthPresetMap = {
        {"HIGH_ACCURACY", dai::node::StereoDepth::PresetMode::HIGH_ACCURACY},
        {"HIGH_DENSITY", dai::node::StereoDepth::PresetMode::HIGH_DENSITY},
    };
    disparityWidthMap = {
        {"DISPARITY_64", dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64},
        {"DISPARITY_96", dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_96},
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

void StereoParamHandler::updateSocketsFromParams(dai::CameraBoardSocket& left, dai::CameraBoardSocket& right) {
    int newLeftS = getParam<int>("i_left_socket_id", static_cast<int>(left));
    int newRightS = getParam<int>("i_right_socket_id", static_cast<int>(right));
    if(newLeftS != static_cast<int>(left) || newRightS != static_cast<int>(right)) {
        ROS_WARN("Left or right socket changed, updating stereo node");
        ROS_WARN("Old left socket: %d, new left socket: %d", static_cast<int>(left), newLeftS);
        ROS_WARN("Old right socket: %d, new right socket: %d", static_cast<int>(right), newRightS);
    }
    left = static_cast<dai::CameraBoardSocket>(newLeftS);
    right = static_cast<dai::CameraBoardSocket>(newRightS);
}

StereoParamHandler::~StereoParamHandler() = default;
void StereoParamHandler::declareParams(std::shared_ptr<dai::node::StereoDepth> stereo, const std::vector<dai::CameraFeatures>& camFeatures) {
    getParam<int>("i_max_q_size");
    stereo->setLeftRightCheck(getParam<bool>("i_lr_check"));
    auto socket = static_cast<dai::CameraBoardSocket>(declareAndLogParam<int>("i_board_socket_id", static_cast<int>(dai::CameraBoardSocket::CAM_A)));
    std::string socketName;
    int width = 1280;
    int height = 720;
    if(getParam<bool>("i_align_depth", true)) {
        socketName = utils::getSocketName(socket, camFeatures);
        width = getOtherNodeParam<int>(socketName, "i_width", width);
        height = getOtherNodeParam<int>(socketName ,"i_height", height);

        setParam<std::string>("i_socket_name", socketName);
        stereo->setDepthAlign(socket);
        setParam<int>("i_board_socket_id", static_cast<int>(socket));
    }
    setParam<int>("i_width", width);
    setParam<int>("i_height", height);
    stereo->setOutputSize(width, height);
    if(getParam<bool>("i_set_input_size")) {
        stereo->setInputResolution(getParam<int>("i_input_width"), getParam<int>("i_input_height"));
    }

    stereo->setDefaultProfilePreset(depthPresetMap.at(getParam<std::string>("i_depth_preset")));
    stereo->enableDistortionCorrection(getParam<bool>("i_enable_distortion_correction"));

    stereo->initialConfig.setBilateralFilterSigma(getParam<int>("i_bilateral_sigma"));
    stereo->initialConfig.setLeftRightCheckThreshold(getParam<int>("i_lrc_threshold"));
    stereo->initialConfig.setMedianFilter(static_cast<dai::MedianFilter>(getParam<int>("i_depth_filter_size")));
    stereo->initialConfig.setConfidenceThreshold(getParam<int>("i_stereo_conf_threshold"));
    if(getParam<bool>("i_subpixel", false)) {
        stereo->initialConfig.setSubpixel(true);
        stereo->initialConfig.setSubpixelFractionalBits(getParam<int>("i_subpixel_fractional_bits"));
    }
    stereo->setRectifyEdgeFillColor(getParam<int>("i_rectify_edge_fill_color"));
    if(declareAndLogParam<bool>("i_enable_alpha_scaling", false)) {
        stereo->setAlphaScaling(declareAndLogParam<float>("i_alpha_scaling", 0.0));
    }
    auto config = stereo->initialConfig.get();
    config.costMatching.disparityWidth = utils::getValFromMap(getParam<std::string>("i_disparity_width", "DISPARITY_96"), disparityWidthMap);
    stereo->setExtendedDisparity(getParam<bool>("i_extended_disp"));
    config.costMatching.enableCompanding = getParam<bool>("i_enable_companding", false);
    config.postProcessing.temporalFilter.enable = getParam<bool>("i_enable_temporal_filter");
    if(config.postProcessing.temporalFilter.enable) {
        config.postProcessing.temporalFilter.alpha = getParam<float>("i_temporal_filter_alpha");
        config.postProcessing.temporalFilter.delta = getParam<int>("i_temporal_filter_delta");
        config.postProcessing.temporalFilter.persistencyMode =
            utils::getValFromMap(getParam<std::string>("i_temporal_filter_persistency"), temporalPersistencyMap);
    }
    config.postProcessing.speckleFilter.enable = getParam<bool>("i_enable_speckle_filter");
    if(config.postProcessing.speckleFilter.enable) {
        config.postProcessing.speckleFilter.speckleRange = getParam<int>("i_speckle_filter_speckle_range");
    }
    if(getParam<bool>("i_enable_disparity_shift", false)) {
        config.algorithmControl.disparityShift = getParam<int>("i_disparity_shift", 0);
    }
    config.postProcessing.spatialFilter.enable = getParam<bool>("i_enable_spatial_filter");
    if(config.postProcessing.spatialFilter.enable) {
        config.postProcessing.spatialFilter.holeFillingRadius = getParam<int>("i_spatial_filter_hole_filling_radius");
        config.postProcessing.spatialFilter.alpha = getParam<float>("i_spatial_filter_alpha");
        config.postProcessing.spatialFilter.delta = getParam<int>("i_spatial_filter_delta");
        config.postProcessing.spatialFilter.numIterations = getParam<int>("i_spatial_filter_iterations");
    }
    if(getParam<bool>("i_enable_threshold_filter")) {
        config.postProcessing.thresholdFilter.minRange = getParam<int>("i_threshold_filter_min_range");
        config.postProcessing.thresholdFilter.maxRange = getParam<int>("i_threshold_filter_max_range");
    }
    if(getParam<bool>("i_enable_brightness_filter")) {
        config.postProcessing.brightnessFilter.minBrightness = getParam<int>("i_brightness_filter_min_brightness");
        config.postProcessing.brightnessFilter.maxBrightness = getParam<int>("i_brightness_filter_max_brightness");
    }
    if(getParam<bool>("i_enable_decimation_filter")) {
        config.postProcessing.decimationFilter.decimationMode =
            utils::getValFromMap(getParam<std::string>("i_decimation_filter_decimation_mode"), decimationModeMap);
        config.postProcessing.decimationFilter.decimationFactor = getParam<int>("i_decimation_filter_decimation_factor");
        int decimatedWidth = width / config.postProcessing.decimationFilter.decimationFactor;
        int decimatedHeight = height / config.postProcessing.decimationFilter.decimationFactor;
        ROS_INFO("Decimation filter enabled with decimation factor %d. Previous width and height: %d x %d, after decimation: %d x %d",
                 config.postProcessing.decimationFilter.decimationFactor,
                 width,
                 height,
                 decimatedWidth,
                 decimatedHeight);
        stereo->setOutputSize(decimatedWidth, decimatedHeight);
        setParam("i_width", decimatedWidth);
        setParam("i_height", decimatedHeight);
    }
    stereo->initialConfig.set(config);
}
dai::CameraControl StereoParamHandler::setRuntimeParams(parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver