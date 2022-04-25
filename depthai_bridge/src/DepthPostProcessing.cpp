#include "depthai_bridge/DepthPostProcessing.hpp"

using TemporalMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode;
using DecimationMode = dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode;

DepthPostProcessing::DepthPostProcessing(ros_node node) {
    set_parameter("median_enable", _median_enable);
    set_parameter("median_mode", _median_mode);
    set_parameter("speckle_enable", _speckle_enable);
    set_parameter("speckle_range", _speckle_range);
    set_parameter("temporal_enable", _temporal_enable);
    set_parameter("temporal_mode", _temporal_mode);
    set_parameter("temporal_alpha", _temporal_alpha);
    set_parameter("temporal_delta", _temporal_delta);
    set_parameter("spatial_enable", _spatial_enable);
    set_parameter("spatial_radius", _spatial_radius);
    set_parameter("spatial_alpha", _spatial_alpha);
    set_parameter("spatial_delta", _spatial_delta);
    set_parameter("spatial_iterations", _spatial_iterations);
    set_parameter("threshold_enable", _threshold_enable);
    set_parameter("threshold_max", _threshold_max);
    set_parameter("threshold_min", _threshold_min);
    set_parameter("decimation_enable", _decimation_enable);
    set_parameter("decimation_mode", _decimation_mode);
    set_parameter("decimation_factor", _decimation_factor);
}

dai::MedianFilter DepthPostProcessing::getMedianFilter() {
    if(_median_mode == "MEDIAN_OFF") return dai::MedianFilter::MEDIAN_OFF;
    if(_median_mode == "KERNEL_3x3") return dai::MedianFilter::KERNEL_3x3;
    if(_median_mode == "KERNEL_5x5") return dai::MedianFilter::KERNEL_5x5;
    if(_median_mode == "KERNEL_7x7") return dai::MedianFilter::KERNEL_7x7;
    return dai::MedianFilter::MEDIAN_OFF;
}

TemporalMode DepthPostProcessing::getTemporalMode() {
    if(_temporal_mode == "PERSISTENCY_OFF") return TemporalMode::PERSISTENCY_OFF;
    if(_temporal_mode == "VALID_8_OUT_OF_8") return TemporalMode::VALID_8_OUT_OF_8;
    if(_temporal_mode == "VALID_2_IN_LAST_3") return TemporalMode::VALID_2_IN_LAST_3;
    if(_temporal_mode == "VALID_2_IN_LAST_4") return TemporalMode::VALID_2_IN_LAST_4;
    if(_temporal_mode == "VALID_2_OUT_OF_8") return TemporalMode::VALID_2_OUT_OF_8;
    if(_temporal_mode == "VALID_1_IN_LAST_2") return TemporalMode::VALID_1_IN_LAST_2;
    if(_temporal_mode == "VALID_1_IN_LAST_5") return TemporalMode::VALID_1_IN_LAST_5;
    if(_temporal_mode == "VALID_1_IN_LAST_8") return TemporalMode::VALID_1_IN_LAST_8;
    if(_temporal_mode == "PERSISTENCY_INDEFINITELY") return TemporalMode::PERSISTENCY_INDEFINITELY;
    return TemporalMode::PERSISTENCY_OFF;
}

DecimationMode DepthPostProcessing::getDecimationMode() {
    if(_decimation_mode == "PIXEL_SKIPPING") return DecimationMode::PIXEL_SKIPPING;
    if(_decimation_mode == "NON_ZERO_MEDIAN") return DecimationMode::NON_ZERO_MEDIAN;
    if(_decimation_mode == "NON_ZERO_MEAN") return DecimationMode::NON_ZERO_MEAN;
    return DecimationMode::PIXEL_SKIPPING;
}

dai::RawStereoDepthConfig DepthPostProcessing::getFilters(dai::RawStereoDepthConfig config) {
    if(_median_enable) config.postProcessing.median = getMedianFilter();
    if(_speckle_enable) {
        config.postProcessing.speckleFilter.enable = _speckle_enable;
        config.postProcessing.speckleFilter.speckleRange = static_cast<std::uint32_t>(_speckle_range);
    }
    if(_temporal_enable) {
        config.postProcessing.temporalFilter.enable = _temporal_enable;
        config.postProcessing.temporalFilter.alpha = _temporal_alpha;
        config.postProcessing.temporalFilter.delta = static_cast<std::int32_t>(_temporal_delta);
        config.postProcessing.temporalFilter.persistencyMode = getTemporalMode();
    }
    if(_spatial_enable) {
        config.postProcessing.spatialFilter.enable = _spatial_enable;
        config.postProcessing.spatialFilter.holeFillingRadius = static_cast<std::uint8_t>(_spatial_radius);
        config.postProcessing.spatialFilter.alpha = _spatial_alpha;
        config.postProcessing.spatialFilter.delta = static_cast<std::int32_t>(_spatial_delta);
        config.postProcessing.spatialFilter.numIterations = static_cast<std::int32_t>(_spatial_iterations);
    }
    if(_threshold_enable) {
        config.postProcessing.thresholdFilter.minRange = _threshold_min;
        config.postProcessing.thresholdFilter.maxRange = _threshold_max;
    }
    if(_decimation_enable) {
        config.postProcessing.decimationFilter.decimationFactor = static_cast<std::uint32_t>(_decimation_factor);
        config.postProcessing.decimationFilter.decimationMode = getDecimationMode();
    }
    setConfig(config);
    return config;
    // _stereo->initialConfig.set(config);
}

void DepthPostProcessing::setConfig(dai::RawStereoDepthConfig config) {
    _config = config;
}

void DepthPostProcessing::setFilters() {
    auto config_message = dai::StereoDepthConfig();
    config_message.set(getFilters(_config));
    auto stereo_config_queue = _device->getInputQueue("config_stereo");
    stereo_config_queue->send(config_message);
}

void DepthPostProcessing::setDevice(std::shared_ptr<dai::Device> device) {
    _device = device;
}

req_type DepthPostProcessing::setPostProcessingRequest(pp_req_msg request, pp_rep_msg response) {
    _median_enable = req_get(median_enable);
    _median_mode = req_get(median_mode);
    _speckle_enable = req_get(speckle_enable);
    _speckle_range = req_get(speckle_range);
    _temporal_enable = req_get(temporal_enable);
    _temporal_alpha = req_get(temporal_alpha);
    _temporal_delta = req_get(temporal_delta);
    _temporal_mode = req_get(temporal_mode);
    _spatial_enable = req_get(spatial_enable);
    _spatial_radius = req_get(spatial_radius);
    _spatial_alpha = req_get(spatial_alpha);
    _spatial_iterations = req_get(spatial_iterations);
    _threshold_enable = req_get(threshold_enable);
    _threshold_min = req_get(threshold_min);
    _threshold_max = req_get(threshold_max);
    _decimation_enable = req_get(decimation_enable);
    _decimation_factor = req_get(decimation_factor);
    _decimation_mode = req_get(decimation_mode);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type DepthPostProcessing::setMedianRequest(med_req_msg request, med_rep_msg response) {
    _median_enable = req_get(median_enable);
    _median_mode = req_get(median_mode);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type DepthPostProcessing::setSpeckleRequest(spk_req_msg request, spk_rep_msg response) {
    _speckle_enable = req_get(speckle_enable);
    _speckle_range = req_get(speckle_range);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type DepthPostProcessing::setTemporalRequest(tmp_req_msg request, tmp_rep_msg response) {
    _temporal_enable = req_get(temporal_enable);
    _temporal_alpha = req_get(temporal_alpha);
    _temporal_delta = req_get(temporal_delta);
    _temporal_mode = req_get(temporal_mode);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type DepthPostProcessing::setSpatialRequest(spt_req_msg request, spt_rep_msg response) {
    _spatial_enable = req_get(spatial_enable);
    _spatial_radius = req_get(spatial_radius);
    _spatial_alpha = req_get(spatial_alpha);
    _spatial_iterations = req_get(spatial_iterations);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type DepthPostProcessing::setThresholdRequest(trh_req_msg request, trh_rep_msg response) {
    _threshold_enable = req_get(threshold_enable);
    _threshold_min = req_get(threshold_min);
    _threshold_max = req_get(threshold_max);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type DepthPostProcessing::setDecimationRequest(dcm_req_msg request, dcm_rep_msg response) {
    _decimation_enable = req_get(decimation_enable);
    _decimation_factor = req_get(decimation_factor);
    _decimation_mode = req_get(decimation_mode);
    setFilters();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}