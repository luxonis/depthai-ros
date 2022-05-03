#include "depthai_bridge/CameraControl.hpp"

CameraControl::CameraControl() {
    _rgb.name = "rgb";
    _stereo.name = "stereo";
}

CameraControl::CameraControl(ros_node node) {
    CameraControl();
    set_parameter("auto_exposure_rgb", _rgb.auto_exposure);
    set_parameter("exposure_start_x_rgb", _rgb.region.at(0));
    set_parameter("exposure_start_y_rgb", _rgb.region.at(1));
    set_parameter("exposure_width_rgb", _rgb.region.at(2));
    set_parameter("exposure_height_rgb", _rgb.region.at(3));
    set_parameter("exposure_compensation_rgb", _rgb.compensation);
    set_parameter("exposure_time_us_rgb", _rgb.time_us);
    set_parameter("exposure_iso_rgb", _rgb.sensitivity_iso);
    set_parameter("auto_exposure_stereo", _stereo.auto_exposure);
    set_parameter("exposure_start_x_stereo", _stereo.region.at(0));
    set_parameter("exposure_start_y_stereo", _stereo.region.at(1));
    set_parameter("exposure_width_stereo", _stereo.region.at(2));
    set_parameter("exposure_height_stereo", _stereo.region.at(3));
    set_parameter("exposure_compensation_stereo", _stereo.compensation);
    set_parameter("exposure_time_us_stereo", _stereo.time_us);
    set_parameter("exposure_iso_stereo", _stereo.sensitivity_iso);
    set_parameter("focus_mode", _focus.mode);
    set_parameter("focus_region_x", _focus.region.at(0));
    set_parameter("focus_region_y", _focus.region.at(1));
    set_parameter("focus_region_width", _focus.region.at(2));
    set_parameter("focus_region_height", _focus.region.at(3));
}

void CameraControl::setRgbExposure(bool value) {
    _exposure_rgb = value;
}

void CameraControl::setExposure() {
    setExposure(_stereo);
    if(_exposure_rgb) setExposure(_rgb);
}

void CameraControl::setExposure(ExposureParameters exposure) {
    auto controlQueue = _device->getInputQueue("control_" + exposure.name);
    auto configQueue = _device->getInputQueue("config_" + exposure.name);
    dai::CameraControl ctrl;
    if(exposure.auto_exposure) {
        ctrl.setAutoExposureEnable();
    } else {
        if(exposure.region.at(2) != 0 || exposure.region.at(3) != 0) {
            dai::ImageManipConfig cfg;
            cfg.setCropRect(exposure.region.at(0), exposure.region.at(1), exposure.region.at(2), exposure.region.at(3));
            configQueue->send(cfg);
        } else {
            ctrl.setManualExposure(exposure.time_us, exposure.sensitivity_iso);
        }
    }
    ctrl.setAutoExposureCompensation(exposure.compensation);
    controlQueue->send(ctrl);
}

void CameraControl::setDevice(std::shared_ptr<dai::Device> device) {
    _device = device;
}

dai::CameraControl::AutoFocusMode CameraControl::getFocusMode() {
    if(_focus.mode == "AUTO") return dai::CameraControl::AutoFocusMode::AUTO;
    if(_focus.mode == "CONTINUOUS_PICTURE") return dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE;
    if(_focus.mode == "CONTINUOUS_VIDEO ") return dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO;
    if(_focus.mode == "EDOF") return dai::CameraControl::AutoFocusMode::EDOF;
    if(_focus.mode == "MACRO") return dai::CameraControl::AutoFocusMode::MACRO;
    if(_focus.mode == "OFF") return dai::CameraControl::AutoFocusMode::OFF;
    return dai::CameraControl::AutoFocusMode::AUTO;
}

int clamp(int v, int min, int max) {
    if(v < min) return min;
    if(v > max) return max;
    return v;
}

void CameraControl::setFocus() {
    dai::CameraControl ctrl;
    auto controlQueue = _device->getInputQueue("control");
    auto mode = getFocusMode();
    ctrl.setAutoFocusMode(mode);

    if(_focus.mode == "OFF") {
        auto configQueue = _device->getInputQueue("config");
        dai::ImageManipConfig cfg;
        cfg.setCropRect(_focus.region.at(0), _focus.region.at(1), 0, 0);
        configQueue->send(cfg);
    } else if(_focus.region.at(2) > 0 && _focus.region.at(3) > 0) {
        ctrl.setAutoFocusRegion(_focus.region.at(0), _focus.region.at(1), _focus.region.at(2), _focus.region.at(3));
    } else if(_focus.mode != "AUTO") {
        ctrl.setManualFocus(clamp(_lens_position, 0, 255));
    }

    controlQueue->send(ctrl);
}

req_type CameraControl::setExposureRequest(exp_req_msg request, exp_rep_msg response) {
    _rgb.auto_exposure = req_get(auto_exposure);
    _rgb.region.at(0) = req_get(exposure_x);
    _rgb.region.at(1) = req_get(exposure_y);
    _rgb.region.at(2) = req_get(exposure_width);
    _rgb.region.at(3) = req_get(exposure_height);
    _rgb.compensation = req_get(compensation);
    _rgb.time_us = clamp(req_get(exposure_time_us), 1, 33000);
    _rgb.sensitivity_iso = clamp(req_get(exposure_time_us), 1, 33000);
    setExposure(_rgb);
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}


req_type CameraControl::setFocusRequest(foc_req_msg request, foc_rep_msg response) {
    _focus.mode = req_get(focus_mode);
    _focus.region.at(0) = req_get(focus_x);
    _focus.region.at(1) = req_get(focus_y);
    _focus.region.at(2) = req_get(focus_width);
    _focus.region.at(3) = req_get(focus_height);
    setFocus();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

dai::CameraControl::AutoWhiteBalanceMode CameraControl::getWhiteBalanceMode() {
    if(_white_balance_mode == "OFF") return dai::CameraControl::AutoWhiteBalanceMode::OFF;
    if(_white_balance_mode == "AUTO") return dai::CameraControl::AutoWhiteBalanceMode::AUTO;
    if(_white_balance_mode == "INCANDESCENT") return dai::CameraControl::AutoWhiteBalanceMode::INCANDESCENT;
    if(_white_balance_mode == "FLUORESCENT") return dai::CameraControl::AutoWhiteBalanceMode::FLUORESCENT;
    if(_white_balance_mode == "WARM_FLUORESCENT") return dai::CameraControl::AutoWhiteBalanceMode::WARM_FLUORESCENT;
    if(_white_balance_mode == "DAYLIGHT") return dai::CameraControl::AutoWhiteBalanceMode::DAYLIGHT;
    if(_white_balance_mode == "CLOUDY_DAYLIGHT") return dai::CameraControl::AutoWhiteBalanceMode::CLOUDY_DAYLIGHT;
    if(_white_balance_mode == "TWILIGHT") return dai::CameraControl::AutoWhiteBalanceMode::TWILIGHT;
    if(_white_balance_mode == "SHADE") return dai::CameraControl::AutoWhiteBalanceMode::SHADE;
    return dai::CameraControl::AutoWhiteBalanceMode::AUTO;
}

void CameraControl::setWhiteBalance() {
    dai::CameraControl ctrl;
    auto controlQueue = _device->getInputQueue("control");
    auto mode = getWhiteBalanceMode();
    ctrl.setAutoWhiteBalanceMode(mode);
    if(mode == dai::CameraControl::AutoWhiteBalanceMode::OFF) ctrl.setManualWhiteBalance(clamp(_color_temperature_k, 1000, 12000));
    controlQueue->send(ctrl);
}

req_type CameraControl::setWhiteBalanceRequest(wb_req_msg request, wb_rep_msg response) {
    _white_balance_mode = req_get(white_balance_mode);
    _color_temperature_k = req_get(color_temperature_k);
    setWhiteBalance();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}
