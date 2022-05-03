#include "depthai_bridge/CameraControl.hpp"

// CameraControl::CameraControl() {
//     .name = "rgb";
//     _stereo.name = "stereo";
// }
namespace dai {

namespace ros {

CameraControl::CameraControl(ros_node node, std::shared_ptr<dai::Device> device, std::string camName) : _device(device), _camName(camName) {
    set_parameter(camName + "/auto_exposure", _exposure.auto_exposure);
    set_parameter(camName + "/exposure_roi_start_x", _exposure.region.at(0));
    set_parameter(camName + "/exposure_roi_start_y", _exposure.region.at(1));
    set_parameter(camName + "/exposure_roi_width", _exposure.region.at(2));
    set_parameter(camName + "/exposure_roi_height", _exposure.region.at(3));
    set_parameter(camName + "/exposure_compensation", _exposure.compensation);
    set_parameter(camName + "/exposure_time_us", _exposure.time_us);
    set_parameter(camName + "/exposure_iso", _exposure.sensitivity_iso);

    set_parameter(camName + "/focus_mode", _focus.mode);
    set_parameter(camName + "/focus_region_x", _focus.region.at(0));
    set_parameter(camName + "/focus_region_y", _focus.region.at(1));
    set_parameter(camName + "/focus_region_width", _focus.region.at(2));
    set_parameter(camName + "/focus_region_height", _focus.region.at(3));
}

void CameraControl::setExposure() {
    setExposure(_exposure);
}

void CameraControl::setExposure(ExposureParameters exposure) {
    auto controlQueue = _device->getInputQueue("control_" + _camName);
    dai::CameraControl ctrl;
    if(exposure.auto_exposure) {
        ctrl.setAutoExposureEnable();
    } else {
        if(exposure.region.at(2) != 0 || exposure.region.at(3) != 0) {
            ctrl.setAutoExposureRegion(exposure.region.at(0), exposure.region.at(1), exposure.region.at(2), exposure.region.at(3));
        } else {
            ctrl.setManualExposure(exposure.time_us, exposure.sensitivity_iso);
        }
    }
    ctrl.setAutoExposureCompensation(exposure.compensation);
    controlQueue->send(ctrl);
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
    auto controlQueue = _device->getInputQueue("control_" + _camName);
    auto mode = getFocusMode();
    ctrl.setAutoFocusMode(mode);

    if(_focus.mode == "OFF") {
        return;
    } else if(_focus.region.at(2) > 0 && _focus.region.at(3) > 0) {
        ctrl.setAutoFocusRegion(_focus.region.at(0), _focus.region.at(1), _focus.region.at(2), _focus.region.at(3));
    } else if(_focus.mode != "AUTO") {
        ctrl.setManualFocus(clamp(_lens_position, 0, 255));
    }

    controlQueue->send(ctrl);
}

req_type CameraControl::setExposureRequest(exp_req_msg request, exp_rep_msg response) {
    _exposure.auto_exposure = req_get(auto_exposure);
    _exposure.region.at(0) = req_get(exposure_x);
    _exposure.region.at(1) = req_get(exposure_y);
    _exposure.region.at(2) = req_get(exposure_width);
    _exposure.region.at(3) = req_get(exposure_height);
    _exposure.compensation = req_get(compensation);
    _exposure.time_us = clamp(req_get(exposure_time_us), 1, 33000);
    _exposure.sensitivity_iso = clamp(req_get(exposure_time_us), 1, 33000);
    setExposure(_exposure);
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

}  // namespace ros
}  // namespace dai