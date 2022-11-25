#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBParamHandler::RGBParamHandler(const std::string& name) : BaseParamHandler(name){};
void RGBParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::ColorCamera> color_cam) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);
    declareAndLogParam<bool>(node, "i_enable_preview", true);
    declareAndLogParam<int>(node, "i_board_socket_id", 0);
    color_cam->setFps(declareAndLogParam<double>(node, "i_fps", 30.0));
    size_t preview_size = declareAndLogParam<int>(node, "i_preview_size", 256);
    color_cam->setPreviewSize(preview_size, preview_size);
    color_cam->setVideoSize(declareAndLogParam<int>(node, "i_rgb_width", 1920), declareAndLogParam<int>(node, "i_rgb_height", 1080));
    color_cam->setResolution(rgb_resolution_map_.at(declareAndLogParam<std::string>(node, "i_resolution", "1080")));
    color_cam->setInterleaved(declareAndLogParam<bool>(node, "i_interleaved", false));
    if(declareAndLogParam<bool>(node, "i_set_isp", false)) {
        color_cam->setIspScale(2, 3);
    }
    color_cam->setPreviewKeepAspectRatio(declareAndLogParam(node, "i_keep_preview_aspect_ratio", true));
    size_t iso = declareAndLogParam(node, "r_iso", 800, get_ranged_int_descriptor(100, 1600));
    size_t exposure = declareAndLogParam(node, "r_exposure", 20000, get_ranged_int_descriptor(1, 33000));
    size_t whitebalance = declareAndLogParam(node, "r_whitebalance", 3300, get_ranged_int_descriptor(1000, 12000));
    size_t focus = declareAndLogParam(node, "r_focus", 1, get_ranged_int_descriptor(0, 255));
    if(declareAndLogParam(node, "r_set_man_focus", false)) {
        color_cam->initialControl.setManualFocus(focus);
    }
    if(declareAndLogParam(node, "r_set_man_exposure", false)) {
        color_cam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam(node, "r_set_man_whitebalance", false)) {
        color_cam->initialControl.setManualWhiteBalance(whitebalance);
    }
}
dai::CameraControl RGBParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    for(const auto& p : params) {
        if(p.get_name() == get_full_param_name("r_set_man_exposure")) {
            if(p.get_value<bool>()) {
                ctrl.setManualExposure(get_param<int>(node, "r_exposure"), get_param<int>(node, "r_iso"));
            } else {
                ctrl.setAutoExposureEnable();
            }
        } else if(p.get_name() == get_full_param_name("r_exposure")) {
            if(get_param<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(p.get_value<int>(), get_param<int>(node, "r_iso"));
            }
        } else if(p.get_name() == get_full_param_name("r_iso")) {
            if(get_param<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(get_param<int>(node, "r_exposure"), p.get_value<int>());
            }
        } else if(p.get_name() == get_full_param_name("r_set_man_focus")) {
            if(p.get_value<bool>()) {
                ctrl.setManualFocus(get_param<int>(node,"r_focus"));
            } else {
                ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
            }
        } else if(p.get_name() == get_full_param_name("r_focus")) {
            if(get_param<bool>(node, "r_set_man_focus")) {
                ctrl.setManualFocus(p.get_value<int>());
            }
        } else if(p.get_name() == get_full_param_name("r_set_man_whitebalance")) {
            if(p.get_value<bool>()){
                ctrl.setManualWhiteBalance(get_param<int>(node, "r_whitebalance"));
            }
            else{
                ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
            }
        } else if(p.get_name() == get_full_param_name("r_whitebalance")) {
            if(get_param<bool>(node, "r_set_man_whitebalance")) {
                ctrl.setManualWhiteBalance(p.get_value<int>());
            }
        }

    return ctrl;
    }
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver