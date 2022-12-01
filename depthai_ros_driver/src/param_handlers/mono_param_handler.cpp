#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
MonoParamHandler::MonoParamHandler(const std::string& name) : BaseParamHandler(name){};
void MonoParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MonoCamera> mono_cam) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);

    declareAndLogParam<int>(node, "i_board_socket_id", 1);
    mono_cam->setFps(declareAndLogParam<double>(node, "i_fps", 30.0));

    mono_cam->setResolution(monoResolutionMap.at(declareAndLogParam<std::string>(node, "i_resolution", "720")));

    size_t iso = declareAndLogParam(node, "r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam(node, "r_exposure", 1000, getRangedIntDescriptor(1, 33000));

    if(declareAndLogParam(node, "r_set_man_exposure", false)) {
        mono_cam->initialControl.setManualExposure(exposure, iso);
    }

}
dai::CameraControl MonoParamHandler::setRuntimeParams(rclcpp::Node* node,const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    for(const auto& p : params) {
        if(p.get_name() == get_full_paramName("r_set_man_exposure")) {
            if(p.get_value<bool>()) {
                ctrl.setManualExposure(get_param<int>(node, "r_exposure"), get_param<int>(node, "r_iso"));
            } else {
                ctrl.setAutoExposureEnable();
            }
        } else if(p.get_name() == get_full_paramName("r_exposure")) {
            if(get_param<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(p.get_value<int>(), get_param<int>(node, "r_iso"));
            }
        } else if(p.get_name() == get_full_paramName("r_iso")) {
            if(get_param<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(get_param<int>(node, "r_exposure"), p.get_value<int>());
            }
    }
    return ctrl;
    }
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver