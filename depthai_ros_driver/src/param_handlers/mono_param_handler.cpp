#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"


#include "depthai/pipeline/nodes.hpp"
#include "depthai/depthai.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
MonoParamHandler::MonoParamHandler(const std::string& dai_node_name) : BaseParamHandler(dai_node_name){};
void MonoParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MonoCamera> mono_cam) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);

    declareAndLogParam<int>(node, "i_board_socket_id", 1);
    mono_cam->setFps(declareAndLogParam<double>(node, "i_fps", 30.0));

    mono_cam->setResolution(mono_resolution_map_.at(declareAndLogParam<std::string>(node, "i_resolution", "720")));

    size_t iso = declareAndLogParam(node, "r_iso", 1000, get_ranged_int_descriptor(1000, 12000));
    size_t exposure = declareAndLogParam(node, "r_exposure", 1000, get_ranged_int_descriptor(10, 30000));
    size_t whitebalance = declareAndLogParam(node, "r_whitebalance", 3300, get_ranged_int_descriptor(3300, 6000));
    size_t focus = declareAndLogParam(node, "r_focus", 1, get_ranged_int_descriptor(0, 255));
    if(declareAndLogParam(node, "r_set_man_focus", false)) {
        mono_cam->initialControl.setManualFocus(focus);
    }
    if(declareAndLogParam(node, "r_set_man_exposure", false)) {
        mono_cam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam(node, "r_set_man_whitebalance", false)) {
        mono_cam->initialControl.setManualWhiteBalance(whitebalance);
    }
} 
dai::CameraControl MonoParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params){
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver