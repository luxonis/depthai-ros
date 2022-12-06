#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
MonoParamHandler::MonoParamHandler(const std::string& name) : BaseParamHandler(name) {
    monoResolutionMap = {
        {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
        {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
        {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
        {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    };
};
MonoParamHandler::~MonoParamHandler() = default;
void MonoParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MonoCamera> mono_cam, dai::CameraBoardSocket socket, dai_nodes::sensor_helpers::ImageSensor) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);
    declareAndLogParam<bool>(node, "i_publish_topic", true);

    declareAndLogParam<int>(node, "i_board_socket_id", static_cast<int>(socket));
    mono_cam->setBoardSocket(socket);
    mono_cam->setFps(declareAndLogParam<double>(node, "i_fps", 30.0));

    mono_cam->setResolution(monoResolutionMap.at(declareAndLogParam<std::string>(node, "i_resolution", "720")));

    size_t iso = declareAndLogParam(node, "r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam(node, "r_exposure", 1000, getRangedIntDescriptor(1, 33000));

    if(declareAndLogParam(node, "r_set_man_exposure", false)) {
        mono_cam->initialControl.setManualExposure(exposure, iso);
    }
}
dai::CameraControl MonoParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    for(const auto& p : params) {
        if(p.get_name() == getFullParamName("r_set_man_exposure")) {
            if(p.get_value<bool>()) {
                ctrl.setManualExposure(getParam<int>(node, "r_exposure"), getParam<int>(node, "r_iso"));
            } else {
                ctrl.setAutoExposureEnable();
            }
        } else if(p.get_name() == getFullParamName("r_exposure")) {
            if(getParam<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(p.get_value<int>(), getParam<int>(node, "r_iso"));
            }
        } else if(p.get_name() == getFullParamName("r_iso")) {
            if(getParam<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(getParam<int>(node, "r_exposure"), p.get_value<int>());
            }
        }
        return ctrl;
    }
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver