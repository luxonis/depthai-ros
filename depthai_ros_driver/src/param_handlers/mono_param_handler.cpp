#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
MonoParamHandler::MonoParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {
    monoResolutionMap = {
        {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
        {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
        {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
        {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    };
}
MonoParamHandler::~MonoParamHandler() = default;
void MonoParamHandler::declareParams(std::shared_ptr<dai::node::MonoCamera> monoCam,
                                     dai::CameraBoardSocket socket,
                                     dai_nodes::sensor_helpers::ImageSensor,
                                     bool publish) {
    declareAndLogParam<int>("i_max_q_size", 30);
    declareAndLogParam<bool>("i_publish_topic", publish);
    declareAndLogParam<bool>("i_low_bandwidth", false);
    declareAndLogParam<int>("i_low_bandwidth_quality", 50);
    declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket));
    declareAndLogParam<std::string>("i_calibration_file", "");
    monoCam->setBoardSocket(socket);
    monoCam->setFps(declareAndLogParam<double>("i_fps", 30.0));

    monoCam->setResolution(monoResolutionMap.at(declareAndLogParam<std::string>("i_resolution", "720")));
    declareAndLogParam<int>("i_width", monoCam->getResolutionWidth());
    declareAndLogParam<int>("i_height", monoCam->getResolutionHeight());
    size_t iso = declareAndLogParam("r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam("r_exposure", 1000, getRangedIntDescriptor(1, 33000));

    if(declareAndLogParam("r_set_man_exposure", false)) {
        monoCam->initialControl.setManualExposure(exposure, iso);
    }
}
dai::CameraControl MonoParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    for(const auto& p : params) {
        if(p.get_name() == getFullParamName("r_set_man_exposure")) {
            if(p.get_value<bool>()) {
                ctrl.setManualExposure(getParam<int>("r_exposure"), getParam<int>("r_iso"));
            } else {
                ctrl.setAutoExposureEnable();
            }
        } else if(p.get_name() == getFullParamName("r_exposure")) {
            if(getParam<bool>("r_set_man_exposure")) {
                ctrl.setManualExposure(p.get_value<int>(), getParam<int>("r_iso"));
            }
        } else if(p.get_name() == getFullParamName("r_iso")) {
            if(getParam<bool>("r_set_man_exposure")) {
                ctrl.setManualExposure(getParam<int>("r_exposure"), p.get_value<int>());
            }
        }
    }
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver