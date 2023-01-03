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
void MonoParamHandler::declareParams(ros::NodeHandle node,
                                     std::shared_ptr<dai::node::MonoCamera> mono_cam,
                                     dai::CameraBoardSocket socket,
                                     dai_nodes::sensor_helpers::ImageSensor) {
    getParam<int>(node, "i_max_q_size");
    getParam<bool>(node, "i_publish_topic");

    mono_cam->setBoardSocket(socket);
    mono_cam->setFps(getParam<double>(node, "i_fps"));

    mono_cam->setResolution(monoResolutionMap.at(getParam<std::string>(node, "i_resolution")));
    getParam<int>(node, "i_width");
    getParam<int>(node, "i_height");
    size_t iso = getParam<int>(node, "r_iso");
    size_t exposure = getParam<int>(node, "r_exposure");

    if(getParam<bool>(node, "r_set_man_exposure")) {
        mono_cam->initialControl.setManualExposure(exposure, iso);
    }
}
dai::CameraControl MonoParamHandler::setRuntimeParams(ros::NodeHandle node, parametersConfig& config) {
    dai::CameraControl ctrl;
    if(getName() == "left") {
        if(config.left_r_set_man_exposure) {
            ctrl.setManualExposure(config.left_r_exposure, config.left_r_iso);
        } else {
            ctrl.setAutoExposureEnable();
        }
    } else if(getName() == "right") {
        if(config.right_r_set_man_exposure) {
            ctrl.setManualExposure(config.right_r_exposure, config.right_r_iso);
        } else {
            ctrl.setAutoExposureEnable();
        }
    }
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver