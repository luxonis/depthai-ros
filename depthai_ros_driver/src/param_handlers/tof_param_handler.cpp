#include "depthai_ros_driver/param_handlers/tof_param_handler.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ToFParamHandler::ToFParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {

    medianFilterMap = {{"MEDIAN_OFF", dai::MedianFilter::MEDIAN_OFF},
                       {"KERNEL_3x3", dai::MedianFilter::KERNEL_3x3},
                       {"KERNEL_5x5", dai::MedianFilter::KERNEL_5x5},
                       {"KERNEL_7x7", dai::MedianFilter::KERNEL_7x7}};
}
ToFParamHandler::~ToFParamHandler() = default;
void ToFParamHandler::declareParams(std::shared_ptr<dai::node::Camera> cam, std::shared_ptr<dai::node::ToF> tof) {
    declareAndLogParam<bool>("i_publish_topic", true);
    int socket = declareAndLogParam<int>("i_board_socket_id", static_cast<int>(dai::CameraBoardSocket::CAM_A));
    cam->setBoardSocket(static_cast<dai::CameraBoardSocket>(socket));
    cam->setSize(declareAndLogParam<int>("i_width", 640), declareAndLogParam<int>("i_height", 480));
	cam->setFps(declareAndLogParam<int>("i_fps", 30));
    auto tofConf = tof->initialConfig.get();
    if(declareAndLogParam<bool>("i_enable_optical_correction", false)) {
        tofConf.enableOpticalCorrection = true;
    }
    if(declareAndLogParam<bool>("i_enable_fppn_correction", false)) {
        tofConf.enableFPPNCorrection = true;
    }
    if(declareAndLogParam<bool>("i_enable_temperature_correction", false)) {
        tofConf.enableTemperatureCorrection = true;
    }
    if(declareAndLogParam<bool>("i_enable_wiggle_correction", false)) {
        tofConf.enableWiggleCorrection = true;
    }
    if(declareAndLogParam<bool>("i_enable_phase_unwrapping", false)) {
        tofConf.enablePhaseUnwrapping = true;
    }

    tofConf.enablePhaseShuffleTemporalFilter = declareAndLogParam<bool>("i_enable_phase_shuffle_temporal_filter", true);
    tofConf.phaseUnwrappingLevel = declareAndLogParam<int>("i_phase_unwrapping_level", 4);
    tofConf.phaseUnwrapErrorThreshold = declareAndLogParam<int>("i_phase_unwrap_error_threshold", 100);
    std::vector<dai::MedianFilter> medianSettings = { dai::MedianFilter::MEDIAN_OFF, dai::MedianFilter::KERNEL_3x3, dai::MedianFilter::KERNEL_5x5, dai::MedianFilter::KERNEL_7x7 };
    tofConf.median = utils::getValFromMap(declareAndLogParam<std::string>("i_median_filter", "MEDIAN_OFF"), medianFilterMap);

    tof->initialConfig.set(tofConf);
}

dai::CameraControl ToFParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
