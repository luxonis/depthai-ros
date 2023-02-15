#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ImuParamHandler::ImuParamHandler(const std::string& name) : BaseParamHandler(name) {}
ImuParamHandler::~ImuParamHandler() = default;
void ImuParamHandler::declareParams(ros::NodeHandle /*node*/, std::shared_ptr<dai::node::IMU> imu) {
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 400);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    //   imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
}
dai::CameraControl ImuParamHandler::setRuntimeParams(ros::NodeHandle /*node*/, parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver