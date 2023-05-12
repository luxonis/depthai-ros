#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"

#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace param_handlers {
ImuParamHandler::ImuParamHandler(ros::NodeHandle node, const std::string& name) : BaseParamHandler(node, name) {}
ImuParamHandler::~ImuParamHandler() = default;
void ImuParamHandler::declareParams(std::shared_ptr<dai::node::IMU> imu, const std::string& imuType) {
    imuSyncMethodMap = {
        {"COPY", dai::ros::ImuSyncMethod::COPY},
        {"LINEAR_INTERPOLATE_GYRO", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO},
        {"LINEAR_INTERPOLATE_ACCEL", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL},
    };
    imuMessagetTypeMap = {
        {"IMU", imu::ImuMsgType::IMU}, {"IMU_WITH_MAG", imu::ImuMsgType::IMU_WITH_MAG}, {"IMU_WITH_MAG_SPLIT", imu::ImuMsgType::IMU_WITH_MAG_SPLIT}};
    bool rotationAvailable = imuType == "BNO086";
    if(getParam<bool>("i_enable_rotation")) {
        if(rotationAvailable) {
            imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, getParam<int>("i_rotation_vec_freq", 400));
            imu->enableIMUSensor(dai::IMUSensor::MAGNETOMETER_CALIBRATED, getParam<int>("i_magnetometer_freq", 100));
        } else {
            ROS_ERROR("Rotation enabled but not available with current sensor");
            setParam<bool>("i_enable_rotation", false);
        }
    }
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, getParam<int>("i_acc_freq", 500));
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, getParam<int>("i_gyro_freq", 400));
    imu->setBatchReportThreshold(getParam<int>("i_batch_report_threshold", 5));
    imu->setMaxBatchReports(getParam<int>("i_max_batch_reports", 20));
}

dai::ros::ImuSyncMethod ImuParamHandler::getSyncMethod() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_sync_method")), imuSyncMethodMap);
}

imu::ImuMsgType ImuParamHandler::getMsgType() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_message_type")), imuMessagetTypeMap);
}

dai::CameraControl ImuParamHandler::setRuntimeParams(parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver