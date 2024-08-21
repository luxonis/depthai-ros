#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"

#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ImuParamHandler::ImuParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name) : BaseParamHandler(node, name) {
    imuSyncMethodMap = {
        {"COPY", dai::ros::ImuSyncMethod::COPY},
        {"LINEAR_INTERPOLATE_GYRO", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO},
        {"LINEAR_INTERPOLATE_ACCEL", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL},
    };
    imuMessagetTypeMap = {
        {"IMU", imu::ImuMsgType::IMU}, {"IMU_WITH_MAG", imu::ImuMsgType::IMU_WITH_MAG}, {"IMU_WITH_MAG_SPLIT", imu::ImuMsgType::IMU_WITH_MAG_SPLIT}};
    rotationVectorTypeMap = {{"ROTATION_VECTOR", dai::IMUSensor::ROTATION_VECTOR},
                             {"GAME_ROTATION_VECTOR", dai::IMUSensor::GAME_ROTATION_VECTOR},
                             {"GEOMAGNETIC_ROTATION_VECTOR", dai::IMUSensor::GEOMAGNETIC_ROTATION_VECTOR},
                             {"ARVR_STABILIZED_ROTATION_VECTOR", dai::IMUSensor::ARVR_STABILIZED_ROTATION_VECTOR},
                             {"ARVR_STABILIZED_GAME_ROTATION_VECTOR", dai::IMUSensor::ARVR_STABILIZED_GAME_ROTATION_VECTOR}};
}
ImuParamHandler::~ImuParamHandler() = default;
void ImuParamHandler::declareParams(std::shared_ptr<dai::node::IMU> imu, const std::string& imuType) {
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);
    declareAndLogParam<int>("i_max_q_size", 8);
    auto messageType = declareAndLogParam<std::string>("i_message_type", "IMU");
    declareAndLogParam<std::string>("i_sync_method", "LINEAR_INTERPOLATE_ACCEL");
    declareAndLogParam<float>("i_acc_cov", 0.0);
    declareAndLogParam<float>("i_gyro_cov", 0.0);
    declareAndLogParam<float>("i_rot_cov", -1.0);
    declareAndLogParam<float>("i_mag_cov", 0.0);
    declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
    bool rotationAvailable = imuType == "BNO086";
    if(declareAndLogParam<bool>("i_enable_rotation", false)) {
        if(rotationAvailable) {
            auto rotationVecType = utils::getValFromMap(utils::getUpperCaseStr(declareAndLogParam<std::string>("i_rotation_vector_type", "ROTATION_VECTOR")),
                                                        rotationVectorTypeMap);
            imu->enableIMUSensor(rotationVecType, declareAndLogParam<int>("i_rot_freq", 400));
            // if imu message type is IMU_WITH_MAG or IMU_WITH_MAG_SPLIT, enable magnetometer
            if(messageType == "IMU_WITH_MAG" || messageType == "IMU_WITH_MAG_SPLIT") {
                imu->enableIMUSensor(dai::IMUSensor::MAGNETOMETER_CALIBRATED, declareAndLogParam<int>("i_mag_freq", 100));
            }
        } else {
            RCLCPP_ERROR(getROSNode()->get_logger(), "Rotation enabled but not available with current sensor");
            declareAndLogParam<bool>("i_enable_rotation", false, true);
        }
    }
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, declareAndLogParam<int>("i_acc_freq", 400));
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, declareAndLogParam<int>("i_gyro_freq", 400));
    imu->setBatchReportThreshold(declareAndLogParam<int>("i_batch_report_threshold", 5));
    imu->setMaxBatchReports(declareAndLogParam<int>("i_max_batch_reports", 10));
}

dai::ros::ImuSyncMethod ImuParamHandler::getSyncMethod() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_sync_method")), imuSyncMethodMap);
}

imu::ImuMsgType ImuParamHandler::getMsgType() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_message_type")), imuMessagetTypeMap);
}

dai::CameraControl ImuParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
