#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"

#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ImuParamHandler::ImuParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name) : BaseParamHandler(node, name) {
    syncMethodMap = {
        {"COPY", dai::ros::ImuSyncMethod::COPY},
        {"LINEAR_INTERPOLATE_GYRO", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO},
        {"LINEAR_INTERPOLATE_ACCEL", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL},
    };
    messagetTypeMap = {
        {"IMU", imu::ImuMsgType::IMU}, {"IMU_WITH_MAG", imu::ImuMsgType::IMU_WITH_MAG}, {"IMU_WITH_MAG_SPLIT", imu::ImuMsgType::IMU_WITH_MAG_SPLIT}};
    rotationVectorTypeMap = {{"ROTATION_VECTOR", dai::IMUSensor::ROTATION_VECTOR},
                             {"GAME_ROTATION_VECTOR", dai::IMUSensor::GAME_ROTATION_VECTOR},
                             {"GEOMAGNETIC_ROTATION_VECTOR", dai::IMUSensor::GEOMAGNETIC_ROTATION_VECTOR},
                             {"ARVR_STABILIZED_ROTATION_VECTOR", dai::IMUSensor::ARVR_STABILIZED_ROTATION_VECTOR},
                             {"ARVR_STABILIZED_GAME_ROTATION_VECTOR", dai::IMUSensor::ARVR_STABILIZED_GAME_ROTATION_VECTOR}};
    accelerometerModeMap = {{"ACCELEROMETER_RAW", dai::IMUSensor::ACCELEROMETER_RAW},
                            {"ACCELEROMETER", dai::IMUSensor::ACCELEROMETER},
                            {"LINEAR_ACCELERATION", dai::IMUSensor::LINEAR_ACCELERATION},
                            {"GRAVITY", dai::IMUSensor::GRAVITY}};
    gyroscopeModeMap = {{"GYROSCOPE_RAW", dai::IMUSensor::GYROSCOPE_RAW},
                        {"GYROSCOPE_CALIBRATED", dai::IMUSensor::GYROSCOPE_CALIBRATED},
                        {"GYROSCOPE_UNCALIBRATED", dai::IMUSensor::GYROSCOPE_UNCALIBRATED}};
    magnetometerModeMap = {{"MAGNETOMETER_RAW", dai::IMUSensor::MAGNETOMETER_RAW},
                           {"MAGNETOMETER_CALIBRATED", dai::IMUSensor::MAGNETOMETER_CALIBRATED},
                           {"MAGNETOMETER_UNCALIBRATED", dai::IMUSensor::MAGNETOMETER_UNCALIBRATED}};
}
ImuParamHandler::~ImuParamHandler() = default;
void ImuParamHandler::declareParams(std::shared_ptr<dai::node::IMU> imu, const std::string& imuType) {
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);
    declareAndLogParam<int>("i_max_q_size", 8);
    auto messageType = declareAndLogParam<std::string>("i_message_type", "IMU");
    declareAndLogParam<std::string>("i_sync_method", "LINEAR_INTERPOLATE_ACCEL");
    declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
    if(declareAndLogParam<bool>("i_enable_acc", true)) {
        const std::string accelerometerModeName = utils::getUpperCaseStr(declareAndLogParam<std::string>("i_acc_mode", "ACCELEROMETER_RAW"));
        const dai::IMUSensor accelerometerMode = utils::getValFromMap(accelerometerModeName, accelerometerModeMap);
        const int accelerometerFreq = declareAndLogParam<int>("i_acc_freq", 400);
        declareAndLogParam<float>("i_acc_cov", 0.0);

        imu->enableIMUSensor(accelerometerMode, accelerometerFreq);
    }

    if(declareAndLogParam<bool>("i_enable_gyro", true)) {
        const std::string gyroscopeModeName = utils::getUpperCaseStr(declareAndLogParam<std::string>("i_gyro_mode", "GYROSCOPE_RAW"));
        const dai::IMUSensor gyroscopeMode = utils::getValFromMap(gyroscopeModeName, gyroscopeModeMap);
        const int gyroscopeFreq = declareAndLogParam<int>("i_gyro_freq", 400);
        declareAndLogParam<float>("i_gyro_cov", 0.0);

        imu->enableIMUSensor(gyroscopeMode, gyroscopeFreq);
    }

    const bool magnetometerAvailable = imuType == "BNO086";
    if(declareAndLogParam<bool>("i_enable_mag", magnetometerAvailable)) {
        if(magnetometerAvailable) {
            const std::string magnetometerModeName = utils::getUpperCaseStr(declareAndLogParam<std::string>("i_mag_mode", "MAGNETOMETER_RAW"));
            const dai::IMUSensor magnetometerMode = utils::getValFromMap(magnetometerModeName, magnetometerModeMap);
            const int magnetometerFreq = declareAndLogParam<int>("i_mag_freq", 100);
            declareAndLogParam<float>("i_mag_cov", 0.0);

            imu->enableIMUSensor(magnetometerMode, magnetometerFreq);
        } else {
            RCLCPP_ERROR(getROSNode()->get_logger(), "Magnetometer enabled but not available with current sensor");
            declareAndLogParam<bool>("i_enable_mag", false, true);
        }
    }

    const bool rotationAvailable = imuType == "BNO086";
    if(declareAndLogParam<bool>("i_enable_rotation", rotationAvailable)) {
        if(rotationAvailable) {
            const std::string rotationModeName = utils::getUpperCaseStr(declareAndLogParam<std::string>("i_rot_mode", "ROTATION_VECTOR"));
            const dai::IMUSensor rotationMode = utils::getValFromMap(rotationModeName, rotationVectorTypeMap);
            const int rotationFreq = declareAndLogParam<int>("i_rot_freq", 400);
            declareAndLogParam<float>("i_rot_cov", -1.0);

            imu->enableIMUSensor(rotationMode, rotationFreq);
        } else {
            RCLCPP_ERROR(getROSNode()->get_logger(), "Rotation enabled but not available with current sensor");
            declareAndLogParam<bool>("i_enable_rotation", false, true);
        }
        imu->setBatchReportThreshold(declareAndLogParam<int>("i_batch_report_threshold", 5));
        imu->setMaxBatchReports(declareAndLogParam<int>("i_max_batch_reports", 10));
    }
}

dai::ros::ImuSyncMethod ImuParamHandler::getSyncMethod() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_sync_method")), syncMethodMap);
}

imu::ImuMsgType ImuParamHandler::getMsgType() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_message_type")), messagetTypeMap);
}

dai::CameraControl ImuParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
