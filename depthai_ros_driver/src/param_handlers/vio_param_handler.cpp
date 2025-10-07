#include "depthai_ros_driver/param_handlers/vio_param_handler.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
VioParamHandler::VioParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
VioParamHandler::~VioParamHandler() = default;
void VioParamHandler::declareParams(std::shared_ptr<dai::node::BasaltVIO> vio) {
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 2);
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(dai::CameraBoardSocket::CAM_B));
    declareAndLogParam<int>("i_imu_update_rate", 400);
    declareAndLogParam<int>(ParamNames::WIDTH, 640);
    declareAndLogParam<int>(ParamNames::HEIGHT, 400);
    declareAndLogParam<double>(ParamNames::FPS, 60.0);
    declareAndLogParam<std::string>("i_vio_config_path", "");
    if(declareAndLogParam<bool>("i_override_local_transform", false)) {
        float x = declareAndLogParam<float>("i_local_transform_x", 0.0);
        float y = declareAndLogParam<float>("i_local_transform_y", 0.0);
        float z = declareAndLogParam<float>("i_local_transform_z", 0.0);
        float qx = declareAndLogParam<float>("i_local_transform_qx", 0.0);
        float qy = declareAndLogParam<float>("i_local_transform_qy", 0.0);
        float qz = declareAndLogParam<float>("i_local_transform_qz", 0.0);
        float qw = declareAndLogParam<float>("i_local_transform_qw", 1.0);
        auto transform = std::make_shared<dai::TransformData>(x, y, z, qx, qy, qz, qw);
        vio->setLocalTransform(transform);
    }
    if(declareAndLogParam<bool>("i_override_imu_extrinsics", false)) {
        float x = declareAndLogParam<float>("i_imu_extr_x", 0.0);
        float y = declareAndLogParam<float>("i_imu_extr_y", 0.0);
        float z = declareAndLogParam<float>("i_imu_extr_z", 0.0);
        float qx = declareAndLogParam<float>("i_imu_extr_qx", 0.0);
        float qy = declareAndLogParam<float>("i_imu_extr_qy", 0.0);
        float qz = declareAndLogParam<float>("i_imu_extr_qz", 0.0);
        float qw = declareAndLogParam<float>("i_imu_extr_qw", 1.0);
        auto transform = std::make_shared<dai::TransformData>(x, y, z, qx, qy, qz, qw);
        vio->setImuExtrinsics(transform);
    }
    if(declareAndLogParam<bool>("i_set_acc_bias", false)) {
        std::vector<double> accBias = declareAndLogParam<std::vector<double>>("i_acc_bias", std::vector<double>(9, 0.0));
        vio->setAccelBias(accBias);
    }
    if(declareAndLogParam<bool>("i_set_gyro_bias", false)) {
        std::vector<double> gyroBias = declareAndLogParam<std::vector<double>>("i_gyro_bias", std::vector<double>(12, 0.0));
        vio->setGyroBias(gyroBias);
    }
    if(declareAndLogParam<bool>("i_set_acc_noise_std", false)) {
        std::vector<double> accNoiseStd = declareAndLogParam<std::vector<double>>("i_acc_noise_std", std::vector<double>(3, 0.0));
        vio->setAccelNoiseStd(accNoiseStd);
    }
    if(declareAndLogParam<bool>("i_set_gyro_noise_std", false)) {
        std::vector<double> gyroNoiseStd = declareAndLogParam<std::vector<double>>("i_gyro_noise_std", std::vector<double>(3, 0.0));
        vio->setGyroNoiseStd(gyroNoiseStd);
    }

    declareAndLogParam<bool>("i_publish_tf", true);
    declareAndLogParam<std::string>("i_frame_id", "odom");
    declareAndLogParam<std::string>("i_child_frame_id", "oak_parent_frame");
    std::string configPath = declareAndLogParam<std::string>("i_config_path", "depthai_ros_driver_default_vio.json");
    if(configPath.find("depthai_ros_driver_default_vio.json") != std::string::npos) {
        configPath = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/custom/" + configPath;
    }
    auto finalPath = declareAndLogParam<std::string>("config_path", configPath, true);

    if(!finalPath.empty()) {
        RCLCPP_DEBUG(getROSNode()->get_logger(), "Loading VIO config from %s", finalPath.c_str());
        vio->setConfigPath(configPath);
    }
    declareAndLogParam<std::vector<double>>("i_covariance", std::vector<double>(36, 0.0));
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
