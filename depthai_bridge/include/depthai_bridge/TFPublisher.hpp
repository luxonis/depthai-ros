#pragma once
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/parameter_client.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace depthai_bridge {
class TFPublisher {
   public:
    explicit TFPublisher(std::shared_ptr<rclcpp::Node> node,
                         const dai::CalibrationHandler& calHandler,
                         const std::vector<dai::CameraFeatures>& camFeatures,
                         const std::string& camName,
                         const std::string& camModel,
                         const std::string& baseFrame = "oak",
                         const std::string& parentFrame = "oak_parent_frame",
                         const std::string& camPosX = "0.0",
                         const std::string& camPosY = "0.0",
                         const std::string& camPosZ = "0.0",
                         const std::string& camRoll = "0.0",
                         const std::string& camPitch = "0.0",
                         const std::string& camYaw = "0.0",
                         const std::string& imuFromDescr = "false",
                         const std::string& customURDFLocation = "",
                         const std::string& customXacroArgs = "",
                         const bool rsCompatibilityMode = false);
    /**
     * @brief Obtain URDF description by running Xacro with provided arguments.
     */
    std::string getURDF();
    geometry_msgs::msg::Vector3 transFromExtr(std::vector<float> translation);
    geometry_msgs::msg::Quaternion quatFromRotM(std::vector<std::vector<float>> extrMat);

   private:
    /**
     * @brief Converts model name to one of the available model families
     */
    void convertModelName();
    /**
     * @brief Prepare arguments for xacro command. If custom URDF location is not provided, check if model name is available in depthai_descriptions package.
     */
    std::string prepareXacroArgs();
    /**
     * @brief Get URDF description and set it as a parameter for robot_state_publisher
     */
    void publishDescription();
    /**
     * @brief Publish camera transforms ("standard" and optical) based on calibration data.
     * Frame names are based on socket names and use following convention: [base_frame]_[socket_name]_camera_frame and
     * [base_frame]_[socket_name]_camera_optical_frame
     */
    void publishCamTransforms(nlohmann::json camData, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler);
    /**
     * @brief Publish IMU transform based on calibration data.
     * Frame name is based on IMU name and uses following convention: [base_frame]_imu_frame.
     * If IMU extrinsics are not set, warning is printed out and imu frame is published with zero translation and rotation.
     */
    void publishImuTransform(nlohmann::json json, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler);
    /**
     * @brief Check if model STL file is available in depthai_descriptions package.
     */
    bool modelNameAvailable();
    std::unique_ptr<rclcpp::AsyncParametersClient> paramClient;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPub;
    std::string nodeName;
    std::string camName;
    std::string camModel;
    std::string baseFrame;
    std::string parentFrame;
    std::string camPosX;
    std::string camPosY;
    std::string camPosZ;
    std::string camRoll;
    std::string camPitch;
    std::string camYaw;
    std::string imuFromDescr;
    std::string customURDFLocation;
    std::string customXacroArgs;
    std::vector<dai::CameraFeatures> camFeatures;
    bool rsCompatibilityMode;
    rclcpp::Logger logger;
};
}  // namespace depthai_bridge
