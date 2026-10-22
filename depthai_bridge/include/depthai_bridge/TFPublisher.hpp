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
    // Develop-compatible constructor; existing clients need no source changes.
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
                         const bool rsCompatibilityMode = false,
                         const std::string& tfPrefix = "");

    explicit TFPublisher(std::shared_ptr<rclcpp::Node> node,
                         const dai::CalibrationHandler& calHandler,
                         const std::vector<dai::CameraFeatures>& camFeatures,
                         const std::string& camName,
                         const std::string& camModel,
                         const std::string& baseFrame,
                         const std::string& parentFrame,
                         const std::string& camPosX,
                         const std::string& camPosY,
                         const std::string& camPosZ,
                         const std::string& camRoll,
                         const std::string& camPitch,
                         const std::string& camYaw,
                         const std::string& imuFromDescr,
                         const std::string& customURDFLocation,
                         const std::string& customXacroArgs,
                         const bool rsCompatibilityMode,
                         const std::string& tfPrefix,
                         const std::string& detectedImu,
                         const std::string& imuSensor = "",
                         const std::string& imuFrame = "auto",
                         const std::string& poseSource = "legacy",
                         const std::string& modelSelection = "auto",
                         const std::string& fallbackModel = "",
                         const std::string& meshModel = "",
                         const std::string& legacyFrameAliases = "true");
    /**
     * @brief Obtain URDF description by running Xacro with provided arguments.
     */
    std::string getURDF();
    geometry_msgs::msg::Vector3 transFromExtr(std::vector<float> translation);
    geometry_msgs::msg::Quaternion quatFromRotM(std::vector<std::vector<float>> extrMat);

   private:
    /// Prepare mappings for the shared board/chip description resolver.
    std::vector<std::string> prepareXacroArgs();
    /**
     * @brief Get URDF description and set it as a parameter for robot_state_publisher
     */
    void publishDescription();
    /**
     * @brief Publish camera transforms ("standard" and optical) based on calibration data.
     * Frame names are based on socket names and use following convention: [tf_prefix]_[socket_name]_camera_frame and
     * [tf_prefix]_[socket_name]_camera_optical_frame
     */
    void publishCamTransforms(nlohmann::json camData, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler);
    /**
     * @brief Publish IMU transform based on calibration data.
     * Frame name is based on IMU name and uses following convention: [tf_prefix]_imu_frame.
     * If IMU extrinsics are not set, warning is printed out and imu frame is published with zero translation and rotation.
     */
    void publishImuTransform(nlohmann::json json, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler);
    std::unique_ptr<rclcpp::AsyncParametersClient> paramClient;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPub;
    std::string camName;
    std::string camModel;
    std::string tfPrefix;
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
    // Serialized private policy occupies the former customXacroArgs string slot.
    // Keep the develop member layout: old clients allocate/destruct this class
    // inline, so adding members would corrupt their allocations after relinking.
    std::string descriptionPolicy;
    std::vector<dai::CameraFeatures> camFeatures;
    bool rsCompatibilityMode;
    rclcpp::Logger logger;
};
}  // namespace depthai_bridge
