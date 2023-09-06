#pragma once
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace dai {
namespace ros {
class TFPublisher {
   public:
    explicit TFPublisher(rclcpp::Node* node,
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
                         const std::string& customXacroArgs);
    /*
     * @brief Obtain URDF description by running Xacro with provided arguments.
     */
    std::string getURDF();
    geometry_msgs::msg::Quaternion quatFromRotM(nlohmann::json rotMatrix);
    geometry_msgs::msg::Vector3 transFromExtr(nlohmann::json translation);

   private:
    /*
     * @brief Converts model name to one of the available model families
     */
    void convertModelName();
    /*
     * @brief Prepare arguments for xacro command. If custom URDF location is not provided, check if model name is available in depthai_descriptions package.
     */
    std::string prepareXacroArgs();
    /*
     * @brief Get URDF description and set it as a parameter for robot_state_publisher
     */
    void publishDescription();
    /*
     * @brief Publish camera transforms ("standard" and optical) based on calibration data.
     * Frame names are based on socket names and use following convention: [base_frame]_[socket_name]_camera_frame and
     * [base_frame]_[socket_name]_camera_optical_frame
     */
    void publishCamTransforms(nlohmann::json camData, rclcpp::Node* node);
    /*
     * @brief Publish IMU transform based on calibration data.
     * Frame name is based on IMU name and uses following convention: [base_frame]_imu_frame.
     * If IMU extrinsics are not set, warning is printed out and imu frame is published with zero translation and rotation.
     */
    void publishImuTransform(nlohmann::json json, rclcpp::Node* node);
    /*
     * @brief Check if model STL file is available in depthai_descriptions package.
     */
    bool modelNameAvailable();
    std::string getCamSocketName(int socketNum);
    std::unique_ptr<rclcpp::AsyncParametersClient> _paramClient;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tfPub;
    std::string _camName;
    std::string _camModel;
    std::string _baseFrame;
    std::string _parentFrame;
    std::string _camPosX;
    std::string _camPosY;
    std::string _camPosZ;
    std::string _camRoll;
    std::string _camPitch;
    std::string _camYaw;
    std::string _imuFromDescr;
    std::string _customURDFLocation;
    std::string _customXacroArgs;
    std::vector<dai::CameraFeatures> _camFeatures;
    rclcpp::Logger _logger;
};
}  // namespace ros
}  // namespace dai