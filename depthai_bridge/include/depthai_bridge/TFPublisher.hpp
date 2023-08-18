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
                         const std::string& camYaw);
    std::string getURDF();
    geometry_msgs::msg::Quaternion quatFromRotM(nlohmann::json rotMatrix);
    geometry_msgs::msg::Vector3 transFromExtr(nlohmann::json translation);

   private:
    std::string prepareXacroArgs();
    void publishDescription();
    void publishCamTransforms(nlohmann::json camData, rclcpp::Node* node);
    void publishImuTransform(nlohmann::json json, rclcpp::Node* node);
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
    std::vector<dai::CameraFeatures> _camFeatures;
    rclcpp::Logger _logger;
};
}  // namespace ros
}  // namespace dai