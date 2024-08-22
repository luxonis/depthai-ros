#include "depthai_bridge/TFPublisher.hpp"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dai {
namespace ros {
TFPublisher::TFPublisher(std::shared_ptr<rclcpp::Node> node,
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
                         const bool rsCompatibilityMode)
    : camName(camName),
      camModel(camModel),
      baseFrame(baseFrame),
      parentFrame(parentFrame),
      camPosX(camPosX),
      camPosY(camPosY),
      camPosZ(camPosZ),
      camRoll(camRoll),
      camPitch(camPitch),
      camYaw(camYaw),
      camFeatures(camFeatures),
      imuFromDescr(imuFromDescr),
      customURDFLocation(customURDFLocation),
      customXacroArgs(customXacroArgs),
      rsCompatibilityMode(rsCompatibilityMode),
      logger(node->get_logger()) {
    tfPub = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    paramClient = std::make_unique<rclcpp::AsyncParametersClient>(node, camName + std::string("_state_publisher"));

    auto json = calHandler.eepromToJson();
    auto camData = json["cameraData"];
    publishDescription();
    publishCamTransforms(camData, node);
    if(imuFromDescr != "true") {
        publishImuTransform(json, node);
    }
}

void TFPublisher::publishDescription() {
    auto urdf = getURDF();
    auto robotDescr = rclcpp::Parameter("robot_description", urdf);
    auto result = paramClient->set_parameters({robotDescr});
    RCLCPP_INFO(logger, "Published URDF");
}

void TFPublisher::publishCamTransforms(nlohmann::json camData, std::shared_ptr<rclcpp::Node> node) {
    for(auto& cam : camData) {
        geometry_msgs::msg::TransformStamped ts;
        geometry_msgs::msg::TransformStamped opticalTS;
        ts.header.stamp = node->get_clock()->now();
        opticalTS.header.stamp = ts.header.stamp;
        auto extrinsics = cam[1]["extrinsics"];

        ts.transform.rotation = quatFromRotM(extrinsics["rotationMatrix"]);
        ts.transform.translation = transFromExtr(extrinsics["translation"]);

        std::string name = getCamSocketName(cam[0]);
        ts.child_frame_id = baseFrame + std::string("_") + name + std::string("_camera_frame");
        // check if the camera is at the end of the chain
        if(extrinsics["toCameraSocket"] != -1) {
            ts.header.frame_id = baseFrame + std::string("_") + getCamSocketName(extrinsics["toCameraSocket"].get<int>()) + std::string("_camera_frame");
        } else {
            ts.header.frame_id = baseFrame;
            ts.transform.rotation.w = 1.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
        }
        // rotate optical fransform
        opticalTS.child_frame_id = baseFrame + std::string("_") + name + std::string("_camera_optical_frame");
        opticalTS.header.frame_id = ts.child_frame_id;
        opticalTS.transform.rotation.w = 0.5;
        opticalTS.transform.rotation.x = -0.5;
        opticalTS.transform.rotation.y = 0.5;
        opticalTS.transform.rotation.z = -0.5;
        tfPub->sendTransform(ts);
        tfPub->sendTransform(opticalTS);
    }
}
void TFPublisher::publishImuTransform(nlohmann::json json, std::shared_ptr<rclcpp::Node> node) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = node->get_clock()->now();
    auto imuExtr = json["imuExtrinsics"];
    if(imuExtr["toCameraSocket"] != -1) {
        ts.header.frame_id = baseFrame + std::string("_") + getCamSocketName(imuExtr["toCameraSocket"].get<int>()) + std::string("_camera_frame");
    } else {
        ts.header.frame_id = baseFrame;
    }
    ts.child_frame_id = baseFrame + std::string("_imu_frame");

    ts.transform.rotation = quatFromRotM(imuExtr["rotationMatrix"]);
    ts.transform.translation = transFromExtr(imuExtr["translation"]);
    bool zeroTrans = ts.transform.translation.x == 0 && ts.transform.translation.y == 0 && ts.transform.translation.z == 0;
    bool zeroRot = ts.transform.rotation.w == 1 && ts.transform.rotation.x == 0 && ts.transform.rotation.y == 0 && ts.transform.rotation.z == 0;
    if(zeroTrans || zeroRot) {
        RCLCPP_WARN(logger, "IMU extrinsics appear to be default. Check if the IMU is calibrated.");
    }
    tfPub->sendTransform(ts);
}

std::string TFPublisher::getCamSocketName(int socketNum) {
    if(rsCompatibilityMode) {
        return rsSocketNameMap.at(static_cast<dai::CameraBoardSocket>(socketNum));
    }
    return socketNameMap.at(static_cast<dai::CameraBoardSocket>(socketNum));
}

geometry_msgs::msg::Vector3 TFPublisher::transFromExtr(nlohmann::json translation) {
    geometry_msgs::msg::Vector3 trans;
    // optical coordinates to ROS
    trans.x = translation["y"].get<double>() / -100.0;
    trans.y = translation["x"].get<double>() / -100.0;
    trans.z = translation["z"].get<double>() / 100.0;
    return trans;
}
geometry_msgs::msg::Quaternion TFPublisher::quatFromRotM(nlohmann::json rotMatrix) {
    tf2::Matrix3x3 m(rotMatrix[0][0],
                     rotMatrix[0][1],
                     rotMatrix[0][2],

                     rotMatrix[1][0],
                     rotMatrix[1][1],
                     rotMatrix[1][2],

                     rotMatrix[2][0],
                     rotMatrix[2][1],
                     rotMatrix[2][2]);

    tf2::Quaternion q;
    m.getRotation(q);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
    return msg_quat;
}

bool TFPublisher::modelNameAvailable() {
    std::string path = ament_index_cpp::get_package_share_directory("depthai_descriptions") + "/urdf/models/";
    DIR* dir;
    struct dirent* ent;
    convertModelName();
    if((dir = opendir(path.c_str())) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            auto name = std::string(ent->d_name);
            RCLCPP_DEBUG(logger, "Found model: %s", name.c_str());
            if(name == camModel + ".stl") {
                return true;
            }
        }
        closedir(dir);
    } else {
        throw std::runtime_error("Could not open depthai_descriptions package directory");
    }
    return false;
}

std::string TFPublisher::prepareXacroArgs() {
    if(!customURDFLocation.empty() || !modelNameAvailable()) {
        RCLCPP_ERROR(
            logger,
            "Model name %s not found in depthai_descriptions package. If camera model is autodetected, please notify developers. Using default model: OAK-D",
            camModel.c_str());
        camModel = "OAK-D";
    }

    std::string xacroArgs = "camera_name:=" + camName;
    xacroArgs += " camera_model:=" + camModel;
    xacroArgs += " base_frame:=" + baseFrame;
    xacroArgs += " parent_frame:=" + parentFrame;
    xacroArgs += " cam_pos_x:=" + camPosX;
    xacroArgs += " cam_pos_y:=" + camPosY;
    xacroArgs += " cam_pos_z:=" + camPosZ;
    xacroArgs += " cam_roll:=" + camRoll;
    xacroArgs += " cam_pitch:=" + camPitch;
    xacroArgs += " cam_yaw:=" + camYaw;
    xacroArgs += " has_imu:=" + imuFromDescr;
    return xacroArgs;
}

void TFPublisher::convertModelName() {
    if(camModel.find("OAK-D-PRO-POE") != std::string::npos || camModel.find("OAK-D-PRO-W-POE") != std::string::npos
       || camModel.find("OAK-D-S2-POE") != std::string::npos) {
        camModel = "OAK-D-POE";
    } else if(camModel.find("OAK-D-LITE") != std::string::npos) {
        camModel = "OAK-D-PRO";
    } else if(camModel.find("OAK-D-S2") != std::string::npos) {
        camModel = "OAK-D-PRO";
    } else if(camModel.find("OAK-D-PRO-W") != std::string::npos) {
        camModel = "OAK-D-PRO";
    } else if(camModel.find("OAK-D-PRO") != std::string::npos) {
        camModel = "OAK-D-PRO";
    } else if(camModel.find("OAK-D-POE")) {
        camModel = "OAK-D-POE";
    } else if(camModel.find("OAK-D") != std::string::npos) {
        camModel = "OAK-D";
    } else {
        RCLCPP_WARN(logger, "Unable to match model name: %s to available model family.", camModel.c_str());
    }
}

std::string TFPublisher::getURDF() {
    std::string args, path;
    if(customXacroArgs.empty()) {
        args = prepareXacroArgs();
    } else {
        args = customXacroArgs;
    }
    if(customURDFLocation.empty()) {
        path = ament_index_cpp::get_package_share_directory("depthai_descriptions") + "/urdf/base_descr.urdf.xacro ";
    } else {
        path = customURDFLocation + " ";
    }
    std::string cmd = "xacro " + path + args;
    RCLCPP_DEBUG(logger, "Xacro command: %s", cmd.c_str());
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if(!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while(fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
}  // namespace ros
}  // namespace dai
