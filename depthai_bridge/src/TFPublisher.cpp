#include "depthai_bridge/TFPublisher.hpp"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <depthai/common/CameraBoardSocket.hpp>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace depthai_bridge {
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
    publishCamTransforms(camData, node, calHandler);
    if(imuFromDescr != "true") {
        publishImuTransform(json, node, calHandler);
    }
}

void TFPublisher::publishDescription() {
    auto urdf = getURDF();
    auto robotDescr = rclcpp::Parameter("robot_description", urdf);
    // check if param server is available
    bool ready = paramClient->service_is_ready();

    if(!ready) {
        RCLCPP_WARN(logger, "Parameter server for [ %s ] is not ready, please check if the node is running", std::string(camName + "_state_publisher").c_str());
    }
    auto result = paramClient->set_parameters({robotDescr});
    RCLCPP_INFO(logger, "Published URDF");
}

void TFPublisher::publishCamTransforms(nlohmann::json camData, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler) {
    for(auto& cam : camData) {
        geometry_msgs::msg::TransformStamped ts;
        geometry_msgs::msg::TransformStamped opticalTS;
        ts.header.stamp = node->get_clock()->now();
        opticalTS.header.stamp = ts.header.stamp;
        auto extrinsics = cam[1]["extrinsics"];
        auto currCam = static_cast<dai::CameraBoardSocket>(cam[0].get<int>());
        if(extrinsics["toCameraSocket"] != -1) {
            auto toCam = static_cast<dai::CameraBoardSocket>(extrinsics["toCameraSocket"].get<int>());
            auto extrMat = calHandler.getCameraExtrinsics(currCam, toCam, false);
            ts.transform.rotation = quatFromRotM(extrMat);
            auto trans = calHandler.getCameraTranslationVector(currCam, toCam, false);
            ts.transform.translation = transFromExtr(trans);
        }

        std::string name = getSocketName(static_cast<dai::CameraBoardSocket>(cam[0]), camModel, rsCompatibilityMode);
        ts.child_frame_id = baseFrame + std::string("_") + name + std::string("_camera_frame");
        // check if the camera is at the end of the chain
        if(extrinsics["toCameraSocket"] != -1) {
            ts.header.frame_id = getFrameName(
                baseFrame,
                getSocketName(static_cast<dai::CameraBoardSocket>(extrinsics["toCameraSocket"].get<int>()), camModel, rsCompatibilityMode) + "_camera_frame");
        } else {
            ts.header.frame_id = baseFrame;
            ts.transform.rotation.w = 1.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
        }
        // rotate optical fransform
        opticalTS.child_frame_id = getOpticalFrameName(baseFrame, name, rsCompatibilityMode);
        opticalTS.header.frame_id = ts.child_frame_id;
        opticalTS.transform.rotation.w = 0.5;
        opticalTS.transform.rotation.x = -0.5;
        opticalTS.transform.rotation.y = 0.5;
        opticalTS.transform.rotation.z = -0.5;
        tfPub->sendTransform(ts);
        tfPub->sendTransform(opticalTS);
    }
}
void TFPublisher::publishImuTransform(nlohmann::json json, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = node->get_clock()->now();
    auto imuExtr = json["imuExtrinsics"];
    ts.child_frame_id = baseFrame + std::string("_imu_frame");
    if(imuExtr["toCameraSocket"] != -1) {
        ts.header.frame_id = getFrameName(
            baseFrame,
            getSocketName(static_cast<dai::CameraBoardSocket>(imuExtr["toCameraSocket"].get<int>()), camModel, rsCompatibilityMode) + "_camera_frame");
        auto extrMat = calHandler.getImuToCameraExtrinsics(static_cast<dai::CameraBoardSocket>(imuExtr["toCameraSocket"].get<int>()));
        // pass parts of 4x4 matrix to transfFromExtr
        std::vector<float> translation = {extrMat[0][3], extrMat[1][3], extrMat[2][3]};
        ts.transform.translation = transFromExtr(translation);
        // imu is already being output in RDF format
        ts.transform.rotation.w = 0.5;
        ts.transform.rotation.x = -0.5;
        ts.transform.rotation.y = 0.5;
        ts.transform.rotation.z = -0.5;
    } else {
        ts.header.frame_id = baseFrame;
        RCLCPP_WARN(logger, "IMU extrinsics are not set. Publishing IMU frame with zero translation and RDF orientation.");
        // imu is already being output in RDF format
        ts.transform.rotation.w = 0.5;
        ts.transform.rotation.x = -0.5;
        ts.transform.rotation.y = 0.5;
        ts.transform.rotation.z = -0.5;
    }
    tfPub->sendTransform(ts);
}

geometry_msgs::msg::Vector3 TFPublisher::transFromExtr(std::vector<float> translation) {
    geometry_msgs::msg::Vector3 trans;
    // optical coordinates to ROS
    trans.x = translation[2] / 100.0;
    trans.y = translation[0] / -100.0;
    trans.z = translation[1] / -100.0;
    return trans;
}
geometry_msgs::msg::Quaternion TFPublisher::quatFromRotM(std::vector<std::vector<float>> extrMat) {
    tf2::Matrix3x3 m(extrMat[0][0],
                     extrMat[0][1],
                     extrMat[0][2],

                     extrMat[1][0],
                     extrMat[1][1],
                     extrMat[1][2],

                     extrMat[2][0],
                     extrMat[2][1],
                     extrMat[2][2]);

    tf2::Quaternion q_extr;
    m.getRotation(q_extr);
    // optical coordinates to ROS
    tf2::Quaternion q_flu(0.0, 0.0, 0.0, 1.0);
    tf2::Quaternion q_rot2rdf(-0.5, 0.5, -0.5, 0.5);
    tf2::Quaternion q_rdf = q_flu * q_rot2rdf;
    q_rdf = q_rdf * q_extr;
    tf2::Quaternion q_final = q_rdf * q_rot2rdf.inverse();
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q_final);
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
    std::map<std::string, std::string> modelMappings = {{"OAK-D-SR-POE", "OAK-D-SR-POE"},
                                                        {"OAK-D-PRO-W-POE", "OAK-D-POE"},
                                                        {"OAK-D-PRO-POE", "OAK-D-POE"},
                                                        {"OAK-D-S2-POE", "OAK-D-POE"},
                                                        {"OAK-D-POE", "OAK-D-POE"},
                                                        {"OAK-D-LITE", "OAK-D-PRO"},
                                                        {"OAK-D-S2", "OAK-D-PRO"},
                                                        {"OAK-D-PRO-W", "OAK-D-PRO"},
                                                        {"OAK-D-PRO", "OAK-D-PRO"},
                                                        {"OAK-D", "OAK-D"},
                                                        {"OAK-T", "OAK-T"}};

    for(const auto& [key, value] : modelMappings) {
        if(camModel == key) {
            camModel = value;
            return;
        }
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
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(cmd.c_str(), "r"), pclose);
    if(!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while(fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
}  // namespace depthai_bridge
