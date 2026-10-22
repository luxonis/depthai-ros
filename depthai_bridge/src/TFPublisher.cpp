#include "depthai_bridge/TFPublisher.hpp"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <depthai/common/CameraBoardSocket.hpp>
#include <memory>
#include <string>
#include <set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_bridge/DescriptionRunner.hpp"
#include "depthai_bridge/FrameAliases.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace depthai_bridge {
// DEPRECATED develop-compatible constructor, retained for source/link symbol compatibility.
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
                         const bool rsCompatibilityMode,
                         const std::string& tfPrefix)
    : TFPublisher(node, calHandler, camFeatures, camName, camModel, baseFrame, parentFrame, camPosX, camPosY, camPosZ, camRoll, camPitch, camYaw, imuFromDescr, customURDFLocation, customXacroArgs, rsCompatibilityMode, tfPrefix, "", "", "auto", "legacy", "auto", "", "", "true") {}

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
                         const bool rsCompatibilityMode,
                         const std::string& tfPrefix,
                         const std::string& detectedImu,
                         const std::string& imuSensor,
                         const std::string& imuFrame,
                         const std::string& poseSource,
                         const std::string& modelSelection,
                         const std::string& fallbackModel,
                         const std::string& meshModel,
                         const std::string& legacyFrameAliases)
    : camName(camName),
      camModel(camModel),
      tfPrefix(resolveFramePrefix(tfPrefix, node->get_name())),
      baseFrame(baseFrame),
      parentFrame(parentFrame),
      camPosX(camPosX),
      camPosY(camPosY),
      camPosZ(camPosZ),
      camRoll(camRoll),
      camPitch(camPitch),
      camYaw(camYaw),
      imuFromDescr(imuFromDescr),
      customURDFLocation(customURDFLocation),
      customXacroArgs(customXacroArgs),
      detectedImu(detectedImu),
      imuSensor(imuSensor),
      imuFrame(imuFrame),
      poseSource(poseSource),
      modelSelection(modelSelection),
      fallbackModel(fallbackModel),
      meshModel(meshModel),
      legacyFrameAliases(legacyFrameAliases),
      camFeatures(camFeatures),
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
    std::set<std::string> occupiedFrames;
    for(const auto& cam : camData) {
        const auto name = getSocketName(static_cast<dai::CameraBoardSocket>(cam[0].get<int>()), camModel, rsCompatibilityMode);
        occupiedFrames.insert(getFrameName(tfPrefix, name + (rsCompatibilityMode ? "_frame" : "_camera_frame")));
        occupiedFrames.insert(getOpticalFrameName(tfPrefix, name, rsCompatibilityMode));
    }
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
        ts.child_frame_id = getFrameName(tfPrefix, name + (rsCompatibilityMode ? "_frame" : "_camera_frame"));
        // check if the camera is at the end of the chain
        if(extrinsics["toCameraSocket"] != -1) {
            ts.header.frame_id =
                getFrameName(tfPrefix,
                             getSocketName(static_cast<dai::CameraBoardSocket>(extrinsics["toCameraSocket"].get<int>()), camModel, rsCompatibilityMode)
                                 + (rsCompatibilityMode ? "_frame" : "_camera_frame"));
        } else {
            ts.header.frame_id = baseFrame;
            ts.transform.rotation.w = 1.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
            const std::string socket = dai::toString(currCam);
            if(cameraPoses.contains(socket)) {
                const auto& pose = cameraPoses.at(socket);
                ts.transform.translation.x = pose.at("xyz").at(0).get<double>();
                ts.transform.translation.y = pose.at("xyz").at(1).get<double>();
                ts.transform.translation.z = pose.at("xyz").at(2).get<double>();
                tf2::Quaternion rotation;
                rotation.setRPY(pose.at("rpy").at(0).get<double>(), pose.at("rpy").at(1).get<double>(), pose.at("rpy").at(2).get<double>());
                ts.transform.rotation = tf2::toMsg(rotation);
            }
        }
        // rotate optical fransform
        opticalTS.child_frame_id = getOpticalFrameName(tfPrefix, name, rsCompatibilityMode);
        opticalTS.header.frame_id = ts.child_frame_id;
        opticalTS.transform.rotation.w = 0.5;
        opticalTS.transform.rotation.x = -0.5;
        opticalTS.transform.rotation.y = 0.5;
        opticalTS.transform.rotation.z = -0.5;
        tfPub->sendTransform(ts);
        tfPub->sendTransform(opticalTS);
        if(legacyFrameAliases == "true") {
            for(const auto& [canonical, deprecated] : detail::legacyCameraFrameAliases(tfPrefix, currCam, camModel, rsCompatibilityMode)) {
                if(!occupiedFrames.insert(deprecated).second) continue;
                geometry_msgs::msg::TransformStamped alias;
                alias.header = ts.header;
                alias.header.frame_id = canonical;
                alias.child_frame_id = deprecated;
                alias.transform.rotation.w = 1.0;
                tfPub->sendTransform(alias);
            }
        }
    }
}
void TFPublisher::publishImuTransform(nlohmann::json json, std::shared_ptr<rclcpp::Node> node, const dai::CalibrationHandler& calHandler) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = node->get_clock()->now();
    auto imuExtr = json["imuExtrinsics"];
    ts.child_frame_id = getFrameName(tfPrefix, "imu_frame");
    if(imuExtr["toCameraSocket"] != -1) {
        ts.header.frame_id =
            getFrameName(tfPrefix,
                         getSocketName(static_cast<dai::CameraBoardSocket>(imuExtr["toCameraSocket"].get<int>()), camModel, rsCompatibilityMode)
                             + (rsCompatibilityMode ? "_frame" : "_camera_frame"));
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

std::vector<std::string> TFPublisher::prepareXacroArgs() {
    // Processed IMU reports use optical/RDF axes, matching publishImuTransform.
    // Keep explicit native selection available for raw reports.
    const auto resolvedImuFrame = imuFrame == "auto" ? std::string("optical") : imuFrame;
    return {"camera_name:=" + tfPrefix,
            "camera_model:=" + camModel,
            "base_frame:=" + baseFrame,
            "parent_frame:=" + parentFrame,
            "cam_pos_x:=" + camPosX,
            "cam_pos_y:=" + camPosY,
            "cam_pos_z:=" + camPosZ,
            "cam_roll:=" + camRoll,
            "cam_pitch:=" + camPitch,
            "cam_yaw:=" + camYaw,
            "has_imu:=" + imuFromDescr,
            "imu_sensor:=" + imuSensor,
            "imu_frame:=" + resolvedImuFrame,
            "pose_source:=" + poseSource,
            "model_selection:=" + modelSelection,
            "fallback_model:=" + fallbackModel,
            "mesh_model:=" + meshModel,
            "legacy_frame_aliases:=" + legacyFrameAliases,
            "use_base_descr:=true"};
}

std::string TFPublisher::getURDF() {
    const auto share = ament_index_cpp::get_package_share_directory("depthai_descriptions");
    std::vector<std::string> arguments = {"python3", share + "/scripts/resolve_description.py", "--output=json", "--detected-imu=" + detectedImu};
    if(!customURDFLocation.empty()) arguments.push_back("--description=" + customURDFLocation);
    const auto mappings = prepareXacroArgs();
    arguments.insert(arguments.end(), mappings.begin(), mappings.end());
    if(!customXacroArgs.empty()) arguments.push_back("--custom-args=" + customXacroArgs);
    const auto result = nlohmann::json::parse(detail::runDescriptionCommand(arguments));
    const auto urdf = result.at("urdf").get<std::string>();
    if(urdf.empty()) throw std::runtime_error("Description resolver returned an empty URDF");
    cameraPoses = result.at("camera_poses");
    // Keep camModel as the physical device name for topic/socket naming.
    const auto diagnostics = result.value("diagnostics", std::string());
    if(!diagnostics.empty()) RCLCPP_WARN(logger, "%s", diagnostics.c_str());
    return urdf;
}
}  // namespace depthai_bridge
