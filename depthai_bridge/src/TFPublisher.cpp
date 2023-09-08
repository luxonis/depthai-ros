#include "depthai_bridge/TFPublisher.hpp"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "nlohmann/json.hpp"
#include "ros/node_handle.h"
#include "ros/package.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "urdf/model.h"

namespace dai {
namespace ros {
TFPublisher::TFPublisher(::ros::NodeHandle node,
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
                         const std::string& customXacroArgs)
    : _camName(camName),
      _camModel(camModel),
      _baseFrame(baseFrame),
      _parentFrame(parentFrame),
      _camPosX(camPosX),
      _camPosY(camPosY),
      _camPosZ(camPosZ),
      _camRoll(camRoll),
      _camPitch(camPitch),
      _camYaw(camYaw),
      _camFeatures(camFeatures),
      _imuFromDescr(imuFromDescr),
      _customURDFLocation(customURDFLocation),
      _customXacroArgs(customXacroArgs) {
    _tfPub = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    auto json = calHandler.eepromToJson();
    auto camData = json["cameraData"];
    publishDescription(node);
    publishCamTransforms(camData, node);
    if(_imuFromDescr != "true") {
        publishImuTransform(json, node);
    }
}

void TFPublisher::publishDescription(::ros::NodeHandle node) {
    auto urdf = getURDF();
    urdf::Model model;
    model.initString(urdf);
    KDL::Tree tree;
    if(!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR("Failed to extract kdl tree from xml robot description");
        throw std::runtime_error("Failed to extract kdl tree from xml robot description");
    }
    _rsp = std::make_shared<robot_state_publisher::RobotStatePublisher>(tree, model);
    _rsp->publishFixedTransforms(true);
    node.setParam("robot_description", urdf);
    ROS_INFO("Published URDF");
}

void TFPublisher::publishCamTransforms(nlohmann::json camData, ::ros::NodeHandle node) {
    for(auto& cam : camData) {
        geometry_msgs::TransformStamped ts;
        geometry_msgs::TransformStamped opticalTS;
        ts.header.stamp = ::ros::Time::now();
        opticalTS.header.stamp = ts.header.stamp;
        auto extrinsics = cam[1]["extrinsics"];

        ts.transform.rotation = quatFromRotM(extrinsics["rotationMatrix"]);
        ts.transform.translation = transFromExtr(extrinsics["translation"]);

        std::string name = getCamSocketName(cam[0]);
        ts.child_frame_id = _baseFrame + std::string("_") + name + std::string("_camera_frame");
        // check if the camera is at the end of the chain
        if(extrinsics["toCameraSocket"] != -1) {
            ts.header.frame_id = _baseFrame + std::string("_") + getCamSocketName(extrinsics["toCameraSocket"].get<int>()) + std::string("_camera_frame");
        } else {
            ts.header.frame_id = _baseFrame;
            ts.transform.rotation.w = 1.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
        }
        // rotate optical fransform
        opticalTS.child_frame_id = _baseFrame + std::string("_") + name + std::string("_camera_optical_frame");
        opticalTS.header.frame_id = ts.child_frame_id;
        opticalTS.transform.rotation.w = 0.5;
        opticalTS.transform.rotation.x = -0.5;
        opticalTS.transform.rotation.y = 0.5;
        opticalTS.transform.rotation.z = -0.5;
        _tfPub->sendTransform(ts);
        _tfPub->sendTransform(opticalTS);
    }
}
void TFPublisher::publishImuTransform(nlohmann::json json, ::ros::NodeHandle node) {
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ::ros::Time::now();
    auto imuExtr = json["imuExtrinsics"];
    if(imuExtr["toCameraSocket"] != -1) {
        ts.header.frame_id = _baseFrame + std::string("_") + getCamSocketName(imuExtr["toCameraSocket"].get<int>()) + std::string("_camera_frame");
    } else {
        ts.header.frame_id = _baseFrame;
    }
    ts.child_frame_id = _baseFrame + std::string("_imu_frame");

    ts.transform.rotation = quatFromRotM(imuExtr["rotationMatrix"]);
    ts.transform.translation = transFromExtr(imuExtr["translation"]);
    bool zeroTrans = ts.transform.translation.x == 0 && ts.transform.translation.y == 0 && ts.transform.translation.z == 0;
    bool zeroRot = ts.transform.rotation.w == 1 && ts.transform.rotation.x == 0 && ts.transform.rotation.y == 0 && ts.transform.rotation.z == 0;
    if(zeroTrans || zeroRot) {
        ROS_WARN("IMU extrinsics appear to be default. Check if the IMU is calibrated.");
        ts.transform.rotation.w = 1.0;
        ts.transform.rotation.x = 0.0;
    }
    _tfPub->sendTransform(ts);
}

std::string TFPublisher::getCamSocketName(int socketNum) {
    return _socketNameMap.at(static_cast<dai::CameraBoardSocket>(socketNum));
}

geometry_msgs::Vector3 TFPublisher::transFromExtr(nlohmann::json translation) {
    geometry_msgs::Vector3 trans;
    // optical coordinates to ROS
    trans.x = translation["y"].get<double>() / -100.0;
    trans.y = translation["x"].get<double>() / -100.0;
    trans.z = translation["z"].get<double>() / 100.0;
    return trans;
}
geometry_msgs::Quaternion TFPublisher::quatFromRotM(nlohmann::json rotMatrix) {
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
    geometry_msgs::Quaternion msg_quat = tf2::toMsg(q);
    return msg_quat;
}

bool TFPublisher::modelNameAvailable() {
    std::string path = ::ros::package::getPath("depthai_descriptions") + "/urdf/models/";
    DIR* dir;
    struct dirent* ent;
    convertModelName();
    if((dir = opendir(path.c_str())) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            auto name = std::string(ent->d_name);
            ROS_DEBUG("Found model: %s", name.c_str());
            if(name == _camModel + ".stl") {
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
    if(!_customURDFLocation.empty() || !modelNameAvailable()) {
        ROS_ERROR(
            "Model name %s not found in depthai_descriptions package. If camera model is autodetected, please notify developers. Using default model: OAK-D",
            _camModel.c_str());
        _camModel = "OAK-D";
    }

    std::string xacroArgs = "camera_name:=" + _camName;
    xacroArgs += " camera_model:=" + _camModel;
    xacroArgs += " base_frame:=" + _baseFrame;
    xacroArgs += " parent_frame:=" + _parentFrame;
    xacroArgs += " cam_pos_x:=" + _camPosX;
    xacroArgs += " cam_pos_y:=" + _camPosY;
    xacroArgs += " cam_pos_z:=" + _camPosZ;
    xacroArgs += " cam_roll:=" + _camRoll;
    xacroArgs += " cam_pitch:=" + _camPitch;
    xacroArgs += " cam_yaw:=" + _camYaw;
    xacroArgs += " has_imu:=" + _imuFromDescr;
    return xacroArgs;
}

void TFPublisher::convertModelName() {
    if(_camModel.find("OAK-D-PRO-POE") != std::string::npos || _camModel.find("OAK-D-PRO-W-POE") != std::string::npos
       || _camModel.find("OAK-D-S2-POE") != std::string::npos) {
        _camModel = "OAK-D-POE";
    } else if(_camModel.find("OAK-D-LITE") != std::string::npos) {
        _camModel = "OAK-D-PRO";
    } else if(_camModel.find("OAK-D-S2") != std::string::npos) {
        _camModel = "OAK-D-PRO";
    } else if(_camModel.find("OAK-D-PRO-W") != std::string::npos) {
        _camModel = "OAK-D-PRO";
    } else if(_camModel.find("OAK-D-PRO") != std::string::npos) {
        _camModel = "OAK-D-PRO";
    } else if(_camModel.find("OAK-D-POE")) {
        _camModel = "OAK-D-POE";
    } else if(_camModel.find("OAK-D") != std::string::npos) {
        _camModel = "OAK-D";
    } else {
        ROS_WARN("Unable to match model name: %s to available model family.", _camModel.c_str());
    }
}

std::string TFPublisher::getURDF() {
    std::string args, path;
    if(_customXacroArgs.empty()) {
        args = prepareXacroArgs();
    } else {
        args = _customXacroArgs;
    }
    if(_customURDFLocation.empty()) {
        path = ::ros::package::getPath("depthai_descriptions") + "/urdf/base_descr.urdf.xacro ";
    } else {
        path = _customURDFLocation + " ";
    }
    std::string cmd = "xacro " + path + args;
    ROS_DEBUG("Xacro command: %s", cmd.c_str());
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