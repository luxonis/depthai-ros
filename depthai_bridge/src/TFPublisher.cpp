#include "depthai_bridge/TFPublisher.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace dai {
namespace ros {
TFPublisher::TFPublisher(rclcpp::Node* node, dai::CalibrationHandler handler, const std::string& xacroArgs) {
    tfPub = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    auto json = handler.eepromToJson();
    auto camData = json["cameraData"];
    std::unordered_map<int, std::string> socketNameMap{{0, "rgb"}, {1, "left"}, {2, "right"}};
    paramClient = std::make_unique<rclcpp::AsyncParametersClient>(node, node->get_name() + std::string("_state_publisher"));
    auto urdf = getXacro(xacroArgs);
    auto urdf_param = rclcpp::Parameter("robot_description", urdf);

    paramClient->set_parameters({urdf_param});
    for(auto& cam : camData) {
        geometry_msgs::msg::TransformStamped ts;
        geometry_msgs::msg::TransformStamped optical_ts;
        ts.header.stamp = node->get_clock()->now();
        optical_ts.header.stamp = ts.header.stamp;
        auto extrinsics = cam[1]["extrinsics"];

        ts.transform.rotation = quatFromRotM(extrinsics["rotationMatrix"]);
        ts.transform.translation = transFromExtr(extrinsics["translation"]);

        std::string name = socketNameMap.at(cam[0]);
        ts.child_frame_id = node->get_name() + std::string("_") + name + std::string("_camera_frame");

        // check if the camera is at the end of the chain
        if(extrinsics["toCameraSocket"] != -1) {
            ts.header.frame_id = node->get_name() + std::string("_") + socketNameMap.at(extrinsics["toCameraSocket"]) + std::string("_camera_frame");
        } else {
            ts.header.frame_id = node->get_name();
            ts.transform.rotation.w = 1.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
        }
        // rotate optical fransform
        optical_ts.child_frame_id = node->get_name() + std::string("_") + name + std::string("_camera_optical_frame");
        optical_ts.header.frame_id = ts.child_frame_id;
        optical_ts.transform.rotation.w = 0.5;
        optical_ts.transform.rotation.x = -0.5;
        optical_ts.transform.rotation.y = 0.5;
        optical_ts.transform.rotation.z = -0.5;
        tfPub->sendTransform(ts);
        tfPub->sendTransform(optical_ts);
    }

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = node->get_clock()->now();
    auto imuExtr = json["imuExtrinsics"];
    if(imuExtr["toCameraSocket"] != -1) {
        ts.header.frame_id = node->get_name() + std::string("_") + socketNameMap.at(imuExtr["toCameraSocket"]) + std::string("_camera_frame");
    } else {
        ts.header.frame_id = node->get_name();
        ts.transform.rotation.w = 1.0;
        ts.transform.rotation.x = 0.0;
        ts.transform.rotation.y = 0.0;
        ts.transform.rotation.z = 0.0;
    }
    ts.child_frame_id = node->get_name() + std::string("_imu_frame");

    ts.transform.rotation = quatFromRotM(imuExtr["rotationMatrix"]);
    ts.transform.translation = transFromExtr(imuExtr["translation"]);
    tfPub->sendTransform(ts);
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
std::string TFPublisher::getXacro(const std::string& xacroArgs) {
    auto path = ament_index_cpp::get_package_share_directory("depthai_descriptions");
    std::string cmd = "xacro /workspaces/ros_2/src/depthai-ros/depthai_descriptions/urdf/base_descr.urdf.xacro ";
    cmd += xacroArgs;
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