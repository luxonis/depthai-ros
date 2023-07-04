#include "depthai_ros_driver/tf_publisher.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace depthai_ros_driver {
TFPublisher::TFPublisher(rclcpp::Node* node, dai::CalibrationHandler handler) {
    tfPub = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    auto json = handler.eepromToJson();
    auto camData = json["cameraData"];
    std::unordered_map<int, std::string> socketNameMap{{0, "rgb"}, {1, "left"}, {2, "right"}};
    for(auto& cam : camData) {
        geometry_msgs::msg::TransformStamped ts;
        geometry_msgs::msg::TransformStamped optical_ts;
        ts.header.stamp = node->get_clock()->now();
        optical_ts.header.stamp = ts.header.stamp;
        tf2::Matrix3x3 m(cam[1]["extrinsics"]["rotationMatrix"][0][0],
                         cam[1]["extrinsics"]["rotationMatrix"][0][1],
                         cam[1]["extrinsics"]["rotationMatrix"][0][2],

                         cam[1]["extrinsics"]["rotationMatrix"][1][0],
                         cam[1]["extrinsics"]["rotationMatrix"][1][1],
                         cam[1]["extrinsics"]["rotationMatrix"][1][2],

                         cam[1]["extrinsics"]["rotationMatrix"][2][0],
                         cam[1]["extrinsics"]["rotationMatrix"][2][1],
                         cam[1]["extrinsics"]["rotationMatrix"][2][2]);

        tf2::Quaternion q;
        m.getRotation(q);
        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
        ts.transform.rotation = msg_quat;
        // optical coordinates to ROS
        ts.transform.translation.x = cam[1]["extrinsics"]["translation"]["y"];
        ts.transform.translation.x /= -100.0;
        ts.transform.translation.y = cam[1]["extrinsics"]["translation"]["x"];
        ts.transform.translation.y /= -100.0;
        ts.transform.translation.z = cam[1]["extrinsics"]["translation"]["z"];
        ts.transform.translation.z /= 100.0;

        std::string name = socketNameMap.at(cam[0]);
        ts.child_frame_id = node->get_name() + std::string("_") + name + std::string("_camera_frame");
        optical_ts.child_frame_id = node->get_name() + std::string("_") + name + std::string("_camera_optical_frame");
        optical_ts.header.frame_id = ts.child_frame_id;
        // check if the camera is at the end of the chain
        if(cam[1]["extrinsics"]["toCameraSocket"] != -1) {
            ts.header.frame_id = node->get_name() + std::string("_") + socketNameMap.at(cam[1]["extrinsics"]["toCameraSocket"]) + std::string("_camera_frame");
        } else {
            ts.header.frame_id = node->get_name();
            ts.transform.rotation.w = 1.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
        }
        // rotate optical fransform
        optical_ts.transform.rotation.w = 0.5;
        optical_ts.transform.rotation.x = -0.5;
        optical_ts.transform.rotation.y = 0.5;
        optical_ts.transform.rotation.z = -0.5;
        tfPub->sendTransform(ts);
        tfPub->sendTransform(optical_ts);
    }
}
}  // namespace depthai_ros_driver