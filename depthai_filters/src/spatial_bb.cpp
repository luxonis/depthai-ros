#include "depthai_filters/spatial_bb.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/duration.hpp"

namespace depthai_filters {

SpatialBB::SpatialBB(const rclcpp::NodeOptions& options) : rclcpp::Node("spatial_bb_node", options) {
    onInit();
}
void SpatialBB::onInit() {
    previewSub.subscribe(this, "nn/passthrough/image_raw");
    infoSub.subscribe(this, "stereo/camera_info");
    detSub.subscribe(this, "nn/spatial_detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(), previewSub, infoSub, detSub);
    sync->registerCallback(std::bind(&SpatialBB::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    markerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("spatial_bb", 10);
}

void SpatialBB::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                          const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);

    visualization_msgs::msg::MarkerArray markerArray;
    double fx = info->k[0];
    double fy = info->k[4];
    double cx = info->k[2];
    double cy = info->k[5];
    int id = 0;
    for(auto& detection : detections->detections) {
        const auto& bbox = detection.bbox;
        auto bboxSizeX = bbox.size.x;
        auto bboxSizeY = bbox.size.y;

        auto bboxCenterX = bbox.center.position.x;
        auto bboxCenterY = bbox.center.position.y;

        // Marker publishing
        visualization_msgs::msg::Marker boxMarker;
        boxMarker.header.frame_id = info->header.frame_id;
        boxMarker.header.stamp = this->get_clock()->now();
        boxMarker.ns = "detections";
        boxMarker.id = id++;
        boxMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        boxMarker.action = visualization_msgs::msg::Marker::ADD;

        boxMarker.scale.x = 0.05;  // Line width
        boxMarker.color.g = 1.0;
        boxMarker.color.a = 1.0;

        // Define bbox corner points in depth image frame
        geometry_msgs::msg::Point32 corners[4];
        corners[0].x = bboxCenterX - bboxSizeX / 2.0;
        corners[0].y = bboxCenterY - bboxSizeY / 2.0;
        corners[1].x = bboxCenterX + bboxSizeX / 2.0;
        corners[1].y = bboxCenterY - bboxSizeY / 2.0;
        corners[2].x = bboxCenterX + bboxSizeX / 2.0;
        corners[2].y = bboxCenterY + bboxSizeY / 2.0;
        corners[3].x = bboxCenterX - bboxSizeX / 2.0;
        corners[3].y = bboxCenterY + bboxSizeY / 2.0;
        // The polygon points are a rectangle, so we need 5 points to close the loop
        boxMarker.points.resize(5);
        for(int i = 0; i < 4; ++i) {
            auto& point = corners[i];
            point.z = detection.results[0].pose.pose.position.z;
            boxMarker.points[i].x = (point.x - cx) * point.z / fx;
            boxMarker.points[i].y = (point.y - cy) * point.z / fy;
            boxMarker.points[i].z = point.z;
        }
        // Repeat the first point to close the loop
        boxMarker.points[4] = boxMarker.points[0];
        boxMarker.lifetime = rclcpp::Duration::from_seconds(0.1);
        markerArray.markers.push_back(boxMarker);

        // Create a text marker for the label
        visualization_msgs::msg::Marker textMarker;
        textMarker.header.frame_id = info->header.frame_id;
        textMarker.header.stamp = this->get_clock()->now();
        textMarker.ns = "detections_label";
        textMarker.id = id++;
        textMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::msg::Marker::ADD;
        textMarker.lifetime = rclcpp::Duration::from_seconds(0.1);

        textMarker.scale.z = 0.3;  // Text size
        textMarker.color.r = 1.0;
        textMarker.color.g = 1.0;
        textMarker.color.b = 1.0;
        textMarker.color.a = 1.0;

        // Position the text above the bounding box
        textMarker.pose.position.x = boxMarker.points[0].x;
        textMarker.pose.position.y = boxMarker.points[0].y;
        textMarker.pose.position.z = boxMarker.points[0].z + 0.1;  // Adjust this value to position the text above the box

        // Set the text to the detection label
        textMarker.text = detection.results[0].hypothesis.class_id;
        markerArray.markers.push_back(textMarker);
    }
    markerPub->publish(markerArray);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SpatialBB);
