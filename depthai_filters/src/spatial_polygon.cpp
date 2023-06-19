#include "depthai_filters/spatial_polygon.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"

namespace depthai_filters {

SpatialPolygon::SpatialPolygon(const rclcpp::NodeOptions& options) : rclcpp::Node("detection_overlay", options) {
    onInit();
}
void SpatialPolygon::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    depthSub.subscribe(this, "stereo/image_raw");
    infoSub.subscribe(this, "stereo/camera_info");
    detSub.subscribe(this, "nn/spatial_detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, depthSub, infoSub, detSub);
    sync->registerCallback(
        std::bind(&SpatialPolygon::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    polyPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("spatial_bb", 10);
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
}

void SpatialPolygon::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                               const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                               const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);
    cv::Mat depth_image = utils::msgToMat(this->get_logger(), depth, sensor_msgs::image_encodings::TYPE_16UC1);
    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);
    auto blue = cv::Scalar(255, 0, 0);
    auto ratio = 720.0 / 300.0;
    int offset_x = (1280.0 - 720.0) / 2;
    int offset_y = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    // Convert to meters using camera info
    double fx = info->k[0];  // Focal length in x (pixel unit)
    double fy = info->k[4];  // Focal length in y (pixel unit)
    double cx = info->k[2];  // Principal point in x (pixel unit)
    double cy = info->k[5];  // Principal point in y (pixel unit)
int id = 0;
    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size.x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size.x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size.y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size.y / 2.0;

        auto labelStr = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        auto confidence = detection.results[0].hypothesis.score;
        cv::putText(previewMat, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(previewMat, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        cv::putText(previewMat, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(previewMat, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);

        const auto& bbox = detection.bbox;

        // Remap bbox from 300x300 frame to 720x720 frame

        auto bbox_size_x = bbox.size.x * ratio;
        auto bbox_size_y = bbox.size.y * ratio;
        auto bbox_center_x = bbox.center.position.x * ratio + offset_x;
        auto bbox_center_y = bbox.center.position.y * ratio + offset_y;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = info->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "detections";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.1;  // Line width
        marker.color.r = 1.0;
        marker.color.a = 1.0;

        // Define bbox corner points in depth image frame
        geometry_msgs::msg::Point32 corners[4];
        corners[0].x = bbox_center_x - bbox_size_x / 2.0;
        corners[0].y = bbox_center_y - bbox_size_y / 2.0;
        corners[1].x = bbox_center_x + bbox_size_x / 2.0;
        corners[1].y = bbox_center_y - bbox_size_y / 2.0;
        corners[2].x = bbox_center_x + bbox_size_x / 2.0;
        corners[2].y = bbox_center_y + bbox_size_y / 2.0;
        corners[3].x = bbox_center_x - bbox_size_x / 2.0;
        corners[3].y = bbox_center_y + bbox_size_y / 2.0;

        // The polygon points are a rectangle, so we need 5 points to close the loop
        marker.points.resize(5);
        for (int i = 0; i < 4; ++i) {
            auto& point = corners[i];
            point.z = detection.results[0].pose.pose.position.z;
            marker.points[i].x = (point.x - cx) * point.z / fx;
            marker.points[i].y = (point.y - cy) * point.z / fy;
            marker.points[i].z = point.z;  // assuming depth_val is in millimeters
        }
        // Repeat the first point to close the loop
        marker.points[4] = marker.points[0];
        marker_array.markers.push_back(marker);
    }
    polyPub->publish(marker_array);
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SpatialPolygon);