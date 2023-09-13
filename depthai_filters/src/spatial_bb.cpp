#include "depthai_filters/spatial_bb.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"

namespace depthai_filters {

SpatialBB::SpatialBB(const rclcpp::NodeOptions& options) : rclcpp::Node("spatial_bb_node", options) {
    onInit();
}
void SpatialBB::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    infoSub.subscribe(this, "stereo/camera_info");
    detSub.subscribe(this, "nn/spatial_detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, infoSub, detSub);
    sync->registerCallback(std::bind(&SpatialBB::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    markerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("spatial_bb", 10);
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    desqueeze = this->declare_parameter<bool>("desqueeze", false);
    labelMap = this->declare_parameter<std::vector<std::string>>("label_map", labelMap);
}

void SpatialBB::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                          const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);
    auto blue = cv::Scalar(255, 0, 0);

    double ratioY = double(info->height) / double(previewMat.rows);
    double ratioX;
    int offsetX;
    if(desqueeze) {
        ratioX = double(info->width) / double(previewMat.cols);
        offsetX = 0;
    } else {
        ratioX = ratioY;
        offsetX = (info->width - info->height) / 2.0;
    }
    int offsetY = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    double fx = info->k[0];
    double fy = info->k[4];
    double cx = info->k[2];
    double cy = info->k[5];
    int id = 0;
    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size.x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size.x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size.y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size.y / 2.0;

        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
        auto labelStr = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        utils::addTextToFrame(previewMat, labelStr, x1 + 10, y1 + 10);
        auto confidence = detection.results[0].hypothesis.score;
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        utils::addTextToFrame(previewMat, confStr.str(), x1 + 10, y1 + 40);

        std::stringstream depthX;
        depthX << "X: " << detection.results[0].pose.pose.position.x << " mm";
        utils::addTextToFrame(previewMat, depthX.str(), x1 + 10, y1 + 60);

        std::stringstream depthY;
        depthY << "Y: " << detection.results[0].pose.pose.position.y << " mm";
        utils::addTextToFrame(previewMat, depthY.str(), x1 + 10, y1 + 75);
        std::stringstream depthZ;
        depthZ << "Z: " << detection.results[0].pose.pose.position.z << " mm";
        utils::addTextToFrame(previewMat, depthZ.str(), x1 + 10, y1 + 90);

        // Marker publishing
        const auto& bbox = detection.bbox;
        auto bbox_size_x = bbox.size.x * ratioX;
        auto bbox_size_y = bbox.size.y * ratioY;

        auto bbox_center_x = bbox.center.position.x * ratioX + offsetX;
        auto bbox_center_y = bbox.center.position.y * ratioY + offsetY;
        visualization_msgs::msg::Marker box_marker;
        box_marker.header.frame_id = info->header.frame_id;
        box_marker.header.stamp = this->get_clock()->now();
        box_marker.ns = "detections";
        box_marker.id = id++;
        box_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        box_marker.action = visualization_msgs::msg::Marker::ADD;

        box_marker.scale.x = 0.05;  // Line width
        box_marker.color.g = 1.0;
        box_marker.color.a = 1.0;

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
        box_marker.points.resize(5);
        for(int i = 0; i < 4; ++i) {
            auto& point = corners[i];
            point.z = detection.results[0].pose.pose.position.z;
            box_marker.points[i].x = (point.x - cx) * point.z / fx;
            box_marker.points[i].y = (point.y - cy) * point.z / fy;
            box_marker.points[i].z = point.z;  // assuming depth_val is in millimeters
        }
        // Repeat the first point to close the loop
        box_marker.points[4] = box_marker.points[0];
        marker_array.markers.push_back(box_marker);

        // Create a text marker for the label
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = info->header.frame_id;
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "detections_label";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.scale.z = 0.3;  // Text size
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        // Position the text above the bounding box
        text_marker.pose.position.x = box_marker.points[0].x;
        text_marker.pose.position.y = box_marker.points[0].y;
        text_marker.pose.position.z = box_marker.points[0].z + 0.1;  // Adjust this value to position the text above the box

        // Set the text to the detection label
        text_marker.text = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        marker_array.markers.push_back(text_marker);
    }
    markerPub->publish(marker_array);
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SpatialBB);