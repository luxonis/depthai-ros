#include "depthai_filters/spatial_polygon.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace depthai_filters {

SpatialPolygon::SpatialPolygon(const rclcpp::NodeOptions& options) : rclcpp::Node("detection_overlay", options) {
    onInit();
}
void SpatialPolygon::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    depthSub.subscribe(this, "stereo/image_raw");
    infoSub.subscribe(this, "stereo/camera_info");
    detSub.subscribe(this, "nn/detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, depthSub, infoSub, detSub);
    sync->registerCallback(
        std::bind(&SpatialPolygon::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    overlayPub = this->create_publisher<geometry_msgs::msg::Polygon>("polygon", 10);
}

void SpatialPolygon::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                               const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                               const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);
    cv::Mat depthMat = utils::msgToMat(this->get_logger(), depth, sensor_msgs::image_encodings::TYPE_16UC1);

    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size_y / 2.0;
        
        //         Factory function to create a pointcloud from an RGB-D image and a camera. Given depth value d at (u, v) image coordinate, the corresponding
        //         3d point is:

        // z = d / depth_scale

        // x = (u - cx) * z / fx

        // y = (v - cy) * z / fy
        geometry_msgs::msg::Polygon polygon;
        geometry_msgs::msg::Point32 p1, p2, p3, p4;
        double z = 3.0;
        p1.x = (x1 - info->k[3]) * z / info->k[0];
        p1.y = (y1 - info->k[5]) * z / info->k[4];
        p1.z = z;
        polygon.points.push_back(p1);

        p2.x = (x1 - info->k[3]) * z / info->k[0];
        p2.y = (y2 - info->k[5]) * z / info->k[4];
        p2.z = z;
        polygon.points.push_back(p2);
        p3.x = (x2 - info->k[3]) * z / info->k[0];
        p3.y = (y2 - info->k[5]) * z / info->k[4];
        p3.z = z;
        polygon.points.push_back(p3);
        p4.x = (x2 - info->k[3]) * z / info->k[0];
        p4.y = (y1 - info->k[5]) * z / info->k[4];
        p4.z = z;
        polygon.points.push_back(p4);
        overlayPub->publish(polygon);
    }
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SpatialPolygon);