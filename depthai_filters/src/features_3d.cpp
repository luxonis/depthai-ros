#include "depthai_filters/features_3d.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace depthai_filters {

Features3D::Features3D(const rclcpp::NodeOptions& options) : rclcpp::Node("features3d", options) {
    onInit();
}
void Features3D::onInit() {
    depthSub.subscribe(this, "stereo/image_raw");
    infoSub.subscribe(this, "stereo/camera_info");
    featureSub.subscribe(this, "feature_tracker/tracked_features");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), depthSub, infoSub, featureSub);
    sync->registerCallback(std::bind(&Features3D::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    pclPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("features", 10);
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    desqueeze = this->declare_parameter<bool>("desqueeze", false);
}
float Features3D::getDepthAt(int x, int y, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image) {
    // Assuming depth image is of encoding type 16UC1 (16-bit depth, unsigned)
    int row_step = depth_image->step;        // bytes per row
    int pixel_index = y * row_step + x * 2;  // 2 bytes per pixel for 16-bit image

    // Get the 16-bit depth value
    uint16_t depth_raw = *(uint16_t*)(&depth_image->data[pixel_index]);

    // Convert to meters or appropriate unit (this depends on your depth sensor configuration)
    // Often, raw values are in millimeters, so convert to meters.
    float depth_meters = depth_raw * 0.001;

    return depth_meters;
}
void Features3D::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                           const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                           const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& features) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = info->header.frame_id;  // Set this to your camera's frame
    cloud.header.stamp = this->get_clock()->now();
    cloud.height = 1;
    cloud.width = features->features.size();  // assuming features is your vector of 2D points
    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
    double fx = info->k[0];
    double fy = info->k[4];
    double cx = info->k[2];
    double cy = info->k[5];
    for(const auto& feature : features->features) {
        float depthVal = getDepthAt(feature.position.x, feature.position.y, depth);  // Define this function based on your depth image structure
        *out_x = (feature.position.x - cx) * depthVal / fx;
        *out_y = (feature.position.y - cy) * depthVal / fy;
        *out_z = depthVal;
        ++out_x;
        ++out_y;
        ++out_z;
    }
    pclPub->publish(cloud);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::Features3D);