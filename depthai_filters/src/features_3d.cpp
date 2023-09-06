#include "depthai_filters/features_3d.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "geometry_msgs/Point32.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/point_cloud2_iterator.h"

namespace depthai_filters {

void Features3D::onInit() {
    auto pNH = getPrivateNodeHandle();
    depthSub.subscribe(pNH, "/stereo/image_raw", 1);
    infoSub.subscribe(pNH, "/stereo/camera_info", 1);
    featureSub.subscribe(pNH, "/feature_tracker/tracked_features", 1);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), depthSub, infoSub, featureSub);
    sync->registerCallback(std::bind(&Features3D::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    pclPub = pNH.advertise<sensor_msgs::PointCloud2>("features", 10);
    overlayPub = pNH.advertise<sensor_msgs::Image>("overlay", 10);
    pNH.getParam("desqueeze", desqueeze);
}
float Features3D::getDepthAt(int x, int y, const sensor_msgs::ImageConstPtr& depth_image) {
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
void Features3D::overlayCB(const sensor_msgs::ImageConstPtr& depth,
                           const sensor_msgs::CameraInfoConstPtr& info,
                           const depthai_ros_msgs::TrackedFeaturesConstPtr& features) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = info->header.frame_id;  // Set this to your camera's frame
    cloud.header.stamp = ros::Time::now();
    cloud.height = 1;
    cloud.width = features->features.size();  // assuming features is your vector of 2D points
    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
    double fx = info->K[0];
    double fy = info->K[4];
    double cx = info->K[2];
    double cy = info->K[5];
    for(const auto& feature : features->features) {
        float depthVal = getDepthAt(feature.position.x, feature.position.y, depth);  // Define this function based on your depth image structure
        *out_x = (feature.position.x - cx) * depthVal / fx;
        *out_y = (feature.position.y - cy) * depthVal / fy;
        *out_z = depthVal;
        ++out_x;
        ++out_y;
        ++out_z;
    }
    pclPub.publish(cloud);
}

}  // namespace depthai_filters
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::Features3D, nodelet::Nodelet)