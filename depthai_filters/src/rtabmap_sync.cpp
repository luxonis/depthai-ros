#include "depthai_filters/rtabmap_sync.hpp"

#include "rtabmap_msgs/msg/key_point.hpp"
#include "rtabmap_msgs/msg/point3f.hpp"
namespace depthai_filters {

RTABMapSync::RTABMapSync(const rclcpp::NodeOptions & options) : rclcpp::Node("rtabmap_sync", options) {
    onInit();
}

void RTABMapSync::onInit() {
    rgbSub.subscribe(this, "rgb/image_raw");
    rgbInfoSub.subscribe(this, "rgb/camera_info");
    depthSub.subscribe(this, "stereo/image_raw");
    depthInfoSub.subscribe(this, "stereo/camera_info");
    featureSub.subscribe(this, "feature_tracker/tracked_features");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), rgbSub, rgbInfoSub, depthSub, depthInfoSub, featureSub);
    sync->registerCallback(std::bind(&RTABMapSync::syncCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    rgbdPub = this->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
}
float RTABMapSync::getDepthAt(int x, int y, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image) {
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
void RTABMapSync::syncCB(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& rgb_info,
                         const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depth_info,
                         const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& features) {
    auto rgbd = std::make_unique<rtabmap_msgs::msg::RGBDImage>();
    rgbd->header = rgb->header;
    rgbd->rgb_camera_info = *rgb_info;
    rgbd->depth_camera_info = *depth_info;
    rgbd->rgb = *rgb;
    rgbd->depth = *depth;
    for (const auto& feature : features->features) {
        rtabmap_msgs::msg::KeyPoint kp;
        kp.pt.x = feature.position.x;
        kp.pt.y = feature.position.y;
        rgbd->key_points.push_back(kp);
        rtabmap_msgs::msg::Point3f p;
        p.x = (feature.position.x - rgb_info->k[2]) * getDepthAt(feature.position.x, feature.position.y, depth) / rgb_info->k[0];
        p.y = (feature.position.y - rgb_info->k[5]) * getDepthAt(feature.position.x, feature.position.y, depth) / rgb_info->k[4];
        p.z = getDepthAt(feature.position.x, feature.position.y, depth);
    }
    rgbdPub->publish(std::move(rgbd));
}

}  // namespace depthai_filters

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::RTABMapSync)
