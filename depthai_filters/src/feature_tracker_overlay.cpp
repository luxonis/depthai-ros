#include "depthai_filters/feature_tracker_overlay.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

FeatureTrackerOverlay::FeatureTrackerOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("feature_overlay", options) {
    onInit();
}
void FeatureTrackerOverlay::onInit() {
    imgSub.subscribe(this, "rgb/preview/image_raw");
    featureSub.subscribe(this, "feature_tracker/tracked_features");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), imgSub, featureSub);
    sync->registerCallback(std::bind(&FeatureTrackerOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
}

void FeatureTrackerOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                                      const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& features) {
    cv::Mat imgMat = utils::msgToMat(this->get_logger(), img, sensor_msgs::image_encodings::BGR8);
    std::vector<depthai_ros_msgs::msg::TrackedFeature> f = features->features;
    trackFeaturePath(f);
    drawFeatures(imgMat);
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(img->header, sensor_msgs::image_encodings::BGR8, imgMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

void FeatureTrackerOverlay::trackFeaturePath(std::vector<depthai_ros_msgs::msg::TrackedFeature>& features) {
    std::unordered_set<featureIdType> newTrackedIDs;
    for(auto& currentFeature : features) {
        auto currentID = currentFeature.id;
        newTrackedIDs.insert(currentID);

        if(!trackedFeaturesPath.count(currentID)) {
            trackedFeaturesPath.insert({currentID, std::deque<geometry_msgs::msg::Point>()});
        }
        std::deque<geometry_msgs::msg::Point>& path = trackedFeaturesPath.at(currentID);

        path.push_back(currentFeature.position);
        while(path.size() > std::max<unsigned int>(1, trackedFeaturesPathLength)) {
            path.pop_front();
        }
    }

    std::unordered_set<featureIdType> featuresToRemove;
    for(auto& oldId : trackedIDs) {
        if(!newTrackedIDs.count(oldId)) {
            featuresToRemove.insert(oldId);
        }
    }

    for(auto& id : featuresToRemove) {
        trackedFeaturesPath.erase(id);
    }

    trackedIDs = newTrackedIDs;
}
void FeatureTrackerOverlay::drawFeatures(cv::Mat& img) {
    for(auto& featurePath : trackedFeaturesPath) {
        std::deque<geometry_msgs::msg::Point>& path = featurePath.second;
        unsigned int j = 0;
        for(j = 0; j < path.size() - 1; j++) {
            auto src = cv::Point(path[j].x, path[j].y);
            auto dst = cv::Point(path[j + 1].x, path[j + 1].y);
            cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
        }

        cv::circle(img, cv::Point(path[j].x, path[j].y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
    }
}
}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::FeatureTrackerOverlay);