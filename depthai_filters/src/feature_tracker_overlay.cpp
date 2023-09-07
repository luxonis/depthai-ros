#include "depthai_filters/feature_tracker_overlay.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

void FeatureTrackerOverlay::onInit() {
    auto pNH = getPrivateNodeHandle();
    imgSub.subscribe(pNH, "/rgb/preview/image_raw", 1);
    featureSub.subscribe(pNH, "/feature_tracker/tracked_features", 1);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), imgSub, featureSub);
    sync->registerCallback(std::bind(&FeatureTrackerOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = pNH.advertise<sensor_msgs::Image>("overlay", 10);
}

void FeatureTrackerOverlay::overlayCB(const sensor_msgs::ImageConstPtr& img, const depthai_ros_msgs::TrackedFeaturesConstPtr& features) {
    cv::Mat imgMat = utils::msgToMat(img, sensor_msgs::image_encodings::BGR8);
    std::vector<depthai_ros_msgs::TrackedFeature> f = features->features;
    trackFeaturePath(f);
    drawFeatures(imgMat);
    sensor_msgs::Image outMsg;
    cv_bridge::CvImage(img->header, sensor_msgs::image_encodings::BGR8, imgMat).toImageMsg(outMsg);

    overlayPub.publish(outMsg);
}

void FeatureTrackerOverlay::trackFeaturePath(std::vector<depthai_ros_msgs::TrackedFeature>& features) {
    std::unordered_set<featureIdType> newTrackedIDs;
    for(auto& currentFeature : features) {
        auto currentID = currentFeature.id;
        newTrackedIDs.insert(currentID);

        if(!trackedFeaturesPath.count(currentID)) {
            trackedFeaturesPath.insert({currentID, std::deque<geometry_msgs::Point>()});
        }
        std::deque<geometry_msgs::Point>& path = trackedFeaturesPath.at(currentID);

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
        std::deque<geometry_msgs::Point>& path = featurePath.second;
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
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::FeatureTrackerOverlay, nodelet::Nodelet)