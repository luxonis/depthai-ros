#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"

namespace depthai_bridge {

class TrackedFeaturesConverter : public BaseConverter {
   public:
    explicit TrackedFeaturesConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    ~TrackedFeaturesConverter();
    void setImageSize(int width, int height) {
        this->width = width;
        this->height = height;
    }

    void toRosMsg(std::shared_ptr<dai::TrackedFeatures> inFeatures, std::deque<depthai_ros_msgs::msg::TrackedFeatures>& featureMsgs);

   private:
    int width = 0, height = 0;
};

}  // namespace depthai_bridge
