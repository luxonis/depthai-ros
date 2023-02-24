#pragma once

#include "camera_info_manager/camera_info_manager.h"
#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace depthai_ros_driver {
namespace dai_nodes {

class SubscriberSensor : public BaseNode {
   public:
    explicit SubscriberSensor(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~SubscriberSensor() = default;
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    std::shared_ptr<dai::DataInputQueue> inputQ;
    std::shared_ptr<dai::node::XLinkIn> xinFrame;
    std::string inputQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver