#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"

namespace dai {
class Pipeline;
class Device;
class DataInputQueue;
namespace node {
class XLinkIn;
}
namespace ros {
class ImageConverter;
}
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler;
}
namespace dai_nodes {

class SensorWrapper : public BaseNode {
   public:
    explicit SensorWrapper(const std::string& daiNodeName,
                           ros::NodeHandle node,
                           std::shared_ptr<dai::Pipeline> pipeline,
                           std::shared_ptr<dai::Device> device,
                           dai::CameraBoardSocket socket,
                           bool publish = true);
    ~SensorWrapper();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    sensor_helpers::ImageSensor getSensorData();

   private:
    void subCB(const sensor_msgs::Image::ConstPtr& img);
    std::unique_ptr<BaseNode> sensorNode, featureTrackerNode, nnNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::unique_ptr<dai::ros::ImageConverter> converter;
    ros::Subscriber sub;
    std::shared_ptr<dai::node::XLinkIn> xIn;
    std::shared_ptr<dai::DataInputQueue> inQ;
    std::string inQName;
    int socketID;
    bool ready;
    sensor_helpers::ImageSensor sensorData;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver