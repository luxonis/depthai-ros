#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "depthai_ros_driver/parametersConfig.h"


namespace depthai_ros_driver {
namespace dai_nodes {

class CameraSensor : public BaseNode {
   public:
    explicit CameraSensor(const std::string& daiNodeName,
                          ros::NodeHandle node,
                          std::shared_ptr<dai::Pipeline> pipeline,
                          std::shared_ptr<dai::Device> device,
                          dai::CameraBoardSocket socket);
    virtual ~CameraSensor() = default;
    void updateParams(parametersConfig &config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<BaseNode> sensorNode;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver