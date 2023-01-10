#pragma once

#include "depthai/depthai.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace depthai_ros_driver {
namespace dai_nodes {

class Imu : public BaseNode {
   public:
    explicit Imu(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~Imu() = default;
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImuConverter> imuConverter;
    void imuQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    ros::Publisher imuPub;
    std::shared_ptr<dai::node::IMU> imuNode;
    std::unique_ptr<param_handlers::ImuParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> imuQ;
    std::shared_ptr<dai::node::XLinkOut> xoutImu;
    std::string imuQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver