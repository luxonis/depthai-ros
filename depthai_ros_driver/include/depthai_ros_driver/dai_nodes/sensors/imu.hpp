#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "ros/publisher.h"
#include "sensor_msgs/Imu.h"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class IMU;
class XLinkOut;
}  // namespace node
namespace ros {
class ImuConverter;
}
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class ImuParamHandler;
}
namespace dai_nodes {

class Imu : public BaseNode {
   public:
    explicit Imu(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device);
    ~Imu();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImuConverter> imuConverter;
    void imuRosQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void imuDaiRosQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void imuMagQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    ros::Publisher rosImuPub, daiImuPub, magPub;
    std::shared_ptr<dai::node::IMU> imuNode;
    std::unique_ptr<param_handlers::ImuParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> imuQ;
    std::shared_ptr<dai::node::XLinkOut> xoutImu;
    std::string imuQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver