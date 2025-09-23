#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_msgs/srv/set_local_transform.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dai {
class Pipeline;
class Device;
class ADatatype;
class MessageQueue;
class InputQueue;
namespace node {
class RTABMapSLAM;
}  // namespace node
}  // namespace dai

namespace depthai_bridge {
class TransformDataConverter;
class GridMapConverter;
};  // namespace depthai_bridge

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp
namespace tf2_ros {
class TransformBroadcaster;
class TransformListener;
class Buffer;
}  // namespace tf2_ros

namespace depthai_ros_driver {
namespace param_handlers {
class SlamParamHandler;
}
namespace dai_nodes {
namespace link_types {
enum class SlamLinkType { rect, depth, transform, map, groundPcl, obstaclePcl };
};
class SensorWrapper;
class Stereo;
class Imu;
class Vio;
class Slam : public BaseNode {
   public:
    explicit Slam(const std::string& daiNodeName,
                  std::shared_ptr<rclcpp::Node> node,
                  std::shared_ptr<dai::Pipeline> pipeline,
                  std::shared_ptr<dai::Device> device,
                  bool rsCompat,
                  SensorWrapper& sens,
                  Vio& vio,
                  Stereo& stereo);
    ~Slam();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input& getInput(int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::shared_ptr<dai::node::RTABMapSLAM> getUnderlyingNode();
    dai::CameraBoardSocket getSocketID();

   private:
    void mapToOdomCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void absolutePoseCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void mapCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void groundPclCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void obstaclePclCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    void triggerNewMapCB(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res);
    void saveDatabaseCB(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res);
    void setLocalTransformCB(const depthai_ros_msgs::srv::SetLocalTransform::Request::SharedPtr req,
                             depthai_ros_msgs::srv::SetLocalTransform::Response::SharedPtr res);

    void tfTimerCB();
    std::unique_ptr<depthai_bridge::TransformDataConverter> mapToOdomConv;
    std::unique_ptr<depthai_bridge::TransformDataConverter> absolutePoseConv;
    std::unique_ptr<depthai_bridge::GridMapConverter> mapConv;
    std::unique_ptr<depthai_bridge::PointCloudConverter> groundPclConv;
    std::unique_ptr<depthai_bridge::PointCloudConverter> obstaclePclConv;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groundPclPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstaclePclPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr absolutePosePub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr triggerNewMapSrv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveDatabaseSrv;
    rclcpp::Service<depthai_ros_msgs::srv::SetLocalTransform>::SharedPtr setLocalTransformSrv;
    std::shared_ptr<dai::node::RTABMapSLAM> slamNode;
    std::unique_ptr<param_handlers::SlamParamHandler> ph;
    std::shared_ptr<dai::MessageQueue> mapToOdomQ, absolutePoseQ, mapQ, groundPclQ, obstaclePclQ;
    std::shared_ptr<dai::InputQueue> externalOdomQ;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBr;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    rclcpp::TimerBase::SharedPtr tfTimer;

    std::string mapFrame, odomFrame, baseFrame, externalOdomFrame, externalBaseFrame;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
