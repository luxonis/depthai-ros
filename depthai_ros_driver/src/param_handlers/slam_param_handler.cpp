#include "depthai_ros_driver/param_handlers/slam_param_handler.hpp"
#include <depthai/common/CameraBoardSocket.hpp>


#include "depthai/rtabmap/RTABMapSLAM.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
SlamParamHandler::SlamParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
SlamParamHandler::~SlamParamHandler() = default;
void SlamParamHandler::declareParams(std::shared_ptr<dai::node::RTABMapSLAM> slam) {
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 2);
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    declareAndLogParam<bool>("i_publish_tf", true);
    declareAndLogParam<bool>("i_publish_absolute_pose", true);
    declareAndLogParam<bool>("i_publish_map", true);
    declareAndLogParam<bool>("i_publish_ground_pcl", true);
    declareAndLogParam<bool>("i_publish_obstacle_pcl", true);
    declareAndLogParam<std::string>("i_config_path", "");
    declareAndLogParam<std::string>("i_map_frame", "map");
    declareAndLogParam<std::string>("i_odom_frame", "odom");
    declareAndLogParam<bool>("i_use_external_odometry", false);
    declareAndLogParam<std::string>("i_external_odom_frame", "odom");
    declareAndLogParam<std::string>("i_external_base_frame", "oak_parent_frame");
    std::map<std::string, std::string> params;
    params.insert({"RGBD/CreateOccupancyGrid", "true"});
    params.insert({"Grid/3D", "true"});
    params.insert({"Rtabmap/SaveWMState", "true"});
    slam->setParams(params);
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
