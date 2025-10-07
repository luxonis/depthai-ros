#include "depthai_ros_driver/param_handlers/slam_param_handler.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/rtabmap/RTABMapSLAM.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "nlohmann/json.hpp"
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
    declareAndLogParam<float>("i_frequency", 1.0);
    declareAndLogParam<bool>("i_publish_tf", true);
    declareAndLogParam<bool>("i_publish_absolute_pose", true);
    declareAndLogParam<bool>("i_publish_map", true);
    declareAndLogParam<bool>("i_publish_ground_pcl", true);
    declareAndLogParam<bool>("i_publish_obstacle_pcl", true);
    if(declareAndLogParam<bool>("i_override_local_transform", false)) {
        float x = declareAndLogParam<float>("i_local_transform_x", 0.0);
        float y = declareAndLogParam<float>("i_local_transform_y", 0.0);
        float z = declareAndLogParam<float>("i_local_transform_z", 0.0);
        float qx = declareAndLogParam<float>("i_local_transform_qx", 0.0);
        float qy = declareAndLogParam<float>("i_local_transform_qy", 0.0);
        float qz = declareAndLogParam<float>("i_local_transform_qz", 0.0);
        float qw = declareAndLogParam<float>("i_local_transform_qw", 1.0);
        auto transform = std::make_shared<dai::TransformData>(x, y, z, qx, qy, qz, qw);
        slam->setLocalTransform(transform);
    }

    declareAndLogParam<std::string>("i_map_frame", "map");
    declareAndLogParam<std::string>("i_odom_frame", "odom");
    declareAndLogParam<std::string>("i_base_frame", "oak_parent_frame");
    declareAndLogParam<bool>("i_use_external_odometry", false);
    declareAndLogParam<std::string>("i_external_odom_frame", "odom");
    declareAndLogParam<std::string>("i_external_base_frame", "oak_parent_frame");
    std::string databasePath = declareAndLogParam<std::string>("i_database_path", "");
    if(!databasePath.empty()) {
        slam->setDatabasePath(databasePath);
    }
    if(declareAndLogParam<bool>("i_load_database_on_start", false)) {
        slam->setLoadDatabaseOnStart(true);
    }
    if(declareAndLogParam<bool>("i_save_database_on_close", false)) {
        slam->setSaveDatabaseOnClose(true);
    }
    if(declareAndLogParam<bool>("i_save_database_periodically", false)) {
        slam->setSaveDatabasePeriodically(true);
    }
    float dataBaseSavePeriod = declareAndLogParam<float>("i_database_save_period", 30.0);
    slam->setSaveDatabasePeriod(dataBaseSavePeriod);
    float alphaScaling = declareAndLogParam<float>("i_alpha_scaling", -1.0);
    slam->setAlphaScaling(alphaScaling);

    std::map<std::string, std::string> params;
    std::string configPath = declareAndLogParam<std::string>("i_config_path", "depthai_ros_driver_default_rtabmap.json");
    if(configPath.find("depthai_ros_driver_default_rtabmap.json") != std::string::npos) {
        configPath = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/custom/" + configPath;
    }
    auto finalPath = declareAndLogParam<std::string>("config_path", configPath, true);

    if(!finalPath.empty()) {
        RCLCPP_DEBUG(getROSNode()->get_logger(), "Loading RTABMap config from %s", finalPath.c_str());
        std::ifstream f(configPath);
        json data = json::parse(f);
        params = data.get<std::map<std::string, std::string>>();
    }
    slam->setParams(params);
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
