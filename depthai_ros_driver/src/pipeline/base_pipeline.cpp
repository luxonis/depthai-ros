#include "depthai_ros_driver/pipeline/base_pipeline.hpp"

#include "depthai/device/Device.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/rgbd.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"

namespace depthai_ros_driver {

namespace pipeline_gen {

bool BasePipeline::checkForImu(std::shared_ptr<param_handlers::PipelineGenParamHandler> ph, std::shared_ptr<dai::Device> device, rclcpp::Logger logger) {
    if(ph->getParam<bool>("i_enable_imu")) {
        if(device->getConnectedIMU() == "NONE" || device->getConnectedIMU().empty()) {
            RCLCPP_WARN(logger, "IMU enabled but not available!");
            return false;
        }
        return true;
    }
    return false;
}
void BasePipeline::addRgbdNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                               std::shared_ptr<rclcpp::Node> node,
                               std::shared_ptr<dai::Device> device,
                               std::shared_ptr<dai::Pipeline> pipeline,
                               std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                               bool rsCompat,
                               dai_nodes::SensorWrapper& rgb,
                               dai_nodes::Stereo& stereo) {
    if(ph->getParam<bool>("i_enable_rgbd")) {
        auto rgbd = std::make_unique<dai_nodes::RGBD>("rgbd", node, pipeline, device, rsCompat, rgb, stereo.getUnderlyingNode(), stereo.isAligned());
        if(device->getPlatform() == dai::Platform::RVC4) {
            stereo.link(rgbd->getInput(static_cast<int>(dai_nodes::link_types::RGBDLinkType::depth)),
                        static_cast<int>(dai_nodes::link_types::StereoLinkType::stereo));
        }
        daiNodes.push_back(std::move(rgbd));
    }
}
void BasePipeline::addRgbdNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                               std::shared_ptr<rclcpp::Node> node,
                               std::shared_ptr<dai::Device> device,
                               std::shared_ptr<dai::Pipeline> pipeline,
                               std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                               bool rsCompat,
                               dai_nodes::SensorWrapper& rgb,
                               dai_nodes::ToF& tof) {
    if(ph->getParam<bool>("i_enable_rgbd")) {
        auto rgbd = std::make_unique<dai_nodes::RGBD>("rgbd", node, pipeline, device, rsCompat, rgb, tof, tof.isAligned());
        if(tof.isAligned()) {
            tof.link(rgbd->getInput(static_cast<int>(dai_nodes::link_types::RGBDLinkType::depth)));
        }
        daiNodes.push_back(std::move(rgbd));
    }
}
void BasePipeline::addNnNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                             std::shared_ptr<rclcpp::Node> node,
                             std::shared_ptr<dai::Pipeline> pipeline,
                             const std::string& deviceName,
                             bool rsCompat,
                             dai_nodes::SensorWrapper& sensor,
                             const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = std::make_unique<dai_nodes::NNWrapper>(
                getNodeName(node, dai_nodes::sensor_helpers::NodeNameEnum::NN), node, pipeline, deviceName, rsCompat, sensor);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            RCLCPP_WARN(node->get_logger(), "Spatial NN selected, but configuration is not compatible. Please change camera.i_nn_type parameter to RGB.");
        }
        default:
            break;
    }
}
void BasePipeline::addNnNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                             std::shared_ptr<rclcpp::Node> node,
                             std::shared_ptr<dai::Pipeline> pipeline,
                             const std::string& deviceName,
                             bool rsCompat,
                             dai_nodes::SensorWrapper& sensor,
                             dai_nodes::Stereo& stereo,
                             const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = std::make_unique<dai_nodes::NNWrapper>(
                getNodeName(node, dai_nodes::sensor_helpers::NodeNameEnum::NN), node, pipeline, deviceName, rsCompat, sensor);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            auto nn = std::make_unique<dai_nodes::SpatialNNWrapper>(
                getNodeName(node, dai_nodes::sensor_helpers::NodeNameEnum::NN), node, pipeline, deviceName, rsCompat, sensor, stereo);
            daiNodes.push_back(std::move(nn));
            break;
        }
        default:
            break;
    }
}
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver
