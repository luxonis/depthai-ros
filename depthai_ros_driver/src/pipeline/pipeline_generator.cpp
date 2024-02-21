#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sys_logger.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {
std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(rclcpp::Node* node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    const std::string& pipelineType,
                                                                                    const std::string& nnType,
                                                                                    bool enableImu) {
    RCLCPP_INFO(node->get_logger(), "Pipeline type: %s", pipelineType.c_str());
    std::string pluginType = pipelineType;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    // Check if one of the default types.
    try {
        std::string pTypeUpCase = utils::getUpperCaseStr(pipelineType);
        auto pTypeValidated = validatePipeline(node, pTypeUpCase, device->getCameraSensorNames().size());
        pluginType = utils::getValFromMap(pTypeValidated, pluginTypeMap);
    } catch(std::out_of_range& e) {
        RCLCPP_DEBUG(node->get_logger(), "Pipeline type [%s] not found in base types, trying to load as a plugin.", pipelineType.c_str());
    }
    pluginlib::ClassLoader<BasePipeline> pipelineLoader("depthai_ros_driver", "depthai_ros_driver::pipeline_gen::BasePipeline");

    try {
        std::shared_ptr<BasePipeline> pipelinePlugin = pipelineLoader.createSharedInstance(pluginType);
        daiNodes = pipelinePlugin->createPipeline(node, device, pipeline, nnType);
    } catch(pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(node->get_logger(), "The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    if(enableImu) {
        if(device->getConnectedIMU() == "NONE" || device->getConnectedIMU().empty()) {
            RCLCPP_WARN(node->get_logger(), "IMU enabled but not available!");
        } else {
            auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device);
            daiNodes.push_back(std::move(imu));
        }
    }
    auto sysLogger = std::make_unique<dai_nodes::SysLogger>("sys_logger", node, pipeline);
    daiNodes.push_back(std::move(sysLogger));
    RCLCPP_INFO(node->get_logger(), "Finished setting up pipeline.");
    return daiNodes;
}

std::string PipelineGenerator::validatePipeline(rclcpp::Node* node, const std::string& typeStr, int sensorNum) {
    auto pType = utils::getValFromMap(typeStr, pipelineTypeMap);
    if(sensorNum == 1) {
        if(pType != PipelineType::RGB) {
            RCLCPP_ERROR(node->get_logger(), "Invalid pipeline chosen for camera as it has only one sensor. Switching to RGB.");
            return "RGB";
        }
    } else if(sensorNum == 2) {
        if(pType != PipelineType::Stereo && pType != PipelineType::Depth) {
            RCLCPP_ERROR(node->get_logger(), "Invalid pipeline chosen for camera as it has only stereo pair. Switching to Depth.");
            return "DEPTH";
        }
    }
    return typeStr;
}
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver