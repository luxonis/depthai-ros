#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace pipeline_gen {
std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(ros::NodeHandle node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    const std::string& pipelineType,
                                                                                    const std::string& nnType,
                                                                                    bool enableImu) {
    ROS_INFO("Pipeline type: %s", pipelineType.c_str());
    std::string pluginType = pipelineType;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    // Check if one of the default types.
    try {
        std::string pTypeUpCase = utils::getUpperCaseStr(pipelineType);
        auto pType = utils::getValFromMap(pTypeUpCase, pipelineTypeMap);
        pType = validatePipeline(pType, device->getCameraSensorNames().size());
        pluginType = utils::getValFromMap(pTypeUpCase, pluginTypeMap);
    } catch(std::out_of_range& e) {
        ROS_DEBUG("Pipeline type [%s] not found in base types, trying to load as a plugin.", pipelineType.c_str());
    }
    pluginlib::ClassLoader<BasePipeline> pipelineLoader("depthai_ros_driver", "depthai_ros_driver::pipeline_gen::BasePipeline");

    try {
        boost::shared_ptr<BasePipeline> pipelinePlugin = pipelineLoader.createInstance(pluginType);
        daiNodes = pipelinePlugin->createPipeline(node, device, pipeline, nnType);
    } catch(pluginlib::PluginlibException& ex) {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    if(enableImu) {
        if(device->getConnectedIMU() == "NONE") {
            ROS_WARN("IMU enabled but not available!");
        } else {
            auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device);
            daiNodes.push_back(std::move(imu));
        }
    }

    ROS_INFO("Finished setting up pipeline.");
    return daiNodes;
}

PipelineType PipelineGenerator::validatePipeline(PipelineType type, int sensorNum) {
    if(sensorNum == 1) {
        if(type != PipelineType::RGB) {
            ROS_ERROR("Wrong pipeline chosen for camera as it has only one sensor. Switching to RGB.");
            return PipelineType::RGB;
        }
    } else if(sensorNum == 2) {
        if(type != PipelineType::Stereo || type != PipelineType::Depth) {
            ROS_ERROR("Wrong pipeline chosen for camera as it has only stereo pair. Switching to Stereo.");
            return PipelineType::Stereo;
        }
    } else if(sensorNum > 3 && type != PipelineType::CamArray) {
        ROS_ERROR("For cameras with more than three sensors you can only use CamArray. Switching to CamArray.");
        return PipelineType::CamArray;
    }
    return type;
}

}  // namespace pipeline_gen
}  // namespace depthai_ros_driver