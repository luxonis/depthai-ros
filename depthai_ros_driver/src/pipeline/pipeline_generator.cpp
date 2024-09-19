#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sync.hpp"
#include "depthai_ros_driver/dai_nodes/sys_logger.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace pipeline_gen {
PipelineGenerator::PipelineGenerator() {
    pluginTypeMap = {{"RGB", "depthai_ros_driver::pipeline_gen::RGB"},
                     {"RGBD", "depthai_ros_driver::pipeline_gen::RGBD"},
                     {"RGBSTEREO", "depthai_ros_driver::pipeline_gen::RGBStereo"},
                     {"STEREO", "depthai_ros_driver::pipeline_gen::Stereo"},
                     {"DEPTH", "depthai_ros_driver::pipeline_gen::Depth"},
                     {"CAMARRAY", "depthai_ros_driver::pipeline_gen::CamArray"},
                     {"DEPTHTOF", "depthai_ros_driver::pipeline_gen::DepthToF"},
                     {"STEREOTOF", "depthai_ros_driver::pipeline_gen::StereoToF"},
                     {"TOF", "depthai_ros_driver::pipeline_gen::ToF"},
                     {"RGBTOF", "depthai_ros_driver::pipeline_gen::RGBToF"}};
    pipelineTypeMap = {{"RGB", PipelineType::RGB},
                       {"RGBD", PipelineType::RGBD},
                       {"RGBSTEREO", PipelineType::RGBStereo},
                       {"STEREO", PipelineType::Stereo},
                       {"DEPTH", PipelineType::Depth},
                       {"CAMARRAY", PipelineType::CamArray},
                       {"DEPTHTOF", PipelineType::DepthToF},
                       {"STEREOTOF", PipelineType::StereoToF},
                       {"TOF", PipelineType::ToF},
                       {"RGBTOF", PipelineType::RGBToF}};
}

PipelineGenerator::~PipelineGenerator() = default;
std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(ros::NodeHandle node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    const std::string& pipelineType,
                                                                                    const std::string& nnType) {
    ph = std::make_unique<param_handlers::PipelineGenParamHandler>(node, "pipeline_gen");
    ph->declareParams();
    ROS_INFO("Pipeline type: %s", pipelineType.c_str());
    std::string pluginType = pipelineType;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    // Check if one of the default types.
    try {
        std::string pTypeUpCase = utils::getUpperCaseStr(pipelineType);
        auto pTypeValidated = validatePipeline(pTypeUpCase, device->getCameraSensorNames().size(), device->getDeviceName());
        pluginType = utils::getValFromMap(pTypeValidated, pluginTypeMap);
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

    if(ph->getParam<bool>("i_enable_imu")) {
        if(device->getConnectedIMU() == "NONE" || device->getConnectedIMU().empty()) {
            ROS_WARN("IMU enabled but not available!");
        } else {
            auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device);
            daiNodes.push_back(std::move(imu));
        }
    }
    if(ph->getParam<bool>("i_enable_sync")) {
        auto sync = std::make_unique<dai_nodes::Sync>("sync", node, pipeline);
        for(auto& daiNode : daiNodes) {
            auto pubs = daiNode->getPublishers();
            ROS_DEBUG("Number of synced publishers found for %s: %zu", daiNode->getName().c_str(), pubs.size());
            if(!pubs.empty()) {
                sync->addPublishers(pubs);
            }
        }
        daiNodes.push_back(std::move(sync));
    }
    if(ph->getParam<bool>("i_enable_diagnostics")) {
        auto sysLogger = std::make_unique<dai_nodes::SysLogger>("sys_logger", node, pipeline);
        daiNodes.push_back(std::move(sysLogger));
    }
    ROS_INFO("Finished setting up pipeline.");
    return daiNodes;
}

std::string PipelineGenerator::validatePipeline(const std::string& typeStr, int sensorNum, const std::string& deviceName) {
    auto pType = utils::getValFromMap(typeStr, pipelineTypeMap);
    if(deviceName == "OAK-D-SR-POE") {
        ROS_WARN("OAK-D-SR-POE device detected. Pipeline types other than StereoToF/ToF/RGBToF might not work without reconfiguration.");
    }
    if(sensorNum == 1) {
        if(pType != PipelineType::RGB) {
            ROS_ERROR("Invalid pipeline chosen for camera as it has only one sensor. Switching to RGB.");
            return "RGB";
        }
    } else if(sensorNum == 2) {
        if(pType != PipelineType::Stereo && pType != PipelineType::Depth) {
            ROS_ERROR("Invalid pipeline chosen for camera as it has only stereo pair. Switching to DEPTH.");
            return "DEPTH";
        }
    }
    return typeStr;
}

}  // namespace pipeline_gen
}  // namespace depthai_ros_driver
