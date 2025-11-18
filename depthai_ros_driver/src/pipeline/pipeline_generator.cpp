#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"

#include <stdexcept>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sync.hpp"
#include "depthai_ros_driver/dai_nodes/sys_logger.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {
PipelineGenerator::PipelineGenerator()
    : pipelineLoader(std::make_shared<pluginlib::ClassLoader<BasePipeline>>("depthai_ros_driver", "depthai_ros_driver::pipeline_gen::BasePipeline")) {
    pluginTypeMap = {{"RGB", "depthai_ros_driver::pipeline_gen::RGB"},
                     {"RGBD", "depthai_ros_driver::pipeline_gen::RGBD"},
                     {"RGBSTEREO", "depthai_ros_driver::pipeline_gen::RGBStereo"},
                     {"STEREO", "depthai_ros_driver::pipeline_gen::Stereo"},
                     {"DEPTH", "depthai_ros_driver::pipeline_gen::Depth"},
                     {"CAMARRAY", "depthai_ros_driver::pipeline_gen::CamArray"},
                     {"DEPTHTOF", "depthai_ros_driver::pipeline_gen::DepthToF"},
                     {"STEREOTOF", "depthai_ros_driver::pipeline_gen::StereoToF"},
                     {"TOF", "depthai_ros_driver::pipeline_gen::ToF"},
                     {"RGBTOF", "depthai_ros_driver::pipeline_gen::RGBToF"},
                     {"THERMAL", "depthai_ros_driver::pipeline_gen::Thermal"}};
    pipelineTypeMap = {{"RGB", PipelineType::RGB},
                       {"RGBD", PipelineType::RGBD},
                       {"RGBSTEREO", PipelineType::RGBStereo},
                       {"STEREO", PipelineType::Stereo},
                       {"DEPTH", PipelineType::Depth},
                       {"CAMARRAY", PipelineType::CamArray},
                       {"DEPTHTOF", PipelineType::DepthToF},
                       {"STEREOTOF", PipelineType::StereoToF},
                       {"TOF", PipelineType::ToF},
                       {"RGBTOF", PipelineType::RGBToF},
                       {"THERMAL", PipelineType::Thermal}};
}

PipelineGenerator::~PipelineGenerator() = default;
void PipelineGenerator::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                       std::shared_ptr<dai::Device> device,
                                       std::shared_ptr<dai::Pipeline> pipeline,
                                       bool rsCompat) {
    auto deviceName = device->getDeviceName();
    RCLCPP_INFO(node->get_logger(), "Creating pipeline for device: %s", deviceName.c_str());
    ph = std::make_shared<param_handlers::PipelineGenParamHandler>(node, "pipeline_gen", deviceName, rsCompat);
    ph->declareParams();
    auto pipelineType = ph->getParam<std::string>("i_pipeline_type");
    auto nnType = ph->getParam<std::string>("i_nn_type");
    RCLCPP_INFO(node->get_logger(), "Pipeline type: %s", pipelineType.c_str());
    std::string pluginType = pipelineType;
    // Check if one of the default types.
    try {
        std::string pTypeUpCase = utils::getUpperCaseStr(pipelineType);
        auto pTypeValidated = validatePipeline(node, pTypeUpCase, device->getCameraSensorNames().size(), device->getDeviceName());
        pluginType = utils::getValFromMap(pTypeValidated, pluginTypeMap);
    } catch(std::out_of_range& e) {
        RCLCPP_DEBUG(node->get_logger(), "Pipeline type [%s] not found in base types, trying to load as a plugin.", pipelineType.c_str());
    }

    try {
        auto pipelinePlugin = pipelineLoader->createSharedInstance(pluginType);
        daiNodes = pipelinePlugin->createPipeline(node, device, pipeline, ph, deviceName, rsCompat, nnType);
        if(daiNodes.empty()) {
            throw std::runtime_error("No nodes created in this pipeline");
        }
    } catch(pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(node->get_logger(), "The plugin failed to load for some reason. Error: %s\n", ex.what());
        throw std::runtime_error("Plugin loading failed.");
    }

    if(ph->getParam<bool>("i_enable_diagnostics")) {
        if(device->getPlatform() == dai::Platform::RVC2) {
            auto sysLogger = std::make_unique<dai_nodes::SysLogger>("sys_logger", node, pipeline, deviceName, rsCompat);
            daiNodes.push_back(std::move(sysLogger));
        } else {
            RCLCPP_WARN(node->get_logger(), "Diagnostics not yet available on RVC4.");
        }
    }
    bool enableSync = false;
    std::unique_ptr<dai_nodes::Sync> sync;
    for(const auto& daiNode : daiNodes) {
        auto pubs = daiNode->getPublishers();
        for(auto& pub : pubs) {
            if(!enableSync && pub->isSynced()) {
                enableSync = true;
                RCLCPP_DEBUG(node->get_logger(), "Found synced publisher, creating Sync node.");
                sync = std::make_unique<dai_nodes::Sync>("sync", node, pipeline, deviceName, rsCompat);
            }
        }
        RCLCPP_DEBUG(node->get_logger(), "Number of publishers found for %s: %zu", daiNode->getName().c_str(), pubs.size());
        if(!pubs.empty() && enableSync) {
            sync->addPublishers(pubs);
        }
    }
    if(enableSync) {
        daiNodes.push_back(std::move(sync));
    }
    for(const auto& node : daiNodes) {
        node->setupQueues(device);
    }
    RCLCPP_INFO(node->get_logger(), "Finished setting up pipeline.");
}

void PipelineGenerator::updateParams(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& node : daiNodes) {
        node->updateParams(params);
    }
}
std::string PipelineGenerator::validatePipeline(std::shared_ptr<rclcpp::Node> node, const std::string& typeStr, int sensorNum, const std::string& deviceName) {
    auto pType = utils::getValFromMap(typeStr, pipelineTypeMap);
    if(deviceName == "OAK-D-SR-POE") {
        RCLCPP_WARN(node->get_logger(), "OAK-D-SR-POE device detected. Pipeline types other than StereoToF/ToF/RGBToF might not work without reconfiguration.");
    }
    if(sensorNum == 1) {
        if(pType != PipelineType::RGB) {
            RCLCPP_ERROR(node->get_logger(), "Invalid pipeline chosen for camera as it has only one sensor. This can result in undefined behavior.");
        }
    } else if(sensorNum == 2) {
        if(pType != PipelineType::Stereo && pType != PipelineType::Depth && pType != PipelineType::CamArray && pType != PipelineType::Thermal) {
            RCLCPP_ERROR(node->get_logger(), "Invalid pipeline chosen for camera as it has only stereo pair. This can result in undefined behavior.");
        }
    }
    return typeStr;
}
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver
