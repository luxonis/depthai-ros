#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sync.hpp"
#include "depthai_ros_driver/dai_nodes/sys_logger.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"

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
std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    const std::string& pipelineType,
                                                                                    const std::string& nnType) {
    ph = std::make_unique<param_handlers::PipelineGenParamHandler>(node, "pipeline_gen");
    ph->declareParams();
    RCLCPP_INFO(node->get_logger(), "Pipeline type: %s", pipelineType.c_str());
    std::string pluginType = pipelineType;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    // Check if one of the default types.
    try {
        std::string pTypeUpCase = utils::getUpperCaseStr(pipelineType);
        auto pTypeValidated = validatePipeline(node, pTypeUpCase, device->getCameraSensorNames().size(), device->getDeviceName());
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
        throw std::runtime_error("Plugin loading failed.");
    }

    if(ph->getParam<bool>("i_enable_imu")) {
        if(device->getConnectedIMU() == "NONE" || device->getConnectedIMU().empty()) {
            RCLCPP_WARN(node->get_logger(), "IMU enabled but not available!");
        } else {
            auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device);
            daiNodes.push_back(std::move(imu));
        }
    }
    if(ph->getParam<bool>("i_enable_diagnostics")) {
        auto sysLogger = std::make_unique<dai_nodes::SysLogger>("sys_logger", node, pipeline);
        daiNodes.push_back(std::move(sysLogger));
    }
    if(ph->getParam<bool>("i_enable_sync")) {
        auto sync = std::make_unique<dai_nodes::Sync>("sync", node, pipeline);
        for(auto& daiNode : daiNodes) {
            auto pubs = daiNode->getPublishers();
            RCLCPP_DEBUG(node->get_logger(), "Number of synced publishers found for %s: %zu", daiNode->getName().c_str(), pubs.size());
            if(!pubs.empty()) {
                sync->addPublishers(pubs);
            }
        }
        daiNodes.push_back(std::move(sync));
    }
    RCLCPP_INFO(node->get_logger(), "Finished setting up pipeline.");
    return daiNodes;
}

std::string PipelineGenerator::validatePipeline(std::shared_ptr<rclcpp::Node> node, const std::string& typeStr, int sensorNum, const std::string& deviceName) {
    auto pType = utils::getValFromMap(typeStr, pipelineTypeMap);
    if(deviceName == "OAK-D-SR-POE") {
        RCLCPP_WARN(node->get_logger(), "OAK-D-SR-POE device detected. Pipeline types other than StereoToF/ToF/RGBToF might not work without reconfiguration.");
    }
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
