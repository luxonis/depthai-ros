#include "depthai_ros_driver/pipeline/base_types.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/stereo.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace pipeline_gen {

std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGB::createPipeline(ros::NodeHandle node,
                                                                      std::shared_ptr<dai::Device> device,
                                                                      std::shared_ptr<dai::Pipeline> pipeline,
                                                                      const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, device, dai::CameraBoardSocket::CAM_A);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            ROS_WARN("Spatial NN selected, but configuration is RGB. Please change camera_i_nn_type parameter to RGB.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBD::createPipeline(ros::NodeHandle node,
                                                                       std::shared_ptr<dai::Device> device,
                                                                       std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, device, dai::CameraBoardSocket::CAM_A);
    auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            auto nn = createSpatialNN(node, pipeline, *rgb, *stereo);
            daiNodes.push_back(std::move(nn));
            break;
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBStereo::createPipeline(ros::NodeHandle node,
                                                                            std::shared_ptr<dai::Device> device,
                                                                            std::shared_ptr<dai::Pipeline> pipeline,
                                                                            const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, device, dai::CameraBoardSocket::CAM_A);
    auto left = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, device, dai::CameraBoardSocket::CAM_B);
    auto right = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, device, dai::CameraBoardSocket::CAM_C);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            ROS_WARN("Spatial NN selected, but configuration is RGBStereo. Please change camera_i_nn_type parameter to RGB.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Stereo::createPipeline(ros::NodeHandle node,
                                                                         std::shared_ptr<dai::Device> device,
                                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                                         const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto left = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, device, dai::CameraBoardSocket::CAM_B);
    auto right = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, device, dai::CameraBoardSocket::CAM_C);
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Depth::createPipeline(ros::NodeHandle node,
                                                                        std::shared_ptr<dai::Device> device,
                                                                        std::shared_ptr<dai::Pipeline> pipeline,
                                                                        const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device);
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> CamArray::createPipeline(ros::NodeHandle node,
                                                                           std::shared_ptr<dai::Device> device,
                                                                           std::shared_ptr<dai::Pipeline> pipeline,
                                                                           const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    for(auto& feature : device->getConnectedCameraFeatures()) {
        auto name = utils::getSocketName(feature.socket);
        auto daiNode = std::make_unique<dai_nodes::SensorWrapper>(name, node, pipeline, device, feature.socket);
        daiNodes.push_back(std::move(daiNode));
    };
    return daiNodes;
}
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGB, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGBD, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGBStereo, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::Stereo, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::Depth, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::CamArray, depthai_ros_driver::pipeline_gen::BasePipeline)