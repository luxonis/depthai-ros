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
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {

std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGB::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                      std::shared_ptr<dai::Device> device,
                                                                      std::shared_ptr<dai::Pipeline> pipeline,
                                                                      const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::RGB), node, pipeline, device, dai::CameraBoardSocket::CAM_A);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            RCLCPP_WARN(node->get_logger(), "Spatial NN selected, but configuration is RGB. Please change camera.i_nn_type parameter to RGB.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBD::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                       std::shared_ptr<dai::Device> device,
                                                                       std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::RGB), node, pipeline, device, dai::CameraBoardSocket::CAM_A);
    auto stereo = std::make_unique<dai_nodes::Stereo>(getNodeName(node, NodeNameEnum::Stereo), node, pipeline, device);
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
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBStereo::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                            std::shared_ptr<dai::Device> device,
                                                                            std::shared_ptr<dai::Pipeline> pipeline,
                                                                            const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::RGB), node, pipeline, device, dai::CameraBoardSocket::CAM_A);
    auto left = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Left), node, pipeline, device, dai::CameraBoardSocket::CAM_B);
    auto right = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Right), node, pipeline, device, dai::CameraBoardSocket::CAM_C);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            RCLCPP_WARN(node->get_logger(), "Spatial NN selected, but configuration is RGBStereo. Please change camera.i_nn_type parameter to RGB.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Stereo::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                         std::shared_ptr<dai::Device> device,
                                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                                         const std::string& /*nnType*/) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto left = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Left), node, pipeline, device, dai::CameraBoardSocket::CAM_B);
    auto right = std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Right), node, pipeline, device, dai::CameraBoardSocket::CAM_C);
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Depth::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                        std::shared_ptr<dai::Device> device,
                                                                        std::shared_ptr<dai::Pipeline> pipeline,
                                                                        const std::string& /*nnType*/) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto stereo = std::make_unique<dai_nodes::Stereo>(getNodeName(node, NodeNameEnum::Stereo), node, pipeline, device);
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> CamArray::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                           std::shared_ptr<dai::Device> device,
                                                                           std::shared_ptr<dai::Pipeline> pipeline,
                                                                           const std::string& /*nnType*/) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;

    for(auto& feature : device->getConnectedCameraFeatures()) {
        auto name = getSocketName(node, feature.socket);
        auto daiNode = std::make_unique<dai_nodes::SensorWrapper>(name, node, pipeline, device, feature.socket);
        daiNodes.push_back(std::move(daiNode));
    };
    return daiNodes;
}

std::vector<std::unique_ptr<dai_nodes::BaseNode>> DepthToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                           std::shared_ptr<dai::Device> device,
                                                                           std::shared_ptr<dai::Pipeline> pipeline,
                                                                           const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline);
    auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device);
    daiNodes.push_back(std::move(tof));
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> StereoToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                            std::shared_ptr<dai::Device> device,
                                                                            std::shared_ptr<dai::Pipeline> pipeline,
                                                                            const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline, dai::CameraBoardSocket::CAM_C);
    auto left = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, device, dai::CameraBoardSocket::CAM_B);
    auto right = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, device, dai::CameraBoardSocket::CAM_C);
    right->link(tof->getInput());
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    daiNodes.push_back(std::move(tof));
    return daiNodes;
}

std::vector<std::unique_ptr<dai_nodes::BaseNode>> ToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                      std::shared_ptr<dai::Device> /*device*/,
                                                                      std::shared_ptr<dai::Pipeline> pipeline,
                                                                      const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline);
    daiNodes.push_back(std::move(tof));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                         std::shared_ptr<dai::Device> device,
                                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                                         const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, device, dai::CameraBoardSocket::CAM_C);
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline, dai::CameraBoardSocket::CAM_C);
    rgb->link(tof->getInput());
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            RCLCPP_WARN(node->get_logger(), "Spatial NN selected, but configuration is RGBToF. Please change camera.i_nn_type parameter to RGB.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(tof));
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
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::StereoToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::DepthToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::ToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGBToF, depthai_ros_driver::pipeline_gen::BasePipeline)
