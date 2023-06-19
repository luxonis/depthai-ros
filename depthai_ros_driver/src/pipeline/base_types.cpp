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
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {

std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGB::createPipeline(rclcpp::Node* node,
                                                                      std::shared_ptr<dai::Device> device,
                                                                      std::shared_ptr<dai::Pipeline> pipeline,
                                                                      const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            RCLCPP_WARN(node->get_logger(), "Spatial NN selected, but configuration is RGB.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBD::createPipeline(rclcpp::Node* node,
                                                                       std::shared_ptr<dai::Device> device,
                                                                       std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);
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
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBStereo::createPipeline(rclcpp::Node* node,
                                                                            std::shared_ptr<dai::Device> device,
                                                                            std::shared_ptr<dai::Pipeline> pipeline,
                                                                            const std::string& nnType) {
    std::string nTypeUpCase = utils::getUpperCaseStr(nnType);
    auto nType = utils::getValFromMap(nTypeUpCase, nnTypeMap);

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);
    auto left = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT);
    auto right = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT);
    switch(nType) {
        case NNType::None:
            break;
        case NNType::RGB: {
            auto nn = createNN(node, pipeline, *rgb);
            daiNodes.push_back(std::move(nn));
            break;
        }
        case NNType::Spatial: {
            RCLCPP_WARN(node->get_logger(), "Spatial NN selected, but configuration is RGBStereo.");
        }
        default:
            break;
    }
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Stereo::createPipeline(rclcpp::Node* node,
                                                                         std::shared_ptr<dai::Device> device,
                                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                                         const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto left = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT);
    auto right = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT);
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Depth::createPipeline(rclcpp::Node* node,
                                                                        std::shared_ptr<dai::Device> device,
                                                                        std::shared_ptr<dai::Pipeline> pipeline,
                                                                        const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device);
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> CamArray::createPipeline(rclcpp::Node* node,
                                                                           std::shared_ptr<dai::Device> device,
                                                                           std::shared_ptr<dai::Pipeline> pipeline,
                                                                           const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    int i = 0;
    int j = 0;
    for(auto& sensor : device->getCameraSensorNames()) {
        // append letter for greater sensor number
        if(i % alphabet.size() == 0) {
            j++;
        }
        std::string nodeName(j, alphabet[i % alphabet.size()]);
        auto daiNode = std::make_unique<dai_nodes::SensorWrapper>(nodeName, node, pipeline, device, sensor.first);
        daiNodes.push_back(std::move(daiNode));
        i++;
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