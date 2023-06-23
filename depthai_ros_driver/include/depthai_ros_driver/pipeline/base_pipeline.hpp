#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/stereo.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace rclcpp {
class Node;
}

namespace depthai_ros_driver {
namespace pipeline_gen {
enum class NNType { None, RGB, Spatial };
class BasePipeline {
   public:
    ~BasePipeline() = default;
    std::unique_ptr<dai_nodes::BaseNode> createNN(rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline, dai_nodes::BaseNode& daiNode) {
        auto nn = std::make_unique<dai_nodes::NNWrapper>("nn", node, pipeline);
        daiNode.link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
        return nn;
    }
    std::unique_ptr<dai_nodes::BaseNode> createSpatialNN(rclcpp::Node* node,
                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                         dai_nodes::BaseNode& daiNode,
                                                         dai_nodes::BaseNode& daiStereoNode) {
        auto nn = std::make_unique<dai_nodes::SpatialNNWrapper>("nn", node, pipeline);
        daiNode.link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::input)),
                     static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
        daiStereoNode.link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::inputDepth)));
        return nn;
    }

    virtual std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                             std::shared_ptr<dai::Device> device,
                                                                             std::shared_ptr<dai::Pipeline> pipeline,
                                                                             const std::string& nnType) = 0;

   protected:
    BasePipeline(){};
    const std::string alphabet = "abcdefghijklmnopqrstuvwxyz";
    std::unordered_map<std::string, NNType> nnTypeMap = {
        {"", NNType::None},
        {"NONE", NNType::None},
        {"RGB", NNType::RGB},
        {"SPATIAL", NNType::Spatial},
    };
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver