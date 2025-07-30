#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"

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
    std::unique_ptr<dai_nodes::BaseNode> createNN(std::shared_ptr<rclcpp::Node> node,
                                                  std::shared_ptr<dai::Pipeline> pipeline,
                                                  dai_nodes::SensorWrapper& camNode,
                                                  const std::string& deviceName,
                                                  bool rsCompat) {
        using namespace dai_nodes::sensor_helpers;
        auto nn = std::make_unique<dai_nodes::NNWrapper>(getNodeName(node, NodeNameEnum::NN), node, pipeline, deviceName, rsCompat, camNode);
        return nn;
    }
    std::unique_ptr<dai_nodes::BaseNode> createSpatialNN(std::shared_ptr<rclcpp::Node> node,
                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                         dai_nodes::SensorWrapper& camNode,
                                                         dai_nodes::Stereo& stereoNode,
                                                         const std::string& deviceName,
                                                         bool rsCompat) {
        using namespace dai_nodes::sensor_helpers;
        auto nn = std::make_unique<dai_nodes::SpatialNNWrapper>(getNodeName(node, NodeNameEnum::NN), node, pipeline, deviceName, rsCompat, camNode, stereoNode);
        return nn;
    }

    virtual std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                             std::shared_ptr<dai::Device> device,
                                                                             std::shared_ptr<dai::Pipeline> pipeline,
                                                                             std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                             const std::string& deviceName,
                                                                             bool rsCompat,
                                                                             const std::string& nnType) = 0;

   protected:
    BasePipeline(){};
    std::unordered_map<std::string, NNType> nnTypeMap = {
        {"", NNType::None},
        {"NONE", NNType::None},
        {"RGB", NNType::RGB},
        {"SPATIAL", NNType::Spatial},
    };
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver
