#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {
enum class PipelineType { RGB, RGBD, RGBStereo, Stereo, Depth, CamArray };
enum class NNType { None, RGB, Spatial };
class PipelineGenerator {
   public:
    PipelineGenerator(){};
    ~PipelineGenerator() = default;
    PipelineType validatePipeline(rclcpp::Node* node, PipelineType type, int sensorNum);
    std::unique_ptr<dai_nodes::BaseNode> createNN(rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline, dai_nodes::BaseNode& daiNode);
    std::unique_ptr<dai_nodes::BaseNode> createSpatialNN(rclcpp::Node* node,
                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                         dai_nodes::BaseNode& daiNode,
                                                         dai_nodes::BaseNode& daiStereoNode);
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& pipelineType,
                                                                     const std::string& nnType,
                                                                     bool enableImu);

   private:
    std::unordered_map<std::string, PipelineType> pipelineTypeMap{{"RGB", PipelineType::RGB},
                                                                  {"RGBD", PipelineType::RGBD},
                                                                  {"RGBStereo", PipelineType::RGBStereo},
                                                                  {"Stereo", PipelineType::Stereo},
                                                                  {"Depth", PipelineType::Depth},
                                                                  {"CamArray", PipelineType::CamArray}};
    std::unordered_map<std::string, NNType> nnTypeMap = {
        {"", NNType::None},
        {"none", NNType::None},
        {"rgb", NNType::RGB},
        {"spatial", NNType::Spatial},
    };
    const std::string alphabet = "abcdefghijklmnopqrstuvwxyz";
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver