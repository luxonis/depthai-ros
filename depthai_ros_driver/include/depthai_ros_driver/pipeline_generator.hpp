#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {
enum class PipelineType { RGB, RGBD, RGBStereo, Stereo };
enum class NNType { None, RGB, Spatial };
class PipelineGenerator {
   public:
    PipelineGenerator(){};
    ~PipelineGenerator() = default;

    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(ros::NodeHandle node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& pipelineType,
                                                                     const std::string& nnType);

   private:
    std::unordered_map<std::string, PipelineType> pipelineTypeMap{
        {"RGB", PipelineType::RGB}, {"RGBD", PipelineType::RGBD}, {"RGBStereo", PipelineType::RGBStereo}, {"Stereo", PipelineType::Stereo}};
    std::unordered_map<std::string, NNType> nnTypeMap = {
        {"", NNType::None},
        {"none", NNType::None},
        {"rgb", NNType::RGB},
        {"spatial", NNType::Spatial},
    };
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver