#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace ros {
class NodeHandle;
}

namespace depthai_ros_driver {
namespace pipeline_gen {
enum class PipelineType { RGB, RGBD, RGBStereo, Depth, Stereo, CamArray };

class PipelineGenerator {
   public:
    PipelineGenerator(){};
    ~PipelineGenerator() = default;
    PipelineType validatePipeline(PipelineType type, int sensorNum);
    std::unique_ptr<dai_nodes::BaseNode> createNN(ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, dai_nodes::BaseNode& daiNode);
    std::unique_ptr<dai_nodes::BaseNode> createSpatialNN(ros::NodeHandle node,
                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                         dai_nodes::BaseNode& daiNode,
                                                         dai_nodes::BaseNode& daiStereoNode);
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(ros::NodeHandle node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& pipelineType,
                                                                     const std::string& nnType,
                                                                     bool enableImu);

   private:
   protected:
    std::unordered_map<std::string, std::string> pluginTypeMap{{"RGB", "depthai_ros_driver::pipeline_gen::RGB"},
                                                               {"RGBD", "depthai_ros_driver::pipeline_gen::RGBD"},
                                                               {"RGBSTEREO", "depthai_ros_driver::pipeline_gen::RGBStereo"},
                                                               {"STEREO", "depthai_ros_driver::pipeline_gen::Stereo"},
                                                               {"DEPTH", "depthai_ros_driver::pipeline_gen::Depth"},
                                                               {"CAMARRAY", "depthai_ros_driver::pipeline_gen::CamArray"}};
    std::unordered_map<std::string, PipelineType> pipelineTypeMap{{"RGB", PipelineType::RGB},
                                                                  {"RGBD", PipelineType::RGBD},
                                                                  {"RGBSTEREO", PipelineType::RGBStereo},
                                                                  {"STEREO", PipelineType::Stereo},
                                                                  {"DEPTH", PipelineType::Depth},
                                                                  {"CAMARRAY", PipelineType::CamArray}};
    const std::string alphabet = "abcdefghijklmnopqrstuvwxyz";
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver