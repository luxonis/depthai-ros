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
    /**
     * @brief      Validates the pipeline type. If the pipeline type is not valid for the number of sensors, it will be changed to the default type.
     *
     * @param[in]  type       The type
     * @param[in]  sensorNum  The sensor number
     *
     * @return     The validated pipeline type.
     */
    std::string validatePipeline(const std::string& typeStr, int sensorNum);

    /**
     * @brief      Creates the pipeline by using a plugin. Plugin types need to be of type depthai_ros_driver::pipeline_gen::BasePipeline.
     *
     * @param      node          The node
     * @param      device        The device
     * @param      pipeline      The pipeline
     * @param[in]  pipelineType  The pipeline type name (plugin name or one of the default types)
     * @param[in]  nnType        The neural network type (none, rgb, spatial)
     * @param[in]  enableImu     Indicates if IMU is enabled
     *
     * @return     Vector BaseNodes created.
     */
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
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver