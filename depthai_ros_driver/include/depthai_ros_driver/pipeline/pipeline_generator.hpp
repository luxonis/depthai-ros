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
namespace param_handlers {
class PipelineGenParamHandler;
}  // namespace param_handlers
namespace pipeline_gen {
enum class PipelineType { RGB, RGBD, RGBStereo, Stereo, Depth, CamArray, DepthToF, StereoToF, ToF, RGBToF, Thermal };

class PipelineGenerator {
   public:
    PipelineGenerator();
    ~PipelineGenerator();
    /**
     * @brief      Validates the pipeline type. If the pipeline type is not valid for the number of sensors, it will be changed to the default type.
     *
     * @param[in]  type       The type
     * @param[in]  sensorNum  The sensor number
     * @param[in]  deviceName The device name
     *
     * @return     The validated pipeline type.
     */
    std::string validatePipeline(const std::string& typeStr, int sensorNum, const std::string& deviceName);

    /**
     * @brief      Creates the pipeline by using a plugin. Plugin types need to be of type depthai_ros_driver::pipeline_gen::BasePipeline.
     *
     * @param      node          The node
     * @param      device        The device
     * @param      pipeline      The pipeline
     * @param[in]  pipelineType  The pipeline type name (plugin name or one of the default types)
     * @param[in]  nnType        The neural network type (none, rgb, spatial)
     *
     * @return     Vector BaseNodes created.
     */
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(ros::NodeHandle node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& pipelineType,
                                                                     const std::string& nnType);

   protected:
    std::unordered_map<std::string, std::string> pluginTypeMap;
    std::unordered_map<std::string, PipelineType> pipelineTypeMap;

   private:
    std::unique_ptr<param_handlers::PipelineGenParamHandler> ph;
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver
