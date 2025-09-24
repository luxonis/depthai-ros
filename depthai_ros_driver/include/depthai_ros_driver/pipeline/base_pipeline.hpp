#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace rclcpp {
class Node;
}

namespace depthai_ros_driver {
namespace dai_nodes {
class SensorWrapper;
class Imu;
class Stereo;
class ToF;
}  // namespace dai_nodes
namespace pipeline_gen {
enum class NNType { None, RGB, Spatial };
class BasePipeline {
   public:
    ~BasePipeline() = default;

    virtual std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                             std::shared_ptr<dai::Device> device,
                                                                             std::shared_ptr<dai::Pipeline> pipeline,
                                                                             std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                             const std::string& deviceName,
                                                                             bool rsCompat,
                                                                             const std::string& nnType) = 0;

   protected:
    bool checkForImu(std::shared_ptr<param_handlers::PipelineGenParamHandler> ph, std::shared_ptr<dai::Device> device, rclcpp::Logger logger);

    void addRgbdNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                     std::shared_ptr<rclcpp::Node> node,
                     std::shared_ptr<dai::Device> device,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                     bool rsCompat,
                     dai_nodes::SensorWrapper& rgb,
                     dai_nodes::Stereo& stereo);

    void addRgbdNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                     std::shared_ptr<rclcpp::Node> node,
                     std::shared_ptr<dai::Device> device,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                     bool rsCompat,
                     dai_nodes::SensorWrapper& rgb,
                     dai_nodes::ToF& tof);

    void addNnNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                   std::shared_ptr<rclcpp::Node> node,
                   std::shared_ptr<dai::Pipeline> pipeline,
                   const std::string& deviceName,
                   bool rsCompat,
                   dai_nodes::SensorWrapper& sensor,
                   const std::string& nnType);
    void addNnNode(std::vector<std::unique_ptr<dai_nodes::BaseNode>>& daiNodes,
                   std::shared_ptr<rclcpp::Node> node,
                   std::shared_ptr<dai::Pipeline> pipeline,
                   const std::string& deviceName,
                   bool rsCompat,
                   dai_nodes::SensorWrapper& sensor,
                   dai_nodes::Stereo& stereo,
                   const std::string& nnType);

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
