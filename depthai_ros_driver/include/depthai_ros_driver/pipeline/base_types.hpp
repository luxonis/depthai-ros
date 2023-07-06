#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace rclcpp {
class Node;
}

namespace depthai_ros_driver {
namespace pipeline_gen {
class RGB : public BasePipeline {
   public:
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& nnType) override;
};
class RGBD : public BasePipeline {
   public:
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& nnType) override;
};
class RGBStereo : public BasePipeline {
   public:
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& nnType) override;
};
class Stereo : public BasePipeline {
   public:
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& nnType) override;
};
class Depth : public BasePipeline {
   public:
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& nnType) override;
};
class CamArray : public BasePipeline {
   public:
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> createPipeline(rclcpp::Node* node,
                                                                     std::shared_ptr<dai::Device> device,
                                                                     std::shared_ptr<dai::Pipeline> pipeline,
                                                                     const std::string& nnType) override;
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver