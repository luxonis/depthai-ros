#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/visibility.h"

namespace depthai_ros_driver {
namespace param_handlers {
class NNParamHandler : public BaseParamHandler {
   public:
    explicit NNParamHandler(const std::string& name);
    ~NNParamHandler(){};
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::NeuralNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip);
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::SpatialDetectionNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip);
    void parseConfigFile(rclcpp::Node *node, const std::string &path, std::shared_ptr<dai::node::NeuralNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip);
    void parseConfigFile(rclcpp::Node *node, const std::string &path, std::shared_ptr<dai::node::SpatialDetectionNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip);

    dai::CameraControl setRuntimeParams(rclcpp::Node* node,const std::vector<rclcpp::Parameter>& params) override;

};
}  // namespace param_handlers
}  // namespace depthai_ros_driver