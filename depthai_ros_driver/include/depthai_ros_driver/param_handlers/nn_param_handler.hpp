#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
namespace nn {
enum class NNFamily { Segmentation, Mobilenet, Yolo };
}
class NNParamHandler : public BaseParamHandler {
   public:
    explicit NNParamHandler(const std::string& name);
    ~NNParamHandler(){};
    nn::NNFamily getNNFamily(rclcpp::Node* node);
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::NeuralNetwork> nn, std::shared_ptr<dai::node::ImageManip> imageManip);
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn, std::shared_ptr<dai::node::ImageManip> imageManip);
    void parseConfigFile(rclcpp::Node* node,
                         const std::string& path,
                         std::shared_ptr<dai::node::NeuralNetwork> nn,
                         std::shared_ptr<dai::node::ImageManip> imageManip);
    void parseConfigFile(rclcpp::Node* node,
                         const std::string& path,
                         std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn,
                         std::shared_ptr<dai::node::ImageManip> imageManip);

    dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) override;

   private:
    void setImageManip(rclcpp::Node* node,const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip);
    std::string getModelPath(const nlohmann::json& data);
    std::unordered_map<std::string, nn::NNFamily> nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"mobilenet", nn::NNFamily::Mobilenet},
        {"yolo", nn::NNFamily::Yolo},
    };
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver