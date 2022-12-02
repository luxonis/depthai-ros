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
    template <typename T>
    void declareParams(rclcpp::Node* node, std::shared_ptr<T> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
        auto nn_path = get_param<std::string>(node, "i_nn_config_path");
        using json = nlohmann::json;
        std::ifstream f(nn_path);
        json data = json::parse(f);
        parseConfigFile(node, nn_path, nn, imageManip);
    }

    void setNNParams(rclcpp::Node *node, nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> nn);
    void setNNParams(rclcpp::Node *node, nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn);
    void setNNParams(rclcpp::Node *node, nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn);

    template <typename T>
    void parseConfigFile(rclcpp::Node* node, const std::string& path, std::shared_ptr<T> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
        using json = nlohmann::json;
        std::ifstream f(path);
        json data = json::parse(f);
        if(data.contains("model") && data.contains("nn_config")) {
            auto modelPath = getModelPath(data);
            declareAndLogParam(node, "i_model_path", modelPath);
            setImageManip(node, modelPath, imageManip);
            nn->setBlobPath(modelPath);
            nn->setNumPoolFrames(declareAndLogParam<int>(node, "i_num_pool_frames", 4));
            nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
            setNNParams(node, data, nn);
        }
    }

    dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) override;

   private:
    void setImageManip(rclcpp::Node* node, const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip);
    std::string getModelPath(const nlohmann::json& data);
    std::unordered_map<std::string, nn::NNFamily> nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"mobilenet", nn::NNFamily::Mobilenet},
        {"YOLO", nn::NNFamily::Yolo},
    };
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver