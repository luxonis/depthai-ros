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
    ~NNParamHandler();
    nn::NNFamily getNNFamily(rclcpp::Node* node);
    template <typename T>
    void declareParams(rclcpp::Node* node, std::shared_ptr<T> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
        auto nn_path = getParam<std::string>(node, "i_nn_config_path");
        using json = nlohmann::json;
        std::ifstream f(nn_path);
        json data = json::parse(f);
        parseConfigFile(node, nn_path, nn, imageManip);
    }

    void setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> nn);
    void setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn);
    void setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn);
    void setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> nn);
    void setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> nn);

    template <typename T>
    void setSpatialParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<T> nn) {
        nn->setBoundingBoxScaleFactor(0.5);
        nn->setDepthLowerThreshold(100);
        nn->setDepthUpperThreshold(10000);
    }

    template <typename T>
    void setYoloParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<T> nn) {
        auto metadata = data["nn_config"]["NN_specific_metadata"];
        int num_classes = 80;
        if(metadata.contains("classes")) {
            num_classes = metadata["classes"].get<int>();
            nn->setNumClasses(num_classes);
        }
        int coordinates = 4;
        if(metadata.contains("coordinates")) {
            coordinates = metadata["coordinates"].get<int>();
            nn->setCoordinateSize(coordinates);
        }
        std::vector<float> anchors = {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
        if(metadata.contains("anchors")) {
            anchors = metadata["anchors"].get<std::vector<float>>();
            nn->setAnchors(anchors);
        }
        std::map<std::string, std::vector<int>> anchor_masks = {{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}};
        if(metadata.contains("anchor_masks")) {
            anchor_masks.clear();
            for(auto& el : metadata["anchor_masks"].items()) {
                anchor_masks.insert({el.key(), el.value()});
            }
        }
        nn->setAnchorMasks(anchor_masks);
        float iou_threshold = 0.5f;
        if(metadata.contains("iou_threshold")) {
            iou_threshold = metadata["iou_threshold"].get<float>();
            nn->setIouThreshold(iou_threshold);
        }
    }

    void setMobilenetParams() {}

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
            nn->input.setBlocking(false);
            declareAndLogParam<int>(node, "i_max_q_size", 30);
            setNNParams(node, data, nn);
        }
    }

    dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) override;

   private:
    void setImageManip(rclcpp::Node* node, const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip);
    std::string getModelPath(const nlohmann::json& data);
    std::unordered_map<std::string, nn::NNFamily> nnFamilyMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver