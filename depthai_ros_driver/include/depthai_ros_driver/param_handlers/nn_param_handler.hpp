#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "nlohmann/json.hpp"

namespace dai {
namespace node {
class NeuralNetwork;
class MobileNetDetectionNetwork;
class MobileNetSpatialDetectionNetwork;
class YoloDetectionNetwork;
class YoloSpatialDetectionNetwork;
class ImageManip;
}  // namespace node
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
namespace nn {
enum class NNFamily { Segmentation, Mobilenet, Yolo };
}
class NNParamHandler : public BaseParamHandler {
   public:
    explicit NNParamHandler(ros::NodeHandle node, const std::string& name, const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A);
    ~NNParamHandler();
    nn::NNFamily getNNFamily();
    std::string getConfigPath();
    template <typename T>
    void declareParams(std::shared_ptr<T> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
        declareAndLogParam<bool>("i_disable_resize", false);
		declareAndLogParam<bool>("i_desqueeze_output", false);
        declareAndLogParam<bool>("i_enable_passthrough", false);
        declareAndLogParam<bool>("i_enable_passthrough_depth", false);
        declareAndLogParam<bool>("i_get_base_device_timestamp", false);
        declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
        declareAndLogParam<int>("i_max_q_size", 30);
        auto nn_path = getParam<std::string>("i_nn_config_path");
        auto nnPath = getConfigPath();
        using json = nlohmann::json;
        std::ifstream f(nnPath);
        json data = json::parse(f);
        parseConfigFile(nnPath, nn, imageManip);
    }

    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> nn);

    template <typename T>
    void setSpatialParams(std::shared_ptr<T> nn) {
        nn->setBoundingBoxScaleFactor(0.5);
        nn->setDepthLowerThreshold(100);
        nn->setDepthUpperThreshold(10000);
    }

    template <typename T>
    void setYoloParams(nlohmann::json data, std::shared_ptr<T> nn) {
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

    template <typename T>
    void parseConfigFile(const std::string& path, std::shared_ptr<T> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
        using json = nlohmann::json;
        std::ifstream f(path);
        json data = json::parse(f);
        if(data.contains("model") && data.contains("nn_config")) {
            auto modelPath = getModelPath(data);
            setImageManip(modelPath, imageManip);
            nn->setBlobPath(modelPath);
            nn->setNumPoolFrames(4);
            nn->setNumInferenceThreads(2);
            nn->input.setBlocking(false);
            setNNParams(data, nn);
        }
    }

    dai::CameraControl setRuntimeParams(parametersConfig& config) override;

   private:
    void setImageManip(const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip);
    std::string getModelPath(const nlohmann::json& data);
    std::unordered_map<std::string, nn::NNFamily> nnFamilyMap;
    std::vector<std::string> labels;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
