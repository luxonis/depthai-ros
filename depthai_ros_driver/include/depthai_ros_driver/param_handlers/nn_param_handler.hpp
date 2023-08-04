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

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
namespace nn {
enum class NNFamily { Segmentation, Mobilenet, Yolo };
}
class NNParamHandler : public BaseParamHandler {
   public:
    explicit NNParamHandler(rclcpp::Node* node, const std::string& name);
    ~NNParamHandler();
    nn::NNFamily getNNFamily();
    template <typename T>
    void declareParams(std::shared_ptr<T> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
        declareAndLogParam<bool>("i_disable_resize", false);
        declareAndLogParam<bool>("i_enable_passthrough", false);
        declareAndLogParam<bool>("i_enable_passthrough_depth", false);
        declareAndLogParam<bool>("i_get_base_device_timestamp", false);
        dai::CameraBoardSocket socket = dai::CameraBoardSocket::RGB;
        try {
            socket = static_cast<dai::CameraBoardSocket>(getROSNode()->get_parameter("rgb.i_board_socket_id").as_int());
        } catch(rclcpp::exceptions::ParameterNotDeclaredException& e) {
            RCLCPP_ERROR(getROSNode()->get_logger(), "Unable to get correct socket for RGB node, defaulting to RGB.");
        }
        declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket));
    
        auto nn_path = getParam<std::string>("i_nn_config_path");
        using json = nlohmann::json;
        std::ifstream f(nn_path);
        json data = json::parse(f);
        parseConfigFile(nn_path, nn, imageManip);
    }

    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> nn);
    void setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> nn);

    template <typename T>
    void setSpatialParams(std::shared_ptr<T> nn) {
        // nn->setBoundingBoxScaleFactor(0.5);
        // nn->setDepthLowerThreshold(100);
        // nn->setDepthUpperThreshold(10000);
    }

    template <typename T>
    void setYoloParams(nlohmann::json data, std::shared_ptr<T> nn) {
        auto metadata = data["nn_config"]["NN_specific_metadata"];
        if(metadata.contains("classes")) {
            int num_classes = 80;
            num_classes = metadata["classes"].get<int>();
            nn->setNumClasses(num_classes);
        }
        if(metadata.contains("coordinates")) {
            int coordinates = 4;
            coordinates = metadata["coordinates"].get<int>();
            nn->setCoordinateSize(coordinates);
        }
        if(metadata.contains("anchors")) {
            std::vector<float> anchors = {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
            anchors = metadata["anchors"].get<std::vector<float>>();
            nn->setAnchors(anchors);
        }
        if(metadata.contains("anchor_masks")) {
            std::map<std::string, std::vector<int>> anchor_masks = {{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}};
            anchor_masks.clear();
            for(auto& el : metadata["anchor_masks"].items()) {
                anchor_masks.insert({el.key(), el.value()});
            }
            nn->setAnchorMasks(anchor_masks);
        }
        if(metadata.contains("iou_threshold")) {
            float iou_threshold = 0.5f;
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
            declareAndLogParam("i_model_path", modelPath);
            if(!getParam<bool>("i_disable_resize")) {
                setImageManip(modelPath, imageManip);
            }
            nn->setBlobPath(modelPath);
            nn->setNumPoolFrames(declareAndLogParam<int>("i_num_pool_frames", 4));
            nn->setNumInferenceThreads(declareAndLogParam<int>("i_num_inference_threads", 2));
            nn->input.setBlocking(false);
            declareAndLogParam<int>("i_max_q_size", 30);
            setNNParams(data, nn);
        }
    }

    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;

   private:
    void setImageManip(const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip);
    std::string getModelPath(const nlohmann::json& data);
    std::unordered_map<std::string, nn::NNFamily> nnFamilyMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver