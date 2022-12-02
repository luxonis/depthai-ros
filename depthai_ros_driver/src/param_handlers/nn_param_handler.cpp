#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(const std::string& name) : BaseParamHandler(name){};

nn::NNFamily NNParamHandler::getNNFamily(rclcpp::Node* node) {
    std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/nn/yolo.json";
    auto nn_path = declareAndLogParam<std::string>(node, "i_nn_config_path", default_nn_path);
    using json = nlohmann::json;
    std::ifstream f(nn_path);
    json data = json::parse(f);
    std::string nnFamily;
    if(data.contains("model") && data.contains("nn_config")) {
        nnFamily = data["nn_config"]["NN_family"].get<std::string>();
        declareAndLogParam<std::string>(node, "i_nn_family", nnFamily);
    } else {
        throw std::runtime_error("No required fields");
    }
    return nnFamilyMap.at(nnFamily);
}

void NNParamHandler::setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> /*nn*/) {
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
}

void NNParamHandler::setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn) {
    if(data["nn_config"].contains("confidence_threshold")) {
        auto conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(declareAndLogParam<float>(node, "i_confidence_threshold", conf_threshold));
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
}

void NNParamHandler::setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn) {
    float conf_threshold = 0.5;
    if(data["nn_config"].contains("confidence_threshold")) {
        conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(declareAndLogParam<float>(node, "i_confidence_threshold", conf_threshold));
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
    if(data["nn_config"].contains("NN_specific_metadata")) {
        auto metadata = data["nn_config"]["NN_specific_metadata"];
        int num_classes = 80;
        if(metadata.contains("classes")) {
            num_classes = metadata["classes"].get<int>();
            nn->setNumClasses(declareAndLogParam<int>(node, "i_num_classes", num_classes));
        }
        int coordinates = 4;
        if(metadata.contains("coordinates")) {
            coordinates = metadata["coordinates"].get<int>();
            nn->setCoordinateSize(declareAndLogParam<int>(node, "i_coordinate_size", coordinates));
        }
        std::vector<float> anchors = {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
        if(metadata.contains("anchors")) {
            anchors = metadata["anchors"].get<std::vector<float>>();
            nn->setAnchors(anchors);
        }
        std::map<std::string, std::vector<int>> anchor_masks = {{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}};
        if(metadata.contains("anchor_masks")) {
            anchor_masks.clear();
            for (auto &el: metadata["anchor_masks"].items()){
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
}

void NNParamHandler::setImageManip(rclcpp::Node* node, const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip) {
    auto blob = dai::OpenVINO::Blob(model_path);
    auto first_info = blob.networkInputs.begin();
    auto input_size = declareAndLogParam<int>(node, "i_input_size", first_info->second.dims[0]);

    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    imageManip->inputImage.setBlocking(false);
    imageManip->inputImage.setQueueSize(8);
    imageManip->initialConfig.setResize(input_size, input_size);
}
std::string NNParamHandler::getModelPath(const nlohmann::json& data) {
    std::string modelPath;
    auto source = data["model"]["zoo"].get<std::string>();
    if(source == "depthai_ros_driver") {
        modelPath = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/models/" + data["model"]["model_name"].get<std::string>() + ".blob";
    } else if(source == "path") {
        modelPath = data["model"]["model_name"].get<std::string>();
    } else {
        throw std::runtime_error("Other options not yet available");
    }
    return modelPath;
}

dai::CameraControl NNParamHandler::setRuntimeParams(rclcpp::Node* /*node*/, const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver