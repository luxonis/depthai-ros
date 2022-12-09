#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(const std::string& name) : BaseParamHandler(name) {
    nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"mobilenet", nn::NNFamily::Mobilenet},
        {"YOLO", nn::NNFamily::Yolo},
    };
};
NNParamHandler::~NNParamHandler() = default;
nn::NNFamily NNParamHandler::getNNFamily(rclcpp::Node* node) {
    std::string config_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/nn/";
    std::string default_nn_conf_name = "mobilenet.json";
    auto nn_path = declareAndLogParam<std::string>(node, "i_nn_config_path", default_nn_conf_name);
    if(nn_path == "depthai_ros_driver/yolo") {
        nn_path = config_path + "yolo.json";
    } else if(nn_path == "depthai_ros_driver/segmentation") {
        nn_path = config_path + "segmentation.json";
    } else if(nn_path == "depthai_ros_driver/mobilenet") {
        nn_path = config_path + "mobilenet.json";
    }
    auto final_path = declareAndLogParam<std::string>(node, "i_nn_config_path", nn_path, true);
    using json = nlohmann::json;
    std::ifstream f(final_path);
    json data = json::parse(f);
    std::string nnFamily;
    if(data.contains("model") && data.contains("nn_config")) {
        nnFamily = data["nn_config"]["NN_family"].get<std::string>();
        RCLCPP_INFO(node->get_logger(), "NN Family: %s", nnFamily.c_str());
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
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
}

void NNParamHandler::setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> nn) {
    if(data["nn_config"].contains("confidence_threshold")) {
        auto conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
    setSpatialParams(node, data, nn);
}
void NNParamHandler::setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> nn) {
    float conf_threshold = 0.5;
    if(data["nn_config"].contains("confidence_threshold")) {
        conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
    setSpatialParams(node, data, nn);
    if(data["nn_config"].contains("NN_specific_metadata")) {
        setYoloParams(node, data, nn);
    }
}

void NNParamHandler::setNNParams(rclcpp::Node* node, nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn) {
    float conf_threshold = 0.5;
    if(data["nn_config"].contains("confidence_threshold")) {
        conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", labels);
    }
    if(data["nn_config"].contains("NN_specific_metadata")) {
        setYoloParams(node, data, nn);
    }
}

void NNParamHandler::setImageManip(rclcpp::Node* node, const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip) {
    auto blob = dai::OpenVINO::Blob(model_path);
    auto first_info = blob.networkInputs.begin();
    auto input_size = first_info->second.dims[0];

    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    imageManip->inputImage.setBlocking(false);
    imageManip->inputImage.setQueueSize(8);
    imageManip->initialConfig.setResize(input_size, input_size);
}
std::string NNParamHandler::getModelPath(const nlohmann::json& data) {
    std::string modelPath;
    auto source = data["model"]["zoo"].get<std::string>();
    if(source == "depthai_examples") {
        modelPath = ament_index_cpp::get_package_share_directory("depthai_examples") + "/resources/" + data["model"]["model_name"].get<std::string>() + ".blob";
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