#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "depthai_ros_driver/dai_nodes/nn.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(const std::string& name) : BaseParamHandler(name){};

void NNParamHandler::setImageManip(rclcpp::Node* node, const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip) {
    auto blob = dai::OpenVINO::Blob(model_path);
    auto first_info = blob.networkInputs.begin();
    RCLCPP_INFO(node->get_logger(), "Input size %d %d", first_info->second.dims[0], first_info->second.dims[1]);
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
void NNParamHandler::parseConfigFile(rclcpp::Node* node,
                                     const std::string& path,
                                     std::shared_ptr<dai::node::NeuralNetwork> nn,
                                     std::shared_ptr<dai::node::ImageManip> imageManip) {
    using json = nlohmann::json;
    std::ifstream f(path);
    json data = json::parse(f);
    if(data.contains("model") && data.contains("nn_config")) {
        auto modelPath = getModelPath(data);
        declareAndLogParam(node, "i_model_path", modelPath);
        setImageManip(node, modelPath, imageManip);
        nn->setBlobPath(modelPath);
        nn->setNumPoolFrames(4);
        nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"].get<std::vector<std::string>>());
    }
}

void NNParamHandler::parseConfigFile(rclcpp::Node* node,
                                     const std::string& path,
                                     std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn,
                                     std::shared_ptr<dai::node::ImageManip> imageManip) {
    using json = nlohmann::json;
    std::ifstream f(path);
    json data = json::parse(f);
    if(data.contains("model") && data.contains("nn_config")) {
        auto modelPath = getModelPath(data);
        setImageManip(node, modelPath, imageManip);
        nn->setBlobPath(modelPath);
        nn->setNumPoolFrames(4);
        nn->setConfidenceThreshold(
            declareAndLogParam<float>(node, "i_confidence_threshold", data["nn_config"]["confidence_threshold"].get<float>()));
        nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
        declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"].get<std::vector<std::string>>());
    }
}

// switch(nn_family) {
//     case types::nn_types::NNFamily::Segmentation: {

//     }
//     case types::nn_types::NNFamily::Mobilenet: {
//         nn->setNumPoolFrames(4);
//         nn->setConfidenceThreshold(
//             declareAndLogParam<float>(node, "i_confidence_threshold", data["nn_config"]["confidence_threshold"].get<float>()));
//         nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
//         declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"]);
//         break;
//     }
//     case types::nn_types::NNFamily::Yolo: {
//         nn->setNumPoolFrames(4);
//         nn->setConfidenceThreshold(
//             declareAndLogParam<float>(node, "i_confidence_threshold", data["nn_config"]["confidence_threshold"].get<float>()));
//         nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
//         declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"]);
//         nn->setConfidenceThreshold(0.5f);
//         nn->setNumClasses(80);
//         nn->setCoordinateSize(4);
//         nn->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
//         nn->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
//         nn->setIouThreshold(0.5f);
//         nn->setNumInferenceThreads(2);
//         break;
//     }
//         nn->input.setBlocking(false);
// }


// void NNParamHandler::parseConfigFile(rclcpp::Node* node,
//                                      const std::string& path,
//                                      std::shared_ptr<dai::node::NeuralNetwork> nn,
//                                      std::shared_ptr<dai::node::ImageManip> imageManip) {
//     using json = nlohmann::json;
//     std::ifstream f(path);
//     json data = json::parse(f);
//     if(data.contains("model") && data.contains("nn_config")) {
//         auto source = data["model"]["zoo"].get<std::string>();
//         if(source == "depthai_ros_driver") {
//             std::string model_path =
//                 ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/models/" + data["model"]["model_name"].get<std::string>() + ".blob";
//             auto blob = dai::OpenVINO::Blob(model_path);
//             auto first_info = blob.networkInputs.begin();
//             RCLCPP_INFO(node->get_logger(), "Input size %d %d", first_info->second.dims[0], first_info->second.dims[1]);
//             auto input_size = declareAndLogParam<int>(node, "i_input_size", first_info->second.dims[0]);

//             imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
//             imageManip->inputImage.setBlocking(false);
//             imageManip->inputImage.setQueueSize(8);
//             imageManip->initialConfig.setResize(input_size, input_size);

//             nn->setBlobPath(model_path);
//             nn->setNumPoolFrames(4);
//             nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
//             declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"].get<std::vector<std::string>>());

//             // switch(nn_family) {
//             //     case types::nn_types::NNFamily::Segmentation: {

//             //     }
//             //     case types::nn_types::NNFamily::Mobilenet: {
//             //         nn->setNumPoolFrames(4);
//             //         nn->setConfidenceThreshold(
//             //             declareAndLogParam<float>(node, "i_confidence_threshold", data["nn_config"]["confidence_threshold"].get<float>()));
//             //         nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
//             //         declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"]);
//             //         break;
//             //     }
//             //     case types::nn_types::NNFamily::Yolo: {
//             //         nn->setNumPoolFrames(4);
//             //         nn->setConfidenceThreshold(
//             //             declareAndLogParam<float>(node, "i_confidence_threshold", data["nn_config"]["confidence_threshold"].get<float>()));
//             //         nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
//             //         declareAndLogParam<std::vector<std::string>>(node, "i_label_map", data["mappings"]["labels"]);
//             //         nn->setConfidenceThreshold(0.5f);
//             //         nn->setNumClasses(80);
//             //         nn->setCoordinateSize(4);
//             //         nn->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
//             //         nn->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
//             //         nn->setIouThreshold(0.5f);
//             //         nn->setNumInferenceThreads(2);
//             //         break;
//             //     }
//             //         nn->input.setBlocking(false);
//             // }
//         }
//     }
// }

nn::NNFamily NNParamHandler::getNNFamily(rclcpp::Node* node) {
    std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/nn/segmentation.json";
    auto nn_path = declareAndLogParam<std::string>(node, "i_nn_path", default_nn_path);
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

void NNParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::NeuralNetwork> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
    auto nn_path = get_param<std::string>(node, "i_nn_path");
    using json = nlohmann::json;
    std::ifstream f(nn_path);
    json data = json::parse(f);
    parseConfigFile(node, nn_path, nn, imageManip);
}

void NNParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn, std::shared_ptr<dai::node::ImageManip> imageManip) {
    auto nn_path = get_param<std::string>(node, "i_nn_path");
    using json = nlohmann::json;
    std::ifstream f(nn_path);
    json data = json::parse(f);
    parseConfigFile(node, nn_path, nn, imageManip);
}
// void NNParamHandler::declareParams(rclcpp::Node* node,
//                                    std::shared_ptr<dai::node::SpatialDetectionNetwork> nn,
//                                    std::shared_ptr<dai::node::ImageManip> imageManip) {
//     auto nn_path = get_param<std::string>(node, "i_nn_path");
//     using json = nlohmann::json;
//     std::ifstream f(nn_path);
//     json data = json::parse(f);
//     parseConfigFile(node, nn_path, nn, imageManip);
// }
dai::CameraControl NNParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver