#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(
    std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat, const dai::CameraBoardSocket& socket)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"detection", nn::NNFamily::Detection},
    };
    declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(socket));
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 8);
}
NNParamHandler::~NNParamHandler() = default;
nn::NNFamily NNParamHandler::getNNFamily() {
    std::string nnFamily = declareAndLogParam<std::string>("i_nn_family", "detection");
    declareAndLogParam<std::string>("i_nn_model", "yolov6-nano");
    return utils::getValFromMap(nnFamily, nnFamilyMap);
}

void NNParamHandler::setNNParams(std::shared_ptr<dai::node::NeuralNetwork> /*nn*/) {}

void NNParamHandler::setNNParams(std::shared_ptr<dai::node::DetectionNetwork> nn) {
    nn->setConfidenceThreshold(declareAndLogParam<float>("i_nn_confidence_threshold", 0.5));
}

void NNParamHandler::setNNParams(std::shared_ptr<dai::node::SpatialDetectionNetwork> nn) {
    nn->setConfidenceThreshold(declareAndLogParam<float>("i_nn_confidence_threshold", 0.5));
    setSpatialParams(nn);
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
