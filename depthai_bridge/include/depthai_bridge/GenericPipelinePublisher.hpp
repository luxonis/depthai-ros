#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
#else
    #include <ros/ros.h>
#endif

#include <depthai/depthai.hpp>
#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/nodes.hpp>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace dai {
namespace ros {
class GenericPipelinePublisher {
    ::ros::NodeHandle& _pnh;
    dai::Device& _device;
    std::string _frame_prefix;
    std::map<CameraBoardSocket, std::string> _frameNames;

    std::map<CameraBoardSocket, ::ros::NodeHandle> _nodeHandles;
    ::ros::NodeHandle& getNodeHandle(CameraBoardSocket socket) {
        if(_nodeHandles.find(socket) == _nodeHandles.end()) {
            _nodeHandles[socket] = ::ros::NodeHandle(_pnh, _frameNames[socket]);
        }
        return _nodeHandles[socket];
    }

    std::vector<std::shared_ptr<void>> keep_alive;
    std::vector<std::shared_ptr<ImageConverter>> converters;
    std::vector<std::shared_ptr<BridgePublisherBase>> publishers;
    ::ros::Subscriber ae_boundingbox, af_boundingbox;
    CalibrationHandler _calibrationHandler;

    bool mapKnownInputNodeType(std::shared_ptr<dai::node::XLinkOut> xLinkOut, std::shared_ptr<dai::node::StereoDepth> inputNode, const std::string& inputName);
    bool mapKnownInputNodeType(std::shared_ptr<dai::node::XLinkOut> xLinkOut, std::shared_ptr<dai::node::IMU> inputNode, const std::string& inputName);
    bool mapKnownInputNodeType(std::shared_ptr<dai::node::XLinkOut> xLinkOut, std::shared_ptr<dai::node::ColorCamera> inputNode, const std::string& inputName);
    bool mapKnownInputNodeType(std::shared_ptr<dai::node::XLinkOut> xLinkOut, std::shared_ptr<dai::node::MonoCamera> inputNode, const std::string& inputName);

    template <int None = 0>
    bool mapKnownInputNodeTypes(std::shared_ptr<dai::node::XLinkOut> xLinkOut, std::shared_ptr<dai::Node> inputNode, const std::string& inputName) {
        return false;
    }

    template <typename T, typename... Args>
    bool mapKnownInputNodeTypes(std::shared_ptr<dai::node::XLinkOut> xLinkOut, std::shared_ptr<dai::Node> inputNode, const std::string& inputName) {
        auto inputNodeTyped = std::dynamic_pointer_cast<T>(inputNode);
        if(inputNodeTyped && mapKnownInputNodeType(xLinkOut, inputNodeTyped, inputName)) {
            return true;
        }
        return mapKnownInputNodeTypes<Args...>(xLinkOut, inputNode, inputName);
    }
    void mapOutputStream(dai::Pipeline& pipeline, std::shared_ptr<dai::node::XLinkOut> xLinkOut, const dai::Node::Connection& connection);

    template<typename T> void setupCameraControlQueue(std::shared_ptr<T> cam, const std::string& prefix);
    template<typename T> void setupCameraControlServer(std::shared_ptr<T> cam, const std::string& prefix);
    void mapNode(dai::Pipeline& pipeline, std::shared_ptr<dai::Node> node);
    void addConfigNodes(dai::Pipeline& pipeline, std::shared_ptr<dai::Node> node);

   public:
    void BuildPublisherFromPipeline(dai::Pipeline& pipeline);
    GenericPipelinePublisher(::ros::NodeHandle& pnh, dai::Device& device, dai::Pipeline& pipeline);
    GenericPipelinePublisher(::ros::NodeHandle& pnh, dai::Device& device, dai::Pipeline& pipeline, const std::string& frame_prefix);
    GenericPipelinePublisher(::ros::NodeHandle& pnh,
                             dai::Device& device,
                             dai::Pipeline& pipeline,
                             const std::map<CameraBoardSocket, std::string>& frameNames,
                             const std::string& frame_prefix);
};
}  // namespace ros
}  // namespace dai