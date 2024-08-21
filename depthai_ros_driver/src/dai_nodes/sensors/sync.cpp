#include "depthai_ros_driver/dai_nodes/sensors/sync.hpp"

#include <chrono>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/sync_param_handler.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

Sync::Sync(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    syncNode = pipeline->create<dai::node::Sync>();
    paramHandler = std::make_unique<param_handlers::SyncParamHandler>(node, daiNodeName);
    paramHandler->declareParams(syncNode);
    setNames();
    setXinXout(pipeline);
}

Sync::~Sync() = default;

void Sync::setNames() {
    syncOutputName = getName() + "_out";
}
void Sync::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutFrame = pipeline->create<dai::node::XLinkOut>();
    xoutFrame->setStreamName(syncOutputName);
    xoutFrame->input.setBlocking(false);
    syncNode->out.link(xoutFrame->input);
}

void Sync::setupQueues(std::shared_ptr<dai::Device> device) {
    outQueue = device->getOutputQueue(syncOutputName, 8, false);
    outQueue->addCallback([this](const std::shared_ptr<dai::ADatatype>& in) {
        auto group = std::dynamic_pointer_cast<dai::MessageGroup>(in);
        if(group) {
            bool firstMsg = true;
            rclcpp::Time timestamp;
            for(auto& msg : *group) {
                // find publisher by message namespace
                for(auto& pub : publishers) {
                    if(pub->getQueueName() == msg.first) {
                        auto data = pub->convertData(msg.second);
                        if(firstMsg) {
                            timestamp = data->info->header.stamp;
                            firstMsg = false;
                        }
                        pub->publish(std::move(data), timestamp);
                    }
                }
            }
        }
    });
}

void Sync::link(dai::Node::Input in, int /* linkType */) {
    syncNode->out.link(in);
}

dai::Node::Input Sync::getInputByName(const std::string& name) {
    syncNode->inputs[name].setBlocking(false);
    return syncNode->inputs[name];
}

void Sync::closeQueues() {
    outQueue->close();
}

void Sync::addPublishers(const std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>>& pubs) {
    for(auto& pub : pubs) {
        pub->link(getInputByName(pub->getQueueName()));
    }
    publishers.insert(publishers.end(), pubs.begin(), pubs.end());
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
