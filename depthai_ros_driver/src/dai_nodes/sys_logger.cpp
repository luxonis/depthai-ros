#include "depthai_ros_driver/dai_nodes/sys_logger.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
SysLogger::SysLogger(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    sysNode = pipeline->create<dai::node::SystemLogger>();
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
SysLogger::~SysLogger() = default;

void SysLogger::setNames() {
    loggerQName = getName() + "_queue";
}

void SysLogger::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutLogger = pipeline->create<dai::node::XLinkOut>();
    xoutLogger->setStreamName(loggerQName);
    sysNode->out.link(xoutLogger->input);
}

void SysLogger::setupQueues(std::shared_ptr<dai::Device> device) {
    loggerQ = device->getOutputQueue(loggerQName, 8, false);
    updater = std::make_shared<diagnostic_updater::Updater>(getROSNode());
    updater->setHardwareID(getROSNode()->get_name() + std::string("_") + device->getMxId() + std::string("_") + device->getDeviceName());
    updater->add("sys_logger", std::bind(&SysLogger::produceDiagnostics, this, std::placeholders::_1));
}

void SysLogger::closeQueues() {
    loggerQ->close();
}

std::string SysLogger::sysInfoToString(const dai::SystemInformation& sysInfo) {
    std::stringstream ss;
    ss << "System Information: " << std::endl;
    ss << "  Leon CSS CPU Usage: " << sysInfo.leonCssCpuUsage.average * 100 << std::endl;
    ss << "  Leon MSS CPU Usage: " << sysInfo.leonMssCpuUsage.average * 100 << std::endl;
    ss << " Ddr Memory Usage: " << sysInfo.ddrMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Ddr Memory Total: " << sysInfo.ddrMemoryUsage.total / (1024.0f * 1024.0f) << std::endl;
    ss << " Cmx Memory Usage: " << sysInfo.cmxMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Cmx Memory Total: " << sysInfo.cmxMemoryUsage.total << std::endl;
    ss << " Leon CSS Memory Usage: " << sysInfo.leonCssMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Leon CSS Memory Total: " << sysInfo.leonCssMemoryUsage.total / (1024.0f * 1024.0f) << std::endl;
    ss << " Leon MSS Memory Usage: " << sysInfo.leonMssMemoryUsage.used / (1024.0f * 1024.0f) << std::endl;
    ss << " Leon MSS Memory Total: " << sysInfo.leonMssMemoryUsage.total / (1024.0f * 1024.0f) << std::endl;
    ss << " Average Chip Temperature: " << sysInfo.chipTemperature.average << std::endl;
    ss << " Leon CSS Chip Temperature: " << sysInfo.chipTemperature.css << std::endl;
    ss << " Leon MSS Chip Temperature: " << sysInfo.chipTemperature.mss << std::endl;
    ss << " UPA Chip Temperature: " << sysInfo.chipTemperature.upa << std::endl;
    ss << " DSS Chip Temperature: " << sysInfo.chipTemperature.dss << std::endl;

    return ss.str();
}

void SysLogger::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    try {
        bool timeout;
        auto logData = loggerQ->get<dai::SystemInformation>(std::chrono::seconds(5), timeout);
        if(!timeout) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "System Information");
            stat.add("System Information", sysInfoToString(*logData));
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No Data");
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(getROSNode()->get_logger(), "No data on logger queue!");
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, e.what());
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
