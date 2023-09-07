#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "diagnostic_updater/diagnostic_updater.h"
namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class SystemLogger;
class XLinkOut;
class VideoEncoder;
}  // namespace node
}  // namespace dai

namespace depthai_ros_driver {

namespace dai_nodes {
class SysLogger : public BaseNode {
   public:
    SysLogger(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline);
    ~SysLogger();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void timerCB();
    std::string sysInfoToString(const dai::SystemInformation& sysInfo);
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    std::shared_ptr<diagnostic_updater::Updater> updater;
    std::shared_ptr<dai::node::XLinkOut> xoutLogger;
    std::shared_ptr<dai::node::SystemLogger> sysNode;
    std::shared_ptr<dai::DataOutputQueue> loggerQ;
    std::string loggerQName;
    ros::Timer timer;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver