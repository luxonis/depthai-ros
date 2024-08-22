#include "depthai_ros_driver/utils.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
namespace depthai_ros_driver {
namespace utils {
std::string getUpperCaseStr(const std::string& string) {
    std::string upper = string;
    for(auto& c : upper) c = toupper(c);
    return upper;
}
std::shared_ptr<dai::node::XLinkOut> setupXout(std::shared_ptr<dai::Pipeline> pipeline, const std::string& name) {
    auto xout = pipeline->create<dai::node::XLinkOut>();
    xout->setStreamName(name);
    xout->input.setBlocking(false);
    xout->input.setWaitForMessage(false);
    xout->input.setQueueSize(1);
    return xout;
};
}  // namespace utils
}  // namespace depthai_ros_driver
