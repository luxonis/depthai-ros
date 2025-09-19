#include "depthai_bridge/TransformDataConverter.hpp"

#include "depthai/utility/matrixOps.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace depthai_bridge {

TransformDataConverter::TransformDataConverter(std::string frameName, std::string childFrameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), childFrameName(childFrameName), convertToSocket(false) {}

TransformDataConverter::~TransformDataConverter() = default;
void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inTransform, std::deque<nav_msgs::msg::Odometry>& odomMsgs) {
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header = getRosHeader(inTransform);
    odomMsg.header.frame_id = frameName;
    odomMsg.child_frame_id = childFrameName;
    auto tf = convertToFrame(inTransform);
    auto trans = inTransform->getTranslation();
    auto quat = inTransform->getQuaternion();
    odomMsg.pose.pose.position.x = trans.x;
    odomMsg.pose.pose.position.y = trans.y;
    odomMsg.pose.pose.position.z = trans.z;
    odomMsg.pose.pose.orientation.x = quat.qx;
    odomMsg.pose.pose.orientation.y = quat.qy;
    odomMsg.pose.pose.orientation.z = quat.qz;
    odomMsg.pose.pose.orientation.w = quat.qw;
    odomMsgs.push_back(odomMsg);
}

void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inTransform, std::deque<geometry_msgs::msg::TransformStamped>& transformMsgs) {
    geometry_msgs::msg::TransformStamped transformMsg;
    transformMsg.header = getRosHeader(inTransform);
    transformMsg.header.frame_id = frameName;
    transformMsg.child_frame_id = childFrameName;
    auto tf = convertToFrame(inTransform);
    auto trans = inTransform->getTranslation();
    auto quat = inTransform->getQuaternion();
    transformMsg.transform.translation.x = trans.x;
    transformMsg.transform.translation.y = trans.y;
    transformMsg.transform.translation.z = trans.z;
    transformMsg.transform.rotation.x = quat.qx;
    transformMsg.transform.rotation.y = quat.qy;
    transformMsg.transform.rotation.z = quat.qz;
    transformMsg.transform.rotation.w = quat.qw;
    transformMsgs.push_back(transformMsg);
}

void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inTransform, std::deque<geometry_msgs::msg::PoseWithCovarianceStamped>& poseMsgs) {
    geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
    poseMsg.header = getRosHeader(inTransform);
    poseMsg.header.frame_id = frameName;
    auto tf = convertToFrame(inTransform);
    auto trans = inTransform->getTranslation();
    auto quat = inTransform->getQuaternion();
    poseMsg.pose.pose.position.x = trans.x;
    poseMsg.pose.pose.position.y = trans.y;
    poseMsg.pose.pose.position.z = trans.z;
    poseMsg.pose.pose.orientation.x = quat.qx;
    poseMsg.pose.pose.orientation.y = quat.qy;
    poseMsg.pose.pose.orientation.z = quat.qz;
    poseMsg.pose.pose.orientation.w = quat.qw;
    poseMsgs.push_back(poseMsg);
}

std::vector<std::vector<float>> TransformDataConverter::convertMatrixToFloat(const std::array<std::array<double, 4>, 4>& matrix) {
    std::vector<std::vector<float>> result;
    for(const auto& row : matrix) {
        std::vector<float> floatRow;
        for(double val : row) {
            floatRow.push_back(static_cast<float>(val));
        }
        result.push_back(floatRow);
    }
    return result;
}
std::vector<std::vector<float>> TransformDataConverter::multiplyMatrices(const std::vector<std::vector<float>>& a, const std::vector<std::vector<float>>& b) {
    std::vector<std::vector<float>> result(4, std::vector<float>(4, 0.0f));
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return result;
}
std::shared_ptr<dai::TransformData> TransformDataConverter::convertToFrame(std::shared_ptr<dai::TransformData> transform) {
    if(convertToSocket) {
        auto tf = std::make_shared<dai::TransformData>();
        auto tf1f = convertMatrixToFloat(transform->transform.matrix);
        auto tf2f = convertMatrixToFloat(transformToSocket->transform.matrix);
        auto tf3 = multiplyMatrices(tf1f, tf2f);

        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                tf->transform.matrix[i][j] = tf3[i][j];
            }
        }
        return tf;
    }
    return transform;
}
void depthai_bridge::TransformDataConverter::convertFromImuFrameToSocket(dai::CameraBoardSocket socket, dai::CalibrationHandler calHandler) {
    socketToTransformTo = socket;
    this->calHandler = calHandler;
    auto T = calHandler.getImuToCameraExtrinsics(socket);
    transformToSocket = std::make_shared<dai::TransformData>();

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            transformToSocket->transform.matrix[i][j] = T[i][j];
        }
    }

    convertToSocket = true;
}
}  // namespace depthai_bridge
