#include "depthai_bridge/PointCloudConverter.hpp"

#include "sensor_msgs/msg/point_field.hpp"

namespace depthai_bridge {

PointCloudConverter::PointCloudConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), scaleFactor(1.0) {}

PointCloudConverter::~PointCloudConverter() = default;

void PointCloudConverter::setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit) {
    // Default is millimeter
    switch(depthUnit) {
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER:
            scaleFactor = 1.0f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER:
            scaleFactor = 0.001f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER:
            scaleFactor = 0.01f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT:
            scaleFactor = 0.3048f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::INCH:
            scaleFactor = 0.0254f;
            break;
        case dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM:
            scaleFactor = 1.0f;
            break;
    }
}
double PointCloudConverter::getScaleFactor() const {
    return scaleFactor;
}
void PointCloudConverter::toRosMsg(std::shared_ptr<dai::PointCloudData> inPcl, std::deque<sensor_msgs::msg::PointCloud2>& pclMsgs) {
    sensor_msgs::msg::PointCloud2 msg;
    bool isColored = inPcl->isColor();
    unsigned int width = inPcl->getWidth();
    unsigned int height = inPcl->getHeight();
    msg.header = getRosHeader(inPcl);
    msg.width = width;
    msg.height = height;
    msg.is_dense = !inPcl->isSparse();

    msg.fields.clear();
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    msg.fields.push_back(field_x);

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    msg.fields.push_back(field_y);

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    msg.fields.push_back(field_z);

    uint32_t pointStep = 12;
    if(isColored) {
        sensor_msgs::msg::PointField field_rgb;
        field_rgb.name = "rgb";
        field_rgb.offset = 12;
        field_rgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_rgb.count = 1;
        msg.fields.push_back(field_rgb);
        pointStep += 4;
    }
    msg.point_step = pointStep;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    if(isColored) {
        auto points = inPcl->getPointsRGB();
        for(size_t i = 0; i < points.size(); ++i) {
            float* ptr = reinterpret_cast<float*>(&msg.data[i * pointStep]);
            const auto& pt = points[i];
            ptr[0] = pt.x * scaleFactor;
            ptr[1] = pt.y * scaleFactor;
            ptr[2] = pt.z * scaleFactor;
            uint32_t rgb = (pt.r << 16) | (pt.g << 8) | pt.b;
            float rgb_float;
            std::memcpy(&rgb_float, &rgb, sizeof(float));
            ptr[3] = rgb_float;
        }
    } else {
        auto points = inPcl->getPoints();
        for(size_t i = 0; i < points.size(); ++i) {
            float* ptr = reinterpret_cast<float*>(&msg.data[i * pointStep]);
            const auto& pt = points[i];
            ptr[0] = pt.x * scaleFactor;
            ptr[1] = pt.y * scaleFactor;
            ptr[2] = pt.z * scaleFactor;
        }
    }

    pclMsgs.push_back(std::move(msg));
}

}  // namespace depthai_bridge
