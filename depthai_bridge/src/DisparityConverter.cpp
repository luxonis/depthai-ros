#include "depthai_bridge/DisparityConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

DisparityConverter::DisparityConverter(
    const std::string frameName, float focalLength, float baseline, float minDepth, float maxDepth, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp),
      focalLength(focalLength),
      baseline(baseline / 100.0),
      minDepth(minDepth / 100.0),
      maxDepth(maxDepth / 100.0) {}

DisparityConverter::~DisparityConverter() = default;

void DisparityConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outDispImageMsgs) {
    DisparityMsgs::DisparityImage outDispImageMsg;
    outDispImageMsg.header = getRosHeader(inData);
    outDispImageMsg.f = focalLength;
    outDispImageMsg.min_disparity = focalLength * baseline / maxDepth;
    outDispImageMsg.max_disparity = focalLength * baseline / minDepth;

    outDispImageMsg.t = baseline / 100.0;  // converting cm to meters

    ImageMsgs::Image& outImageMsg = outDispImageMsg.image;
    outImageMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    outImageMsg.header = outDispImageMsg.header;
    if(inData->getType() == dai::ImgFrame::Type::RAW8) {
        outDispImageMsg.delta_d = 1.0;
        size_t size = inData->getData().size() * sizeof(float);
        outImageMsg.data.resize(size);
        outImageMsg.height = inData->getHeight();
        outImageMsg.width = inData->getWidth();
        outImageMsg.step = size / inData->getHeight();
        outImageMsg.is_bigendian = true;

        std::vector<float> convertedData(inData->getData().begin(), inData->getData().end());
        outImageMsg.data.assign(convertedData.begin(), convertedData.end());
    } else {
        outDispImageMsg.delta_d = 1.0 / 32.0;
        size_t size = inData->getHeight() * inData->getWidth() * sizeof(float);
        outImageMsg.data.resize(size);
        outImageMsg.height = inData->getHeight();
        outImageMsg.width = inData->getWidth();
        outImageMsg.step = size / inData->getHeight();
        outImageMsg.is_bigendian = true;

        std::vector<float> convertedData;
        convertedData.reserve(inData->getHeight() * inData->getWidth());

        std::transform(reinterpret_cast<const int16_t*>(inData->getData().data()),
                       reinterpret_cast<const int16_t*>(inData->getData().data() + inData->getData().size()),
                       std::back_inserter(convertedData),
                       [](int16_t disp) -> float { return static_cast<float>(disp) / 32.0; });

        outImageMsg.data.assign(convertedData.begin(), convertedData.end());
    }
    outDispImageMsgs.push_back(outDispImageMsg);
    return;
}

DisparityImagePtr DisparityConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
    std::deque<DisparityMsgs::DisparityImage> msgQueue;
    toRosMsg(inData, msgQueue);
    auto msg = msgQueue.front();

    DisparityImagePtr ptr = std::make_shared<DisparityMsgs::DisparityImage>(msg);

    return ptr;
}

// Getter methods
float DisparityConverter::getFocalLength() const {
    return focalLength;
}

float DisparityConverter::getBaseline() const {
    return baseline;
}

float DisparityConverter::getMinDepth() const {
    return minDepth;
}

float DisparityConverter::getMaxDepth() const {
    return maxDepth;
}
}  // namespace depthai_bridge
