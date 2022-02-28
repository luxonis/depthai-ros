
#include <depthai_bridge/DisparityConverter.hpp>

namespace dai {

namespace ros {

DisparityConverter::DisparityConverter(const std::string frameName, float focalLength, float baseline, float minDepth, float maxDepth)
    : _frameName(frameName), _focalLength(focalLength), _baseline(baseline / 100.0), _minDepth(minDepth / 100.0), _maxDepth(maxDepth / 100.0) {}

void DisparityConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outDispImageMsgs) {
    auto tstamp = inData->getTimestamp();
    DisparityMsgs::DisparityImage outDispImageMsg;
    outDispImageMsg.header.frame_id = _frameName;
    outDispImageMsg.f = _focalLength;
    outDispImageMsg.min_disparity = _focalLength * _baseline / _maxDepth;
    outDispImageMsg.max_disparity = _focalLength * _baseline / _minDepth;

#ifdef IS_ROS2
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    outDispImageMsg.header.stamp = rclStamp;
    outDispImageMsg.t = _baseline / 100;  // converting cm to meters
    sensor_msgs::msg::Image& outImageMsg = outDispImageMsg.image;
#else
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    uint64_t nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    outDispImageMsg.header.stamp = rosStamp;

    outDispImageMsg.header.seq = inData->getSequenceNum();
    outDispImageMsg.T = _baseline / 100;  // converting cm to meters
    sensor_msgs::Image& outImageMsg = outDispImageMsg.image;
#endif

    // copying the data to ros msg
    // outDispImageMsg.header       = imgHeader;
    // std::string temp_str(encodingEnumMap[inData->getType()]);
    outImageMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    outImageMsg.header = outDispImageMsg.header;
    if(inData->getType() == dai::RawImgFrame::Type::RAW8) {
        outDispImageMsg.delta_d = 1;
        size_t size = inData->getData().size() * sizeof(float);
        outImageMsg.data.resize(size);
        outImageMsg.height = inData->getHeight();
        outImageMsg.width = inData->getWidth();
        outImageMsg.step = size / inData->getHeight();
        outImageMsg.is_bigendian = true;

        std::vector<float> convertedData(inData->getData().begin(), inData->getData().end());
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(outImageMsg.data.data());

        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(convertedData.data());

        // TODO(Sachin): Try using assign since it is a vector
        // img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);

    } else {
        outDispImageMsg.delta_d = 1 / 32;
        size_t size = inData->getHeight() * inData->getWidth() * sizeof(float);
        outImageMsg.data.resize(size);
        outImageMsg.height = inData->getHeight();
        outImageMsg.width = inData->getWidth();
        outImageMsg.step = size / inData->getHeight();
        outImageMsg.is_bigendian = true;
        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());

        std::vector<int16_t> raw16Data(inData->getHeight() * inData->getWidth());
        unsigned char* raw16DataPtr = reinterpret_cast<unsigned char*>(raw16Data.data());
        memcpy(raw16DataPtr, daiImgData, inData->getData().size());
        std::vector<float> convertedData;
        std::transform(
            raw16Data.begin(), raw16Data.end(), std::back_inserter(convertedData), [](int16_t disp) -> std::size_t { return static_cast<float>(disp) / 32.0; });

        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(outImageMsg.data.data());
        unsigned char* convertedDataPtr = reinterpret_cast<unsigned char*>(convertedData.data());
        memcpy(imageMsgDataPtr, convertedDataPtr, size);
    }
    outDispImageMsgs.push_back(outDispImageMsg);
    return;
}

/* void DisparityConverter::toDaiMsg(const DisparityMsgs::DisparityImage &inMsg,
                              dai::ImgFrame& outData) {

  std::unordered_map<dai::RawImgFrame::Type, std::string>::iterator
      revEncodingIter;
  if (_daiInterleaved) {
    revEncodingIter = std::find_if(
        encodingEnumMap.begin(), encodingEnumMap.end(),
        [&](const std::pair<dai::RawImgFrame::Type, std::string> &pair) {
          return pair.second == inMsg.encoding;
        });
    if (revEncodingIter == encodingEnumMap.end())
      std::runtime_error("Unable to find DAI encoding for the corresponding "
                         "DisparityMsgs::DisparityImage.encoding stream");

    outData.setData(inMsg.data);
  } else {
    revEncodingIter = std::find_if(
        encodingEnumMap.begin(), encodingEnumMap.end(),
        [&](const std::pair<dai::RawImgFrame::Type, std::string> &pair) {
          return pair.second.find(inMsg.encoding) != std::string::npos;
        });

    std::istringstream f(revEncodingIter->second);
    std::vector<std::string> encoding_info;
    std::string s;

    while (getline(f, s, '_'))
      encoding_info.push_back(s);

    std::vector<std::uint8_t> opData(inMsg.data.size());
    interleavedToPlanar(inMsg.data, opData, inMsg.height, inMsg.width,
                        std::stoi(encoding_info[0]),
                        std::stoi(encoding_info[1]));
    outData.setData(opData);
  }


  TimePoint ts(std::chrono::seconds((int)inMsg.header.stamp.toSec()) +
               std::chrono::nanoseconds(inMsg.header.stamp.toNSec()));
  outData.setTimestamp(ts);
  outData.setSequenceNum(inMsg.header.seq);
  outData.setWidth(inMsg.width);
  outData.setHeight(inMsg.height);
  outData.setType(revEncodingIter->first);
} */

DisparityImagePtr DisparityConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
    std::deque<DisparityMsgs::DisparityImage> msgQueue;
    toRosMsg(inData, msgQueue);
    auto msg = msgQueue.front();
#ifdef IS_ROS2
    DisparityImagePtr ptr = std::make_shared<DisparityMsgs::DisparityImage>(msg);
#else
    DisparityImagePtr ptr = boost::make_shared<DisparityMsgs::DisparityImage>(msg);
#endif
    return ptr;
}

}  // namespace ros
}  // namespace dai