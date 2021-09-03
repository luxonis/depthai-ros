#include <depthai_bridge/ImageConverter.hpp>
#include "rclcpp/rclcpp.hpp"
#include <chrono>

// FIXME(Sachin): Do I need to convert the encodings that are available in dai
// to only that ros support ? I mean we can publish whatever it is and decode it
// on the other side but howver maybe we should have option to convert planar to
// interleaved before publishing ???

// By default everthing form dai is changed to interleaved when publishing over
// ros. and if we subscribe to a previously published ros msg as input to
// xlinkin node then we need to convert it back to planar if xlinkin node needs
// it planar
namespace dai::rosBridge {

 std::unordered_map<dai::RawImgFrame::Type, std::string> ImageConverter::encodingEnumMap = {
            {dai::RawImgFrame::Type::YUV422i        , "yuv422"               },
            {dai::RawImgFrame::Type::RGBA8888       , "rgba8"                },
            {dai::RawImgFrame::Type::RGB888i        , "rgb8"                 },
            {dai::RawImgFrame::Type::BGR888i        , "bgr8"                 },
            {dai::RawImgFrame::Type::GRAY8          , "mono8"                },
            {dai::RawImgFrame::Type::RAW8           , "mono8"                 },
            {dai::RawImgFrame::Type::RAW16          , "16UC1"                },
            // {dai::RawImgFrame::Type::NV12           : "CV_bridge" },
        };

std::unordered_map<dai::RawImgFrame::Type, std::string> ImageConverter::planarEncodingEnumMap = {
                                    {dai::RawImgFrame::Type::BGR888p, "3_1_bgr8"}, // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
                                    {dai::RawImgFrame::Type::RGB888p, "3_1_bgr8"}, // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
                                    {dai::RawImgFrame::Type::NV12   , "nv12"                 } 

                                };

ImageConverter::ImageConverter(bool interleaved)
    : _daiInterleaved(interleaved) {}

ImageConverter::ImageConverter(const std::string frameName,
                               bool interleaved)
    : _frameName(frameName), _daiInterleaved(interleaved) {}

void ImageConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData,
                              sensor_msgs::msg::Image &outImageMsg) {

  if (_daiInterleaved && encodingEnumMap.find(inData->getType()) == encodingEnumMap.end()){
      if (planarEncodingEnumMap.find(inData->getType()) != planarEncodingEnumMap.end())
        throw std::runtime_error("Encoding value found for planar dataformat but object was created with 'interleaved = true'. ");
      else
        throw std::runtime_error("Encoding value not found. ");
  } 
    
  if (!_daiInterleaved && planarEncodingEnumMap.find(inData->getType()) == planarEncodingEnumMap.end()){
      if (encodingEnumMap.find(inData->getType()) != encodingEnumMap.end())
        throw std::runtime_error("Encoding value found for Interleaved dataformat but object was created with 'Interleaved = false'. ");
      else
        throw std::runtime_error("Encoding convertion not found. ");
  } 

  auto tstamp = inData->getTimestamp();
  auto rclNow = rclcpp::Clock().now();
  auto steadyTime = std::chrono::steady_clock::now();
  auto diffTime = steadyTime - tstamp;
  auto rclStamp = rclNow - diffTime;

  // std::cout << "Image TIme stamp  ->" << rclStamp << std::endl;
  // std::cout << "rcl Cpp stamp  ->" << rclNow << std::endl;
  // std::cout << "steady Cpp stamp  ->" << std::chrono::steady_clock::now() << std::endl;

  int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(
                    tstamp.time_since_epoch())
                    .count();
  int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     tstamp.time_since_epoch())
                     .count() %
                 1000000000UL;

  outImageMsg.header.stamp = rclStamp;
  outImageMsg.header.frame_id = _frameName;
    
  if (!_daiInterleaved) {

    std::istringstream f(planarEncodingEnumMap[inData->getType()]);
    std::vector<std::string> encoding_info;
    std::string s;

    while (getline(f, s, '_'))
      encoding_info.push_back(s);
    outImageMsg.height   = inData->getHeight();
    outImageMsg.width    = inData->getWidth();
    // FIXME(sachin): This might be wrong for NV12. Fix it
    outImageMsg.step     = inData->getData().size() / inData->getHeight(); 
    outImageMsg.is_bigendian = true;
    size_t size = inData->getData().size();
    outImageMsg.data.resize(size);
      if(planarEncodingEnumMap[inData->getType()] == "nv12"){
        outImageMsg.encoding = planarEncodingEnumMap[inData->getType()];
        outImageMsg.data = std::move(inData->getData());
      }
      else{
        outImageMsg.encoding = encoding_info[2];
        planarToInterleaved(inData->getData(), outImageMsg.data, outImageMsg.width,
                            outImageMsg.height, std::stoi(encoding_info[0]),
                            std::stoi(encoding_info[1]));
      }
    } 
    else {
    // copying the data to ros msg
    // outImageMsg.header       = imgHeader;
    std::string temp_str(encodingEnumMap[inData->getType()]);
    outImageMsg.encoding = temp_str;
    outImageMsg.height = inData->getHeight();
    outImageMsg.width = inData->getWidth();
    outImageMsg.step = inData->getData().size() / inData->getHeight();
    if (outImageMsg.encoding == "16UC1")
      outImageMsg.is_bigendian = false;
    else
      outImageMsg.is_bigendian = true;

    size_t size = inData->getData().size();
    outImageMsg.data.resize(size);
    unsigned char *imageMsgDataPtr =
        reinterpret_cast<unsigned char *>(&outImageMsg.data[0]);
    unsigned char *daiImgData =
        reinterpret_cast<unsigned char *>(inData->getData().data());

    // TODO(Sachin): Try using assign since it is a vector
    // img->data.assign(packet.data->cbegin(), packet.data->cend());
    memcpy(imageMsgDataPtr, daiImgData, size);
  }
  return;
}

// TODO(sachin): Not tested
void ImageConverter::toDaiMsg(const sensor_msgs::msg::Image &inMsg,
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
                         "sensor_msgs::image.encoding stream");

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

  /** FIXME(sachin) : is this time convertion correct ???
   * Print the original time and ros time in seconds in
   * ImageFrame::toRosMsg(std::shared_ptr<dai::ImgFrame> inData,
   *sensor_msgs::Image& opMsg) to cross verify..
   **/
  TimePoint ts(std::chrono::seconds((int)inMsg.header.stamp.sec) +
               std::chrono::nanoseconds(inMsg.header.stamp.nanosec));
  outData.setTimestamp(ts);
  outData.setWidth(inMsg.width);
  outData.setHeight(inMsg.height);
  outData.setType(revEncodingIter->first);
  return;
}

sensor_msgs::msg::Image::SharedPtr 
ImageConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
  sensor_msgs::msg::Image::SharedPtr ptr = std::make_shared<sensor_msgs::msg::Image>();
  toRosMsg(inData, *ptr);
  return ptr;
}

void ImageConverter::planarToInterleaved(const std::vector<uint8_t> &srcData,
                                         std::vector<uint8_t> &destData, int w,
                                         int h, int numPlanes, int bpp) {

  if (numPlanes == 3) {
    // optimization (cache)
    for (int i = 0; i < w * h; i++) {
      uint8_t b = srcData.data()[i + w * h * 0];
      destData[i * 3 + 0] = b;
    }
    for (int i = 0; i < w * h; i++) {
      uint8_t g = srcData.data()[i + w * h * 1];
      destData[i * 3 + 1] = g;
    }
    for (int i = 0; i < w * h; i++) {
      uint8_t r = srcData.data()[i + w * h * 2];
      destData[i * 3 + 2] = r;
    }
  } else {
    std::runtime_error("If you encounter the scenario where you need this "
                       "please create an issue on github");
  }
  return;
}

void ImageConverter::interleavedToPlanar(const std::vector<uint8_t> &srcData,
                                         std::vector<uint8_t> &destData, int w,
                                         int h, int numPlanes, int bpp) {
  if (numPlanes == 3) {
    // optimization (cache)
    for (int i = 0; i < w * h; i++) {

      uint8_t b = srcData[i * 3 + 0];
      uint8_t g = srcData[i * 3 + 1];
      uint8_t r = srcData[i * 3 + 2];

      destData[i + w * h * 0] = b;
      destData[i + w * h * 1] = g;
      destData[i + w * h * 2] = r;
    }
    // for(int i = 0; i < w*h; i++) {
    //     uint8_t g = srcData.data()[i + w*h * 1];
    //     destData[i*3+1] = g;
    // }
    // for(int i = 0; i < w*h; i++) {
    //     uint8_t r = srcData.data()[i + w*h * 2];
    //     destData[i*3+2] = r;
    // }
  } else {
    std::runtime_error("If you encounter the scenario where you need this "
                       "please create an issue on github");
  }
  return;
}

cv::Mat ImageConverter::rosMsgtoCvMat(sensor_msgs::msg::Image &inMsg) {
  cv::Mat rgb(inMsg.height, inMsg.width, CV_8UC3);
  if(inMsg.encoding == "nv12"){
    cv::Mat nv_frame(inMsg.height * 3 / 2, inMsg.width, CV_8UC1, inMsg.data.data());    
    cv::cvtColor(nv_frame, rgb, cv::COLOR_YUV2BGR_NV12);
    return rgb; 
  }
  else{
    std::runtime_error("THis frature is still WIP");
  }
  return rgb;
}

} // namespace dai::rosBridge
