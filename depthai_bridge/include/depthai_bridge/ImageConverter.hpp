#pragma once

#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>

#include "cv_bridge/cv_bridge.h"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace dai {

namespace ros {

namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
namespace FFMPEGMsgs = ffmpeg_image_transport_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using FFMPEGImagePtr = FFMPEGMsgs::FFMPEGPacket::SharedPtr;
using CompImagePtr = ImageMsgs::CompressedImage::SharedPtr;

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class ImageConverter {
   public:
    // ImageConverter() = default;
    ImageConverter(const std::string frameName, bool interleaved, bool getBaseDeviceTimestamp = false);
    ~ImageConverter();
    ImageConverter(bool interleaved, bool getBaseDeviceTimestamp = false);

    /**
     * @brief Handles cases in which the ROS time shifts forward or backward
     *  Should be called at regular intervals or on-change of ROS time, depending
     *  on monitoring.
     *
     */
    void updateRosBaseTime();

    /**
     * @brief Commands the converter to automatically update the ROS base time on message conversion based on variable
     *
     * @param update: bool whether to automatically update the ROS base time on message conversion
     */
    void setUpdateRosBaseTimeOnToRosMsg(bool update = true) {
        updateRosBaseTimeOnToRosMsg = update;
    }

    /**
     * @brief Sets converter behavior to convert from bitstream to raw data.
     * @param srcType: The type of the bitstream data used for conversion.
     */
    void convertFromBitstream(dai::RawImgFrame::Type srcType);

    /**
     * @brief Sets exposure offset when getting timestamps from the message.
     * @param offset: The exposure offset to be added to the timestamp.
     */
    void addExposureOffset(dai::CameraExposureOffset& offset);

    /**
     * @brief Sets converter behavior to convert from disparity to depth when converting messages from bitstream.
     * @param baseline: The baseline of the stereo pair.
     */
    void convertDispToDepth(double baseline);

    /**
     * @brief Reverses the order of the stereo sockets when creating CameraInfo to calculate Tx component of Projection matrix.
     * By default the right socket is used as the base, calling this function will set left as base.
     */
    void reverseStereoSocketOrder();

    /**
     * @brief Sets the alpha scaling factor for the image.
     * @param alphaScalingFactor: The alpha scaling factor to be used.
     */
    void setAlphaScaling(double alphaScalingFactor = 0.0);

    /**
     * @brief Sets the encoding of the image when converting to FFMPEG message. Default is libx264.
     * @param encoding: The encoding to be used.
     */
    void setFFMPEGEncoding(const std::string& encoding);

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs);
    ImageMsgs::Image toRosMsgRawPtr(std::shared_ptr<dai::ImgFrame> inData, const sensor_msgs::msg::CameraInfo& info = sensor_msgs::msg::CameraInfo());
    ImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    FFMPEGMsgs::FFMPEGPacket toRosFFMPEGPacket(std::shared_ptr<dai::EncodedFrame> inData);

    ImageMsgs::CompressedImage toRosCompressedMsg(std::shared_ptr<dai::ImgFrame> inData);

    void toDaiMsg(const ImageMsgs::Image& inMsg, dai::ImgFrame& outData);

    /** TODO(sachin): Add support for ros msg to cv mat since we have some
     *  encodings which cv supports but ros doesn't
     **/
    cv::Mat rosMsgtoCvMat(ImageMsgs::Image& inMsg);

    ImageMsgs::CameraInfo calibrationToCameraInfo(dai::CalibrationHandler calibHandler,
                                                  dai::CameraBoardSocket cameraId,
                                                  int width = -1,
                                                  int height = -1,
                                                  Point2f topLeftPixelId = Point2f(),
                                                  Point2f bottomRightPixelId = Point2f());

   private:
    void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    void interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    static std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap;
    static std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap;

    // dai::RawImgFrame::Type _srcType;
    bool daiInterleaved;
    // bool c
    const std::string frameName = "";
    std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime;

    rclcpp::Time rosBaseTime;
    bool getBaseDeviceTimestamp;
    // For handling ROS time shifts and debugging
    int64_t totalNsChange{0};
    // Whether to update the ROS base time on each message conversion
    bool updateRosBaseTimeOnToRosMsg{false};
    dai::RawImgFrame::Type srcType;
    bool fromBitstream = false;
    bool dispToDepth = false;
    bool addExpOffset = false;
    bool alphaScalingEnabled = false;
    dai::CameraExposureOffset expOffset;
    bool reversedStereoSocketOrder = false;
    double baseline;
    double alphaScalingFactor = 0.0;
    int camHeight = -1;
    int camWidth = -1;
    std::string ffmpegEncoding = "libx264";
};

}  // namespace ros

}  // namespace dai
