#pragma once

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_bridge {

namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
namespace FFMPEGMsgs = ffmpeg_image_transport_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using FFMPEGImagePtr = FFMPEGMsgs::FFMPEGPacket::SharedPtr;
using CompImagePtr = ImageMsgs::CompressedImage::SharedPtr;

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class ImageConverter : public BaseConverter {
   public:
    ImageConverter(const std::string& frameName, bool interleaved, bool getBaseDeviceTimestamp = false);
    ~ImageConverter();

    /**
     * @brief Sets converter behavior to convert from bitstream to raw data.
     * @param srcType: The type of the bitstream data used for conversion.
     */
    void convertFromBitstream(dai::ImgFrame::Type srcType);

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

    void toRosMsg(std::shared_ptr<dai::EncodedFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs);
    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs);
    ImageMsgs::Image toRosMsgRawPtr(std::shared_ptr<dai::EncodedFrame> inData, const sensor_msgs::msg::CameraInfo& info = sensor_msgs::msg::CameraInfo());
    ImageMsgs::Image toRosMsgRawPtr(std::shared_ptr<dai::ImgFrame> inData, const sensor_msgs::msg::CameraInfo& info = sensor_msgs::msg::CameraInfo());
    ImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    void toRosFFMPEGPacket(std::shared_ptr<dai::EncodedFrame> inData, std::deque<FFMPEGMsgs::FFMPEGPacket>& outImageMsgs);

    void toRosCompressedMsg(std::shared_ptr<dai::EncodedFrame> inData, std::deque<ImageMsgs::CompressedImage>& outImageMsgs);

    void toDaiMsg(const ImageMsgs::Image& inMsg, dai::ImgFrame& outData);
    sensor_msgs::msg::CameraInfo generateCameraInfo(std::shared_ptr<dai::ImgFrame> imgFrame) const;

    /** TODO(sachin): Add support for ros msg to cv mat since we have some
     *  encodings which cv supports but ros doesn't
     **/
    cv::Mat rosMsgtoCvMat(ImageMsgs::Image& inMsg);

    ImageMsgs::CameraInfo calibrationToCameraInfo(dai::CalibrationHandler calibHandler,
                                                  dai::CameraBoardSocket cameraId,
                                                  int width = -1,
                                                  int height = -1,
                                                  dai::Point2f topLeftPixelId = dai::Point2f(),
                                                  dai::Point2f bottomRightPixelId = dai::Point2f());

    void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    void interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);

    bool isDaiInterleaved() const {
        return daiInterleaved;
    }
    bool isFromBitstream() const {
        return fromBitstream;
    }
    dai::ImgFrame::Type getSrcType() const {
        return srcType;
    }
    bool isDispToDepth() const {
        return dispToDepth;
    }
    double getBaseline() const {
        return baseline;
    }
    bool isAddExpOffset() const {
        return addExpOffset;
    }
    dai::CameraExposureOffset getExpOffset() const {
        return expOffset;
    }
    bool isReversedStereoSocketOrder() const {
        return reversedStereoSocketOrder;
    }
    bool isAlphaScalingEnabled() const {
        return alphaScalingEnabled;
    }
    double getAlphaScalingFactor() const {
        return alphaScalingFactor;
    }
    std::string getFFMPEGEncoding() const {
        return ffmpegEncoding;
    }

   private:
    static std::unordered_map<dai::ImgFrame::Type, std::string> encodingEnumMap;
    static std::unordered_map<dai::ImgFrame::Type, std::string> planarEncodingEnumMap;

    bool daiInterleaved;
    dai::ImgFrame::Type srcType;
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

}  // namespace depthai_bridge
