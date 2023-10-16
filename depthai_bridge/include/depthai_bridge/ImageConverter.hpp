#pragma once

#include <deque>
#include <memory>
#include <tuple>
#include <unordered_map>

#include "cv_bridge/cv_bridge.h"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "ros/time.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"

namespace dai {

namespace ros {

namespace StdMsgs = std_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class ImageConverter {
   public:
    // ImageConverter() = default;
    ImageConverter(const std::string frameName, bool interleaved, bool getBaseDeviceTimestamp = false);
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
        _updateRosBaseTimeOnToRosMsg = update;
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

    ImageMsgs::Image toRosMsgRawPtr(std::shared_ptr<dai::ImgFrame> inData, const sensor_msgs::CameraInfo& info = sensor_msgs::CameraInfo());
    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs);
    ImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

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
    static std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap;
    static std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap;

    // dai::RawImgFrame::Type _srcType;
    bool _daiInterleaved;
    // bool c
    const std::string _frameName = "";
    void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    void interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;

    ::ros::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
    // For handling ROS time shifts and debugging
    int64_t _totalNsChange{0};
    // Whether to update the ROS base time on each message conversion
    bool _updateRosBaseTimeOnToRosMsg{false};
    dai::RawImgFrame::Type _srcType;
    bool _fromBitstream = false;
    bool _convertDispToDepth = false;
    bool _addExpOffset = false;
    dai::CameraExposureOffset _expOffset;
    bool _reverseStereoSocketOrder = false;
    double _baseline;
    bool _alphaScalingEnabled = false;
    double _alphaScalingFactor = 0.0;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
