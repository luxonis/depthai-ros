#include "depthai_bridge/ImageConverter.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace depthai_bridge {

std::unordered_map<dai::ImgFrame::Type, std::string> ImageConverter::encodingEnumMap = {{dai::ImgFrame::Type::YUV422i, "yuv422"},
                                                                                        {dai::ImgFrame::Type::RGBA8888, "rgba8"},
                                                                                        {dai::ImgFrame::Type::RGB888i, "rgb8"},
                                                                                        {dai::ImgFrame::Type::BGR888i, "bgr8"},
                                                                                        {dai::ImgFrame::Type::GRAY8, "mono8"},
                                                                                        {dai::ImgFrame::Type::RAW8, "mono8"},
                                                                                        {dai::ImgFrame::Type::RAW16, "16UC1"},
                                                                                        {dai::ImgFrame::Type::YUV420p, "YUV420"},
                                                                                        {dai::ImgFrame::Type::GRAYF16, "32FC1"}};
std::unordered_map<dai::ImgFrame::Type, std::string> ImageConverter::planarEncodingEnumMap = {
    {dai::ImgFrame::Type::BGR888p, "rgb8"},  // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
    {dai::ImgFrame::Type::RGB888p, "rgb8"},
    {dai::ImgFrame::Type::NV12, "rgb8"},
    {dai::ImgFrame::Type::YUV420p, "rgb8"}};

ImageConverter::ImageConverter(const std::string& frameName, bool interleaved, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), daiInterleaved(interleaved) {}

ImageConverter::~ImageConverter() = default;

void ImageConverter::convertFromBitstream(dai::ImgFrame::Type srcType) {
    fromBitstream = true;
    this->srcType = srcType;
}

void ImageConverter::convertDispToDepth(double baseline) {
    dispToDepth = true;
    this->baseline = baseline;
}

void ImageConverter::addExposureOffset(dai::CameraExposureOffset& offset) {
    expOffset = offset;
    addExpOffset = true;
}

void ImageConverter::reverseStereoSocketOrder() {
    reversedStereoSocketOrder = true;
}

void ImageConverter::setAlphaScaling(double alphaScalingFactor) {
    alphaScalingEnabled = true;
    this->alphaScalingFactor = alphaScalingFactor;
}

void ImageConverter::setFFMPEGEncoding(const std::string& encoding) {
    ffmpegEncoding = encoding;
}

ImageMsgs::Image ImageConverter::toRosMsgRawPtr(std::shared_ptr<dai::EncodedFrame> inData, const sensor_msgs::msg::CameraInfo& info) {
    ImageMsgs::Image outImageMsg;
    StdMsgs::Header header = getRosHeader(inData, addExpOffset, expOffset);
    outImageMsg.header = header;

    if(fromBitstream) {
        std::string encoding;
        int decodeFlags;
        int channels;
        cv::Mat output;
        switch(srcType) {
            case dai::ImgFrame::Type::BGR888i: {
                encoding = sensor_msgs::image_encodings::BGR8;
                decodeFlags = cv::IMREAD_COLOR;
                channels = CV_8UC3;
                break;
            }
            case dai::ImgFrame::Type::NV12: {
                encoding = sensor_msgs::image_encodings::BGR8;
                decodeFlags = cv::IMREAD_COLOR;
                channels = CV_8UC1;
                break;
            }
            case dai::ImgFrame::Type::RGB888i: {
                encoding = sensor_msgs::image_encodings::RGB8;
                decodeFlags = cv::IMREAD_COLOR;
                channels = CV_8UC3;
                break;
            }
            case dai::ImgFrame::Type::GRAY8: {
                encoding = sensor_msgs::image_encodings::MONO8;
                decodeFlags = cv::IMREAD_GRAYSCALE;
                channels = CV_8UC1;
                break;
            }
            case dai::ImgFrame::Type::RAW8: {
                encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                decodeFlags = cv::IMREAD_ANYDEPTH;
                channels = CV_16UC1;
                break;
            }
            default: {
                std::cout << frameName << static_cast<int>(srcType) << std::endl;
                throw(std::runtime_error("Converted type not supported!"));
            }
        }

        output = cv::imdecode(cv::Mat(1, inData->getData().size(), channels, inData->getData().data()), decodeFlags);

        // converting disparity
        if(dispToDepth) {
            auto factor = std::abs(baseline * 10) * info.p[0];
            cv::Mat depthOut = cv::Mat(cv::Size(output.cols, output.rows), CV_16UC1);
            depthOut.forEach<uint16_t>([&output, &factor](uint16_t& pixel, const int* position) -> void {
                auto disp = output.at<uint8_t>(position);
                if(disp == 0)
                    pixel = 0;
                else
                    pixel = factor / disp;
            });
            output = depthOut.clone();
        }
        cv_bridge::CvImage(header, encoding, output).toImageMsg(outImageMsg);
    }
    return outImageMsg;
}

ImageMsgs::Image ImageConverter::toRosMsgRawPtr(std::shared_ptr<dai::ImgFrame> inData, const sensor_msgs::msg::CameraInfo& info) {
    ImageMsgs::Image outImageMsg;
    StdMsgs::Header header = getRosHeader(inData, addExpOffset, expOffset);
    outImageMsg.header = header;

    if(planarEncodingEnumMap.find(inData->getType()) != planarEncodingEnumMap.end()) {
        cv::Mat mat, output;
        cv::Size size = cv::Size(inData->getWidth(), inData->getHeight());

        int type = 0;
        switch(inData->getType()) {
            case dai::ImgFrame::Type::RGB888p: {
                cv::Mat m1 = cv::Mat(size, CV_8UC1, inData->getData().data() + size.area() * 2);
                cv::Mat m2 = cv::Mat(size, CV_8UC1, inData->getData().data() + size.area() * 1);
                cv::Mat m3 = cv::Mat(size, CV_8UC1, inData->getData().data() + size.area() * 0);
                cv::Mat channels[3] = {m1, m2, m3};
                type = CV_8UC3;
                cv::merge(channels, 3, output);
            } break;

            case dai::ImgFrame::Type::BGR888p: {
                cv::Mat m1 = cv::Mat(size, CV_8UC1, inData->getData().data() + size.area() * 0);
                cv::Mat m2 = cv::Mat(size, CV_8UC1, inData->getData().data() + size.area() * 1);
                cv::Mat m3 = cv::Mat(size, CV_8UC1, inData->getData().data() + size.area() * 2);
                cv::Mat channels[3] = {m1, m2, m3};
                type = CV_8UC3;
                cv::merge(channels, 3, output);
            } break;

            case dai::ImgFrame::Type::YUV420p:
                type = CV_8UC1;

                size = cv::Size(inData->getWidth(), inData->getHeight() * 3 / 2);
                mat = cv::Mat(size, type, inData->getData().data());
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
                break;

            case dai::ImgFrame::Type::NV12: {
                type = CV_8UC1;
                int step = inData->getStride();
                cv::Mat frameY(size, type, inData->getData().data(), step);
                cv::Mat frameUV(size / 2, type, inData->getData().data() + inData->getPlaneStride(), step);
                cv::cvtColorTwoPlane(frameY, frameUV, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
                break;
            } break;

            default:
                output = cv::Mat(size, type, inData->getData().data());
                break;
        }
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, output).toImageMsg(outImageMsg);

    } else if(encodingEnumMap.find(inData->getType()) != encodingEnumMap.end()) {
        outImageMsg.header = header;
        if(inData->getType() == dai::ImgFrame::Type::GRAYF16) {
            // we need to convert from FP16 to FP32
            cv::Mat mat = cv::Mat(inData->getHeight(), inData->getWidth(), CV_16F, inData->getData().data());
            cv::Mat frameFp32(inData->getHeight(), inData->getWidth(), CV_32F);
            mat.convertTo(frameFp32, CV_32F);
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, frameFp32).toImageMsg(outImageMsg);
        } else {
            std::string temp_str(encodingEnumMap[inData->getType()]);
            outImageMsg.encoding = temp_str;
            outImageMsg.height = inData->getHeight();
            outImageMsg.width = inData->getWidth();
            outImageMsg.step = inData->getStride();
            ;
            if(outImageMsg.encoding == "16UC1" || outImageMsg.encoding == "32FC1")
                outImageMsg.is_bigendian = false;
            else
                outImageMsg.is_bigendian = true;

            size_t size = inData->getData().size();
            outImageMsg.data.reserve(size);
            outImageMsg.data.assign(inData->getData().begin(), inData->getData().end());
        }
    }
    return outImageMsg;
}

void ImageConverter::toRosCompressedMsg(std::shared_ptr<dai::EncodedFrame> inData, std::deque<ImageMsgs::CompressedImage>& outImageMsgs) {
    ImageMsgs::CompressedImage outImageMsg;
    StdMsgs::Header header = getRosHeader(inData, addExpOffset, expOffset);

    outImageMsg.header = header;
    outImageMsg.format = "jpeg";
    outImageMsg.data.reserve(inData->getData().size());
    outImageMsg.data.assign(inData->getData().begin(), inData->getData().end());
    outImageMsgs.push_back(outImageMsg);
}

void ImageConverter::toRosFFMPEGPacket(std::shared_ptr<dai::EncodedFrame> inData, std::deque<FFMPEGMsgs::FFMPEGPacket>& outImageMsgs) {
    FFMPEGMsgs::FFMPEGPacket outFrameMsg;
    StdMsgs::Header header = getRosHeader(inData, addExpOffset, expOffset);
    outFrameMsg.header = header;
    auto ft = inData->getFrameType();

    outFrameMsg.width = camWidth;
    outFrameMsg.height = camHeight;
    outFrameMsg.encoding = ffmpegEncoding;
    outFrameMsg.pts = header.stamp.sec * 1000000000 + header.stamp.nanosec;  // in nanoseconds
    outFrameMsg.flags = (int)(ft == dai::EncodedFrame::FrameType::I);
    outFrameMsg.is_bigendian = false;
    outFrameMsg.data.reserve(inData->getData().size());
    outFrameMsg.data.assign(inData->getData().begin(), inData->getData().end());

    outImageMsgs.push_back(outFrameMsg);
}
void ImageConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs) {
    auto outImageMsg = toRosMsgRawPtr(inData);
    outImageMsgs.push_back(outImageMsg);
    return;
}

void ImageConverter::toRosMsg(std::shared_ptr<dai::EncodedFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs) {
    auto outImageMsg = toRosMsgRawPtr(inData);
    outImageMsgs.push_back(outImageMsg);
    return;
}

ImagePtr ImageConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
    auto msg = toRosMsgRawPtr(inData);

    ImagePtr ptr = std::make_shared<ImageMsgs::Image>(msg);
    return ptr;
}

void ImageConverter::toDaiMsg(const ImageMsgs::Image& inMsg, dai::ImgFrame& outData) {
    std::unordered_map<dai::ImgFrame::Type, std::string>::iterator revEncodingIter;
    if(daiInterleaved) {
        revEncodingIter = std::find_if(encodingEnumMap.begin(), encodingEnumMap.end(), [&](const std::pair<dai::ImgFrame::Type, std::string>& pair) {
            return pair.second == inMsg.encoding;
        });
        if(revEncodingIter == encodingEnumMap.end())
            throw std::runtime_error(
                "Unable to find DAI encoding for the corresponding "
                "sensor_msgs::image.encoding stream");

        outData.setData(inMsg.data);
    } else {
        revEncodingIter = std::find_if(encodingEnumMap.begin(), encodingEnumMap.end(), [&](const std::pair<dai::ImgFrame::Type, std::string>& pair) {
            return pair.second.find(inMsg.encoding) != std::string::npos;
        });

        std::istringstream f(revEncodingIter->second);
        std::vector<std::string> encoding_info;
        std::string s;

        while(getline(f, s, '_')) encoding_info.push_back(s);

        std::vector<std::uint8_t> opData(inMsg.data.size());
        interleavedToPlanar(inMsg.data, opData, inMsg.height, inMsg.width, std::stoi(encoding_info[0]), std::stoi(encoding_info[1]));
        outData.setData(opData);
    }

    outData.setWidth(inMsg.width);
    outData.setHeight(inMsg.height);
    outData.setType(revEncodingIter->first);
}

void ImageConverter::planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp) {
    if(numPlanes == 3) {
        // optimization (cache)
        for(int i = 0; i < w * h; i++) {
            destData[i * 3 + 0] = srcData.data()[i + w * h * 0];
            destData[i * 3 + 1] = srcData.data()[i + w * h * 1];
            destData[i * 3 + 2] = srcData.data()[i + w * h * 2];
        }
    } else {
        throw std::runtime_error(
            "If you encounter the scenario where you need this "
            "please create an issue on github");
    }
    return;
}

void ImageConverter::interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp) {
    if(numPlanes == 3) {
        // optimization (cache)
        for(int i = 0; i < w * h; i++) {
            uint8_t b = srcData[i * 3 + 0];
            uint8_t g = srcData[i * 3 + 1];
            uint8_t r = srcData[i * 3 + 2];

            destData[i + w * h * 0] = b;
            destData[i + w * h * 1] = g;
            destData[i + w * h * 2] = r;
        }
    } else {
        throw std::runtime_error(
            "If you encounter the scenario where you need this "
            "please create an issue on github");
    }
    return;
}

cv::Mat ImageConverter::rosMsgtoCvMat(ImageMsgs::Image& inMsg) {
    cv::Mat rgb(inMsg.height, inMsg.width, CV_8UC3);
    if(inMsg.encoding == "nv12") {
        cv::Mat nv_frame(inMsg.height * 3 / 2, inMsg.width, CV_8UC1, inMsg.data.data());
        cv::cvtColor(nv_frame, rgb, cv::COLOR_YUV2BGR_NV12);
        return rgb;
    } else if(inMsg.encoding == "yuv420p") {
        cv::Mat yuv_frame(inMsg.height * 3 / 2, inMsg.width, CV_8UC1, inMsg.data.data());
        cv::cvtColor(yuv_frame, rgb, cv::COLOR_YUV2BGR_IYUV);
        return rgb;
    } else if(inMsg.encoding == "rgb8") {
        cv::Mat rgb_frame(inMsg.height, inMsg.width, CV_8UC3, inMsg.data.data());
        cv::cvtColor(rgb_frame, rgb, cv::COLOR_RGB2BGR);
        return rgb;
    } else if(inMsg.encoding == "bgr8") {
        cv::Mat bgr_frame(inMsg.height, inMsg.width, CV_8UC3, inMsg.data.data());
        return bgr_frame;
    } else {
        throw std::runtime_error("Unsupported encoding");
    }
}

sensor_msgs::msg::CameraInfo ImageConverter::generateCameraInfo(std::shared_ptr<dai::ImgFrame> imgFrame) const {
    sensor_msgs::msg::CameraInfo cameraInfo;

    // Get the ImgTransformation from the ImgFrame
    const auto& transformation = imgFrame->transformation;

    // Set the width and height
    cameraInfo.width = transformation.getSize().first;
    cameraInfo.height = transformation.getSize().second;

    // Set the intrinsic matrix
    const auto& intrinsicMatrix = transformation.getIntrinsicMatrix();
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraInfo.k[i * 3 + j] = intrinsicMatrix[i][j];
        }
    }

    // Set the distortion model
    cameraInfo.distortion_model = "rational_polynomial";

    // Set the distortion coefficients
    const auto& distortionCoeffs = transformation.getDistortionCoefficients();
    cameraInfo.d.resize(distortionCoeffs.size());
    for(size_t i = 0; i < distortionCoeffs.size(); ++i) {
        cameraInfo.d[i] = distortionCoeffs[i];
    }

    // Set the projection matrix
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraInfo.p[i * 4 + j] = intrinsicMatrix[i][j];
        }
        // We take extrinsic projection params from initial calibration estimate here
        cameraInfo.p[i * 4 + 3] = camInfo.p[i * 4 + 3];
    }

    // Set the rectification matrix (identity matrix)
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraInfo.r[i * 3 + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    return cameraInfo;
}

ImageMsgs::CameraInfo ImageConverter::calibrationToCameraInfo(dai::CalibrationHandler calibHandler,
                                                              dai::CameraBoardSocket cameraId,
                                                              int width,
                                                              int height,
                                                              dai::Point2f topLeftPixelId,
                                                              dai::Point2f bottomRightPixelId) {
    std::vector<std::vector<float>> camIntrinsics, rectifiedRotation;
    std::vector<float> distCoeffs;
    std::vector<double> flatIntrinsics, distCoeffsDouble;
    int defWidth, defHeight;
    ImageMsgs::CameraInfo cameraData;
    std::tie(std::ignore, defWidth, defHeight) = calibHandler.getDefaultIntrinsics(cameraId);

    if(width == -1) {
        cameraData.width = static_cast<uint32_t>(defWidth);
    } else {
        cameraData.width = static_cast<uint32_t>(width);
    }

    if(height == -1) {
        cameraData.height = static_cast<uint32_t>(defHeight);
    } else {
        cameraData.height = static_cast<uint32_t>(height);
    }

    camWidth = cameraData.width;
    camHeight = cameraData.height;
    camIntrinsics = calibHandler.getCameraIntrinsics(cameraId, cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);

    flatIntrinsics.resize(9);
    for(int i = 0; i < 3; i++) {
        std::copy(camIntrinsics[i].begin(), camIntrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
    }

    auto& intrinsics = cameraData.k;
    auto& distortions = cameraData.d;
    auto& projection = cameraData.p;
    auto& rotation = cameraData.r;
    // Set rotation to reasonable default even for non-stereo pairs
    rotation[0] = rotation[4] = rotation[8] = 1;
    for(size_t i = 0; i < 3; i++) {
        std::copy(flatIntrinsics.begin() + i * 3, flatIntrinsics.begin() + (i + 1) * 3, projection.begin() + i * 4);
    }
    std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), intrinsics.begin());

    distCoeffs = calibHandler.getDistortionCoefficients(cameraId);

    for(size_t i = 0; i < 8; i++) {
        distortions.push_back(static_cast<double>(distCoeffs[i]));
    }

    // Setting Projection matrix if the cameras are stereo pair. Right as the first and left as the second.
    if(calibHandler.getStereoRightCameraId() != dai::CameraBoardSocket::AUTO && calibHandler.getStereoLeftCameraId() != dai::CameraBoardSocket::AUTO) {
        if(calibHandler.getStereoRightCameraId() == cameraId || calibHandler.getStereoLeftCameraId() == cameraId) {
            std::vector<std::vector<float>> stereoIntrinsics =
                calibHandler.getCameraIntrinsics(cameraId, cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);

            if(alphaScalingEnabled) {
                cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F);
                for(int i = 0; i < 3; i++) {
                    for(int j = 0; j < 3; j++) {
                        cameraMatrix.at<double>(i, j) = stereoIntrinsics[i][j];
                    }
                }
                cv::Mat distCoefficients(distCoeffs);

                cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoefficients, cv::Size(width, height), alphaScalingFactor);
                // Copying the contents of newCameraMatrix to stereoIntrinsics
                for(int i = 0; i < 3; i++) {
                    for(int j = 0; j < 3; j++) {
                        float newValue = static_cast<float>(newCameraMatrix.at<double>(i, j));
                        stereoIntrinsics[i][j] = newValue;
                        intrinsics[i * 3 + j] = newValue;
                    }
                }
            }
            std::vector<double> stereoFlatIntrinsics(12), flatRectifiedRotation(9);
            for(int i = 0; i < 3; i++) {
                std::copy(stereoIntrinsics[i].begin(), stereoIntrinsics[i].end(), stereoFlatIntrinsics.begin() + 4 * i);
                stereoFlatIntrinsics[(4 * i) + 3] = 0;
            }

            // Check stereo socket order
            dai::CameraBoardSocket stereoSocketFirst = calibHandler.getStereoLeftCameraId();
            dai::CameraBoardSocket stereoSocketSecond = calibHandler.getStereoRightCameraId();
            double factor = 1.0;
            if(reversedStereoSocketOrder) {
                stereoSocketFirst = calibHandler.getStereoRightCameraId();
                stereoSocketSecond = calibHandler.getStereoLeftCameraId();
                factor = -1.0;
            }

            if(stereoSocketFirst == cameraId) {
                // This defines where the first camera is w.r.t second camera coordinate system giving it a translation to place all the points in the first
                // camera to second camera by multiplying that translation vector using transformation function.
                stereoFlatIntrinsics[3] = factor * stereoFlatIntrinsics[0] * calibHandler.getCameraExtrinsics(stereoSocketFirst, stereoSocketSecond)[0][3]
                                          / 100.0;  // Converting to meters
                rectifiedRotation = calibHandler.getStereoLeftRectificationRotation();
            } else {
                rectifiedRotation = calibHandler.getStereoRightRectificationRotation();
            }

            for(int i = 0; i < 3; i++) {
                std::copy(rectifiedRotation[i].begin(), rectifiedRotation[i].end(), flatRectifiedRotation.begin() + 3 * i);
            }

            std::copy(stereoFlatIntrinsics.begin(), stereoFlatIntrinsics.end(), projection.begin());
            std::copy(flatRectifiedRotation.begin(), flatRectifiedRotation.end(), rotation.begin());
        }
    }
    cameraData.distortion_model = "rational_polynomial";

    camInfo = cameraData;
    return cameraData;
}
}  // namespace depthai_bridge
