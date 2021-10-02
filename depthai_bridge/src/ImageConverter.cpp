#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <boost/range/algorithm.hpp>
#include <depthai/depthai.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>

// FIXME(Sachin): Do I need to convert the encodings that are available in dai
// to only that ros support ? I mean we can publish whatever it is and decode it
// on the other side but howver maybe we should have option to convert planar to
// interleaved before publishing ???

// By default everthing form dai is changed to interleaved when publishing over
// ros. and if we subscribe to a previously published ros msg as input to
// xlinkin node then we need to convert it back to planar if xlinkin node needs
// it planar
namespace dai::rosBridge {

std::unordered_map<dai::RawImgFrame::Type, std::string> ImageConverter::encodingEnumMap = {{dai::RawImgFrame::Type::YUV422i, "yuv422"},
                                                                                           {dai::RawImgFrame::Type::RGBA8888, "rgba8"},
                                                                                           {dai::RawImgFrame::Type::RGB888i, "rgb8"},
                                                                                           {dai::RawImgFrame::Type::BGR888i, "bgr8"},
                                                                                           {dai::RawImgFrame::Type::GRAY8, "mono8"},
                                                                                           {dai::RawImgFrame::Type::RAW8, "mono8"},
                                                                                           {dai::RawImgFrame::Type::RAW16, "16UC1"},
                                                                                           {dai::RawImgFrame::Type::YUV420p, "YUV420"}};
// TODO(sachin) : Move Planare to encodingEnumMap and use default planar namings. And convertt those that are not supported in ROS using ImageTransport in the
// bridge.
std::unordered_map<dai::RawImgFrame::Type, std::string> ImageConverter::planarEncodingEnumMap = {
    {dai::RawImgFrame::Type::BGR888p, "rgb8"},  // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
    {dai::RawImgFrame::Type::RGB888p, "rgb8"},
    {dai::RawImgFrame::Type::NV12, "rgb8"},
    {dai::RawImgFrame::Type::YUV420p, "rgb8"}};

ImageConverter::ImageConverter(bool interleaved) : _daiInterleaved(interleaved) {}

ImageConverter::ImageConverter(const std::string frameName, bool interleaved) : _frameName(frameName), _daiInterleaved(interleaved) {}

void ImageConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, ImageMsgs::Image& outImageMsg) {
    auto tstamp = inData->getTimestamp();
    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp.time_since_epoch()).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp.time_since_epoch()).count() % 1000000000UL;

    std_msgs::Header header;
    header.seq = inData->getSequenceNum();
    header.stamp = ros::Time(sec, nsec);
    header.frame_id = _frameName;

    if(planarEncodingEnumMap.find(inData->getType()) != planarEncodingEnumMap.end()) {
        // cv::Mat inImg = inData->getCvFrame();
        cv::Mat mat, output;
        cv::Size size = {0, 0};
        int type = 0;
        switch(inData->getType()) {
            case dai::RawImgFrame::Type::BGR888p:
            case dai::RawImgFrame::Type::RGB888p:
                size = cv::Size(inData->getWidth(), inData->getHeight());
                type = CV_8UC3;
                break;
            case dai::RawImgFrame::Type::YUV420p:
            case dai::RawImgFrame::Type::NV12:
                size = cv::Size(inData->getWidth(), inData->getHeight() * 3 / 2);
                type = CV_8UC1;
                break;

            default:
                std::runtime_error("Invalid dataType inputs..");
                break;
        }
        mat = cv::Mat(size, type, inData->getData().data());

        switch(inData->getType()) {
            case dai::RawImgFrame::Type::RGB888p: {
                cv::Size s(inData->getWidth(), inData->getHeight());
                std::vector<cv::Mat> channels;
                // RGB
                channels.push_back(cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 2));
                channels.push_back(cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 1));
                channels.push_back(cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 0));
                cv::merge(channels, output);
            } break;

            case dai::RawImgFrame::Type::BGR888p: {
                cv::Size s(inData->getWidth(), inData->getHeight());
                std::vector<cv::Mat> channels;
                // BGR
                channels.push_back(cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 0));
                channels.push_back(cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 1));
                channels.push_back(cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 2));
                cv::merge(channels, output);
            } break;

            case dai::RawImgFrame::Type::YUV420p:
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
                break;

            case dai::RawImgFrame::Type::NV12:
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
                break;

            default:
                output = mat.clone();
                break;
        }
        cv_bridge::CvImage(header, ImageMsgs::image_encodings::BGR8, output).toImageMsg(outImageMsg);

    } else if(encodingEnumMap.find(inData->getType()) != encodingEnumMap.end()) {
        // copying the data to ros msg
        outImageMsg.header = header;
        std::string temp_str(encodingEnumMap[inData->getType()]);
        outImageMsg.encoding = temp_str;
        outImageMsg.height = inData->getHeight();
        outImageMsg.width = inData->getWidth();
        outImageMsg.step = inData->getData().size() / inData->getHeight();
        if(outImageMsg.encoding == "16UC1")
            outImageMsg.is_bigendian = false;
        else
            outImageMsg.is_bigendian = true;

        size_t size = inData->getData().size();
        outImageMsg.data.resize(size);
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&outImageMsg.data[0]);
        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());

        // TODO(Sachin): Try using assign since it is a vector
        // img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);
    }
    return;
}

// TODO(sachin): Not tested
void ImageConverter::toDaiMsg(const ImageMsgs::Image& inMsg, dai::ImgFrame& outData) {
    std::unordered_map<dai::RawImgFrame::Type, std::string>::iterator revEncodingIter;
    if(_daiInterleaved) {
        revEncodingIter = std::find_if(encodingEnumMap.begin(), encodingEnumMap.end(), [&](const std::pair<dai::RawImgFrame::Type, std::string>& pair) {
            return pair.second == inMsg.encoding;
        });
        if(revEncodingIter == encodingEnumMap.end())
            std::runtime_error(
                "Unable to find DAI encoding for the corresponding "
                "sensor_msgs::image.encoding stream");

        outData.setData(inMsg.data);
    } else {
        revEncodingIter = std::find_if(encodingEnumMap.begin(), encodingEnumMap.end(), [&](const std::pair<dai::RawImgFrame::Type, std::string>& pair) {
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

    /** FIXME(sachin) : is this time convertion correct ???
     * Print the original time and ros time in seconds in
     * ImageFrame::toRosMsg(std::shared_ptr<dai::ImgFrame> inData,
     *ImageMsgs::Image& opMsg) to cross verify..
     **/
    TimePoint ts(std::chrono::seconds((int)inMsg.header.stamp.toSec()) + std::chrono::nanoseconds(inMsg.header.stamp.toNSec()));
    outData.setTimestamp(ts);
    outData.setSequenceNum(inMsg.header.seq);
    outData.setWidth(inMsg.width);
    outData.setHeight(inMsg.height);
    outData.setType(revEncodingIter->first);
}

ImageMsgs::ImagePtr ImageConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
    ImageMsgs::ImagePtr ptr = boost::make_shared<ImageMsgs::Image>();
    toRosMsg(inData, *ptr);
    return ptr;
}

void ImageConverter::planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp) {
    if(numPlanes == 3) {
        // optimization (cache)
        for(int i = 0; i < w * h; i++) {
            uint8_t b = srcData.data()[i + w * h * 0];
            destData[i * 3 + 0] = b;
        }
        for(int i = 0; i < w * h; i++) {
            uint8_t g = srcData.data()[i + w * h * 1];
            destData[i * 3 + 1] = g;
        }
        for(int i = 0; i < w * h; i++) {
            uint8_t r = srcData.data()[i + w * h * 2];
            destData[i * 3 + 2] = r;
        }
    } else {
        std::runtime_error(
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
        // for(int i = 0; i < w*h; i++) {
        //     uint8_t g = srcData.data()[i + w*h * 1];
        //     destData[i*3+1] = g;
        // }
        // for(int i = 0; i < w*h; i++) {
        //     uint8_t r = srcData.data()[i + w*h * 2];
        //     destData[i*3+2] = r;
        // }
    } else {
        std::runtime_error(
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
    } else {
        std::runtime_error("THis frature is still WIP");
        return rgb;
    }
}

ImageMsgs::CameraInfo ImageConverter::calibrationToCameraInfo(
    dai::CalibrationHandler calibHandler, dai::CameraBoardSocket cameraId, int width, int height, Point2f topLeftPixelId, Point2f bottomRightPixelId) {
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

    camIntrinsics = calibHandler.getCameraIntrinsics(cameraId, cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);

    flatIntrinsics.resize(9);
    for(int i = 0; i < 3; i++) {
        std::copy(camIntrinsics[i].begin(), camIntrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
    }
    std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), cameraData.K.begin());

    // TODO(sachin): plumb_bob takes only 5 parameters. Should I change from Plum_bob? if so which model represents best ?
    distCoeffs = calibHandler.getDistortionCoefficients(cameraId);

    for(size_t i = 0; i < 5; i++) {
        cameraData.D.push_back(static_cast<double>(distCoeffs[i]));
    }

    // Setting Projection matrix if the cameras are stereo pair
    if(calibHandler.getStereoRightCameraId() != dai::CameraBoardSocket::AUTO && calibHandler.getStereoLeftCameraId() != dai::CameraBoardSocket::AUTO) {
        if(calibHandler.getStereoRightCameraId() == cameraId || calibHandler.getStereoLeftCameraId() == cameraId) {
            std::vector<std::vector<float>> stereoIntrinsics = calibHandler.getCameraIntrinsics(
                calibHandler.getStereoRightCameraId(), cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);
            std::vector<double> stereoFlatIntrinsics(12), flatRectifiedRotation(9);
            for(int i = 0; i < 3; i++) {
                std::copy(stereoIntrinsics[i].begin(), stereoIntrinsics[i].end(), stereoFlatIntrinsics.begin() + 4 * i);
                stereoFlatIntrinsics[(4 * i) + 3] = 0;
            }

            if(calibHandler.getStereoLeftCameraId() == cameraId) {
                stereoFlatIntrinsics[3] = stereoFlatIntrinsics[0]
                                          * calibHandler.getCameraExtrinsics(calibHandler.getStereoLeftCameraId(), calibHandler.getStereoRightCameraId())[0][3];
                rectifiedRotation = calibHandler.getStereoLeftRectificationRotation();
            } else {
                rectifiedRotation = calibHandler.getStereoRightRectificationRotation();
            }

            for(int i = 0; i < 3; i++) {
                std::copy(rectifiedRotation[i].begin(), rectifiedRotation[i].end(), flatRectifiedRotation.begin() + 3 * i);
            }

            std::copy(stereoFlatIntrinsics.begin(), stereoFlatIntrinsics.end(), cameraData.P.begin());
            std::copy(flatRectifiedRotation.begin(), flatRectifiedRotation.end(), cameraData.R.begin());
        }
    }
    cameraData.distortion_model = "plumb_bob";

    return cameraData;
}

}  // namespace dai::rosBridge
