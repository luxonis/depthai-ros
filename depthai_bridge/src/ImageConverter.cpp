
#include <depthai_bridge/ImageConverter.hpp>

namespace dai {

namespace ros {

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

ImageConverter::ImageConverter(const std::string frameName, bool interleaved, std::shared_ptr<Filters> filters) : _frameName(frameName), _daiInterleaved(interleaved), _filters(filters) {}

void ImageConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<ImageMsgs::Image>& outImageMsgs) {
    // auto start = std::chrono::system_clock::now();

    auto tstamp = inData->getTimestamp();
    ImageMsgs::Image outImageMsg;
    StdMsgs::Header header;
    header.frame_id = _frameName;

#ifdef IS_ROS2
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    header.stamp = rclStamp;
#else
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    uint64_t nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    header.stamp = rosStamp;
    header.seq = inData->getSequenceNum();
#endif

    // auto ts = std::chrono::time_point_cast<std::chrono::milliseconds>(steadyTime);
    // auto steady_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // auto dataStamp = inData->getTimestamp();
    // auto dataTs = std::chrono::time_point_cast<std::chrono::milliseconds>(dataStamp);
    // auto data_ts = std::chrono::duration_cast<std::chrono::milliseconds >(dataTs.time_since_epoch()).count();
    // std::cout << "header stamp " << header.stamp << " : data stamp " << data_ts << " : steady stamp " << steady_ts << " : sequence number " << header.seq << std::endl;


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
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, output).toImageMsg(outImageMsg);

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
    outImageMsgs.push_back(outImageMsg);

    // auto end = std::chrono::system_clock::now();
    // double elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds >(end - start).count();
    // std::cout << "time to create image messsage " << elapsed_seconds << '\n';
    
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
    /* #ifdef IS_ROS2
        TimePoint ts(std::chrono::seconds((int)inMsg.header.stamp.seconds ()) + std::chrono::nanoseconds(inMsg.header.stamp.nanoseconds()));
    #else
        TimePoint ts(std::chrono::seconds((int)inMsg.header.stamp.toSec()) + std::chrono::nanoseconds(inMsg.header.stamp.toNSec()));
    #endif

    outData.setTimestamp(ts);
    outData.setSequenceNum(inMsg.header.seq); */
    outData.setWidth(inMsg.width);
    outData.setHeight(inMsg.height);
    outData.setType(revEncodingIter->first);
}

ImagePtr ImageConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData) {
    std::deque<ImageMsgs::Image> msgQueue;
    toRosMsg(inData, msgQueue);
    auto msg = msgQueue.front();

#ifdef IS_ROS2
    ImagePtr ptr = std::make_shared<ImageMsgs::Image>(msg);
#else
    ImagePtr ptr = boost::make_shared<ImageMsgs::Image>(msg);
#endif
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
        std::runtime_error("This frature is still WIP");
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

#ifdef IS_ROS2
    auto& intrinsics = cameraData.k;
    auto& distortions = cameraData.d;
    auto& projection = cameraData.p;
    auto& rotation = cameraData.r;
#else
    auto& intrinsics = cameraData.K;
    auto& distortions = cameraData.D;
    auto& projection = cameraData.P;
    auto& rotation = cameraData.R;
#endif
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
            std::vector<std::vector<float>> stereoIntrinsics = calibHandler.getCameraIntrinsics(
                calibHandler.getStereoRightCameraId(), cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);
            std::vector<double> stereoFlatIntrinsics(12), flatRectifiedRotation(9);
            for(int i = 0; i < 3; i++) {
                std::copy(stereoIntrinsics[i].begin(), stereoIntrinsics[i].end(), stereoFlatIntrinsics.begin() + 4 * i);
                stereoFlatIntrinsics[(4 * i) + 3] = 0;
            }

            if(calibHandler.getStereoLeftCameraId() == cameraId) {
                // This defines where the first camera is w.r.t second camera coordinate system giving it a translation to place all the points in the first
                // camera to second camera by multiplying that translation vector using transformation function.
                stereoFlatIntrinsics[3] = stereoFlatIntrinsics[0]
                                          * calibHandler.getCameraExtrinsics(calibHandler.getStereoRightCameraId(), calibHandler.getStereoLeftCameraId())[0][3]
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

    cx = cameraData.K[2];
    cy = cameraData.K[5];
    fx = 1.0f / (cameraData.K[0]);
    fy = 1.0f / (cameraData.K[4]);
    return cameraData;
}

void ImageConverter::toRosPointcloudMsg(std::shared_ptr<dai::ImgFrame> depthData, std::shared_ptr<dai::ImgFrame> colorData, sensor_msgs::PointCloud2& outPointcloudMsg) {
    
    // auto start = std::chrono::system_clock::now();

    // auto tstampTmp = depthData->getTimestamp();
    // auto ts = std::chrono::time_point_cast<std::chrono::milliseconds>(tstampTmp);
    // auto depth_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // ts = std::chrono::time_point_cast<std::chrono::milliseconds>(colorData->getTimestamp());
    // auto color_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // auto steadyTimeTmp = std::chrono::steady_clock::now();
    // ts = std::chrono::time_point_cast<std::chrono::milliseconds>(steadyTimeTmp);
    // auto steady_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // std::cout << "depth: " << depth_ts << " : steady time " << steady_ts << " : " << depthData->getSequenceNum() << std::endl;
    // std::cout << "rgb:   " << color_ts << " : steady time " << steady_ts + 20 << " : " << colorData->getSequenceNum() << std::endl;
    
    // auto wait_time = std::chrono::steady_clock::now();
    // auto ts_loop = std::chrono::time_point_cast<std::chrono::milliseconds>(wait_time);
    // auto wait_time_loop = std::chrono::duration_cast<std::chrono::milliseconds >(ts_loop.time_since_epoch()).count();
    // std::cout << "loop time: " << wait_time_loop << std::endl;
    // while( (steady_ts + 55) > wait_time_loop) {
    //     wait_time = std::chrono::steady_clock::now();
    //     ts_loop = std::chrono::time_point_cast<std::chrono::milliseconds>(wait_time);
    //     wait_time_loop = std::chrono::duration_cast<std::chrono::milliseconds >(ts_loop.time_since_epoch()).count();
    //     // std::cout << "loop time: " << wait_time_loop << std::endl;
    // }
    // wait_time = std::chrono::steady_clock::now();
    // ts_loop = std::chrono::time_point_cast<std::chrono::milliseconds>(wait_time);
    // wait_time_loop = std::chrono::duration_cast<std::chrono::milliseconds >(ts_loop.time_since_epoch()).count();
    // std::cout << "loop time: " << wait_time_loop << std::endl;

    StdMsgs::Header header;
    header.frame_id = _frameName;
    auto tstamp = depthData->getTimestamp();

#ifdef IS_ROS2
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    header.stamp = rclStamp;
#else
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    uint64_t nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    header.stamp = rosStamp;
    header.seq = depthData->getSequenceNum();
#endif

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->width    = depthData->getWidth();
    cloud->height   = depthData->getHeight();
    cloud->is_dense = false;
    cloud->resize (cloud->width * cloud->height);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    int image_idx = 0;
    const uint16_t* depth_buffer = reinterpret_cast<const uint16_t*>(depthData->getData().data());
    const uint8_t* color_buffer = reinterpret_cast<const uint8_t*>(colorData->getData().data());

    for (int v = 0; v < cloud->height; ++v) {
        for (int u = 0; u < cloud->width; ++u, ++image_idx) {
            
            float Z = depth_buffer[image_idx] / 1000.0;
            pcl::PointXYZI *pt = &cloud->at(u,v);
            
            if (std::isnan(Z)) {
                // pt->x = pt->y = pt->z = Z;
            } else {
                pt->x = ((u - cx) * Z * fx);
                pt->y = ((v - cy) * Z * fy);
                pt->z = Z;
                pt->intensity = color_buffer[image_idx];
            }
        }
    }

    if (_filters->voxelGrid.useFilter) {
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(_filters->voxelGrid.leafX, _filters->voxelGrid.leafY, _filters->voxelGrid.leafZ);
        sor.setMinimumPointsNumberPerVoxel(_filters->voxelGrid.minPointsPerVoxel);
        sor.setFilterFieldName(_filters->voxelGrid.filterFieldName);
        sor.setFilterLimits(_filters->voxelGrid.filterMinLimit,_filters->voxelGrid.filterMaxLimit);
        sor.filter(*cloud_filtered);

        pcl::toROSMsg(*cloud_filtered, outPointcloudMsg);
    } else {
        pcl::toROSMsg(*cloud, outPointcloudMsg);
    }

    outPointcloudMsg.header = header;

    // auto end = std::chrono::system_clock::now();
    // double elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds >(end - start).count();

    // std::cout << "time to create pointcloud " << elapsed_seconds << '\n';

}

void ImageConverter::toRosPointcloudMsgRGB(std::shared_ptr<dai::ImgFrame> depthData, std::shared_ptr<dai::ImgFrame> colorData, sensor_msgs::PointCloud2& outPointcloudMsg) {

    // auto start = std::chrono::system_clock::now();

    // auto tstampTmp = depthData->getTimestamp();
    // auto ts = std::chrono::time_point_cast<std::chrono::milliseconds>(tstampTmp);
    // auto depth_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // ts = std::chrono::time_point_cast<std::chrono::milliseconds>(colorData->getTimestamp());
    // auto color_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // auto steadyTimeTmp = std::chrono::steady_clock::now();
    // ts = std::chrono::time_point_cast<std::chrono::milliseconds>(steadyTimeTmp);
    // auto steady_ts = std::chrono::duration_cast<std::chrono::milliseconds >(ts.time_since_epoch()).count();

    // std::cout << "depth: " << depth_ts << " : steady time " << steady_ts << " : " << depthData->getSequenceNum() << std::endl;
    // std::cout << "rgb:   " << color_ts << " : steady time " << steady_ts << " : " << colorData->getSequenceNum() << std::endl;
    

    StdMsgs::Header header;
    header.frame_id = _frameName;
    auto tstamp = depthData->getTimestamp();

#ifdef IS_ROS2
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    header.stamp = rclStamp;
#else
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    uint64_t nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    header.stamp = rosStamp;
    header.seq = depthData->getSequenceNum();
#endif

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->width    = depthData->getWidth();
    cloud->height   = depthData->getHeight();
    cloud->is_dense = false;
    cloud->resize (cloud->width * cloud->height);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    int depth_idx = 0;
    int color_idx = 0;
    
    const uint16_t* depth_buffer = reinterpret_cast<const uint16_t*>(depthData->getData().data());
    
    cv::Mat mat, output;
        cv::Size size = {0, 0};
        int type = 0;
        size = cv::Size(colorData->getWidth(), colorData->getHeight() * 3 / 2);
                type = CV_8UC1;
        mat = cv::Mat(size, type, colorData->getData().data());
    cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);

    for (int v = 0; v < cloud->height; ++v) {
        for (int u = 0; u < cloud->width; ++u, ++depth_idx, color_idx+=3 ) {

            float Z = depth_buffer[depth_idx] / 1000.0;
            pcl::PointXYZRGB *pt = &cloud->at(u,v);
            
            if (std::isnan(Z)) {
                // pt->x = pt->y = pt->z = Z;
            } else {
                pt->x = ((u - cx) * Z * fx);
                pt->y = ((v - cy) * Z * fy);
                pt->z = Z;
                pt->r = output.data[color_idx + 2];
                pt->g = output.data[color_idx + 1];
                pt->b = output.data[color_idx];
            }
        }
    }

    if (_filters->voxelGrid.useFilter) {
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(_filters->voxelGrid.leafX, _filters->voxelGrid.leafY, _filters->voxelGrid.leafZ);
        sor.setMinimumPointsNumberPerVoxel(_filters->voxelGrid.minPointsPerVoxel);
        sor.setFilterFieldName(_filters->voxelGrid.filterFieldName);
        sor.setFilterLimits(_filters->voxelGrid.filterMinLimit,_filters->voxelGrid.filterMaxLimit);
        sor.filter(*cloud_filtered);

        pcl::toROSMsg(*cloud_filtered, outPointcloudMsg);
    } else {
        pcl::toROSMsg(*cloud, outPointcloudMsg);
    }

    outPointcloudMsg.header = header;

    // auto end = std::chrono::system_clock::now();
    // double elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds >(end - start).count();
    // std::cout << elapsed_seconds << '\n';

}

}  // namespace ros
}  // namespace dai