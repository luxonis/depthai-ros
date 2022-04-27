#pragma once

#include <cv_bridge/cv_bridge.h>

#include <depthai-shared/common/CameraBoardSocket.hpp>
#include <depthai/depthai.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/camera_info.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include "std_msgs/msg/header.hpp"
#else
    #include <ros/ros.h>

    #include <boost/make_shared.hpp>
    #include <boost/range/algorithm.hpp>

    #include "sensor_msgs/CameraInfo.h"
    #include "sensor_msgs/Image.h"
    #include <sensor_msgs/PointCloud2.h>
    #include "std_msgs/Header.h"

#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2
namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
#else
namespace StdMsgs = std_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
#endif
using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

// struct VoxelGrid {
//     float leafX = 0.06;
//     float leafY = 0.06;
//     float leafZ = 0.06;
//     int minPointsPerVoxel = 50;
//     std::string filterFieldName = "";
//     double filterMinLimit = 0;
//     double filterMaxLimit = 5;
// };

// struct Filters {
//     VoxelGrid voxelGrid; 
// };

class ImageConverter {
   public:
    struct VoxelGrid {
        bool useFilter = false;
        int minPointsPerVoxel = 50;
        float leafX = 0.06;
        float leafY = 0.06;
        float leafZ = 0.06;
        std::string filterFieldName = "";
        double filterMinLimit = 0;
        double filterMaxLimit = 5;
    };

    struct Filters {
        VoxelGrid voxelGrid; 
    };
    // ImageConverter() = default;
    ImageConverter(const std::string frameName, bool interleaved, std::shared_ptr<Filters> filters = nullptr );
    ImageConverter(bool interleaved);

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

    void toRosPointcloudMsg(std::shared_ptr<dai::ImgFrame> depthData, std::shared_ptr<dai::ImgFrame> colorData, sensor_msgs::PointCloud2& outImageMsg);
    void toRosPointcloudMsgRGB(std::shared_ptr<dai::ImgFrame> depthData, std::shared_ptr<dai::ImgFrame> colorData, sensor_msgs::PointCloud2& outImageMsg);

   private:
    static std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap;
    static std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap;

    std::shared_ptr<Filters> _filters;

    // dai::RawImgFrame::Type _srcType;
    bool _daiInterleaved;
    // bool c
    const std::string _frameName = "";
    float cx, cy, fx, fy;
    void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    void interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
