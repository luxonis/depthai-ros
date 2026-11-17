#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>

#include "depthai_bridge/OutputRotation.hpp"

using depthai_bridge::OutputRotation;

TEST(OutputRotation, RejectsNonQuarterTurns) {
    EXPECT_THROW(OutputRotation(45), std::invalid_argument);
    EXPECT_THROW(OutputRotation(-90), std::invalid_argument);
}

TEST(OutputRotation, ProjectionAndDistortionAgreeInRotatedFrame) {
    for(int angle : {0, 90, 180, 270}) {
        OutputRotation rotation(angle);
        sensor_msgs::msg::CameraInfo info;
        info.width = 640;
        info.height = 400;
        info.k = {500, 0, 301, 0, 480, 189, 0, 0, 1};
        info.p = {500, 0, 301, -37, 0, 480, 189, 0, 0, 0, 1, 0};
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        info.distortion_model = "rational_polynomial";
        info.d = {0.01, -0.02, 0.003, -0.004, 0.001, 0.002, 0.003, 0.004, 0.005, -0.006, 0.007, -0.008};
        info.header.frame_id = "optical";
        auto native = info;
        rotation.cameraInfo(info);
        EXPECT_GT(info.k[0], 0);
        EXPECT_GT(info.k[4], 0);
        EXPECT_EQ(info.width, rotation.swapsAxes() ? 400 : 640);
        EXPECT_EQ(info.header.frame_id, rotation.frame("optical"));
        for(auto p : std::vector<cv::Point3d>{{0.1, 0.2, 1}, {-0.3, 0.4, 2}, {0.5, -0.2, 3}}) {
            auto xy = rotation.vector(p.x, p.y);
            std::vector<cv::Point2d> original, rotated;
            cv::projectPoints(std::vector<cv::Point3d>{p}, cv::Vec3d(), cv::Vec3d(), cv::Mat(3, 3, CV_64F, native.k.data()), native.d, original);
            cv::projectPoints(
                std::vector<cv::Point3d>{{xy.first, xy.second, p.z}}, cv::Vec3d(), cv::Vec3d(), cv::Mat(3, 3, CV_64F, info.k.data()), info.d, rotated);
            auto expected = rotation.pixel(original[0].x, original[0].y, 640., 400.);
            EXPECT_NEAR(rotated[0].x, expected.first, 1e-9);
            EXPECT_NEAR(rotated[0].y, expected.second, 1e-9);
            auto tf = rotation.transform("optical");
            double c = 1 - 2 * tf.transform.rotation.z * tf.transform.rotation.z;
            double s = 2 * tf.transform.rotation.z * tf.transform.rotation.w;
            EXPECT_NEAR(c * xy.first - s * xy.second, p.x, 1e-12);
            EXPECT_NEAR(s * xy.first + c * xy.second, p.y, 1e-12);
        }
        auto deviceInfo = native;
        auto h = rotation.matrix(640, 400);
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++) {
                deviceInfo.k[i * 3 + j] = 0;
                for(int k = 0; k < 3; k++) deviceInfo.k[i * 3 + j] += h[i * 3 + k] * native.k[k * 3 + j];
                deviceInfo.p[i * 4 + j] = deviceInfo.k[i * 3 + j];
            }
        deviceInfo.width = info.width;
        deviceInfo.height = info.height;
        rotation.cameraInfo(deviceInfo, true);
        for(int i = 0; i < 9; i++) EXPECT_DOUBLE_EQ(info.k[i], deviceInfo.k[i]);
        for(int i = 0; i < 12; i++) EXPECT_DOUBLE_EQ(info.p[i], deviceInfo.p[i]);
    }
}

TEST(OutputRotation, RoiAndBinningFollowQuarterTurn) {
    sensor_msgs::msg::CameraInfo info;
    info.width = 640;
    info.height = 400;
    info.roi.x_offset = 10;
    info.roi.y_offset = 20;
    info.roi.width = 30;
    info.roi.height = 40;
    info.binning_x = 2;
    info.binning_y = 4;
    OutputRotation(90).cameraInfo(info);
    EXPECT_EQ(info.roi.x_offset, 340);
    EXPECT_EQ(info.roi.y_offset, 10);
    EXPECT_EQ(info.roi.width, 40);
    EXPECT_EQ(info.roi.height, 30);
    EXPECT_EQ(info.binning_x, 4);
    EXPECT_EQ(info.binning_y, 2);
}

TEST(OutputRotation, FisheyeDistortionRemainsRadiallySymmetric) {
    for(int angle : {90, 180, 270}) {
        sensor_msgs::msg::CameraInfo info;
        info.width = 640;
        info.height = 400;
        info.k = {500, 0, 301, 0, 480, 189, 0, 0, 1};
        info.distortion_model = "equidistant";
        info.d = {0.01, -0.02, 0.003, -0.004};
        auto native = info;
        OutputRotation rotation(angle);
        rotation.cameraInfo(info);
        std::vector<cv::Point3d> point{{0.3, -0.4, 1}};
        std::vector<cv::Point2d> original, rotated;
        cv::fisheye::projectPoints(point, original, cv::Vec3d(), cv::Vec3d(), cv::Mat(3, 3, CV_64F, native.k.data()), native.d);
        auto xy = rotation.vector(point[0].x, point[0].y);
        point[0].x = xy.first;
        point[0].y = xy.second;
        cv::fisheye::projectPoints(point, rotated, cv::Vec3d(), cv::Vec3d(), cv::Mat(3, 3, CV_64F, info.k.data()), info.d);
        auto expected = rotation.pixel(original[0].x, original[0].y, 640., 400.);
        EXPECT_NEAR(rotated[0].x, expected.first, 1e-9);
        EXPECT_NEAR(rotated[0].y, expected.second, 1e-9);
    }
}

TEST(OutputRotation, RejectsUnsupportedDistortion) {
    sensor_msgs::msg::CameraInfo info;
    info.distortion_model = "rational_polynomial";
    info.d.resize(14, 0.0);
    info.d[12] = 0.01;
    EXPECT_THROW(OutputRotation(90).cameraInfo(info), std::invalid_argument);
    EXPECT_NO_THROW(OutputRotation(0).cameraInfo(info));
    info.distortion_model = "unknown";
    EXPECT_THROW(OutputRotation(90).cameraInfo(info), std::invalid_argument);
}
