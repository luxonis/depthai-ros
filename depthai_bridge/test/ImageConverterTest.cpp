#include <gtest/gtest.h>

#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/OutputRotation.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_bridge {

class ImageConverterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Set up any necessary data or configurations here
    }

    void TearDown() override {
        // Clean up any resources if needed
    }
};

TEST_F(ImageConverterTest, CameraInfoTracksCropAndResize) {
    const std::array<std::array<float, 3>, 3> intrinsics = {{{400, 0, 320}, {0, 420, 240}, {0, 0, 1}}};
    dai::ImgTransformation transformation(640, 480, intrinsics);
    transformation.setDistortionCoefficients({0.1f, 0.2f, 0.3f, 0.4f});
    transformation.addCrop(100, 60, 400, 300).addScale(0.5f, 0.5f).setSize(200, 150);
    sensor_msgs::msg::CameraInfo initialInfo;
    initialInfo.width = 640;
    initialInfo.height = 480;
    initialInfo.k[0] = 999;
    initialInfo.d = {9.0};
    initialInfo.r[0] = 0.5;
    auto info = ImageConverter::transformationToCameraInfo(transformation, initialInfo);
    EXPECT_EQ(info.width, 200u);
    EXPECT_EQ(info.height, 150u);
    EXPECT_DOUBLE_EQ(info.k[0], 200);
    EXPECT_DOUBLE_EQ(info.k[4], 210);
    EXPECT_DOUBLE_EQ(info.k[2], 110);
    EXPECT_DOUBLE_EQ(info.k[5], 90);
    EXPECT_EQ(info.d.size(), 4u);
    EXPECT_FLOAT_EQ(info.d[0], 0.1f);
    EXPECT_DOUBLE_EQ(info.p[0], info.k[0]);
    EXPECT_DOUBLE_EQ(info.p[2], info.k[2]);
    EXPECT_DOUBLE_EQ(info.p[3], 0);
    EXPECT_DOUBLE_EQ(info.r[0], 1);
    EXPECT_DOUBLE_EQ(info.r[4], 1);
    EXPECT_DOUBLE_EQ(info.r[8], 1);
}

TEST_F(ImageConverterTest, CameraInfoProjectsStereoBaselineAtCurrentFocalLength) {
    sensor_msgs::msg::CameraInfo calibration;
    calibration.p[0] = 400;
    calibration.p[5] = 420;
    calibration.p[3] = -30;
    const std::array<std::array<float, 3>, 3> intrinsics = {{{400, 0, 320}, {0, 420, 240}, {0, 0, 1}}};
    dai::ImgTransformation transformation(640, 480, intrinsics);
    transformation.addScale(0.5f, 0.5f).setSize(320, 240);
    auto info = ImageConverter::transformationToCameraInfo(transformation, calibration);
    EXPECT_DOUBLE_EQ(info.p[0], 200);
    EXPECT_DOUBLE_EQ(info.p[3], -15);
    EXPECT_DOUBLE_EQ(info.p[3] / info.p[0], calibration.p[3] / calibration.p[0]);
}

TEST_F(ImageConverterTest, RotatedCameraInfoProjectsStereoBaselineOnce) {
    const std::array<std::array<float, 3>, 3> intrinsics = {{{400, 0, 320}, {0, 420, 240}, {0, 0, 1}}};
    for(int angle : {0, 90, 180, 270}) {
        SCOPED_TRACE(angle);
        OutputRotation rotation(angle);
        for(bool vertical : {false, true}) {
            SCOPED_TRACE(vertical);
            sensor_msgs::msg::CameraInfo calibration;
            calibration.p = {400, 0, 320, vertical ? 0.0 : -30.0, 0, 420, 240, vertical ? -21.0 : 0.0, 0, 0, 1, 0};
            dai::ImgTransformation transformation(640, 480, intrinsics);
            transformation.addScale(0.5f, 0.5f).setSize(320, 240);
            auto expected = ImageConverter::transformationToCameraInfo(transformation, calibration);
            rotation.cameraInfo(expected);

            // Model the device homography before conversion, including a resize.
            const auto h = rotation.matrix(320, 240);
            transformation.addTransformation({{{h[0], h[1], h[2]}, {h[3], h[4], h[5]}, {h[6], h[7], h[8]}}});
            transformation.setSize(rotation.swapsAxes() ? 240 : 320, rotation.swapsAxes() ? 320 : 240);
            auto actual = ImageConverter::transformationToCameraInfo(transformation, calibration);
            rotation.cameraInfo(actual, true, true);
            EXPECT_EQ(actual.width, expected.width);
            EXPECT_EQ(actual.height, expected.height);
            for(size_t i = 0; i < actual.k.size(); ++i) EXPECT_DOUBLE_EQ(actual.k[i], expected.k[i]);
            for(size_t i = 0; i < actual.p.size(); ++i) EXPECT_DOUBLE_EQ(actual.p[i], expected.p[i]);
        }
    }
}

TEST_F(ImageConverterTest, CameraInfoUsesFisheyeModelAndMatchesEncodedFrames) {
    const std::array<std::array<float, 3>, 3> intrinsics = {{{400, 0, 320}, {0, 420, 240}, {0, 0, 1}}};
    dai::ImgTransformation transformation(640, 480, intrinsics, dai::CameraModel::Fisheye, {0.1f, 0.2f, 0.3f, 0.4f});
    auto frame = std::make_shared<dai::ImgFrame>();
    auto encoded = std::make_shared<dai::EncodedFrame>();
    frame->setWidth(640);
    frame->setHeight(480);
    frame->transformation = transformation;
    encoded->transformation = transformation;
    ImageConverter converter("test_frame", true);
    auto info = converter.generateCameraInfo(frame);
    EXPECT_EQ(info.distortion_model, "equidistant");
    EXPECT_EQ(info, converter.generateCameraInfo(encoded));
    frame->transformation.addScale(0.5f, 0.5f).setSize(320, 240);
    frame->setWidth(320);
    frame->setHeight(240);
    auto resizedInfo = converter.generateCameraInfo(frame);
    EXPECT_EQ(resizedInfo.width, 320u);
    EXPECT_DOUBLE_EQ(resizedInfo.k[0], 200);
}

TEST_F(ImageConverterTest, ConstructorTest) {
    ImageConverter converter("test_frame", true, false);
    EXPECT_EQ(converter.getFrameName(), "test_frame");
    EXPECT_TRUE(converter.isDaiInterleaved());
    EXPECT_FALSE(converter.isGetBaseDeviceTimestamp());
}

TEST_F(ImageConverterTest, ConvertFromBitstreamTest) {
    ImageConverter converter("test_frame", true, false);
    converter.convertFromBitstream(dai::ImgFrame::Type::BGR888i);
    EXPECT_TRUE(converter.isFromBitstream());
    EXPECT_EQ(converter.getSrcType(), dai::ImgFrame::Type::BGR888i);
}

TEST_F(ImageConverterTest, ConvertDispToDepthTest) {
    ImageConverter converter("test_frame", true, false);
    converter.convertDispToDepth(0.1);
    EXPECT_TRUE(converter.isDispToDepth());
    EXPECT_EQ(converter.getBaseline(), 0.1);
}

TEST_F(ImageConverterTest, AddExposureOffsetTest) {
    ImageConverter converter("test_frame", true, false);
    dai::CameraExposureOffset offset;
    converter.addExposureOffset(offset);
    EXPECT_TRUE(converter.isAddExpOffset());
    EXPECT_EQ(converter.getExpOffset(), offset);
}

TEST_F(ImageConverterTest, ReverseStereoSocketOrderTest) {
    ImageConverter converter("test_frame", true, false);
    converter.reverseStereoSocketOrder();
    EXPECT_TRUE(converter.isReversedStereoSocketOrder());
}

TEST_F(ImageConverterTest, SetAlphaScalingTest) {
    ImageConverter converter("test_frame", true, false);
    converter.setAlphaScaling(0.5);
    EXPECT_TRUE(converter.isAlphaScalingEnabled());
    EXPECT_EQ(converter.getAlphaScalingFactor(), 0.5);
}

TEST_F(ImageConverterTest, SetFFMPEGEncodingTest) {
    ImageConverter converter("test_frame", true, false);
    converter.setFFMPEGEncoding("h264");
    EXPECT_EQ(converter.getFFMPEGEncoding(), "h264");
}

TEST_F(ImageConverterTest, ToRosMsgRawPtrTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::ImgFrame> inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setType(dai::ImgFrame::Type::BGR888i);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    sensor_msgs::msg::CameraInfo info;
    auto outImageMsg = converter.toRosMsgRawPtr(inData, info);
    EXPECT_EQ(outImageMsg.header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsg.width, 640);
    EXPECT_EQ(outImageMsg.height, 480);
    EXPECT_EQ(outImageMsg.encoding, "bgr8");
    EXPECT_EQ(outImageMsg.step, 640 * 3);
    EXPECT_EQ(outImageMsg.data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToRosCompressedMsgTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::EncodedFrame> inData = std::make_shared<dai::EncodedFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    std::deque<sensor_msgs::msg::CompressedImage> outImageMsgs;
    converter.toRosCompressedMsg(inData, outImageMsgs);
    auto outImageMsg = outImageMsgs.front();
    EXPECT_EQ(outImageMsg.header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsg.format, "jpeg");
    EXPECT_EQ(outImageMsg.data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToRosFFMPEGPacketTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::EncodedFrame> inData = std::make_shared<dai::EncodedFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    std::deque<FFMPEGMsgs::FFMPEGPacket> outImageMsgs;
    converter.toRosFFMPEGPacket(inData, outImageMsgs);
    auto outFrameMsg = outImageMsgs.front();

    EXPECT_EQ(outFrameMsg.header.frame_id, "test_frame");
    EXPECT_EQ(outFrameMsg.encoding, "libx264");
    EXPECT_EQ(outFrameMsg.data.size(), 640 * 480 * 3);
    EXPECT_EQ(outFrameMsg.width, 640u);
    EXPECT_EQ(outFrameMsg.height, 480u);

    inData->setWidth(320);
    inData->setHeight(240);
    outImageMsgs.clear();
    converter.toRosFFMPEGPacket(inData, outImageMsgs);
    EXPECT_EQ(outImageMsgs.front().width, 320u);
    EXPECT_EQ(outImageMsgs.front().height, 240u);
}

TEST_F(ImageConverterTest, ToRosMsgTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::ImgFrame> inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setType(dai::ImgFrame::Type::BGR888i);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    std::deque<sensor_msgs::msg::Image> outImageMsgs;
    converter.toRosMsg(inData, outImageMsgs);
    EXPECT_EQ(outImageMsgs.size(), 1);
    EXPECT_EQ(outImageMsgs.front().header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsgs.front().width, 640);
    EXPECT_EQ(outImageMsgs.front().height, 480);
    EXPECT_EQ(outImageMsgs.front().encoding, "bgr8");
    EXPECT_EQ(outImageMsgs.front().step, 640 * 3);
    EXPECT_EQ(outImageMsgs.front().data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToRosMsgPtrTest) {
    ImageConverter converter("test_frame", true, false);
    std::shared_ptr<dai::ImgFrame> inData = std::make_shared<dai::ImgFrame>();
    inData->setWidth(640);
    inData->setHeight(480);
    inData->setType(dai::ImgFrame::Type::BGR888i);
    inData->setData(std::vector<uint8_t>(640 * 480 * 3, 128));
    auto outImageMsg = converter.toRosMsgPtr(inData);
    EXPECT_EQ(outImageMsg->header.frame_id, "test_frame");
    EXPECT_EQ(outImageMsg->width, 640);
    EXPECT_EQ(outImageMsg->height, 480);
    EXPECT_EQ(outImageMsg->encoding, "bgr8");
    EXPECT_EQ(outImageMsg->step, 640 * 3);
    EXPECT_EQ(outImageMsg->data.size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, ToDaiMsgTest) {
    ImageConverter converter("test_frame", true, false);
    sensor_msgs::msg::Image inMsg;
    inMsg.width = 640;
    inMsg.height = 480;
    inMsg.encoding = "bgr8";
    inMsg.step = 640 * 3;
    inMsg.data = std::vector<uint8_t>(640 * 480 * 3, 128);
    dai::ImgFrame outData;
    converter.toDaiMsg(inMsg, outData);
    EXPECT_EQ(outData.getWidth(), 640);
    EXPECT_EQ(outData.getHeight(), 480);
    EXPECT_EQ(outData.getType(), dai::ImgFrame::Type::BGR888i);
    EXPECT_EQ(outData.getData().size(), 640 * 480 * 3);
}

TEST_F(ImageConverterTest, RosMsgtoCvMatTest) {
    ImageConverter converter("test_frame", true, false);
    sensor_msgs::msg::Image inMsg;
    inMsg.encoding = "bgr8";
    inMsg.width = 3;
    inMsg.height = 3;
    inMsg.data = {
        0, 0, 0, 1, 1, 1, 2, 2, 2,  // Row 0
        3, 3, 3, 4, 4, 4, 5, 5, 5,  // Row 1
        6, 6, 6, 7, 7, 7, 8, 8, 8   // Row 2
    };
    auto cvMat = converter.rosMsgtoCvMat(inMsg);
    EXPECT_EQ(cvMat.rows, 3);
    EXPECT_EQ(cvMat.cols, 3);
    EXPECT_EQ(cvMat.type(), CV_8UC3);
    EXPECT_EQ(cvMat.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(0, 1), cv::Vec3b(1, 1, 1));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(0, 2), cv::Vec3b(2, 2, 2));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(1, 0), cv::Vec3b(3, 3, 3));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(1, 1), cv::Vec3b(4, 4, 4));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(1, 2), cv::Vec3b(5, 5, 5));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(2, 0), cv::Vec3b(6, 6, 6));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(2, 1), cv::Vec3b(7, 7, 7));
    EXPECT_EQ(cvMat.at<cv::Vec3b>(2, 2), cv::Vec3b(8, 8, 8));
}

TEST_F(ImageConverterTest, CalibrationToCameraInfoTest) {
    ImageConverter converter("test_frame", true, false);
    std::ifstream f(ament_index_cpp::get_package_share_directory("depthai_bridge_v3") + "/resources/cal.json");
    nlohmann::json data = nlohmann::json::parse(f);
    dai::CalibrationHandler calibHandler = dai::CalibrationHandler::fromJson(data);
    dai::CameraBoardSocket cameraId = dai::CameraBoardSocket::CAM_A;
    dai::Point2f topLeftPixelId = {0, 0};
    dai::Point2f bottomRightPixelId = {640, 480};
    auto cameraInfo = converter.calibrationToCameraInfo(calibHandler, cameraId, 640, 480, topLeftPixelId, bottomRightPixelId);
    EXPECT_EQ(cameraInfo.width, 640);
    EXPECT_EQ(cameraInfo.height, 480);
    EXPECT_EQ(cameraInfo.distortion_model, "rational_polynomial");
    EXPECT_EQ(cameraInfo.k.size(), 9);
    const auto coefficients = calibHandler.getDistortionCoefficients(cameraId);
    EXPECT_EQ(cameraInfo.d, std::vector<double>(coefficients.begin(), coefficients.begin() + 8));
    EXPECT_EQ(cameraInfo.p.size(), 12);
    EXPECT_EQ(cameraInfo.r.size(), 9);
}

TEST_F(ImageConverterTest, FisheyeCalibrationUsesFourCoefficients) {
    std::ifstream f(ament_index_cpp::get_package_share_directory("depthai_bridge") + "/resources/cal.json");
    auto calibration = dai::CalibrationHandler::fromJson(nlohmann::json::parse(f));
    const auto socket = dai::CameraBoardSocket::CAM_B;
    calibration.setCameraType(socket, dai::CameraModel::Fisheye);
    calibration.setDistortionCoefficients(socket, {0.01f, -0.02f, 0.003f, -0.004f});
    ImageConverter converter("test_frame", true);
    for(bool alphaScaling : {false, true}) {
        if(alphaScaling) converter.setAlphaScaling(0.5);
        auto info = converter.calibrationToCameraInfo(calibration, socket);
        EXPECT_EQ(info.distortion_model, "equidistant");
        ASSERT_EQ(info.d.size(), 4u);
        EXPECT_FLOAT_EQ(info.d[0], 0.01f);
        EXPECT_FLOAT_EQ(info.d[3], -0.004f);
        EXPECT_GT(info.k[0], 0.0);
        EXPECT_GT(info.k[4], 0.0);
    }
}

TEST_F(ImageConverterTest, RejectsIncompletePerspectiveCalibration) {
    std::ifstream f(ament_index_cpp::get_package_share_directory("depthai_bridge") + "/resources/cal.json");
    auto data = nlohmann::json::parse(f);
    const auto socket = dai::CameraBoardSocket::CAM_A;
    for(auto& camera : data["cameraData"]) {
        if(camera[0].get<int>() == static_cast<int>(socket)) {
            camera[1]["distortionCoeff"] = {0.1, 0.2};
        }
    }
    auto calibration = dai::CalibrationHandler::fromJson(data);
    ImageConverter converter("test_frame", true);
    EXPECT_THROW(converter.calibrationToCameraInfo(calibration, socket), std::runtime_error);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
}  // namespace depthai_bridge

TEST(ImageConverterRotationTest, ExcludesAllocationPaddingFromRawImage) {
    depthai_bridge::ImageConverter converter("optical", false);
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setType(dai::ImgFrame::Type::GRAY8);
    frame->setWidth(4);
    frame->setHeight(3);
    frame->setStride(8);
    frame->setData(std::vector<uint8_t>(32, 7));
    auto image = converter.toRosMsgRawPtr(frame);
    EXPECT_EQ(image.data.size(), 24);
    EXPECT_EQ(image.step, 8);
}

TEST(ImageConverterRotationTest, EncodedDimensionsComeFromOutputFrame) {
    depthai_bridge::ImageConverter converter("optical_rotated", false);
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->setWidth(400);
    frame->setHeight(640);
    std::deque<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> packets;
    converter.toRosFFMPEGPacket(frame, packets);
    EXPECT_EQ(packets.front().width, 400);
    EXPECT_EQ(packets.front().height, 640);
}

TEST(ImageConverterRotationTest, CameraInfoIncludesFramePadding) {
    depthai_bridge::ImageConverter converter("optical", false);
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(224);
    frame->setHeight(133);
    frame->transformation.setSize(213, 133);
    frame->transformation.setIntrinsicMatrix({{{100, 0, 106}, {0, 100, 66}, {0, 0, 1}}});
    const auto info = converter.generateCameraInfo(frame);
    EXPECT_EQ(info.width, 224);
    EXPECT_EQ(info.height, 133);
    EXPECT_EQ(info.k[0], 100);
    EXPECT_EQ(info.k[2], 106);
}
