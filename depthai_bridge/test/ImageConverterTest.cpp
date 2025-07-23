#include <gtest/gtest.h>

#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/ImageConverter.hpp"
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
    std::ifstream f(ament_index_cpp::get_package_share_directory("depthai_bridge") + "/resources/cal.json");
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
    EXPECT_EQ(cameraInfo.d.size(), 8);
    EXPECT_EQ(cameraInfo.p.size(), 12);
    EXPECT_EQ(cameraInfo.r.size(), 9);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
}  // namespace depthai_bridge
