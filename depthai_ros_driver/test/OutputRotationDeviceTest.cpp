#include <gtest/gtest.h>

#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "depthai/device/Device.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/output_rotation.hpp"

TEST(OutputRotationDevice, PreservesDepthAndOutputGeometry) {
    const auto* deviceId = std::getenv("DEPTHAI_ROTATION_TEST_DEVICE");
    if(!deviceId) GTEST_SKIP() << "Set DEPTHAI_ROTATION_TEST_DEVICE to run the hardware rotation test";
    for(auto type : {dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::RAW16, dai::ImgFrame::Type::BGR888i}) {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        auto device = std::make_shared<dai::Device>(dai::DeviceInfo(deviceId));
        auto pipeline = std::make_shared<dai::Pipeline>(device);
        // A pass-through manip gives the test a device output to rotate.
        auto source = pipeline->create<dai::node::ImageManip>();
        source->setRunOnHost(false);
        source->setMaxOutputFrameSize(640 * 400 * 4);
        auto input = source->inputImage.createInputQueue();
        auto native = source->out.createOutputQueue();
        std::vector<std::unique_ptr<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>> rotations;
        std::vector<std::shared_ptr<dai::MessageQueue>> outputs;
        for(int degrees : {90, 180, 270}) {
            auto rotation = std::make_unique<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>(pipeline, source->out, degrees);
            rotation->configure(640, 400);
            outputs.push_back(rotation->out().createOutputQueue());
            rotations.push_back(std::move(rotation));
        }
        pipeline->start();
        for(int sequence = 0; sequence < 5; ++sequence) {
            int cvType = type == dai::ImgFrame::Type::RAW16 ? CV_16UC1 : type == dai::ImgFrame::Type::GRAY8 ? CV_8UC1 : CV_8UC3;
            // NN-aligned and decimated outputs differ from the configured stereo input size.
            const std::array<cv::Size, 5> depthSizes = {cv::Size(640, 400), {300, 300}, {320, 200}, {512, 384}, {640, 400}};
            cv::Mat pixels(type == dai::ImgFrame::Type::RAW16 ? depthSizes[sequence] : cv::Size(640, 400), cvType);
            cv::RNG rng(sequence + 1);
            rng.fill(pixels, cv::RNG::UNIFORM, 0, type == dai::ImgFrame::Type::RAW16 ? 65536 : 256);
            if(type != dai::ImgFrame::Type::RAW16 && device->getPlatform() == dai::Platform::RVC2) {
                pixels.setTo(0);
                const std::array<cv::Point, 5> centers = {cv::Point(160, 80), {320, 200}, {480, 320}, {80, 320}, {560, 80}};
                cv::circle(pixels, centers[sequence], 16, cv::Scalar::all(200), -1);
            }
            auto frame = std::make_shared<dai::ImgFrame>();
            frame->setType(type);
            frame->setWidth(pixels.cols);
            frame->setHeight(pixels.rows);
            frame->setStride(pixels.step);
            frame->setData(std::vector<uint8_t>(pixels.data, pixels.data + pixels.total() * pixels.elemSize()));
            frame->setSequenceNum(sequence);
            frame->setTimestamp(std::chrono::steady_clock::now());
            input->send(frame);
            auto original = native->get<dai::ImgFrame>();
            for(int i = 0; i < 3; i++) {
                auto result = outputs[i]->get<dai::ImgFrame>();
                cv::Mat expected;
                cv::rotate(original->getFrame(), expected, i == 0 ? cv::ROTATE_90_CLOCKWISE : i == 1 ? cv::ROTATE_180 : cv::ROTATE_90_COUNTERCLOCKWISE);
                EXPECT_EQ(result->getSequenceNum(), sequence);
                EXPECT_EQ(result->getTimestamp(), original->getTimestamp());
                EXPECT_EQ(result->getType(), type);
                ASSERT_EQ(result->getWidth(), expected.cols);
                ASSERT_EQ(result->getHeight(), expected.rows);
                if(type == dai::ImgFrame::Type::RAW16 || device->getPlatform() != dai::Platform::RVC2) {
                    EXPECT_EQ(cv::norm(expected, result->getFrame(), cv::NORM_INF), 0);
                } else {
                    cv::Mat e = expected, a = result->getFrame();
                    if(e.channels() == 3) {
                        cv::cvtColor(expected, e, cv::COLOR_BGR2GRAY);
                        cv::cvtColor(a, a, cv::COLOR_BGR2GRAY);
                    }
                    auto em = cv::moments(e), am = cv::moments(a);
                    // Stock RVC2 firmware uses bicubic interpolation for 8-bit images,
                    // including integer rotations. Check geometry, not bit identity.
                    // RAW16 and the RVC4 CPU backend are checked exactly above.
                    EXPECT_NEAR(am.m10 / am.m00, em.m10 / em.m00, 2.0);
                    EXPECT_NEAR(am.m01 / am.m00, em.m01 / em.m00, 2.0);
                    EXPECT_NEAR(am.m00 / em.m00, 1.0, 0.05);
                }
            }
        }
        pipeline->stop();
        pipeline->wait();
    }
}
