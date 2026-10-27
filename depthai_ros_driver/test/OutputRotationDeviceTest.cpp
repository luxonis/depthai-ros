#include <gtest/gtest.h>

#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_ros_driver/dai_nodes/nn/rotated_input.hpp"
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

// Compare the actual NN input with its passthrough: rotating only publication
// would leave the model consuming the native orientation and fail these checks.
TEST(OutputRotationDevice, RotatesBeforeInference) {
    const auto* deviceId = std::getenv("DEPTHAI_ROTATION_TEST_DEVICE");
    const auto* model = std::getenv("DEPTHAI_ROTATION_TEST_MODEL");
    if(!deviceId || !model) GTEST_SKIP() << "Set DEPTHAI_ROTATION_TEST_DEVICE and DEPTHAI_ROTATION_TEST_MODEL";
    for(int degrees : {90, 180, 270}) {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        auto device = std::make_shared<dai::Device>(dai::DeviceInfo(deviceId));
        auto pipeline = std::make_shared<dai::Pipeline>(device);
        auto camera = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto network = pipeline->create<dai::node::DetectionNetwork>();
        dai::NNArchive archive(model);
        network->setModelPath(model);
        depthai_ros_driver::dai_nodes::nn_helpers::RotatedInput input(pipeline, camera, *network->neuralNetwork, degrees);
        auto before = input.image->out().createOutputQueue(4, true);
        auto after = network->passthrough.createOutputQueue(4, true);
        auto detections = network->out.createOutputQueue(4, true);
        pipeline->start();
        for(int i = 0; i < 3; ++i) {
            auto passed = after->get<dai::ImgFrame>();
            auto source = before->get<dai::ImgFrame>();
            while(source->getSequenceNum() < passed->getSequenceNum()) source = before->get<dai::ImgFrame>();
            ASSERT_EQ(source->getSequenceNum(), passed->getSequenceNum());
            EXPECT_EQ(passed->getTimestamp(), source->getTimestamp());
            const auto size = archive.getInputSize().value();
            ASSERT_EQ(passed->getWidth(), size.first);
            ASSERT_EQ(passed->getHeight(), size.second);
            EXPECT_EQ(cv::norm(source->getFrame(), passed->getFrame(), cv::NORM_INF), 0);
            EXPECT_TRUE(passed->transformation.isEqualTransformation(source->transformation));
            const auto detectedSize = detections->get<dai::ImgDetections>()->transformation->getSize();
            EXPECT_EQ(detectedSize.first, size.first);
            EXPECT_EQ(detectedSize.second, size.second);
        }
        pipeline->stop();
        pipeline->wait();
    }
}

TEST(OutputRotationDevice, NativeNodesProduceRotatedGeometry) {
    const auto* deviceId = std::getenv("DEPTHAI_ROTATION_TEST_DEVICE");
    if(!deviceId) GTEST_SKIP() << "Set DEPTHAI_ROTATION_TEST_DEVICE";
    auto device = std::make_shared<dai::Device>(dai::DeviceInfo(deviceId));
    auto pipeline = std::make_shared<dai::Pipeline>(device);
    auto depthSource = pipeline->create<dai::node::ImageManip>();
    auto colorSource = pipeline->create<dai::node::ImageManip>();
    for(auto source : {depthSource, colorSource}) {
        source->setRunOnHost(false);
        source->setMaxOutputFrameSize(64 * 32 * 3);
    }
    auto depthInput = depthSource->inputImage.createInputQueue();
    auto colorInput = colorSource->inputImage.createInputQueue();
    std::vector<std::shared_ptr<dai::MessageQueue>> depths, clouds, spatials;
    std::vector<std::shared_ptr<dai::InputQueue>> detections;
    std::vector<std::shared_ptr<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>> rotations;
    for(int degrees : {90, 180, 270}) {
        auto depth = std::make_shared<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>(pipeline, depthSource->out, degrees, true);
        auto color = std::make_shared<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>(pipeline, colorSource->out, degrees, true);
        depth->configure(64, 32);
        color->configure(64, 32);
        rotations.insert(rotations.end(), {depth, color});
        auto cloud = pipeline->create<dai::node::RGBD>()->build();
        cloud->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
        depth->out().link(cloud->inDepth);
        color->out().link(cloud->inColor);
        clouds.push_back(cloud->pcl.createOutputQueue());
        depths.push_back(depth->out().createOutputQueue());
        auto spatial = pipeline->create<dai::node::SpatialLocationCalculator>();
        spatial->initialConfig->setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
        depth->out().link(spatial->inputDepth);
        detections.push_back(spatial->inputDetections.createInputQueue());
        spatials.push_back(spatial->outputDetections.createOutputQueue());
    }
    pipeline->start();
    // Exercise repeated frames with cached rotation metadata.
    for(int frameIndex = 0; frameIndex < 3; ++frameIndex) {
        const float focal = 100.f;

        auto depth = std::make_shared<dai::ImgFrame>();
        depth->setType(dai::ImgFrame::Type::RAW16);
        depth->setWidth(64);
        depth->setHeight(32);
        depth->setStride(128);
        std::vector<uint16_t> values(64 * 32, 1500);
        const auto* bytes = reinterpret_cast<const uint8_t*>(values.data());
        depth->setData(std::vector<uint8_t>(bytes, bytes + values.size() * 2));
        depth->transformation = dai::ImgTransformation(64, 32, {{{focal, 0, 31}, {0, 90, 15}, {0, 0, 1}}});
        depth->transformation.setExtrinsics(dai::Extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, dai::Point3f(0, 0, 0), dai::CameraBoardSocket::CAM_A));
        depth->setInstanceNum(0);
        depth->setTimestamp(std::chrono::steady_clock::now());
        auto color = std::make_shared<dai::ImgFrame>(*depth);
        color->setType(dai::ImgFrame::Type::RGB888i);
        color->setStride(192);
        color->setData(std::vector<uint8_t>(64 * 32 * 3, 100));
        depthInput->send(depth);
        colorInput->send(color);
        for(int i = 0; i < 3; ++i) {
            const int degrees = (i + 1) * 90;
            const auto rotated = depths[i]->get<dai::ImgFrame>();
            auto boxes = std::make_shared<dai::ImgDetections>();
            boxes->transformation = rotated->transformation;
            dai::ImgDetection box;
            box.xmin = .125f;
            box.xmax = .375f;
            box.ymin = .25f;
            box.ymax = .75f;
            boxes->detections.push_back(box);
            detections[i]->send(boxes);
            const auto cloud = clouds[i]->get<dai::PointCloudData>();
            ASSERT_EQ(cloud->getWidth(), rotated->getWidth());
            ASSERT_EQ(cloud->getHeight(), rotated->getHeight());
            const auto points = cloud->getPointsRGB();
            const auto& centerColor = points[(cloud->getHeight() / 2) * cloud->getWidth() + cloud->getWidth() / 2];
            EXPECT_NEAR(centerColor.r, 100, 2);
            EXPECT_NEAR(centerColor.g, 100, 2);
            EXPECT_NEAR(centerColor.b, 100, 2);
            ASSERT_EQ(points.size(), 64 * 32);
            for(int y = 0; y < 32; ++y) {
                for(int x = 0; x < 64; ++x) {
                    auto pixel = depthai_bridge::OutputRotation(degrees).pixel(x, y, 64, 32);
                    const auto& point = points[pixel.second * cloud->getWidth() + pixel.first];
                    const auto expected = depthai_bridge::OutputRotation(degrees).vector((x - 31) * 1.5 / focal, (y - 15) * 1.5 / 90);
                    ASSERT_NEAR(point.x, expected.first, 1e-5);
                    ASSERT_NEAR(point.y, expected.second, 1e-5);
                    EXPECT_NEAR(point.z, 1.5, 1e-5);
                }
            }
            const auto spatial = spatials[i]->get<dai::SpatialImgDetections>();
            ASSERT_EQ(spatial->detections.size(), 1);
            const auto& xyz = spatial->detections.front().spatialCoordinates;
            const auto& expected = points[(cloud->getHeight() / 2) * cloud->getWidth() + cloud->getWidth() / 4];
            EXPECT_NEAR(xyz.x / 1000, expected.x, 1e-4);
            EXPECT_NEAR(xyz.y / 1000, expected.y, 1e-4);
            EXPECT_NEAR(xyz.z / 1000, expected.z, 1e-4);
        }
    }
    pipeline->stop();
    pipeline->wait();
}

// Live feature counts depend on the scene. A textured input verifies that every
// orientation produces features in the rotated pixel coordinate system.
TEST(OutputRotationDevice, TracksRotatedImages) {
    const auto* deviceId = std::getenv("DEPTHAI_ROTATION_TEST_DEVICE");
    if(!deviceId) GTEST_SKIP() << "Set DEPTHAI_ROTATION_TEST_DEVICE";
    for(int degrees : {90, 180, 270}) {
        SCOPED_TRACE(degrees);
        auto device = std::make_shared<dai::Device>(dai::DeviceInfo(deviceId));
        auto pipeline = std::make_shared<dai::Pipeline>(device);
        depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation rotation(pipeline, degrees);
        rotation.configure(640, 384);
        auto input = rotation.input().createInputQueue();
        auto tracker = pipeline->create<dai::node::FeatureTracker>();
        tracker->setHardwareResources(2, 2);
        rotation.out().link(tracker->inputImage);
        auto output = tracker->outputFeatures.createOutputQueue();
        pipeline->start();
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setType(dai::ImgFrame::Type::GRAY8);
        frame->setWidth(640);
        frame->setHeight(384);
        frame->setStride(640);
        std::vector<uint8_t> pixels(640 * 384);
        for(int y = 0; y < 384; ++y)
            for(int x = 0; x < 640; ++x) pixels[y * 640 + x] = ((x / 24 + y / 24) % 2) ? 220 : 30;
        frame->setData(pixels);
        size_t maxFeatures = 0;
        for(int index = 0; index < 5; ++index) {
            frame->setSequenceNum(index);
            frame->setTimestamp(std::chrono::steady_clock::now());
            input->send(frame);
            auto features = output->get<dai::TrackedFeatures>();
            maxFeatures = std::max(maxFeatures, features->trackedFeatures.size());
            for(const auto& feature : features->trackedFeatures) {
                EXPECT_GE(feature.position.x, 0);
                EXPECT_GE(feature.position.y, 0);
                EXPECT_LT(feature.position.x, degrees == 180 ? 640 : 384);
                EXPECT_LT(feature.position.y, degrees == 180 ? 384 : 640);
            }
        }
        EXPECT_GT(maxFeatures, 20);
        pipeline->stop();
        pipeline->wait();
    }
}
