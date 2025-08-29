#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"

#include <rclcpp/logging.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
ImagePublisher::ImagePublisher(std::shared_ptr<rclcpp::Node> node,
                               std::shared_ptr<dai::Pipeline> pipeline,
                               const std::string& qName,
                               dai::Node::Output* out,
                               bool synced,
                               bool ipcEnabled,
                               const utils::VideoEncoderConfig& encoderConfig)
    : node(node), encConfig(encoderConfig), out(out), qName(qName), ipcEnabled(ipcEnabled), synced(synced) {
    if(encoderConfig.enabled) {
        encoder = createEncoder(pipeline, encoderConfig);
        this->out->link(encoder->input);
    }
}
void ImagePublisher::setup(std::shared_ptr<dai::Device> device, const utils::ImgConverterConfig& convConf, const utils::ImgPublisherConfig& pubConf) {
    convConfig = convConf;
    pubConfig = pubConf;
    createImageConverter(device);
    createInfoManager(device);
    if(pubConfig.topicName.empty()) {
        throw std::runtime_error("Topic name cannot be empty!");
    }
    rclcpp::PublisherOptions pubOptions;
    pubOptions.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub =
                node->create_publisher<sensor_msgs::msg::CompressedImage>(pubConfig.topicName + pubConfig.compressedTopicSuffix, rclcpp::QoS(10), pubOptions);
        } else {
            ffmpegPub = node->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
                pubConfig.topicName + pubConfig.compressedTopicSuffix, rclcpp::QoS(10), pubOptions);
        }
        infoPub =
            node->create_publisher<sensor_msgs::msg::CameraInfo>(pubConfig.topicName + pubConfig.infoSuffix + "/camera_info", rclcpp::QoS(10), pubOptions);
    } else {
        imgPubIT = image_transport::create_camera_publisher(node.get(), pubConfig.topicName + pubConfig.topicSuffix);
    }
    if(!synced) {
        dataQ = out->createOutputQueue(pubConf.maxQSize, pubConf.qBlocking);
        addQueueCB();
    }
}

void ImagePublisher::createImageConverter(std::shared_ptr<dai::Device> device) {
    converter = std::make_shared<depthai_bridge::ImageConverter>(convConfig.tfPrefix, convConfig.interleaved, convConfig.getBaseDeviceTimestamp);
    converter->setUpdateRosBaseTimeOnToRosMsg(convConfig.updateROSBaseTimeOnRosMsg);
    if(convConfig.lowBandwidth) {
        converter->convertFromBitstream(convConfig.encoding);
        if(convConfig.isStereo && !convConfig.outputDisparity) {
            try {
                auto calHandler = device->readCalibration();
                double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
                if(convConfig.reverseSocketOrder) {
                    baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
                }
                converter->convertDispToDepth(baseline);
            } catch(const std::exception& e) {
                RCLCPP_DEBUG(node->get_logger(), "Failed to convert disparity to depth: %s", e.what());
            }
        }
    }
    if(convConfig.addExposureOffset) {
        converter->addExposureOffset(convConfig.expOffset);
    }
    if(convConfig.reverseSocketOrder) {
        converter->reverseStereoSocketOrder();
    }
    if(convConfig.alphaScalingEnabled) {
        converter->setAlphaScaling(convConfig.alphaScaling);
    }
    if(convConfig.isStereo && !convConfig.outputDisparity) {
        auto calHandler = device->readCalibration();
        double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
        if(convConfig.reverseSocketOrder) {
            baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
        }
        converter->convertDispToDepth(baseline);
    }
    converter->setFFMPEGEncoding(convConfig.ffmpegEncoder);
}

std::shared_ptr<dai::node::VideoEncoder> ImagePublisher::createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const utils::VideoEncoderConfig& encoderConfig) {
    auto enc = pipeline->create<dai::node::VideoEncoder>();
    enc->setQuality(encoderConfig.quality);
    enc->setProfile(encoderConfig.profile);
    if(encoderConfig.profile != dai::VideoEncoderProperties::Profile::MJPEG) {
        enc->setBitrate(encoderConfig.bitrate);
        enc->setKeyframeFrequency(encoderConfig.frameFreq);
    }
    return enc;
}
void ImagePublisher::createInfoManager(std::shared_ptr<dai::Device> device) {
    infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
        node->create_sub_node(std::string(node->get_name()) + "/" + pubConfig.daiNodeName).get(), "/" + pubConfig.daiNodeName + pubConfig.infoMgrSuffix);
    if(pubConfig.calibrationFile.empty()) {
        auto calHandler = device->readCalibration();
        auto info = sensor_helpers::getCalibInfo(node->get_logger(), converter, calHandler, pubConfig.socket, pubConfig.width, pubConfig.height);
        if(pubConfig.rectified) {
            std::fill(info.d.begin(), info.d.end(), 0.0);
            info.r[0] = info.r[4] = info.r[8] = 1.0;
        }
        infoManager->setCameraInfo(info);
    } else {
        infoManager->loadCameraInfo(pubConfig.calibrationFile);
    }
};
ImagePublisher::~ImagePublisher() {
    closeQueue();
};

void ImagePublisher::closeQueue() {
    if(dataQ) dataQ->close();
}
void ImagePublisher::link(dai::Node::Input& in) {
    out->link(in);
}
std::shared_ptr<dai::MessageQueue> ImagePublisher::getQueue() {
    return dataQ;
}
bool ImagePublisher::isSynced() {
    return synced;
}
void ImagePublisher::addQueueCB() {
    cbID = dataQ->addCallback([this](const std::shared_ptr<dai::ADatatype>& data) { publish(data); });
}

std::string ImagePublisher::getQueueName() {
    return qName;
}
std::shared_ptr<Image> ImagePublisher::convertData(const std::shared_ptr<dai::ADatatype>& data) {
    auto daiImg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    auto info = converter->generateCameraInfo(daiImg);
    if(pubConfig.rectified) {
        info.r[0] = info.r[4] = info.r[8] = 1.0;
    }
    if(pubConfig.undistorted) {
        std::fill(info.d.begin(), info.d.end(), 0.0);
    }
    auto img = std::make_shared<Image>();
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            auto daiImg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
            std::deque<sensor_msgs::msg::CompressedImage> deq;
            converter->toRosCompressedMsg(daiImg, deq);
            img->compressedImg = std::make_unique<sensor_msgs::msg::CompressedImage>(deq.front());
        } else {
            auto daiImg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
            std::deque<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> deq;
            converter->toRosFFMPEGPacket(daiImg, deq);
            img->ffmpegPacket = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(deq.front());
        }
    } else {
        auto daiImg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
        info.header = rawMsg.header;
        sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>(rawMsg);
        img->image = std::move(msg);
    }
    sensor_msgs::msg::CameraInfo::UniquePtr infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
    img->info = std::move(infoMsg);
    return img;
}
void ImagePublisher::publish(std::shared_ptr<Image> img) {
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub->publish(std::move(img->compressedImg));
        } else {
            ffmpegPub->publish(std::move(img->ffmpegPacket));
        }
        infoPub->publish(std::move(img->info));
    } else {
        if(ipcEnabled && (!pubConfig.lazyPub || detectSubscription(imgPub, infoPub))) {
            imgPub->publish(std::move(img->image));
            infoPub->publish(std::move(img->info));
        } else {
            if(!pubConfig.lazyPub || imgPubIT.getNumSubscribers() > 0) imgPubIT.publish(*img->image, *img->info);
        }
    }
}
void ImagePublisher::publish(std::shared_ptr<Image> img, rclcpp::Time timestamp) {
    img->info->header.stamp = timestamp;
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            img->compressedImg->header.stamp = timestamp;
        } else {
            img->ffmpegPacket->header.stamp = timestamp;
        }
    } else {
        img->image->header.stamp = timestamp;
    }
    publish(img);
}

void ImagePublisher::publish(const std::shared_ptr<dai::ADatatype>& data) {
    if(rclcpp::ok()) {
        auto img = convertData(data);
        publish(img);
    }
}

bool ImagePublisher::detectSubscription(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                                        const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& infoPub) {
    return (pub->get_subscription_count() > 0 || pub->get_intra_process_subscription_count() > 0 || infoPub->get_subscription_count() > 0
            || infoPub->get_intra_process_subscription_count() > 0);
}
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
