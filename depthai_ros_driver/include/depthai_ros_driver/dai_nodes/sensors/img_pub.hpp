#pragma once

#include <string>

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "depthai_ros_msgs/FFMPEGPacket.h"
#include "image_transport/camera_publisher.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

namespace dai {
class Device;
class Pipeline;
class DataOutputQueue;
namespace node {
class VideoEncoder;
class XLinkOut;
}  // namespace node
namespace ros {
class ImageConverter;
}
}  // namespace dai


namespace camera_info_manager {
class CameraInfoManager;
}
namespace depthai_ros_driver {
namespace dai_nodes {

namespace sensor_helpers {
/**
 * @brief Image struct
 *
 * This struct is used to store the image, camera info, header and ffmpeg packet.
 */
class Image {
   public:
    sensor_msgs::Image::Ptr image;
    sensor_msgs::CameraInfo::Ptr info;
    depthai_ros_msgs::FFMPEGPacket::Ptr ffmpegPacket;
    sensor_msgs::CompressedImage::Ptr compressedImg;
};
/**
 * @brief ImagePublisher class
 *
 * This class is used to publish images from the device to ROS2. It creates a publisher for the image and camera info topics.
 */
class ImagePublisher {
   public:
    /**
     * @brief Construct a new Image Publisher object
     *
     * Creates XLinkOut if synced and VideoEncoder if lowBandwidth is enabled. linkFunc is stored and returned when link is called.
     */
    ImagePublisher(ros::NodeHandle node,
                   std::shared_ptr<dai::Pipeline> pipeline,
                   const std::string& qName,
                   std::function<void(dai::Node::Input in)> linkFunc,
                   bool synced = false,
                   const utils::VideoEncoderConfig& encoderConfig = {});

    ~ImagePublisher();
    /**
     * @brief Setup the image publisher
     *
     * Creates Publishers, ImageConverter and CameraInfoManager. Creates a Queue and adds a callback if not synced.
     */
    void setup(std::shared_ptr<dai::Device> device, const utils::ImgConverterConfig& convConf, const utils::ImgPublisherConfig& pubConf);
    void createImageConverter(std::shared_ptr<dai::Device> device);
    void createInfoManager(std::shared_ptr<dai::Device> device);
    void addQueueCB(const std::shared_ptr<dai::DataOutputQueue>& queue);
    void closeQueue();
    std::shared_ptr<dai::DataOutputQueue> getQueue();
    void link(dai::Node::Input in);
    std::string getQueueName();
    void publish(const std::shared_ptr<dai::ADatatype>& data);
    void publish(std::shared_ptr<Image> img);
    void publish(std::shared_ptr<Image> img, ros::Time timestamp);
    std::shared_ptr<Image> convertData(const std::shared_ptr<dai::ADatatype>& data);
    std::shared_ptr<dai::node::VideoEncoder> createEncoder(std::shared_ptr<dai::Pipeline> pipeline, const utils::VideoEncoderConfig& encoderConfig);

   private:
    bool detectSubscription(const ros::Publisher pub,
                            const ros::Publisher infoPub);
	ros::NodeHandle node;
    image_transport::ImageTransport it;
    utils::VideoEncoderConfig encConfig;
    utils::ImgPublisherConfig pubConfig;
    utils::ImgConverterConfig convConfig;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::ros::ImageConverter> converter;
    std::shared_ptr<dai::node::XLinkOut> xout;
    std::shared_ptr<dai::node::VideoEncoder> encoder;
    std::function<void(dai::Node::Input in)> linkCB;
    ros::Publisher imgPub;
    ros::Publisher infoPub;
    ros::Publisher ffmpegPub;
    ros::Publisher compressedImgPub;
    image_transport::CameraPublisher imgPubIT;
    std::shared_ptr<dai::DataOutputQueue> dataQ;
    int cbID;
    std::string qName;
    bool ipcEnabled;
    bool synced;
};
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
