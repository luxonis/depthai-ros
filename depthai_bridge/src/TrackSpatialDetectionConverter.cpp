#include <depthai_bridge/TrackSpatialDetectionConverter.hpp>

#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai/depthai.hpp"

namespace dai {

namespace ros {

TrackSpatialDetectionConverter::TrackSpatialDetectionConverter(std::string frameName, int width, int height, bool normalized, float thresh, bool getBaseDeviceTimestamp)
	:	_frameName(frameName),
		_width(width),
		_height(height),
		_normalized(normalized),
		_thresh(thresh),
		_steadyBaseTime(std::chrono::steady_clock::now()),
		_getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
		_rosBaseTime = rclcpp::Clock().now();
}

TrackSpatialDetectionConverter::~TrackSpatialDetectionConverter() = default;

void TrackSpatialDetectionConverter::toRosMsg(
	std::shared_ptr<dai::Tracklets> trackData, 
	std::deque<DepthaiMsgs::TrackDetection2DArray>& opDetectionMsgs) {

	// setting the header
	std::chrono::_V2::steady_clock::time_point tstamp;
	if(_getBaseDeviceTimestamp)
		tstamp = trackData->getTimestampDevice();
	else
		tstamp = trackData->getTimestamp();

	DepthaiMsgs::TrackDetection2DArray opDetectionMsg;
	opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
	opDetectionMsg.header.frame_id = _frameName;
	opDetectionMsg.detections.resize(trackData->tracklets.size());

	// publishing
	for (int i = 0; i < trackData->tracklets.size(); ++i)
	{
		dai::Tracklet t = trackData->tracklets[i];
		dai::Rect roi;
		float xMin, yMin, xMax, yMax;

		 if (_normalized)
		 	roi = t.roi;
		else
			roi = t.roi.denormalize(_width, _height);

		xMin = roi.topLeft().x;
		yMin = roi.topLeft().y;
		xMax = roi.bottomRight().x;
		yMax = roi.bottomRight().y;

		float xSize = xMax - xMin;
		float ySize = yMax - yMin;
		float xCenter = xMin + xSize / 2.;
		float yCenter = yMin + ySize / 2.;

		opDetectionMsg.detections[i].results.resize(1);

#if defined(IS_GALACTIC) || defined(IS_HUMBLE)
		opDetectionMsg.detections[i].results[0].class_id = std::to_string(t.label);
#elif IS_ROS2
		opDetectionMsg.detections[i].results[0].id = std::to_string(t.label);
#else
		opDetectionMsg.detections[i].results[0].id = t.label;
#endif

		opDetectionMsg.detections[i].results[0].score = _thresh;

#ifdef IS_HUMBLE
		opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
		opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
#else
		opDetectionMsg.detections[i].bbox.center.x = xCenter;
		opDetectionMsg.detections[i].bbox.center.y = yCenter;
#endif

		opDetectionMsg.detections[i].bbox.size_x = xSize;
		opDetectionMsg.detections[i].bbox.size_y = ySize;
		opDetectionMsg.detections[i].is_tracking = true;
		std::stringstream track_id_str;
        track_id_str << "" << t.id;
		opDetectionMsg.detections[i].tracking_id = track_id_str.str();
		opDetectionMsg.detections[i].tracking_age = t.age;
		opDetectionMsg.detections[i].tracking_status = static_cast<int32_t>(t.status);

		// converting mm to meters since per ros rep-103 lenght should always be in meters
		opDetectionMsg.detections[i].results[0].pose.pose.position.x = t.spatialCoordinates.x / 1000.0;
		opDetectionMsg.detections[i].results[0].pose.pose.position.y = t.spatialCoordinates.y / 1000.0;
		opDetectionMsg.detections[i].results[0].pose.pose.position.z = t.spatialCoordinates.z / 1000.0;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

TrackDetection2DArrayPtr TrackSpatialDetectionConverter::toRosMsgPtr(
	std::shared_ptr<dai::Tracklets> trackData) {

	std::deque<DepthaiMsgs::TrackDetection2DArray> msgQueue;
    toRosMsg(trackData, msgQueue);
    auto msg = msgQueue.front();

#ifdef IS_ROS2
    TrackDetection2DArrayPtr ptr = std::make_shared<DepthaiMsgs::TrackDetection2DArray>(msg);
#else
    TrackDetection2DArrayPtr ptr = boost::make_shared<DepthaiMsgs::TrackDetection2DArray>(msg);
#endif
    return ptr;
}

}  // namespace ros

}  // namespace dai