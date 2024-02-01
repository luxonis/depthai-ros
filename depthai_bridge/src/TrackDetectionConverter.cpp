#include <depthai_bridge/TrackDetectionConverter.hpp>

#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai/depthai.hpp"

namespace dai {

namespace ros {

TrackDetectionConverter::TrackDetectionConverter(std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp)
	:	_frameName(frameName),
		_width(width),
		_height(height),
		_normalized(normalized),
		_steadyBaseTime(std::chrono::steady_clock::now()),
		_getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
		_rosBaseTime = rclcpp::Clock().now();
}

TrackDetectionConverter::~TrackDetectionConverter() = default;

void TrackDetectionConverter::toRosMsg(
	std::shared_ptr<dai::Tracklets> trackData, 
	std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs) {

	// setting the header
	std::chrono::_V2::steady_clock::time_point tstamp;
	if(_getBaseDeviceTimestamp)
		tstamp = trackData->getTimestampDevice();
	else
		tstamp = trackData->getTimestamp();

	VisionMsgs::Detection2DArray opDetectionMsg;
	opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
	opDetectionMsg.header.frame_id = _frameName;
	opDetectionMsg.detections.resize(trackData->tracklets.size());

	// publishing
	for(int i = 0; i < trackData->tracklets.size(); ++i)
	{
		dai::Tracklet t = trackData->tracklets[i];
		dai::Rect roi;
		int xMin, yMin, xMax, yMax;

		if (_normalized)
			roi = t.roi;
		else
			roi = t.roi.denormalize(_width, _height);

		xMin = t.roi.topLeft().x;
		yMin = t.roi.topLeft().y;
		xMax = t.roi.bottomRight().x;
		yMax = t.roi.bottomRight().y;

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

		opDetectionMsg.detections[i].results[0].score = -1.0;

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

		// converting mm to meters since per ros rep-103 lenght should always be in meters
		// opDetectionMsg.detections[i].position.x = 0;
		// opDetectionMsg.detections[i].position.y = 0;
		// opDetectionMsg.detections[i].position.z = 0;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

Detection2DArrayPtr TrackDetectionConverter::toRosMsgPtr(
	std::shared_ptr<dai::Tracklets> trackData) {

	std::deque<VisionMsgs::Detection2DArray> msgQueue;
    toRosMsg(trackData, msgQueue);
    auto msg = msgQueue.front();

#ifdef IS_ROS2
    Detection2DArrayPtr ptr = std::make_shared<VisionMsgs::Detection2DArray>(msg);
#else
    Detection2DArrayPtr ptr = boost::make_shared<VisionMsgs::Detection2DArray>(msg);
#endif
    return ptr;
}

}  // namespace ros

}  // namespace dai