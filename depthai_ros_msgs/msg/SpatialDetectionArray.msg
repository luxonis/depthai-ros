# A list of 2D detections, for a multi-object 2D detector along with 3D depth information.

std_msgs/Header header
# A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information
#   this list with many candidate detections generated from a single input.
SpatialDetection[] detections

## These messages are created as close as possible to the official vision_msgs(http://wiki.ros.org/vision_msgs)