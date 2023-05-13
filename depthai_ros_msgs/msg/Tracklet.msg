# TrackingStatus
uint8 NEW=0
uint8 TRACKED=1
uint8 LOST=2
uint8 REMOVED=3

# Rect roi
vision_msgs/BoundingBox2D roi

# Tracklet's ID.
int32 id

# Tracklet's label ID.
int32 label

# Number of frames it is being tracked for.
int32 age

# Status of tracklet.
int32 status

# Image detection that is tracked.
ImgDetection srcImgDetection

# Spatial coordinates of tracklet.
geometry_msgs/Point spatialCoordinates
