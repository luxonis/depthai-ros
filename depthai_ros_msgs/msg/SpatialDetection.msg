
# Class probabilities
vision_msgs/ObjectHypothesis[] results

# 2D bounding box surrounding the object.
vision_msgs/BoundingBox2D bbox

# Center of the detected object in meters 
geometry_msgs/Point position

# If true, this message contains object tracking information.
bool is_tracking

# ID used for consistency across multiple detection messages. This value will
# likely differ from the id field set in each individual ObjectHypothesis.
# If you set this field, be sure to also set is_tracking to True.
string tracking_id