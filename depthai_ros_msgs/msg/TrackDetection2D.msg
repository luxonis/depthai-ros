
# Class probabilities
vision_msgs/ObjectHypothesisWithPose[] results

# 2D bounding box surrounding the object.
vision_msgs/BoundingBox2D bbox

# If true, this message contains object tracking information.
bool is_tracking

# ID used for consistency across multiple detection messages. This value will
# likely differ from the id field set in each individual ObjectHypothesis.
# If you set this field, be sure to also set is_tracking to True.
string tracking_id

# Age: number of frames the object is being tracked
int32 tracking_age

# Status of the tracking:
# 0 = NEW -> the object is newly added.
# 1 = TRACKED -> the object is being tracked.
# 2 = LOST -> the object gets lost now. The object can be tracked again automatically (long term tracking) 
#			  or by specifying detected object manually (short term and zero term tracking).
# 3 = REMOVED -> the object is removed.
int32 tracking_status
