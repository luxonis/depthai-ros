
# Center of the 

string label

# Landmarks score.
float32 lm_score

# 21 2D hand-knuckles on the detected hand
geometry_msgs/Pose2D[] landmark

# Spatial Position of the palm 
geometry_msgs/Point position

# detected hand-gesture using hand-knukles landmarks
string gesture

# is set to true if Spatial position is enabled
bool is_spatial
