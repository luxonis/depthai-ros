# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

# Boolean value true if an point was detected with this ray
bool hit

# Horizontal angle relative to sensors main direction of this ray [rad]
float32 h_angle

# Vertical angle relative to sensors main direction of this ray
float32 v_angle

# Position of the intersection point. Valid only if hit is true
geometry_msgs/Point intersection_point

# Distance from sensor of the hit point. Valid only if hit is true [m]
float32 distance
