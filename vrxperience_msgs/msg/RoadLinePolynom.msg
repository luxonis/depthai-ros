# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

# The ID of the line
int16 line_id

# The polynomial coefficients
float64 c0
float64 c1
float64 c2
float64 c3

# The line curvature radius, got from the RND data. If the line is straight,
# then this field is set to 99000.0 (infinite curve radius)
float64 curvature_radius

# 1/2c2, or 99000.0 (infinite curve radius) if c2=0.
float64 estimated_curvature_radius
