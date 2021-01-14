# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# The ID of the associated sensor target
int16 id

# Corner coordinates of the bounding box
geometry_msgs/Point rear_bottom_right
geometry_msgs/Point rear_top_right
geometry_msgs/Point rear_bottom_left
geometry_msgs/Point rear_top_left
geometry_msgs/Point front_bottom_right
geometry_msgs/Point front_top_right
geometry_msgs/Point front_bottom_left
geometry_msgs/Point front_top_left
