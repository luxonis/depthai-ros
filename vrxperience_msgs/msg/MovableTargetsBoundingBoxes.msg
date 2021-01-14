# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Unique identifier of the sensor
int32 global_id

# The ID of the equipped vehicle
int16 ego_vehicle_id

# Array of target bounding boxes detected by the sensor
vrxperience_msgs/MovableTargetBoundingBox[<=30] bounding_boxes
