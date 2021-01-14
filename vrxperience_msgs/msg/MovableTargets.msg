# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Unique identifier of the line road markings sensor
int32 global_id

# The ID of the equipped vehicle
int16 ego_vehicle_id

# Index in targets of the nearest target detected.
# If no target detected, the value is -1
int16 nearest_target

# Array of targets detected by the sensor
vrxperience_msgs/MovableTarget[<=30] targets
