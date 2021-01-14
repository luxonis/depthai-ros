# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Unique identifier of the sensor
int32 global_id

# The index of the vehicle
int16 ego_vehicle_id

# The laser index
int16 sensor_id

# Counter of emitted results
int16 frame_count

# Nearest point detected, if no point detected, the value is -1
int16 nearest_point

# Array of points detected by the laser sensors
vrxperience_msgs/LaserMeterPoint[<=720] results
