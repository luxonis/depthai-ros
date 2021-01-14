# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Unique identifier of the sensor
int32 global_id

# The index of the vehicle
int16 ego_vehicle_id

# The GPS index
int16 sensor_id

# Latitude of the vehicle's sensor position [deg]
float64 latitude

# Longitude of the vehicle's sensor position [deg]
float64 longitude

# Altitude of the vehicle's sensor position [deg]
float64 altitude

# Number of satellites used
int16 satellites

# GPS sensor horizontal precision
float32 hdop
