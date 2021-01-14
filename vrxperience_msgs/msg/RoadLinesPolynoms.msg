# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Unique identifier of the line road markings sensor
int32 global_id

# The ID of the equipped vehicle
int16 ego_vehicle_id

# Boolean value, true if the sent polynoms are noisy, false if not
bool is_noisy

# Array of structures describing each line polynom,
# from left to the right of the ego-vehicle
vrxperience_msgs/RoadLinePolynom[<=3] polynoms
