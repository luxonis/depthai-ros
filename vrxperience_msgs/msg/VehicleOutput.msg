# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Center of gravity speed vector, positive to the front of the vehicle
geometry_msgs/Vector3 cdg_speed

# Center of gravity angular speed vector (RPY)
geometry_msgs/Vector3 cdg_angular_speed

# Center of gravity acceleration vector, positive to the front of the vehicle
geometry_msgs/Vector3 cdg_accel

# Center of gravity angular acceleration vector (RPY)
geometry_msgs/Vector3 cdg_angular_accel

# Position of the vehicle's cabin CoG in the absolute referential
geometry_msgs/Vector3 cdg_position

# Orientation of the vehicle's cabin CoG in the absolute referential (RPY)
geometry_msgs/Vector3 cdg_orientation

# Masks of flags corresponding to the active lights
int32 lights

# Masks of flags corresponding to the active rosidl_generate_interfaces
int32 sirens

# Masks of flags corresponding to the active auxilary_lights
int32 auxilary_lights

# State of the engine (range: 0 OFF, 1 ON)
bool engine_state

# Engine fuel consumption (m3/s)
float64 consumption

# Engine CO2 emissions (kg/s)
float64 pollution

# Engine speed (rad/s)
float64 engine_speed

# Gear really engaged in the motor (range: from -1 to 5)
int32 gear_engaged

# Effective gas pedal position [0 1] (after regulations)
float64 gas_pedal

# Effective brake pedal force (after regulations) [N]
float64 brake_pedal

# Effective hand_brake force (after regulations) [N]
float64 hand_brake

# Effective clutch pedal position [0 1] (after regulations)
float64 clutch_pedal

# Effective gear box mode
int32 gear_box_mode

# Effective ignition key position
int32 effective_ignition_key_position

bool is_engine_retarder
int32 brake_retarder_notch
float64 trailer_brake
int32 transfer_choice

# Lock differentials mode
int32 lock_diff_mode

# Is transmission locked up
bool force_lockup

# Effective plow mode
int32 plow_mode

float64 steering_wheel_angle
float64 steering_wheel_speed
float64 steering_wheel_accel
float64 steering_wheel_torque

geometry_msgs/Point[4] hub_position

# RPY
geometry_msgs/Vector3[4] hub_orientation

float64[4] wheel_rotation
float64[4] wheel_rotation_speed

# Degrees Celsius
float64[4] brake_temperature

bool[4] abs_is_active
bool[4] is_in_contact
geometry_msgs/Point[4] contact_points
geometry_msgs/Vector3[4] ground_normals
float64[4] grip

# Material type of the road under the tire
int32[4] ground_index

# Longitudinal slip ratio of the tires
float64[4] lsr

# Tire slip angle
float64[4] slip_angle

geometry_msgs/Vector3[4] tire_force

int32[4] lane_type
float64[4] roughness_height
float64[4] roughness_length
float64[4] roughness_height_stddev
float64[4] roughness_length_stddev

int32 E_GROUND_ASPHALT = 0
int32 E_GROUND_CONRETE = 1
int32 E_GROUND_GRASS = 2
int32 E_GROUND_COBBLESTONE = 3

int32[4] ground_type

# Water height on the road uner the tire
float64[4] water_height

float64[512] custom_output
