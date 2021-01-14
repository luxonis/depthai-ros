# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Additive value for CabToModel Accelerator value (default is 0) [%]
float64 accelerator_additive

# Multiplicative coefficient for CabToModel Accelerator value (default is 1)
float64 accelerator_multiplicative

# Additive value for CabToModel Brake value (default is 0) [N]
float64 brake_additive

# Multiplicative coefficient for CabToModel Clutch value (default is 1)
float64 brake_multiplicative

# Additive value for CabToModel Clutch value (default is 0)
float64 clutch_additive

# Multiplicative coefficient for CabToModel Clutch value (default is 1)
float64 clutch_multiplicative

# Additive value for CabToModel ParkingBrake value (default is 0) [N]
float64 parking_brake_additive

# Multiplicative coefficient for CabToModel ParkingBrake value (default is 1)
float64 parking_brake_multiplicative

# True to take over gear box mode from CabToModel (default is false)
bool gear_box_take_over

int16 T_AUTO_MODE_MANUAL = 0
int16 T_AUTO_MODE_REVERSE2 = 1
int16 T_AUTO_MODE_REVERSE1 = 2
int16 T_AUTO_MODE_NEUTRAL = 3
int16 T_AUTO_MODE_PARK = 4
int16 T_AUTO_MODE_LIMIT1 = 5
int16 T_AUTO_MODE_LIMIT2 = 6
int16 T_AUTO_MODE_LIMIT3 = 7
int16 T_AUTO_MODE_LIMIT4 = 8
int16 T_AUTO_MODE_RACE = 9
int16 T_AUTO_MODE_DRIVE = 10
int16 T_AUTO_MODE_OVER_DRIVE = 11
int16 SEQUENTIAL = 12

# Gear box mode
int16 gear_box_auto_mode

# Wanted gear (range: -2 to max_gear), used if GearBoxAutoMode is 0 (TAutoModeManual)
int16 wanted_gear

bool shift_up
bool shift_down

bool is_ratio_limit
int16 min_ratio
int16 max_ratio
