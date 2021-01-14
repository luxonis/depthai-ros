# Copyright (c) 2020 Adam Gotlib (WUT Driverless)
# For internal use of Indy Autonomous Challenge participants
# DO NOT REDISTRIBUTE. See README.md for details

std_msgs/Header header

# Additional angle value to apply on input of dynamics model [rad]
float64 additive_steering_wheel_angle

# Multiplicative coefficient to apply on input of dynamics model
float64 multiplicative_steering_wheel_angle

# Additional speed value to apply on input of dynamics model [rad/s]
float64 additive_steering_wheel_speed

# Multiplicative coefficient to apply on input of dynamics model
float64 multiplicative_steering_wheel_speed

# Additional angular acceleration to apply on input of dynamics model
float64 additive_steering_wheel_accel

# Multiplicative coefficient to apply on input of dynamics model
float64 multiplicative_steering_wheel_accel

# Additional torque value to apply on input of dynamics model
float64 additive_steering_wheel_torque

# Multiplicative coefficient to apply on input of dynamics model
float64 multiplicative_steering_wheel_torque
