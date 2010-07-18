# This message holds the state from the 9 degree of freedom Razor IMU from Sparkfun
#
# Magnetometer readings are assumed to be in Gauss
# Angular velocities are in rads/sec
# Linear acceleration is in m/s^2
# 
# +X axis points towards the Razor's serial connection
# +Y axis points to the right of +X
# +Z axis points downwards

Header header

geometry_msgs/Vector3 magnetometer
float64 mag_error

geometry_msgs/Vector3 angular_velocity 
float64 omega_error

geometry_msgs/Vector3 linear_acceleration
float64 accel_error
