# Altitude MPC Message for logging
# For Ciconia - Hybrid VTOL Aircraft.

Header header

float64 time

float64 reference_altitude
float64 altitude
float64 home_altitude
float64 velocity
float64 acceleration_signal
float64 acceleration
float64 throttle
float64 reference_throttle

float64 Q_Z
float64 Q_ZDOT

float64 R

float64 Np
float64 Nc

float64 const_z_min
float64 const_z_max
float64 const_dz_min
float64 const_dz_max
float64 const_u_min
float64 const_u_max
float64 const_du_min
float64 const_du_max

float64 acc_to_thrust_p
float64 acc_to_thrust_i
float64 acc_to_thrust_d
