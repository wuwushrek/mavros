# Message for MPC_FULL_STATE
# This is used in model predictve control to get the full state of the robot
# It is also used for system identification of the system dynamics

# time_usec is the timestamp of the data
# x, y, z are the position of the robot
# vx, vy, vz are the velocity of the robot
# qw qx qy qz are the quaternion orientation of the robot
# wx wy wz are the angular velocity of the robot
# Tb, Mx, My, Mz are the thrust and moments of the vehcile

uint64 time_usec
float32 x
float32 y
float32 z
float32 vx
float32 vy
float32 vz
float32 qw
float32 qx
float32 qy
float32 qz
float32 wx
float32 wy
float32 wz
float32 tb
float32 mx
float32 my
float32 mz
