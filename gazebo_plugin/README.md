# Gazebo LQR Controller Plugin

This directory contains the C++ Gazebo plugin used to execute the Crazyflie discrete LQR controller.

## Controller workflow

1. Read the precomputed LQR gain matrix K.
2. Read the current Crazyflie state from Gazebo.
3. Compute the state error relative to the desired state.
4. Apply the control law:

   u = -K(x - x_des)

5. Convert the LQR control inputs into motor angular velocities.
6. Apply the motor commands to the Crazyflie model in Gazebo.

## Controlled states

- Position: x, y, z
- Linear velocity: vx, vy, vz
- Attitude: roll, pitch, yaw
- Angular rates: p, q, r

## Source file

- `source/crazyflie_lqr_controller.cpp`
