# Crazyflie Gazebo: Outer-Loop LQR + USC Inner PID

This folder contains the working Crazyflie Gazebo forward controller.

## Control architecture

The original USC cascaded position/velocity PID outer loop is replaced by a
6-state LQR outer-loop controller.

State error:

e = [ex, ey, ez, evx, evy, evz]^T

LQR output:

u = [roll_cmd, pitch_cmd, az_cmd]^T

Control structure:

Position / Velocity Error
        |
        v
6-State Outer-Loop LQR
        |
        +--> desired roll
        +--> desired pitch
        +--> vertical acceleration command
        |
        v
USC Attitude PID
        |
        v
USC Rate PID
        |
        v
USC Motor Mixer
        |
        v
Gazebo Crazyflie Motors

The USC attitude and angular-rate PID controllers are retained.

## LQR model

Continuous-time hover model:

xdot = vx
ydot = vy
zdot = vz

vxdot = g * theta
vydot = -g * phi
vzdot = az

State:

[ex, ey, ez, evx, evy, evz]

Input:

[roll_cmd, pitch_cmd, az_cmd]

LQR design timestep:

0.01 s

Q:

diag([8, 8, 12, 1.2, 1.2, 10])

R:

diag([6, 6, 2])

## Working Hover Test

Target:
x = 0 m
y = 0 m
z = 1 m
yaw = 0 deg

Takeoff:
10 s

Hover:
8 s

Landing:
8 s

Run inside the existing Gazebo environment:

python3 usc_lqr_position_gazebo.py \
  --x 0.0 \
  --y 0.0 \
  --z 1.0 \
  --yaw-deg 0.0 \
  --takeoff-seconds 10 \
  --hover-seconds 8 \
  --land-seconds 8 \
  --land-z 0.05
