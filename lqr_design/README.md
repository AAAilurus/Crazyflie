# Discrete LQR Design

This directory contains the Python tools used to design the Crazyflie hover controller.

## Workflow

1. Build the nonlinear and linear hover model.
2. Calculate the continuous-time state-space matrices A and B.
3. Discretize the model to obtain Ad and Bd.
4. Solve the discrete algebraic Riccati equation.
5. Compute the discrete LQR gain K.
6. Send desired position and yaw commands to the Gazebo simulation.

## Files

- `compute_dlqr.py`: model linearization, discretization, DARE solution, and gain calculation.
- `lqr_sim_command.py`: sends desired x, y, z, and yaw commands to the simulated Crazyflie.
