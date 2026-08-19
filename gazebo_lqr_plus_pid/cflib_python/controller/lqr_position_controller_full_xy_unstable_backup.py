"""
Physical outer-loop LQR position controller for Crazyflie.

This replaces the USC position + velocity PID part with a hover-linearized LQR.

Original USC:
    position PID -> velocity PID -> attitude PID -> rate PID -> motors

Modified USC:
    physical LQR -> attitude PID -> rate PID -> motors

The LQR is designed from:
    A = df/dx
    B = df/du

State:
    x = [px_error, py_error, pz_error, vx_error, vy_error, vz_error]

Input:
    u = [roll_cmd, pitch_cmd, az_cmd]

where:
    roll_cmd  = desired roll angle [rad]
    pitch_cmd = desired pitch angle [rad]
    az_cmd    = vertical acceleration correction [m/s^2]
"""

import math
from typing import Tuple

import numpy as np
from scipy.linalg import expm, solve_discrete_are

from .pid_constants import *
from .pid import (
    PIDObject,
    constrain,
    pid_reset,
)
from .controller_types import (
    POSITION_RATE,
    Attitude,
    Setpoint,
    StabMode,
    State,
)


class PIDAxis:
    """
    Dummy PID axis for compatibility.

    The LQR version does not use position or velocity PID,
    but controller_pid.py may still call reset_all_pid().
    """

    def __init__(self):
        self.pid = PIDObject()
        self.previous_mode = StabMode.MODE_DISABLE
        self.setpoint = 0.0
        self.output = 0.0


def build_hover_lqr_gain(dt: float):
    """
    Build physical hover-linearized LQR gain.

    Continuous-time translational hover model:

        px_dot = vx
        py_dot = vy
        pz_dot = vz

        vx_dot = g * pitch
        vy_dot = -g * roll
        vz_dot = az

    State:
        x = [px, py, pz, vx, vy, vz]

    Input:
        u = [roll, pitch, az]

    Therefore:
        A = df/dx
        B = df/du
    """

    g = 9.81

    n = 6
    m = 3

    Ac = np.zeros((n, n))
    Bc = np.zeros((n, m))

    # Position derivatives
    Ac[0, 3] = 1.0
    Ac[1, 4] = 1.0
    Ac[2, 5] = 1.0

    # Small-angle hover acceleration coupling
    # vx_dot = g * pitch
    # vy_dot = -g * roll
    # vz_dot = az
    Bc[3, 1] = g
    Bc[4, 0] = -g
    Bc[5, 2] = 1.0

    # Exact zero-order-hold discretization, same style as independent LQR
    block = np.zeros((n + m, n + m))
    block[:n, :n] = Ac
    block[:n, n:] = Bc

    discrete_block = expm(block * dt)
    Ad = discrete_block[:n, :n]
    Bd = discrete_block[:n, n:]

    # LQR weights.
    # Increase position weights for faster response.
    # Increase R for smoother smaller roll/pitch/thrust commands.
    Q = np.diag([
        8.0, 8.0, 12.0,     # position errors: x, y, z
        1.2, 1.2, 10.0,     # velocity errors: vx, vy, vz
    ])

    R = np.diag([
        6.0, 6.0, 2.0,      # roll, pitch, az
    ])

    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)

    return Ac, Bc, Ad, Bd, Q, R, P, K


class LQRPositionController:
    """
    USC-compatible physical LQR position controller.

    It keeps the same interface as the original USC PositionController:

        position_controller(setpoint, state) -> thrust, attitude_desired

    But internally it does:

        error = [p - p_des, v - v_des]
        u = -K error
        u = [roll_cmd, pitch_cmd, az_cmd]

    Then it returns:
        thrust
        attitude_desired.roll
        attitude_desired.pitch

    The USC main controller will continue with:
        attitude PID -> rate PID -> motor mixing
    """

    def __init__(self):
        self.dt = 1.0 / POSITION_RATE

        # Build physical LQR matrices
        (
            self.Ac,
            self.Bc,
            self.Ad,
            self.Bd,
            self.Q_lqr,
            self.R_lqr,
            self.P_lqr,
            self.K_lqr,
        ) = build_hover_lqr_gain(self.dt)

        # Thrust parameters from USC PID defaults
        if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD:
            self.thrust_base = PID_VEL_THRUST_BASE_BARO_Z_HOLD
        else:
            self.thrust_base = PID_VEL_THRUST_BASE

        self.thrust_min = PID_VEL_THRUST_MIN
        self.thrust_max = UINT16_MAX

        # LQR output limits
        # USC attitude controller expects roll/pitch in degrees later.
        # Conservative LQR attitude limits for initial x/y position-control tests.
        self.max_roll_rad = math.radians(0.5)
        self.max_pitch_rad = math.radians(0.5)
        self.max_az = 15.0

        # Map vertical acceleration command to USC thrust unit.
        # This is a Gazebo/USC-interface calibration parameter.
        # If takeoff is too slow/fast, tune this first.
        self.az_to_thrust = 3500.0

        # Debug values
        self.last_error = np.zeros(6)
        self.last_u = np.zeros(3)
        self.last_roll_cmd_rad = 0.0
        self.last_pitch_cmd_rad = 0.0
        self.last_az_cmd = 0.0
        self.last_thrust = self.thrust_base

        # Compatibility dummy PID objects
        self.pid_x = PIDAxis()
        self.pid_y = PIDAxis()
        self.pid_z = PIDAxis()
        self.pid_vx = PIDAxis()
        self.pid_vy = PIDAxis()
        self.pid_vz = PIDAxis()

        print("========== LQR POSITION CONTROLLER ==========")
        print("Model: physical hover linearization")
        print("State x = [ex, ey, ez, evx, evy, evz]")
        print("Input u = [roll_cmd, pitch_cmd, az_cmd]")
        print(f"dt = {self.dt:.4f} s")
        print(f"Ac shape = {self.Ac.shape}")
        print(f"Bc shape = {self.Bc.shape}")
        print(f"Ad shape = {self.Ad.shape}")
        print(f"Bd shape = {self.Bd.shape}")
        print(f"K shape  = {self.K_lqr.shape}")
        print("K =")
        print(self.K_lqr)
        print("=============================================")

    def init(self):
        """C-style compatibility."""
        pass

    def position_controller(self, setpoint: Setpoint, state: State) -> Tuple[float, Attitude]:
        """
        Physical LQR position controller.

        Args:
            setpoint: Desired position/velocity/yaw
            state: Current Crazyflie state

        Returns:
            thrust, attitude_desired
        """

        attitude_desired = Attitude()

        # Use global-frame translational error.
        # This matches the physical hover model:
        # x_dot = vx, y_dot = vy, z_dot = vz.
        ex = state.position.x - setpoint.position.x
        ey = state.position.y - setpoint.position.y
        ez = state.position.z - setpoint.position.z

        evx = state.velocity.x - setpoint.velocity.x
        evy = state.velocity.y - setpoint.velocity.y
        evz = state.velocity.z - setpoint.velocity.z

        error = np.array([ex, ey, ez, evx, evy, evz], dtype=float)

        # LQR control law
        # u = [roll_cmd_rad, pitch_cmd_rad, az_cmd]
        u = -self.K_lqr @ error

        # Full position/velocity LQR outer loop.
        # The hover reference has zero roll and pitch at equilibrium,
        # but the outer-loop LQR must command temporary roll/pitch
        # during transients to correct x/y position and velocity errors.
        roll_cmd = float(u[0])
        pitch_cmd = -float(u[1])
        az_cmd = float(u[2])

        # Respect disabled/manual modes.
        if setpoint.mode.x != StabMode.MODE_ABS:
            pitch_cmd = 0.0

        if setpoint.mode.y != StabMode.MODE_ABS:
            roll_cmd = 0.0

        if setpoint.mode.z != StabMode.MODE_ABS:
            az_cmd = 0.0

        # Saturate commands for safety
        roll_cmd = constrain(roll_cmd, -self.max_roll_rad, self.max_roll_rad)
        pitch_cmd = constrain(pitch_cmd, -self.max_pitch_rad, self.max_pitch_rad)
        az_cmd = constrain(az_cmd, -self.max_az, self.max_az)

        # Convert rad to degrees for USC attitude controller.
        #
        # Sign convention:
        # From physical model:
        #   vx_dot = g * pitch
        #   vy_dot = -g * roll
        #
        # If signs look reversed in Gazebo, flip these two lines only.
        attitude_desired.roll = math.degrees(roll_cmd)
        attitude_desired.pitch = math.degrees(pitch_cmd)

        # Convert vertical acceleration correction to USC thrust command.
        thrust = self.thrust_base + az_cmd * self.az_to_thrust
        thrust = constrain(thrust, self.thrust_min, self.thrust_max)

        # Yaw reference is passed to the USC attitude controller.
        attitude_desired.yaw = setpoint.attitude.yaw

        # Store latest LQR values for logging and diagnostics.
        self.last_error = error
        self.last_u = np.array([roll_cmd, pitch_cmd, az_cmd], dtype=float)
        self.last_roll_cmd_rad = roll_cmd
        self.last_pitch_cmd_rad = pitch_cmd
        self.last_az_cmd = az_cmd
        self.last_thrust = thrust

        return thrust, attitude_desired

    def reset_all_pid(self, x_actual: float, y_actual: float, z_actual: float):
        """
        Reset compatibility.

        No position/velocity PID is used in this LQR version.
        """
        pid_reset(self.pid_x.pid, x_actual)
        pid_reset(self.pid_y.pid, y_actual)
        pid_reset(self.pid_z.pid, z_actual)
        pid_reset(self.pid_vx.pid, 0.0)
        pid_reset(self.pid_vy.pid, 0.0)
        pid_reset(self.pid_vz.pid, 0.0)

    def get_lqr_debug_data(self):
        """Return LQR matrices and latest command for debugging."""
        return {
            "Ac": self.Ac,
            "Bc": self.Bc,
            "Ad": self.Ad,
            "Bd": self.Bd,
            "Q": self.Q_lqr,
            "R": self.R_lqr,
            "P": self.P_lqr,
            "K": self.K_lqr,
            "last_error": self.last_error,
            "last_u": self.last_u,
            "last_thrust": self.last_thrust,
        }
