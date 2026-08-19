#!/usr/bin/env python3

"""
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

import gz.transport13 as gz_transport
import gz.msgs10.actuators_pb2 as actuators_pb2


# ---------------------------------------------------------------------
# Import USC controller code
# ---------------------------------------------------------------------
USC_PYTHON_DIR = "/root/usc_lqr_gazebo_test/usc_code/sala4_crazyflie/cflib_python"
if USC_PYTHON_DIR not in sys.path:
    sys.path.insert(0, USC_PYTHON_DIR)

from controller.controller_lqr_position import ControllerPID
from controller.controller_types import (
    AccData,
    Attitude,
    AttitudeRate,
    Axis3f,
    Control,
    GyroData,
    Position,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
)


# ---------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------
def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quat_to_euler_rad(qx, qy, qz, qw):
    # Roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ---------------------------------------------------------------------
# Main Gazebo wrapper
# ---------------------------------------------------------------------
class USCLQRPositionGazebo(Node):
    def __init__(self, args):
        super().__init__("usc_lqr_position_gazebo")

        self.target_x = args.x
        self.target_y = args.y
        self.target_z = args.z
        self.target_yaw_deg = args.yaw_deg

        self.takeoff_time = args.takeoff_seconds
        self.hover_time = args.hover_seconds
        self.land_time = args.land_seconds
        self.land_z = args.land_z

        self.start_time = time.time()
        self.last_print_time = 0.0
        self.log_path = Path("usc_lqr_position_gazebo_log.csv")
        self.log_data = []

        # USC controller objects
        self.controller = ControllerPID()
        self.controller.init()

        self.control = Control()
        self.state = State(
            attitude=Attitude(),
            position=Position(),
            velocity=Velocity(),
            acc=Axis3f(),
        )
        self.sensors = SensorData(
            gyro=GyroData(),
            acc=AccData(),
        )
        self.setpoint = Setpoint()
        self.stabilizer_step = 0

        # We run at 500 Hz, same as USC attitude loop.
        self.control_dt = 0.002

        # Gazebo motor calibration.
        # USC outputs PWM-like thrust values around 35000.
        # Gazebo expects motor angular velocity around 2321.5 rad/s for hover.
        self.pwm_hover = 35000.0
        self.omega_hover = 2321.5

        # This scale maps PWM perturbations around hover into Gazebo rad/s.
        # If takeoff is too weak/strong, tune only this value first.
        self.pwm_to_omega = args.pwm_to_omega

        self.motor_min = 0.0
        self.motor_max = 2618.0

        self.gz_node = gz_transport.Node()
        self.motor_pub = self.gz_node.advertise(
            "/crazyflie/python_motor_speed",
            actuators_pb2.Actuators,
        )

        self.create_subscription(Odometry, "/crazyflie/odom", self.odom_callback, 10)
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info("========== USC PHYSICAL LQR-POSITION GAZEBO ==========")
        self.get_logger().info("controller = physical LQR outer loop + USC attitude/rate PID")
        self.get_logger().info("odom topic = /crazyflie/odom")
        self.get_logger().info("motor topic = /crazyflie/python_motor_speed")
        self.get_logger().info(f"target = ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f})")
        self.get_logger().info(f"pwm_hover = {self.pwm_hover:.1f}")
        self.get_logger().info(f"omega_hover = {self.omega_hover:.1f}")
        self.get_logger().info(f"pwm_to_omega = {self.pwm_to_omega:.4f}")
        self.get_logger().info("====================================")

    def odom_callback(self, msg):
        # Position in meters
        self.state.position.x = msg.pose.pose.position.x
        self.state.position.y = msg.pose.pose.position.y
        self.state.position.z = msg.pose.pose.position.z

        # Linear velocity in m/s
        self.state.velocity.x = msg.twist.twist.linear.x
        self.state.velocity.y = msg.twist.twist.linear.y
        self.state.velocity.z = msg.twist.twist.linear.z

        # Orientation: Gazebo quaternion -> USC attitude in degrees
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = quat_to_euler_rad(qx, qy, qz, qw)

        self.state.attitude.roll = math.degrees(roll)
        self.state.attitude.pitch = math.degrees(pitch)
        self.state.attitude.yaw = math.degrees(yaw)

        # Angular velocity: Gazebo rad/s -> USC gyro deg/s
        self.sensors.gyro.x = math.degrees(msg.twist.twist.angular.x)
        self.sensors.gyro.y = math.degrees(msg.twist.twist.angular.y)
        self.sensors.gyro.z = math.degrees(msg.twist.twist.angular.z)

        # Acc is not critical for this PID path, but keep nominal values.
        self.sensors.acc.x = 0.0
        self.sensors.acc.y = 0.0
        self.sensors.acc.z = 1.0

    def smoothstep(self, s):
        """
        Smoothstep position profile:
        s=0 -> 0, s=1 -> 1
        velocity is zero at both ends.
        """
        s = max(0.0, min(1.0, s))
        return s * s * (3.0 - 2.0 * s)

    def smoothstep_dot(self, s):
        """
        Derivative of smoothstep w.r.t. normalized time s.
        This is high in the middle and zero near start/end.
        """
        s = max(0.0, min(1.0, s))
        return 6.0 * s * (1.0 - s)

    def smoothstep(self, s):
        """
        Smooth 0->1 transition.
        Velocity is zero at the beginning and end.
        """
        s = max(0.0, min(1.0, s))
        return s * s * (3.0 - 2.0 * s)

    def smoothstep_dot(self, s):
        """
        Derivative of smoothstep with respect to normalized time.
        """
        s = max(0.0, min(1.0, s))
        return 6.0 * s * (1.0 - s)

    def reference(self, elapsed):
        """
        Real USC architecture reference.

        We use smooth takeoff and smooth landing because the inner
        attitude/rate PID is still active. A hard z_ref step can make
        the mixer split motors and flip the drone.
        """
        takeoff_end = self.takeoff_time
        hover_end = self.takeoff_time + self.hover_time
        total_time = self.takeoff_time + self.hover_time + self.land_time

        z_start = 0.0

        if elapsed < takeoff_end:
            T = max(self.takeoff_time, 1e-6)
            s = elapsed / T
            h = self.smoothstep(s)
            hdot = self.smoothstep_dot(s)

            z_ref = z_start + h * (self.target_z - z_start)
            vz_ref = (self.target_z - z_start) / T * hdot

        elif elapsed < hover_end:
            z_ref = self.target_z
            vz_ref = 0.0

        elif elapsed < total_time:
            T = max(self.land_time, 1e-6)
            s = (elapsed - hover_end) / T
            h = self.smoothstep(s)
            hdot = self.smoothstep_dot(s)

            z_ref = self.target_z + h * (self.land_z - self.target_z)
            vz_ref = (self.land_z - self.target_z) / T * hdot

        else:
            z_ref = self.land_z
            vz_ref = 0.0

        return self.target_x, self.target_y, z_ref, self.target_yaw_deg, vz_ref

    def update_setpoint(self, elapsed):
        x_ref, y_ref, z_ref, yaw_ref, vz_ref = self.reference(elapsed)

        self.setpoint.position = Position(x=x_ref, y=y_ref, z=z_ref)
        self.setpoint.velocity = Velocity(x=0.0, y=0.0, z=vz_ref)
        self.setpoint.attitude = Attitude(roll=0.0, pitch=0.0, yaw=yaw_ref)
        self.setpoint.attitude_rate = AttitudeRate(roll=0.0, pitch=0.0, yaw=0.0)
        self.setpoint.velocity_body = False

        self.setpoint.mode = SetpointMode()
        self.setpoint.mode.x = StabMode.MODE_ABS
        self.setpoint.mode.y = StabMode.MODE_ABS
        self.setpoint.mode.z = StabMode.MODE_ABS
        self.setpoint.mode.roll = StabMode.MODE_ABS
        self.setpoint.mode.pitch = StabMode.MODE_ABS
        self.setpoint.mode.yaw = StabMode.MODE_ABS

    def usc_control_to_pwm(self):
        """
        Full USC-style motor mixing.

        Important:
        The previous mixer had the wrong sign/order for the Gazebo motor layout.
        That created positive feedback: small pitch error -> motor split -> larger pitch error.

        This version reverses roll/pitch correction signs for the Gazebo layout.
        """
        thrust = float(getattr(self.control, "thrust", 0.0))
        roll = float(getattr(self.control, "roll", 0.0))
        pitch = float(getattr(self.control, "pitch", 0.0))
        yaw = float(getattr(self.control, "yaw", 0.0))

        mix_limit = 600.0
        roll = max(-mix_limit, min(mix_limit, roll))
        pitch = max(-mix_limit, min(mix_limit, pitch))
        yaw = max(-mix_limit, min(mix_limit, yaw))

        # Reversed roll/pitch signs compared with previous attempt.
        pwm = [
            thrust + roll - pitch + yaw,
            thrust + roll + pitch - yaw,
            thrust - roll + pitch + yaw,
            thrust - roll - pitch - yaw,
        ]

        pwm_min = 0.0
        pwm_max = 60000.0
        pwm = [max(pwm_min, min(pwm_max, float(x))) for x in pwm]

        return pwm

    def pwm_to_motor_speed(self, pwm):
        """
        Convert USC PWM-like command to Gazebo motor angular speed.

        Matches the scaling used in this script:
            pwm_hover = 35000
            omega_hover = 2321.5
            pwm_to_omega = 0.0200
        """
        omega = []
        for p in pwm:
            w = self.omega_hover + (float(p) - self.pwm_hover) * self.pwm_to_omega

            # keep inside same practical motor range seen in previous logs
            w = max(0.0, min(2618.0, w))
            omega.append(w)

        return omega

    def publish_motors(self, omega):
        msg = actuators_pb2.Actuators()
        for w in omega:
            msg.velocity.append(float(w))
        self.motor_pub.publish(msg)

    def stop_motors(self):
        self.publish_motors(np.zeros(4))

    def log_sample(self, elapsed, x_ref, y_ref, z_ref, yaw_ref, vz_ref, pwm, omega):
        self.log_data.append({
            "time": elapsed,
            "x": self.state.position.x,
            "y": self.state.position.y,
            "z": self.state.position.z,
            "vx": self.state.velocity.x,
            "vy": self.state.velocity.y,
            "vz": self.state.velocity.z,
            "roll_deg": self.state.attitude.roll,
            "pitch_deg": self.state.attitude.pitch,
            "yaw_deg": self.state.attitude.yaw,
            "x_ref": x_ref,
            "y_ref": y_ref,
            "z_ref": z_ref,
            "yaw_ref": yaw_ref,
            "vz_ref": vz_ref,
            "err_z": self.state.position.z - z_ref,
            "control_thrust": self.control.thrust,
            "control_roll": self.control.roll,
            "control_pitch": self.control.pitch,
            "control_yaw": self.control.yaw,
            "pwm_1": pwm[0],
            "pwm_2": pwm[1],
            "pwm_3": pwm[2],
            "pwm_4": pwm[3],
            "omega_1": omega[0],
            "omega_2": omega[1],
            "omega_3": omega[2],
            "omega_4": omega[3],
        })

    def save_log(self):
        if not self.log_data:
            return

        keys = list(self.log_data[0].keys())
        with open(self.log_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.log_data)

        self.get_logger().info(f"Saved log: {self.log_path}")

    def control_loop(self):
        elapsed = time.time() - self.start_time
        total_time = self.takeoff_time + self.hover_time + self.land_time

        if elapsed > total_time:
            self.stop_motors()
            self.save_log()
            self.get_logger().info("Done. Sent zero motor speeds.")
            rclpy.shutdown()
            return

        self.update_setpoint(elapsed)
        x_ref, y_ref, z_ref, yaw_ref, vz_ref = self.reference(elapsed)

        # Run USC PID controller
        self.controller.controller_pid(
            self.control,
            self.setpoint,
            self.sensors,
            self.state,
            self.stabilizer_step,
        )

        self.stabilizer_step += 1

        pwm = self.usc_control_to_pwm()
        omega = self.pwm_to_motor_speed(pwm)

        self.publish_motors(omega)
        self.log_sample(elapsed, x_ref, y_ref, z_ref, yaw_ref, vz_ref, pwm, omega)

        if elapsed - self.last_print_time > 0.25:
            self.last_print_time = elapsed
            self.get_logger().info(
                f"t={elapsed:4.1f} "
                f"z={self.state.position.z:5.2f} "
                f"z_ref={z_ref:4.2f} "
                f"err_z={self.state.position.z - z_ref: .2f} "
                f"thrust={self.control.thrust:7.1f} "
                f"rpy=({self.state.attitude.roll: .1f},"
                f"{self.state.attitude.pitch: .1f},"
                f"{self.state.attitude.yaw: .1f}) "
                f"omega=({omega[0]:.1f},{omega[1]:.1f},{omega[2]:.1f},{omega[3]:.1f})"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=1.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)

    parser.add_argument("--takeoff-seconds", type=float, default=15.0)
    parser.add_argument("--hover-seconds", type=float, default=10.0)
    parser.add_argument("--land-seconds", type=float, default=10.0)
    parser.add_argument("--land-z", type=float, default=0.05)

    parser.add_argument("--pwm-to-omega", type=float, default=0.020)

    args = parser.parse_args()

    rclpy.init()
    node = USCLQRPositionGazebo(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
        node.save_log()
        print("KeyboardInterrupt: motors stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
