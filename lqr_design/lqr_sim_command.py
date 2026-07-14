#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time


TOPIC = "/crazyflie/desired_state"
MESSAGE_TYPE = "gz.msgs.Pose"


def send_setpoint(
    x: float,
    y: float,
    z: float,
    yaw: float,
    vx: float = 0.0,
    vy: float = 0.0,
    vz: float = 0.0,
) -> None:
    message = (
        f"position: {{x: {x}, y: {y}, z: {z}}} "
        f"orientation: {{x: {vx}, y: {vy}, z: {vz}, w: {yaw}}}"
    )

    command = [
        "gz",
        "topic",
        "-t",
        TOPIC,
        "-m",
        MESSAGE_TYPE,
        "-p",
        message,
    ]

    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        raise RuntimeError(
            result.stderr.strip()
            or "Failed to publish Gazebo setpoint."
        )


def hold_setpoint(
    x: float,
    y: float,
    z: float,
    yaw: float,
    duration: float,
    rate_hz: float = 10.0,
) -> None:
    period = 1.0 / rate_hz
    start = time.monotonic()

    while time.monotonic() - start < duration:
        send_setpoint(x, y, z, yaw)
        time.sleep(period)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send position and yaw commands to the Gazebo LQR plugin."
    )

    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=1.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=10.0)

    args = parser.parse_args()

    print(
        f"Sending desired state: "
        f"x={args.x:.3f}, "
        f"y={args.y:.3f}, "
        f"z={args.z:.3f}, "
        f"yaw={args.yaw:.3f}"
    )

    try:
        hold_setpoint(
            args.x,
            args.y,
            args.z,
            args.yaw,
            args.duration,
            args.rate,
        )
    except KeyboardInterrupt:
        print("\nCommand stopped.")
        return 130
    except Exception as exc:
        print(f"Error: {exc}")
        return 1

    print("Command completed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
