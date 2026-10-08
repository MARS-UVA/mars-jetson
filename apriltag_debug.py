#!/usr/bin/env python3

import math
import json


def rpy_to_quaternion(roll, pitch, yaw):
    """Convert Gazebo/ROS RPY (radians) -> quaternion [qx, qy, qz, qw]."""

    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    # Normalize
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)

    return (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm
    )


def gazebo_to_your_format(x, y, z, roll, pitch, yaw):
    qx, qy, qz, qw = rpy_to_quaternion(
        roll, pitch, yaw
    )

    return {
        "x": x,
        "y": y,
        "z": z,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw
    }


# ============================================================
# EDIT THESE
# ============================================================

x = 0.35
y = -0.05
z = 0.3

roll = 0.61
pitch = 1.57
yaw = 0.0


# ============================================================
# CONVERT
# ============================================================



result = gazebo_to_your_format(
    x, y, z,
    roll, pitch, yaw
)

print(json.dumps(result, indent=4))