#!/usr/bin/env python3
"""
Small math helpers shared by runtime code and unit tests.
"""

import math


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (radians) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
