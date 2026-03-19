#!/usr/bin/env python3
"""
Pure helpers for mapping external control inputs into vehicle motion targets.
"""

from dataclasses import dataclass
import math
from typing import Tuple


TANK_DRIVE_MODE = 'tank'
ACKERMANN_DRIVE_MODE = 'ackermann'
PRIUS_DRIVE_MODE = 'prius'
SUPPORTED_DRIVE_MODES = {TANK_DRIVE_MODE, ACKERMANN_DRIVE_MODE, PRIUS_DRIVE_MODE}

DEFAULT_ACKERMANN_STEERING_LIMIT = 0.6458
DEFAULT_ACKERMANN_WHEEL_BASE = 2.86

GEAR_UNKNOWN = 0
GEAR_PARKING = 1
GEAR_REVERSE = 2
GEAR_NEUTRAL = 3
GEAR_DRIVE = 4
SUPPORTED_GEARS = {
    GEAR_PARKING,
    GEAR_REVERSE,
    GEAR_NEUTRAL,
    GEAR_DRIVE,
}


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a value into the inclusive [lower, upper] range."""
    return max(lower, min(upper, value))


def normalize_drive_mode(mode: str) -> str:
    """Normalize the configured drive mode, falling back to tank."""
    normalized = (mode or TANK_DRIVE_MODE).strip().lower()
    if normalized not in SUPPORTED_DRIVE_MODES:
        return TANK_DRIVE_MODE
    return normalized


@dataclass(frozen=True)
class ControlTargets:
    """Motion targets derived from external control inputs."""

    linear_velocity: float
    angular_velocity: float
    steering_angle: float
    throttle: float = 0.0
    brake: float = 0.0
    steering_input: float = 0.0
    gear: int = GEAR_UNKNOWN


def steering_input_to_angle(
    steering_input: float,
    steering_limit: float = DEFAULT_ACKERMANN_STEERING_LIMIT,
) -> float:
    """
    Convert normalized external steering into a wheel steering angle.

    The external UDP contract uses -1 for left and +1 for right, while
    ROS/Gazebo positive yaw is left, so the sign is inverted here.
    """
    normalized = clamp(float(steering_input), -1.0, 1.0)
    return -normalized * steering_limit


def steering_angle_to_yaw_rate(
    linear_velocity: float,
    steering_angle: float,
    wheel_base: float = DEFAULT_ACKERMANN_WHEEL_BASE,
) -> float:
    """Convert a steering angle into an Ackermann-compatible yaw rate."""
    if abs(wheel_base) < 1e-9:
        return 0.0
    return float(linear_velocity) * math.tan(float(steering_angle)) / wheel_base


def normalize_gear(gear: int) -> int:
    """Normalize Ottopia gear values, falling back to neutral."""
    normalized = int(gear)
    if normalized not in SUPPORTED_GEARS:
        return GEAR_NEUTRAL
    return normalized


def throttle_to_prius_pedals(throttle: float, gear: int) -> Tuple[float, float]:
    """Split signed Ottopia gas pedal input into Prius throttle/brake commands."""
    normalized_throttle = clamp(float(throttle), -1.0, 1.0)
    normalized_gear = normalize_gear(gear)

    if normalized_throttle < 0.0:
        return 0.0, abs(normalized_throttle)

    if normalized_gear in {GEAR_DRIVE, GEAR_REVERSE}:
        return normalized_throttle, 0.0

    return 0.0, 0.0


def udp_command_to_targets(
    throttle: float,
    steering: float,
    drive_mode: str,
    max_linear_speed: float,
    max_angular_speed: float,
    gear: int = GEAR_UNKNOWN,
    ackermann_steering_limit: float = DEFAULT_ACKERMANN_STEERING_LIMIT,
    ackermann_wheel_base: float = DEFAULT_ACKERMANN_WHEEL_BASE,
) -> ControlTargets:
    """Map normalized UDP control values into motion targets."""
    normalized_throttle = clamp(float(throttle), -1.0, 1.0)
    normalized_steering = clamp(float(steering), -1.0, 1.0)
    normalized_gear = normalize_gear(gear)
    linear_velocity = normalized_throttle * float(max_linear_speed)
    mode = normalize_drive_mode(drive_mode)

    if mode == PRIUS_DRIVE_MODE:
        prius_throttle, prius_brake = throttle_to_prius_pedals(
            normalized_throttle,
            normalized_gear,
        )
        return ControlTargets(
            linear_velocity=0.0,
            angular_velocity=0.0,
            steering_angle=0.0,
            throttle=prius_throttle,
            brake=prius_brake,
            steering_input=normalized_steering,
            gear=normalized_gear,
        )

    if mode == ACKERMANN_DRIVE_MODE:
        steering_angle = steering_input_to_angle(
            normalized_steering,
            ackermann_steering_limit,
        )
        angular_velocity = steering_angle_to_yaw_rate(
            linear_velocity,
            steering_angle,
            ackermann_wheel_base,
        )
        return ControlTargets(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            steering_angle=steering_angle,
            steering_input=normalized_steering,
            gear=normalized_gear,
        )

    angular_velocity = -normalized_steering * float(max_angular_speed)
    return ControlTargets(
        linear_velocity=linear_velocity,
        angular_velocity=angular_velocity,
        steering_angle=0.0,
        steering_input=normalized_steering,
        gear=normalized_gear,
    )


def adapter_command_to_targets(
    linear_velocity: float,
    angular_velocity: float,
    steering_angle: float,
    drive_mode: str,
    max_linear_speed: float = 10.0,
    ackermann_steering_limit: float = DEFAULT_ACKERMANN_STEERING_LIMIT,
    ackermann_wheel_base: float = DEFAULT_ACKERMANN_WHEEL_BASE,
) -> ControlTargets:
    """Map adapter commands into the values that should be published to Gazebo."""
    mode = normalize_drive_mode(drive_mode)
    if mode == PRIUS_DRIVE_MODE:
        requested_linear_velocity = float(linear_velocity)
        normalized_steering = 0.0
        if abs(ackermann_steering_limit) > 1e-9:
            normalized_steering = clamp(
                -float(steering_angle) / float(ackermann_steering_limit),
                -1.0,
                1.0,
            )
        normalized_throttle = clamp(
            abs(requested_linear_velocity) / max(float(max_linear_speed), 1e-9),
            0.0,
            1.0,
        )
        if requested_linear_velocity > 1e-6:
            gear = GEAR_DRIVE
        elif requested_linear_velocity < -1e-6:
            gear = GEAR_REVERSE
        else:
            gear = GEAR_NEUTRAL

        return ControlTargets(
            linear_velocity=0.0,
            angular_velocity=0.0,
            steering_angle=0.0,
            throttle=normalized_throttle,
            brake=0.0,
            steering_input=normalized_steering,
            gear=gear,
        )

    if mode == ACKERMANN_DRIVE_MODE:
        clamped_steering = clamp(
            float(steering_angle),
            -float(ackermann_steering_limit),
            float(ackermann_steering_limit),
        )
        return ControlTargets(
            linear_velocity=float(linear_velocity),
            angular_velocity=steering_angle_to_yaw_rate(
                linear_velocity,
                clamped_steering,
                ackermann_wheel_base,
            ),
            steering_angle=clamped_steering,
            steering_input=clamp(
                -clamped_steering / float(ackermann_steering_limit),
                -1.0,
                1.0,
            ),
            gear=GEAR_UNKNOWN,
        )

    return ControlTargets(
        linear_velocity=float(linear_velocity),
        angular_velocity=float(angular_velocity),
        steering_angle=float(steering_angle),
        gear=GEAR_UNKNOWN,
    )
